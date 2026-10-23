//! # Автономный фоновый поток приема и сквозной обработки телеметрии (`ingestion`)

use crate::state::{NodeTelemetry, SharedStateHandle};
use mocap_core::client::MocapProcessor;
use std::io::Read;
use std::sync::atomic::Ordering;
use std::thread;
use std::time::{Duration, Instant};

/// Запускает фоновый поток опроса физического порта через `MocapProcessor`.
pub fn spawn_background_ingestion(
    shared: SharedStateHandle,
    target_endpoint: String,
    baud_rate: u32,
) {
    thread::spawn(move || {
        let mut processor = MocapProcessor::new();

        while shared.is_running.load(Ordering::Relaxed) {
            if !shared.user_wants_connected.load(Ordering::Relaxed) {
                reset_shared_metrics(&shared, Some("Отключено пользователем".to_string()));
                thread::sleep(Duration::from_millis(150));
                continue;
            }

            let mut stream = match open_stream(&target_endpoint, baud_rate, &shared) {
                Some(s) => s,
                None => {
                    let interval = shared.reconnect_interval_ms.load(Ordering::Relaxed);
                    thread::sleep(Duration::from_millis(interval));
                    continue;
                }
            };

            run_stream_loop(&mut stream, &mut processor, &shared);

            let interval = shared.reconnect_interval_ms.load(Ordering::Relaxed);
            thread::sleep(Duration::from_millis(interval));
        }
    });
}

fn open_stream(
    target: &str,
    baud_rate: u32,
    shared: &SharedStateHandle,
) -> Option<Box<dyn Read>> {
    #[cfg(target_os = "linux")]
    if target.contains(':') && target.len() == 17 {
        for ch in [1, 2] {
            if !shared.user_wants_connected.load(Ordering::Relaxed) {
                break;
            }
            if let Ok(s) = mocap_core::transport::BtRfcommStream::connect(target, ch, Duration::from_millis(80)) {
                clear_connection_error(shared);
                return Some(Box::new(s));
            }
        }

        reset_shared_metrics(shared, Some(format!("Поиск устройства {target}...")));
        if !shared.auto_reconnect.load(Ordering::Relaxed) {
            shared.user_wants_connected.store(false, Ordering::SeqCst);
        }
        return None;
    }

    match serialport::new(target, baud_rate).timeout(Duration::from_millis(50)).open() {
        Ok(p) => {
            clear_connection_error(shared);
            Some(Box::new(p))
        }
        Err(e) => {
            reset_shared_metrics(shared, Some(format!("Ожидание порта {target}: {e}")));
            if !shared.auto_reconnect.load(Ordering::Relaxed) {
                shared.user_wants_connected.store(false, Ordering::SeqCst);
            }
            None
        }
    }
}

fn run_stream_loop(
    stream: &mut Box<dyn Read>,
    processor: &mut MocapProcessor,
    shared: &SharedStateHandle,
) {
    let mut read_chunk = [0u8; 1024];
    let mut last_seq_id: Option<u16> = None;
    let mut frame_count_sec = 0u32;
    let mut last_fps_timer = Instant::now();
    let mut last_activity = Instant::now();
    let mut last_step_time = Instant::now();
    let mut last_raw_nodes = Vec::new();

    while shared.is_running.load(Ordering::Relaxed) && shared.user_wants_connected.load(Ordering::Relaxed) {
        if shared.is_paused.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(25));
            continue;
        }

        if shared.request_calibration.swap(false, Ordering::SeqCst) {
            processor.start_calibration(200);
        }

        if shared.request_npose_alignment.swap(false, Ordering::SeqCst) {
            processor.capture_npose_alignment(&last_raw_nodes);
        }

        if let Ok(profile) = shared.mapping_profile.try_lock() {
            processor.set_mapping_profile(profile.clone());
        }

        let dt_sec = last_step_time.elapsed().as_secs_f64().clamp(0.001, 0.05);
        last_step_time = Instant::now();

        match stream.read(&mut read_chunk) {
            Ok(0) => {
                reset_shared_metrics(shared, Some("Устройство отключилось (EOF)".to_string()));
                break;
            }
            Ok(bytes_read) => {
                last_activity = Instant::now();

                let frames = match processor.ingest_bytes(&read_chunk[..bytes_read], dt_sec) {
                    Ok(f) => f,
                    Err(_) => {
                        shared.stats.crc_errors.fetch_add(1, Ordering::Relaxed);
                        continue;
                    }
                };

                for frame in frames {
                    shared.detected_mux_mask.store(frame.mux_mask, Ordering::Relaxed);

                    if let Some(last_id) = last_seq_id {
                        let diff = frame.sequence_id.wrapping_sub(last_id);
                        if diff > 1 {
                            shared.stats.packets_dropped.fetch_add((diff - 1) as u64, Ordering::Relaxed);
                        }
                    }
                    last_seq_id = Some(frame.sequence_id);
                    shared.stats.packets_total.fetch_add(1, Ordering::Relaxed);

                    if let Ok(mut inst_guard) = shared.last_frame_instant.lock() {
                        *inst_guard = Some(Instant::now());
                    }

                    let mut telemetry_nodes = Vec::with_capacity(frame.nodes.len());
                    last_raw_nodes.clear();

                    for node in frame.nodes {
                        let energy = node.sample.kinetic_energy();
                        last_raw_nodes.push(mocap_core::protocol::RawNodeData {
                            mux_address: node.mux_address,
                            channel: node.channel,
                            sensor_address: node.sensor_address,
                            gyr_x: node.sample.raw_gyro[0],
                            gyr_y: node.sample.raw_gyro[1],
                            gyr_z: node.sample.raw_gyro[2],
                            acc_x: node.sample.raw_accel[0],
                            acc_y: node.sample.raw_accel[1],
                            acc_z: node.sample.raw_accel[2],
                        });

                        telemetry_nodes.push(NodeTelemetry {
                            mux_address: node.mux_address,
                            channel: node.channel,
                            sensor_address: node.sensor_address,
                            raw: last_raw_nodes.last().copied().unwrap_or_default(),
                            calibrated: node.sample,
                            orientation_quat: node.orientation.to_f32_array(),
                            bone_tag: node.bone_name,
                            peak_energy: energy,
                        });
                    }

                    if let Ok(mut nodes_guard) = shared.nodes.lock() {
                        for new_node in &mut telemetry_nodes {
                            if let Some(old_node) = nodes_guard.iter().find(|n| {
                                n.mux_address == new_node.mux_address
                                    && n.channel == new_node.channel
                                    && n.sensor_address == new_node.sensor_address
                            }) {
                                new_node.peak_energy = new_node.peak_energy.max(old_node.peak_energy * 0.95);
                            }
                        }
                        *nodes_guard = telemetry_nodes;
                    }

                    frame_count_sec += 1;
                }

                if let Ok(mut prog) = shared.calibration_progress.lock() {
                    *prog = processor.calibration_progress();
                }

                if !processor.is_calibrating() && !processor.biases().is_empty() {
                    if let Ok(mut biases) = shared.active_biases.lock() {
                        *biases = processor.biases().to_vec();
                    }
                }

                if last_fps_timer.elapsed() >= Duration::from_secs(1) {
                    if let Ok(mut fps_guard) = shared.calculated_fps.lock() {
                        *fps_guard = frame_count_sec as f32;
                    }
                    frame_count_sec = 0;
                    last_fps_timer = Instant::now();
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                if last_activity.elapsed() > Duration::from_millis(500) {
                    reset_shared_metrics(shared, Some("Потеря связи: таймаут потока".to_string()));
                    break;
                }
            }
            Err(e) => {
                reset_shared_metrics(shared, Some(format!("Ошибка связи: {e}")));
                break;
            }
        }
    }
}

fn reset_shared_metrics(shared: &SharedStateHandle, error_msg: Option<String>) {
    if let Ok(mut err_guard) = shared.connection_error.lock() {
        *err_guard = error_msg;
    }
    if let Ok(mut nodes_guard) = shared.nodes.lock() {
        nodes_guard.clear();
    }
    if let Ok(mut fps_guard) = shared.calculated_fps.lock() {
        *fps_guard = 0.0;
    }
    shared.detected_mux_mask.store(0, Ordering::Relaxed);
}

fn clear_connection_error(shared: &SharedStateHandle) {
    if let Ok(mut err_guard) = shared.connection_error.lock() {
        *err_guard = None;
    }
}