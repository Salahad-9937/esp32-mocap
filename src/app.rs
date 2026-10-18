//! Главная оконная стейт-машина MoCapApp и автономный фоновый поток приема телеметрии.

use crate::config::AppConfig;
use crate::state::{
    CalibratedMeasurement, ConnectionStatus, NodeTelemetry, RawNodeMeasurement,
    SharedStateHandle, SharedTelemetryState,
};
use crate::transport::BtRfcommStream;
use crate::views::{CalibrationView, MapperView, TopologyView};
use crate::visuals::{render_3d_scene, OrbitalCamera};
use egui::{Color32, Frame, Margin, RichText, Stroke, TextWrapMode, Visuals};
use std::io::Read;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveTab {
    Topology,
    Mapper,
    Calibration,
}

pub struct MocapApp {
    pub config: AppConfig,
    pub shared: SharedStateHandle,
    pub connection_status: ConnectionStatus,
    pub active_tab: ActiveTab,
    pub camera: OrbitalCamera,
    pub topology_view: TopologyView,
    pub mapper_view: MapperView,
    pub calibration_view: CalibrationView,
}

impl MocapApp {
    #[must_use]
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let config = AppConfig::default();
        let shared = Arc::new(SharedTelemetryState::default());

        shared.is_running.store(true, Ordering::SeqCst);
        shared.user_wants_connected.store(true, Ordering::SeqCst);
        shared.auto_reconnect.store(config.auto_reconnect, Ordering::SeqCst);
        shared.reconnect_interval_ms.store(500, Ordering::SeqCst);

        Self::spawn_background_ingestion(
            Arc::clone(&shared),
            config.default_port.clone(),
            config.baud_rate,
        );

        let mut mapper_view = MapperView::default();
        mapper_view.load_profile(&config.mapping_profile_path);

        Self {
            config,
            shared,
            connection_status: ConnectionStatus::Connecting,
            active_tab: ActiveTab::Topology,
            camera: OrbitalCamera::default(),
            topology_view: TopologyView::default(),
            mapper_view,
            calibration_view: CalibrationView::default(),
        }
    }

    fn spawn_background_ingestion(shared: SharedStateHandle, target_endpoint: String, baud_rate: u32) {
        thread::spawn(move || {
            while shared.is_running.load(Ordering::Relaxed) {
                if !shared.user_wants_connected.load(Ordering::Relaxed) {
                    if let Ok(mut err_guard) = shared.connection_error.lock() {
                        *err_guard = Some("Отключено пользователем".to_string());
                    }
                    if let Ok(mut nodes_guard) = shared.nodes.lock() {
                        nodes_guard.clear();
                    }
                    if let Ok(mut fps_guard) = shared.calculated_fps.lock() {
                        *fps_guard = 0.0;
                    }
                    thread::sleep(Duration::from_millis(150));
                    continue;
                }

                let mut stream: Box<dyn Read> = if target_endpoint.contains(':') && target_endpoint.len() == 17 {
                    let mut connected_socket = None;
                    for ch in [1, 2] {
                        if !shared.user_wants_connected.load(Ordering::Relaxed) {
                            break;
                        }
                        if let Ok(s) = BtRfcommStream::connect(&target_endpoint, ch, Duration::from_millis(80)) {
                            connected_socket = Some(s);
                            break;
                        }
                    }

                    match connected_socket {
                        Some(s) => {
                            if let Ok(mut err_guard) = shared.connection_error.lock() {
                                *err_guard = None;
                            }
                            Box::new(s)
                        }
                        None => {
                            if let Ok(mut nodes_guard) = shared.nodes.lock() {
                                nodes_guard.clear();
                            }
                            if let Ok(mut fps_guard) = shared.calculated_fps.lock() {
                                *fps_guard = 0.0;
                            }
                            if let Ok(mut err_guard) = shared.connection_error.lock() {
                                *err_guard = Some(format!("Поиск устройства {target_endpoint}..."));
                            }

                            if !shared.auto_reconnect.load(Ordering::Relaxed) {
                                shared.user_wants_connected.store(false, Ordering::SeqCst);
                                continue;
                            }

                            let interval = shared.reconnect_interval_ms.load(Ordering::Relaxed);
                            thread::sleep(Duration::from_millis(interval));
                            continue;
                        }
                    }
                } else {
                    let port_builder = serialport::new(&target_endpoint, baud_rate)
                        .timeout(Duration::from_millis(50));

                    match port_builder.open() {
                        Ok(p) => {
                            if let Ok(mut err_guard) = shared.connection_error.lock() {
                                *err_guard = None;
                            }
                            Box::new(p)
                        }
                        Err(e) => {
                            if let Ok(mut nodes_guard) = shared.nodes.lock() {
                                nodes_guard.clear();
                            }
                            if let Ok(mut fps_guard) = shared.calculated_fps.lock() {
                                *fps_guard = 0.0;
                            }
                            if let Ok(mut err_guard) = shared.connection_error.lock() {
                                *err_guard = Some(format!("Ожидание порта {target_endpoint}: {e}"));
                            }

                            if !shared.auto_reconnect.load(Ordering::Relaxed) {
                                shared.user_wants_connected.store(false, Ordering::SeqCst);
                                continue;
                            }

                            let interval = shared.reconnect_interval_ms.load(Ordering::Relaxed);
                            thread::sleep(Duration::from_millis(interval));
                            continue;
                        }
                    }
                };

                let mut ring_buffer = Vec::with_capacity(2048);
                let mut read_chunk = [0u8; 512];
                let mut last_seq_id: Option<u16> = None;
                let mut frame_count_sec = 0u32;
                let mut last_fps_timer = Instant::now();
                let mut last_activity = Instant::now();

                while shared.is_running.load(Ordering::Relaxed) && shared.user_wants_connected.load(Ordering::Relaxed) {
                    if shared.is_paused.load(Ordering::Relaxed) {
                        thread::sleep(Duration::from_millis(25));
                        continue;
                    }

                    match stream.read(&mut read_chunk) {
                        Ok(0) => {
                            if let Ok(mut nodes_guard) = shared.nodes.lock() {
                                nodes_guard.clear();
                            }
                            if let Ok(mut fps_guard) = shared.calculated_fps.lock() {
                                *fps_guard = 0.0;
                            }
                            if let Ok(mut err_guard) = shared.connection_error.lock() {
                                *err_guard = Some("Устройство отключилось (EOF)".to_string());
                            }
                            break;
                        }
                        Ok(bytes_read) => {
                            last_activity = Instant::now();
                            ring_buffer.extend_from_slice(&read_chunk[..bytes_read]);

                            while ring_buffer.len() >= 7 {
                                if ring_buffer[0] != 0xAA || ring_buffer[1] != 0x55 {
                                    ring_buffer.remove(0);
                                    continue;
                                }

                                let node_count = ring_buffer[4] as usize;
                                if !(1..=64).contains(&node_count) {
                                    ring_buffer.remove(0);
                                    continue;
                                }

                                let frame_len_14b = 7 + node_count * 14;
                                let frame_len_12b = 7 + node_count * 12;

                                let (frame_len, bytes_per_node, is_14b) = if ring_buffer.len() >= frame_len_14b
                                    && calculate_crc16(&ring_buffer[2..frame_len_14b - 2])
                                        == ((ring_buffer[frame_len_14b - 2] as u16)
                                            | ((ring_buffer[frame_len_14b - 1] as u16) << 8))
                                {
                                    (frame_len_14b, 14, true)
                                } else if ring_buffer.len() >= frame_len_12b
                                    && calculate_crc16(&ring_buffer[2..frame_len_12b - 2])
                                        == ((ring_buffer[frame_len_12b - 2] as u16)
                                            | ((ring_buffer[frame_len_12b - 1] as u16) << 8))
                                {
                                    (frame_len_12b, 12, false)
                                } else {
                                    if ring_buffer.len() >= frame_len_14b.max(frame_len_12b) {
                                        ring_buffer.remove(0);
                                        continue;
                                    }
                                    break;
                                };

                                let seq_id = (ring_buffer[2] as u16) | ((ring_buffer[3] as u16) << 8);
                                if let Some(last_id) = last_seq_id {
                                    let diff = seq_id.wrapping_sub(last_id);
                                    if diff > 1 {
                                        shared.stats.packets_dropped.fetch_add((diff - 1) as u64, Ordering::Relaxed);
                                    }
                                }
                                last_seq_id = Some(seq_id);
                                shared.stats.packets_total.fetch_add(1, Ordering::Relaxed);

                                if let Ok(mut inst_guard) = shared.last_frame_instant.lock() {
                                    *inst_guard = Some(Instant::now());
                                }

                                let mut telemetry_nodes = Vec::with_capacity(node_count);
                                for i in 0..node_count {
                                    let offset = 5 + i * bytes_per_node;
                                    let (mux_address, channel, imu_offset) = if is_14b {
                                        (ring_buffer[offset], ring_buffer[offset + 1], offset + 2)
                                    } else {
                                        (0x70 + (i as u8 / 8), i as u8 % 8, offset)
                                    };

                                    let gx = i16::from_le_bytes([ring_buffer[imu_offset], ring_buffer[imu_offset + 1]]);
                                    let gy = i16::from_le_bytes([ring_buffer[imu_offset + 2], ring_buffer[imu_offset + 3]]);
                                    let gz = i16::from_le_bytes([ring_buffer[imu_offset + 4], ring_buffer[imu_offset + 5]]);
                                    let ax = i16::from_le_bytes([ring_buffer[imu_offset + 6], ring_buffer[imu_offset + 7]]);
                                    let ay = i16::from_le_bytes([ring_buffer[imu_offset + 8], ring_buffer[imu_offset + 9]]);
                                    let az = i16::from_le_bytes([ring_buffer[imu_offset + 10], ring_buffer[imu_offset + 11]]);

                                    let sa = 9.80665 / 8192.0;
                                    let sg = (std::f32::consts::PI / 180.0) / 32.8;
                                    let g_rads = [gx as f32 * sg, gy as f32 * sg, gz as f32 * sg];
                                    let current_energy = (g_rads[0].powi(2) + g_rads[1].powi(2) + g_rads[2].powi(2)).sqrt();

                                    telemetry_nodes.push(NodeTelemetry {
                                        mux_address,
                                        channel,
                                        raw: RawNodeMeasurement { gx, gy, gz, ax, ay, az },
                                        calibrated: CalibratedMeasurement {
                                            accel_mps2: [ax as f32 * sa, ay as f32 * sa, az as f32 * sa],
                                            gyro_rads: g_rads,
                                        },
                                        orientation_quat: [1.0, 0.0, 0.0, 0.0],
                                        bone_tag: None,
                                        peak_energy: current_energy,
                                    });
                                }

                                if let Ok(mut nodes_guard) = shared.nodes.lock() {
                                    for new_node in &mut telemetry_nodes {
                                        if let Some(old_node) = nodes_guard.iter().find(|n| n.mux_address == new_node.mux_address && n.channel == new_node.channel) {
                                            new_node.bone_tag = old_node.bone_tag.clone();
                                            new_node.peak_energy = new_node.peak_energy.max(old_node.peak_energy * 0.95);
                                        }
                                    }
                                    *nodes_guard = telemetry_nodes;
                                }

                                frame_count_sec += 1;
                                ring_buffer.drain(..frame_len);
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
                                if let Ok(mut nodes_guard) = shared.nodes.lock() {
                                    nodes_guard.clear();
                                }
                                if let Ok(mut fps_guard) = shared.calculated_fps.lock() {
                                    *fps_guard = 0.0;
                                }
                                if let Ok(mut err_guard) = shared.connection_error.lock() {
                                    *err_guard = Some("Потеря связи: таймаут потока".to_string());
                                }
                                break;
                            }
                        }
                        Err(e) => {
                            if let Ok(mut nodes_guard) = shared.nodes.lock() {
                                nodes_guard.clear();
                            }
                            if let Ok(mut fps_guard) = shared.calculated_fps.lock() {
                                *fps_guard = 0.0;
                            }
                            if let Ok(mut err_guard) = shared.connection_error.lock() {
                                *err_guard = Some(format!("Ошибка связи: {e}"));
                            }
                            break;
                        }
                    }
                }

                let interval = shared.reconnect_interval_ms.load(Ordering::Relaxed);
                thread::sleep(Duration::from_millis(interval));
            }
        });
    }
}

impl eframe::App for MocapApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let bg_color = Color32::from_rgb(16, 20, 28);
        ctx.set_visuals(Visuals {
            dark_mode: true,
            panel_fill: bg_color,
            window_fill: bg_color,
            extreme_bg_color: Color32::from_rgb(10, 14, 20),
            faint_bg_color: Color32::from_rgb(22, 27, 36),
            ..Visuals::dark()
        });

        if let Ok(mut inst_guard) = self.shared.last_frame_instant.lock() {
            if let Some(instant) = *inst_guard {
                if instant.elapsed() > Duration::from_millis(500) {
                    if let Ok(mut nodes_guard) = self.shared.nodes.lock() {
                        nodes_guard.clear();
                    }
                    if let Ok(mut fps_guard) = self.shared.calculated_fps.lock() {
                        *fps_guard = 0.0;
                    }
                    if let Ok(mut err_guard) = self.shared.connection_error.lock() {
                        if err_guard.is_none() || err_guard.as_deref() == Some("") {
                            *err_guard = Some("Поиск устройства (таймаут данных)...".to_string());
                        }
                    }
                }
            }
        }

        let fps = self.shared.calculated_fps.lock().map(|f| *f).unwrap_or(0.0);
        let wants_connected = self.shared.user_wants_connected.load(Ordering::Relaxed);

        if let Ok(err_guard) = self.shared.connection_error.lock() {
            if !wants_connected {
                self.connection_status = ConnectionStatus::Disconnected;
            } else if fps > 0.0 {
                self.connection_status = ConnectionStatus::Connected;
            } else if let Some(ref err_msg) = *err_guard {
                if err_msg.starts_with("Поиск")
                    || err_msg.starts_with("Ожидание")
                    || err_msg.contains("таймаут")
                    || err_msg.contains("отключилось")
                {
                    self.connection_status = ConnectionStatus::Connecting;
                } else {
                    self.connection_status = ConnectionStatus::Error(err_msg.clone());
                }
            } else {
                self.connection_status = ConnectionStatus::Connecting;
            }
        }

        if let Ok(mut nodes_guard) = self.shared.nodes.lock() {
            for node in nodes_guard.iter_mut() {
                let key = format!("0x{:02X}:{}", node.mux_address, node.channel);
                node.bone_tag = self.mapper_view.profile.mappings.get(&key).cloned();
            }
        }

        // Фиксированная ширина панели 400px: полностью вмещает матрицу 8x8 (34x26) и одинакова для всех вкладок
        egui::SidePanel::left("mocap_side_panel_static")
            .exact_width(425.0)
            .resizable(false)
            .frame(Frame {
                inner_margin: Margin::same(8.0),
                fill: bg_color,
                stroke: Stroke::new(1.0, Color32::from_rgb(38, 46, 62)),
                ..Default::default()
            })
            .show(ctx, |ui| {
                ui.style_mut().wrap_mode = Some(TextWrapMode::Wrap);

                // Верхний блок заголовка и статуса подключения
                ui.horizontal(|ui| {
                    ui.heading("ESP32 MoCap");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let (status_text, status_color) = match &self.connection_status {
                            ConnectionStatus::Connected => ("ПОТОК АКТИВЕН", Color32::GREEN),
                            ConnectionStatus::Connecting => ("ОЖИДАНИЕ", Color32::YELLOW),
                            ConnectionStatus::Paused => ("ПАУЗА", Color32::KHAKI),
                            ConnectionStatus::Disconnected => ("ОТКЛЮЧЕНО", Color32::GRAY),
                            ConnectionStatus::Error(err) => (err.as_str(), Color32::RED),
                        };
                        ui.label(RichText::new(status_text).color(status_color).strong());
                    });
                });

                ui.add_space(2.0);

                // Управление подключением
                ui.horizontal(|ui| {
                    let btn_text = match self.connection_status {
                        ConnectionStatus::Connected => "Отключиться",
                        ConnectionStatus::Connecting => "Подключиться",
                        _ => "🔌 Подключиться",
                    };

                    if ui.button(RichText::new(btn_text).color(Color32::WHITE).strong()).clicked() {
                        match self.connection_status {
                            ConnectionStatus::Connected | ConnectionStatus::Connecting => {
                                self.shared.user_wants_connected.store(false, Ordering::SeqCst);
                            }
                            _ => {
                                self.shared.user_wants_connected.store(true, Ordering::SeqCst);
                                if let Ok(mut err_guard) = self.shared.connection_error.lock() {
                                    *err_guard = None;
                                }
                            }
                        }
                    }

                    let mut auto_rec = self.shared.auto_reconnect.load(Ordering::Relaxed);
                    if ui.checkbox(&mut auto_rec, "Авто-реконнект").changed() {
                        self.shared.auto_reconnect.store(auto_rec, Ordering::SeqCst);
                    }
                });

                ui.add_space(2.0);

                // Переключатель вкладок
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut self.active_tab, ActiveTab::Topology, "🖧 Топология I2C");
                    ui.selectable_value(&mut self.active_tab, ActiveTab::Mapper, "🦴 Маппинг");
                    ui.selectable_value(&mut self.active_tab, ActiveTab::Calibration, "⚖ Калибровка");
                });

                ui.separator();

                // Содержимое активной вкладки
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        match self.active_tab {
                            ActiveTab::Topology => {
                                self.topology_view.show(ui, &self.shared, &self.config.default_port);
                            }
                            ActiveTab::Mapper => {
                                self.mapper_view.show(ui, &self.shared, &self.config.mapping_profile_path);
                            }
                            ActiveTab::Calibration => {
                                self.calibration_view.show(ui, &self.shared);
                            }
                        }
                    });
            });

        // Центральная область 3D сцены
        egui::CentralPanel::default()
            .frame(Frame {
                inner_margin: Margin::same(0.0),
                fill: bg_color,
                ..Default::default()
            })
            .show(ctx, |ui| {
                let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::drag());

                let scroll_delta = ui.input(|i| i.raw_scroll_delta.y);
                self.camera.handle_input(&response, scroll_delta);

                if response.dragged() || (response.hovered() && scroll_delta != 0.0) {
                    ui.ctx().request_repaint();
                }

                painter.rect_filled(response.rect, 0.0_f32, bg_color);

                let nodes = self.shared.nodes.lock().map(|g| g.clone()).unwrap_or_default();
                render_3d_scene(&painter, &self.camera, response.rect, &nodes);
            });

        if self.connection_status == ConnectionStatus::Connected {
            ctx.request_repaint_after(Duration::from_millis(14));
        }
    }
}

#[must_use]
pub fn calculate_crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}