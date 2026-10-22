//! # Состояние пользовательской сессии UI и потокобезопасные буферы (`state`)

use mocap_core::calibration::NodeBias;
use mocap_core::mapping::MappingProfile;
use mocap_core::protocol::RawNodeData;
use mocap_core::types::ImuSample;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8};
use std::sync::{Arc, Mutex};
use std::time::Instant;

pub type RawNodeMeasurement = RawNodeData;
pub type CalibratedMeasurement = ImuSample;

/// Полный дескриптор состояния активного узла с ориентацией и привязкой к кости.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeTelemetry {
    pub mux_address: u8,
    pub channel: u8,
    pub sensor_address: u8,
    pub raw: RawNodeData,
    pub calibrated: ImuSample,
    pub orientation_quat: [f32; 4],
    pub bone_tag: Option<String>,
    pub peak_energy: f32,
}

impl Default for NodeTelemetry {
    fn default() -> Self {
        Self {
            mux_address: 0x70,
            channel: 0,
            sensor_address: 0x68,
            raw: RawNodeData::default(),
            calibrated: ImuSample::default(),
            orientation_quat: [1.0, 0.0, 0.0, 0.0],
            bone_tag: None,
            peak_energy: 0.0,
        }
    }
}

/// Статус подключения к источнику телеметрии.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ConnectionStatus {
    Disconnected,
    Connecting,
    Connected,
    Paused,
    Error(String),
}

/// Диагностические метрики радиоканала и целостности кадров.
#[derive(Debug, Default)]
pub struct StreamStats {
    pub packets_total: AtomicU64,
    pub packets_dropped: AtomicU64,
    pub crc_errors: AtomicU64,
}

/// Совокупный снимок телеметрии и команд, разделяемый между потоками.
#[derive(Debug)]
pub struct SharedTelemetryState {
    pub nodes: Mutex<Vec<NodeTelemetry>>,
    pub detected_mux_mask: AtomicU8,
    pub stats: StreamStats,
    pub is_running: AtomicBool,
    pub is_paused: AtomicBool,
    pub user_wants_connected: AtomicBool,
    pub auto_reconnect: AtomicBool,
    pub reconnect_interval_ms: AtomicU64,
    pub calculated_fps: Mutex<f32>,
    pub last_frame_instant: Mutex<Option<Instant>>,
    pub connection_error: Mutex<Option<String>>,
    pub request_calibration: AtomicBool,
    pub calibration_progress: Mutex<Option<f32>>,
    pub active_biases: Mutex<Vec<NodeBias>>,
    pub mapping_profile: Mutex<MappingProfile>,
}

impl Default for SharedTelemetryState {
    fn default() -> Self {
        Self {
            nodes: Mutex::new(Vec::new()),
            detected_mux_mask: AtomicU8::new(0),
            stats: StreamStats::default(),
            is_running: AtomicBool::new(false),
            is_paused: AtomicBool::new(false),
            user_wants_connected: AtomicBool::new(true),
            auto_reconnect: AtomicBool::new(true),
            reconnect_interval_ms: AtomicU64::new(2000),
            calculated_fps: Mutex::new(0.0),
            last_frame_instant: Mutex::new(None),
            connection_error: Mutex::new(None),
            request_calibration: AtomicBool::new(false),
            calibration_progress: Mutex::new(None),
            active_biases: Mutex::new(Vec::new()),
            mapping_profile: Mutex::new(MappingProfile::default()),
        }
    }
}

pub type SharedStateHandle = Arc<SharedTelemetryState>;