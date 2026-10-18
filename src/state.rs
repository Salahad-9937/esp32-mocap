//! Потокобезопасные буферы телеметрии, структуры измерений и статистика потока.

use std::sync::atomic::{AtomicBool, AtomicU64};
use std::sync::{Arc, Mutex};
use std::time::Instant;

/// Сырые 16-битные знаковые показания АЦП одного узла BMI160.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RawNodeMeasurement {
    pub gx: i16,
    pub gy: i16,
    pub gz: i16,
    pub ax: i16,
    pub ay: i16,
    pub az: i16,
}

/// Физически нормализованные кинематические величины в единицах СИ.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct CalibratedMeasurement {
    pub accel_mps2: [f32; 3],
    pub gyro_rads: [f32; 3],
}

/// Полный дескриптор состояния активного узла с ориентацией и привязкой к кости.
#[derive(Debug, Clone, PartialEq)]
pub struct NodeTelemetry {
    pub mux_address: u8,
    pub channel: u8,
    pub raw: RawNodeMeasurement,
    pub calibrated: CalibratedMeasurement,
    pub orientation_quat: [f32; 4],
    pub bone_tag: Option<String>,
    pub peak_energy: f32,
}

impl Default for NodeTelemetry {
    fn default() -> Self {
        Self {
            mux_address: 0x70,
            channel: 0,
            raw: RawNodeMeasurement::default(),
            calibrated: CalibratedMeasurement::default(),
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

/// Совокупный снимок телеметрии, разделяемый между потоками.
#[derive(Debug)]
pub struct SharedTelemetryState {
    pub nodes: Mutex<Vec<NodeTelemetry>>,
    pub stats: StreamStats,
    pub is_running: AtomicBool,
    pub is_paused: AtomicBool,
    pub user_wants_connected: AtomicBool,
    pub auto_reconnect: AtomicBool,
    pub reconnect_interval_ms: AtomicU64,
    pub calculated_fps: Mutex<f32>,
    pub last_frame_instant: Mutex<Option<Instant>>,
    pub connection_error: Mutex<Option<String>>,
}

impl Default for SharedTelemetryState {
    fn default() -> Self {
        Self {
            nodes: Mutex::new(Vec::new()),
            stats: StreamStats::default(),
            is_running: AtomicBool::new(false),
            is_paused: AtomicBool::new(false),
            user_wants_connected: AtomicBool::new(true),
            auto_reconnect: AtomicBool::new(true),
            reconnect_interval_ms: AtomicU64::new(2000),
            calculated_fps: Mutex::new(0.0),
            last_frame_instant: Mutex::new(None),
            connection_error: Mutex::new(None),
        }
    }
}

pub type SharedStateHandle = Arc<SharedTelemetryState>;