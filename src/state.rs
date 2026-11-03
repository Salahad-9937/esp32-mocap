//! # Состояние пользовательской сессии UI и доменные буферы (`state`)

use mocap_core::calibration::NodeBias;
use mocap_core::mapping::MappingProfile;
use mocap_core::protocol::RawNodeData;
use mocap_core::types::ImuSample;
use std::sync::atomic::{AtomicBool, AtomicU64, AtomicU8};
use std::sync::{Arc, Mutex};
use std::time::Instant;

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

/// Домен физического соединения и контроля радиоканала.
#[derive(Debug)]
pub struct ConnectionDomain {
    pub status: Mutex<ConnectionStatus>,
    pub stats: StreamStats,
    pub is_running: AtomicBool,
    pub is_paused: AtomicBool,
    pub user_wants_connected: AtomicBool,
    pub auto_reconnect: AtomicBool,
    pub reconnect_interval_ms: AtomicU64,
}

impl Default for ConnectionDomain {
    fn default() -> Self {
        Self {
            status: Mutex::new(ConnectionStatus::Connecting),
            stats: StreamStats::default(),
            is_running: AtomicBool::new(true),
            is_paused: AtomicBool::new(false),
            user_wants_connected: AtomicBool::new(true),
            auto_reconnect: AtomicBool::new(true),
            reconnect_interval_ms: AtomicU64::new(500),
        }
    }
}

/// Домен входящей телеметрии и частотных характеристик.
#[derive(Debug, Default)]
pub struct TelemetryDomain {
    pub nodes: Mutex<Vec<NodeTelemetry>>,
    pub detected_mux_mask: AtomicU8,
    pub calculated_fps: Mutex<f32>,
    pub last_frame_instant: Mutex<Option<Instant>>,
}

/// Домен калибровочного автомата (ноль гироскопов и соосность N-позы).
#[derive(Debug, Default)]
pub struct CalibrationDomain {
    pub request_calibration: AtomicBool,
    pub request_npose_alignment: AtomicBool,
    pub progress: Mutex<Option<f32>>,
    pub active_biases: Mutex<Vec<NodeBias>>,
}

/// Домен профиля анатомической привязки каналов к скелету.
#[derive(Debug, Default)]
pub struct MappingDomain {
    pub profile: Mutex<MappingProfile>,
}

/// Совокупное состояние сессии, разделяемое между потоками по доменным границам.
#[derive(Debug, Default)]
pub struct SharedTelemetryState {
    pub connection: ConnectionDomain,
    pub telemetry: TelemetryDomain,
    pub calibration: CalibrationDomain,
    pub mapping: MappingDomain,
}

impl SharedTelemetryState {
    #[must_use]
    pub fn connection_status(&self) -> ConnectionStatus {
        self.connection
            .status
            .lock()
            .map(|s| s.clone())
            .unwrap_or(ConnectionStatus::Disconnected)
    }

    pub fn set_connection_status(&self, status: ConnectionStatus) {
        if let Ok(mut lock) = self.connection.status.lock() {
            *lock = status;
        }
    }

    #[must_use]
    pub fn snapshot_nodes(&self) -> Vec<NodeTelemetry> {
        self.telemetry
            .nodes
            .lock()
            .map(|g| g.clone())
            .unwrap_or_default()
    }
}

pub type SharedStateHandle = Arc<SharedTelemetryState>;