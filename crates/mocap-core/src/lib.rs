//! # `mocap-core`
//!
//! Автономная headless-библиотека приема, декодирования бинарных кадров,
//! калибровки и открытого конвейера ориентации (Sensor Fusion) для ESP32 MoCap.

pub mod calibration;
pub mod client;
pub mod error;
pub mod fusion;
pub mod mapping;
pub mod protocol;
pub mod transport;
pub mod types;

// Плоский публичный реэкспорт основных интерфейсов
pub use calibration::{BiasEstimator, Calibrator, NodeBias, ACCEL_SCALE_4G, GYRO_SCALE_1000DPS};
pub use client::{MocapProcessor, ProcessedFrame, ProcessedNode};
pub use error::{MocapError, Result};
pub use fusion::{
    create_orientation_filter, BypassFilter, FilterAlgorithm, MadgwickFilter, MahonyFilter,
    OrientationFilter,
};
pub use mapping::{HardwareChannel, MappingEntry, MappingProfile};
pub use protocol::{crc16_ccitt, RawFrame, RawNodeData, StreamFramer};
pub use transport::{ByteStream, SerialTransport};
pub use types::{ImuSample, Quaternion, Vector3};