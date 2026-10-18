//! Типизированные ошибки библиотеки захвата движений `mocap-core`.

use std::fmt;

/// Результат выполнения операций библиотеки.
pub type Result<T> = std::result::Result<T, MocapError>;

/// Перечисление возможных сбоев при работе с потоком данных и датчиками.
#[derive(Debug)]
pub enum MocapError {
    /// Ошибка ввода-вывода последовательного порта
    Transport(String),
    /// Переполнение внутреннего кольцевого буфера кадрировщика
    BufferOverflow,
    /// Несовпадение контрольной суммы кадра CRC-16
    CrcMismatch { expected: u16, actual: u16 },
    /// Некорректное значение количества узлов в заголовке кадра
    InvalidNodeCount(u8),
    /// Процедура статической калибровки отклонена из-за движения оператора
    CalibrationFailed(String),
    /// Ошибка привязки топологии или маппинга каналов
    MappingError(String),
    /// Ошибка сериализации/десериализации профилей конфигурации
    Serialization(String),
}

impl fmt::Display for MocapError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Transport(msg) => write!(f, "Ошибка транспорта последовательного порта: {msg}"),
            Self::BufferOverflow => write!(f, "Переполнение буфера входящего потока данных"),
            Self::CrcMismatch { expected, actual } => write!(
                f,
                "Сбой контрольной суммы кадра CRC-16 (ожидалось: 0x{expected:04X}, получено: 0x{actual:04X})"
            ),
            Self::InvalidNodeCount(count) => write!(f, "Недопустимое количество узлов в кадре: {count}"),
            Self::CalibrationFailed(reason) => write!(f, "Сбой процедуры калибровки нуля: {reason}"),
            Self::MappingError(msg) => write!(f, "Ошибка маппинга топологии: {msg}"),
            Self::Serialization(msg) => write!(f, "Ошибка сериализации профиля конфигурации: {msg}"),
        }
    }
}

impl std::error::Error for MocapError {}

impl From<serialport::Error> for MocapError {
    fn from(err: serialport::Error) -> Self {
        Self::Transport(err.to_string())
    }
}

impl From<toml::de::Error> for MocapError {
    fn from(err: toml::de::Error) -> Self {
        Self::Serialization(err.to_string())
    }
}

impl From<toml::ser::Error> for MocapError {
    fn from(err: toml::ser::Error) -> Self {
        Self::Serialization(err.to_string())
    }
}