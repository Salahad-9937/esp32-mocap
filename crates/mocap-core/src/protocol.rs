//! Разбор низкоуровневого протокола передачи и циклический кадрировщик потока байтов.

use crate::error::{MocapError, Result};

pub const FRAME_PREAMBLE: [u8; 2] = [0xAA, 0x55];
pub const MAX_NODES: usize = 64;
pub const HEADER_LEN: usize = 5; // 2 байта преамбула + 2 байта sequence + 1 байт node_count
pub const NODE_DATA_LEN: usize = 12; // 6 байт гироскоп + 6 байт акселерометр
pub const CRC_LEN: usize = 2;

/// Вычисляет контрольную сумму CRC-16-CCITT (полином 0x1021, инициализация 0xFFFF).
#[must_use]
pub fn crc16_ccitt(data: &[u8]) -> u16 {
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

/// Необработанные целочисленные данные одного узла из бинарного кадра.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RawNodeData {
    pub gyr_x: i16,
    pub gyr_y: i16,
    pub gyr_z: i16,
    pub acc_x: i16,
    pub acc_y: i16,
    pub acc_z: i16,
}

impl RawNodeData {
    /// Распаковывает 12 байт последовательности BMI160 (GYR_X..Z, затем ACC_X..Z, Little-Endian).
    #[must_use]
    pub fn from_bytes(slice: &[u8]) -> Option<Self> {
        if slice.len() < NODE_DATA_LEN {
            return None;
        }

        let gyr_x = i16::from_le_bytes([slice[0], slice[1]]);
        let gyr_y = i16::from_le_bytes([slice[2], slice[3]]);
        let gyr_z = i16::from_le_bytes([slice[4], slice[5]]);
        let acc_x = i16::from_le_bytes([slice[6], slice[7]]);
        let acc_y = i16::from_le_bytes([slice[8], slice[9]]);
        let acc_z = i16::from_le_bytes([slice[10], slice[11]]);

        Some(Self { gyr_x, gyr_y, gyr_z, acc_x, acc_y, acc_z })
    }
}

/// Полный верифицированный бинарный кадр от концентратора ESP32.
#[derive(Debug, Clone, PartialEq)]
pub struct RawFrame {
    pub sequence_id: u16,
    pub node_count: u8,
    pub nodes: Vec<RawNodeData>,
}

/// Потоковый кадрировщик для выделения пакетов из произвольного потока байтов.
pub struct StreamFramer {
    buffer: Vec<u8>,
    max_capacity: usize,
}

impl Default for StreamFramer {
    fn default() -> Self {
        Self::new()
    }
}

impl StreamFramer {
    #[must_use]
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(2048),
            max_capacity: 4096,
        }
    }

    /// Добавляет порцию считанных байтов во внутренний накопитель.
    pub fn push_bytes(&mut self, bytes: &[u8]) -> Result<()> {
        if self.buffer.len() + bytes.len() > self.max_capacity {
            self.buffer.clear();
            return Err(MocapError::BufferOverflow);
        }
        self.buffer.extend_from_slice(bytes);
        Ok(())
    }

    /// Извлекает следующий готовый кадр из буфера при его доступности.
    pub fn poll_frame(&mut self) -> Result<Option<RawFrame>> {
        loop {
            if self.buffer.len() < HEADER_LEN {
                return Ok(None);
            }

            // Поиск стартовой преамбулы 0xAA 0x55
            let preamble_pos = self.buffer.windows(2).position(|w| w == FRAME_PREAMBLE);
            let Some(pos) = preamble_pos else {
                let keep_last = if self.buffer.last().copied() == Some(FRAME_PREAMBLE[0]) { 1 } else { 0 };
                let drain_len = self.buffer.len() - keep_last;
                self.buffer.drain(..drain_len);
                return Ok(None);
            };

            if pos > 0 {
                self.buffer.drain(..pos);
            }

            if self.buffer.len() < HEADER_LEN {
                return Ok(None);
            }

            let node_count = self.buffer[4];
            if node_count == 0 || node_count as usize > MAX_NODES {
                // Ложная преамбула: сдвигаем буфер на 1 байт и продолжаем поиск
                self.buffer.drain(..1);
                continue;
            }

            let frame_len = HEADER_LEN + (node_count as usize) * NODE_DATA_LEN + CRC_LEN;
            if self.buffer.len() < frame_len {
                // Кадр еще не дочитан целиком
                return Ok(None);
            }

            // Извлечение полей и расчет CRC
            let seq_id = u16::from_le_bytes([self.buffer[2], self.buffer[3]]);
            let crc_offset = frame_len - CRC_LEN;
            let expected_crc = u16::from_le_bytes([self.buffer[crc_offset], self.buffer[crc_offset + 1]]);

            // CRC считается по срезу от sequence_id до конца полезной нагрузки
            let crc_slice = &self.buffer[2..crc_offset];
            let actual_crc = crc16_ccitt(crc_slice);

            if expected_crc != actual_crc {
                // Повреждение кадра: смещаемся на 1 байт для ресинхронизации
                self.buffer.drain(..1);
                return Err(MocapError::CrcMismatch {
                    expected: expected_crc,
                    actual: actual_crc,
                });
            }

            // Распаковка полезной нагрузки узлов
            let mut nodes = Vec::with_capacity(node_count as usize);
            let payload_start = HEADER_LEN;

            for i in 0..(node_count as usize) {
                let offset = payload_start + i * NODE_DATA_LEN;
                if let Some(node) = RawNodeData::from_bytes(&self.buffer[offset..offset + NODE_DATA_LEN]) {
                    nodes.push(node);
                }
            }

            self.buffer.drain(..frame_len);

            return Ok(Some(RawFrame {
                sequence_id: seq_id,
                node_count,
                nodes,
            }));
        }
    }
}