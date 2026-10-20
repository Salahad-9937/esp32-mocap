//! # Разбор протокола и циклический кадрировщик потока байтов (`protocol`)

use crate::error::{MocapError, Result};

pub const FRAME_PREAMBLE: [u8; 2] = [0xAA, 0x55];
pub const MAX_NODES: usize = 64;
pub const HEADER_LEN: usize = 5;
pub const NODE_DATA_LEN_12B: usize = 12;
pub const NODE_DATA_LEN_14B: usize = 14;
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
    pub mux_address: u8,
    pub channel: u8,
    pub gyr_x: i16,
    pub gyr_y: i16,
    pub gyr_z: i16,
    pub acc_x: i16,
    pub acc_y: i16,
    pub acc_z: i16,
}

impl RawNodeData {
    /// Распаковывает 14-байтовый срез BMI160 (1B MUX + 1B CH + 12B IMU).
    #[must_use]
    pub fn from_bytes_14b(slice: &[u8]) -> Option<Self> {
        if slice.len() < NODE_DATA_LEN_14B {
            return None;
        }

        let mux_address = slice[0];
        let channel = slice[1];
        let gyr_x = i16::from_le_bytes([slice[2], slice[3]]);
        let gyr_y = i16::from_le_bytes([slice[4], slice[5]]);
        let gyr_z = i16::from_le_bytes([slice[6], slice[7]]);
        let acc_x = i16::from_le_bytes([slice[8], slice[9]]);
        let acc_y = i16::from_le_bytes([slice[10], slice[11]]);
        let acc_z = i16::from_le_bytes([slice[12], slice[13]]);

        Some(Self {
            mux_address,
            channel,
            gyr_x,
            gyr_y,
            gyr_z,
            acc_x,
            acc_y,
            acc_z,
        })
    }

    /// Распаковывает 12-байтовый срез BMI160 с внешним указанием адреса и канала.
    #[must_use]
    pub fn from_bytes_12b(mux_address: u8, channel: u8, slice: &[u8]) -> Option<Self> {
        if slice.len() < NODE_DATA_LEN_12B {
            return None;
        }

        let gyr_x = i16::from_le_bytes([slice[0], slice[1]]);
        let gyr_y = i16::from_le_bytes([slice[2], slice[3]]);
        let gyr_z = i16::from_le_bytes([slice[4], slice[5]]);
        let acc_x = i16::from_le_bytes([slice[6], slice[7]]);
        let acc_y = i16::from_le_bytes([slice[8], slice[9]]);
        let acc_z = i16::from_le_bytes([slice[10], slice[11]]);

        Some(Self {
            mux_address,
            channel,
            gyr_x,
            gyr_y,
            gyr_z,
            acc_x,
            acc_y,
            acc_z,
        })
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
            max_capacity: 8192,
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

    /// Извлекает следующий готовый кадр из буфера с валидацией 14-байтных и 12-байтных пакетов.
    pub fn poll_frame(&mut self) -> Result<Option<RawFrame>> {
        loop {
            if self.buffer.len() < HEADER_LEN {
                return Ok(None);
            }

            let preamble_pos = self.buffer.windows(2).position(|w| w == FRAME_PREAMBLE);
            let Some(pos) = preamble_pos else {
                let keep_last = usize::from(self.buffer.last().copied() == Some(FRAME_PREAMBLE[0]));
                let drain_len = self.buffer.len().saturating_sub(keep_last);
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
                self.buffer.drain(..1);
                continue;
            }

            let n_nodes = node_count as usize;
            let frame_len_14b = HEADER_LEN + n_nodes * NODE_DATA_LEN_14B + CRC_LEN;
            let frame_len_12b = HEADER_LEN + n_nodes * NODE_DATA_LEN_12B + CRC_LEN;

            let (frame_len, is_14b) = if self.buffer.len() >= frame_len_14b
                && Self::check_crc_slice(&self.buffer[..frame_len_14b])
            {
                (frame_len_14b, true)
            } else if self.buffer.len() >= frame_len_12b
                && Self::check_crc_slice(&self.buffer[..frame_len_12b])
            {
                (frame_len_12b, false)
            } else {
                if self.buffer.len() >= frame_len_14b.max(frame_len_12b) {
                    self.buffer.drain(..1);
                    continue;
                }
                return Ok(None);
            };

            let seq_id = u16::from_le_bytes([self.buffer[2], self.buffer[3]]);
            let mut nodes = Vec::with_capacity(n_nodes);
            let payload_start = HEADER_LEN;

            for i in 0..n_nodes {
                let node = if is_14b {
                    let offset = payload_start + i * NODE_DATA_LEN_14B;
                    RawNodeData::from_bytes_14b(&self.buffer[offset..offset + NODE_DATA_LEN_14B])
                } else {
                    let offset = payload_start + i * NODE_DATA_LEN_12B;
                    let def_mux = 0x70 + (i as u8 / 8);
                    let def_ch = i as u8 % 8;
                    RawNodeData::from_bytes_12b(def_mux, def_ch, &self.buffer[offset..offset + NODE_DATA_LEN_12B])
                };

                if let Some(n) = node {
                    nodes.push(n);
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

    fn check_crc_slice(frame_slice: &[u8]) -> bool {
        let len = frame_slice.len();
        if len < HEADER_LEN + CRC_LEN {
            return false;
        }
        let expected_crc = u16::from_le_bytes([frame_slice[len - 2], frame_slice[len - 1]]);
        let actual_crc = crc16_ccitt(&frame_slice[2..len - 2]);
        expected_crc == actual_crc
    }
}