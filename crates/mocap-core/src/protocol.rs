//! # Разбор протокола и циклический кадрировщик потока байтов (`protocol`)

use crate::error::{MocapError, Result};

pub const FRAME_PREAMBLE: [u8; 2] = [0xAA, 0x55];
pub const MAX_NODES: usize = 128;
pub const HEADER_LEN_V1: usize = 5;
pub const HEADER_LEN_V2: usize = 6;
pub const NODE_DATA_LEN_12B: usize = 12;
pub const NODE_DATA_LEN_14B: usize = 14;
pub const NODE_DATA_LEN_15B: usize = 15;
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
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RawNodeData {
    pub mux_address: u8,
    pub channel: u8,
    pub sensor_address: u8,
    pub gyr_x: i16,
    pub gyr_y: i16,
    pub gyr_z: i16,
    pub acc_x: i16,
    pub acc_y: i16,
    pub acc_z: i16,
}

impl Default for RawNodeData {
    fn default() -> Self {
        Self {
            mux_address: 0x70,
            channel: 0,
            sensor_address: 0x68,
            gyr_x: 0,
            gyr_y: 0,
            gyr_z: 0,
            acc_x: 0,
            acc_y: 0,
            acc_z: 0,
        }
    }
}

impl RawNodeData {
    /// Распаковывает 15-байтовый срез BMI160 (1B MUX + 1B CH + 1B ADDR + 12B IMU).
    #[must_use]
    pub fn from_bytes_15b(slice: &[u8]) -> Option<Self> {
        if slice.len() < NODE_DATA_LEN_15B {
            return None;
        }

        Some(Self {
            mux_address: slice[0],
            channel: slice[1],
            sensor_address: slice[2],
            gyr_x: i16::from_le_bytes([slice[3], slice[4]]),
            gyr_y: i16::from_le_bytes([slice[5], slice[6]]),
            gyr_z: i16::from_le_bytes([slice[7], slice[8]]),
            acc_x: i16::from_le_bytes([slice[9], slice[10]]),
            acc_y: i16::from_le_bytes([slice[11], slice[12]]),
            acc_z: i16::from_le_bytes([slice[13], slice[14]]),
        })
    }

    /// Распаковывает 14-байтовый срез BMI160 (1B MUX + 1B CH/ADDR + 12B IMU).
    #[must_use]
    pub fn from_bytes_14b(slice: &[u8]) -> Option<Self> {
        if slice.len() < NODE_DATA_LEN_14B {
            return None;
        }

        let mux_address = slice[0];
        let raw_ch = slice[1];
        let channel = raw_ch & 0x7F;
        let sensor_address = if (raw_ch & 0x80) != 0 { 0x69 } else { 0x68 };

        Some(Self {
            mux_address,
            channel,
            sensor_address,
            gyr_x: i16::from_le_bytes([slice[2], slice[3]]),
            gyr_y: i16::from_le_bytes([slice[4], slice[5]]),
            gyr_z: i16::from_le_bytes([slice[6], slice[7]]),
            acc_x: i16::from_le_bytes([slice[8], slice[9]]),
            acc_y: i16::from_le_bytes([slice[10], slice[11]]),
            acc_z: i16::from_le_bytes([slice[12], slice[13]]),
        })
    }

    /// Распаковывает 12-байтовый срез BMI160 с внешним указанием адресов.
    #[must_use]
    pub fn from_bytes_12b(mux_address: u8, channel: u8, sensor_address: u8, slice: &[u8]) -> Option<Self> {
        if slice.len() < NODE_DATA_LEN_12B {
            return None;
        }

        Some(Self {
            mux_address,
            channel,
            sensor_address,
            gyr_x: i16::from_le_bytes([slice[0], slice[1]]),
            gyr_y: i16::from_le_bytes([slice[2], slice[3]]),
            gyr_z: i16::from_le_bytes([slice[4], slice[5]]),
            acc_x: i16::from_le_bytes([slice[6], slice[7]]),
            acc_y: i16::from_le_bytes([slice[8], slice[9]]),
            acc_z: i16::from_le_bytes([slice[10], slice[11]]),
        })
    }
}

/// Полный верифицированный бинарный кадр от концентратора ESP32.
#[derive(Debug, Clone, PartialEq)]
pub struct RawFrame {
    pub sequence_id: u16,
    pub node_count: u8,
    pub mux_mask: u8,
    pub nodes: Vec<RawNodeData>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PayloadLayout {
    Len15,
    Len14,
    Len12,
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
            buffer: Vec::with_capacity(4096),
            max_capacity: 16384,
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

    /// Извлекает следующий готовый кадр из буфера с поддержкой форматов 15B, 14B и 12B.
    pub fn poll_frame(&mut self) -> Result<Option<RawFrame>> {
        loop {
            if self.buffer.len() < HEADER_LEN_V1 {
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

            if self.buffer.len() < HEADER_LEN_V1 {
                return Ok(None);
            }

            let node_count = self.buffer[4];
            if node_count == 0 || node_count as usize > MAX_NODES {
                self.buffer.drain(..1);
                continue;
            }

            let n_nodes = node_count as usize;
            let frame_len_15b_v2 = HEADER_LEN_V2 + n_nodes * NODE_DATA_LEN_15B + CRC_LEN;
            let frame_len_15b_v1 = HEADER_LEN_V1 + n_nodes * NODE_DATA_LEN_15B + CRC_LEN;
            let frame_len_14b = HEADER_LEN_V1 + n_nodes * NODE_DATA_LEN_14B + CRC_LEN;
            let frame_len_12b = HEADER_LEN_V1 + n_nodes * NODE_DATA_LEN_12B + CRC_LEN;

            let (frame_len, header_len, layout) = if self.buffer.len() >= frame_len_15b_v2
                && Self::check_crc_slice(&self.buffer[..frame_len_15b_v2])
            {
                (frame_len_15b_v2, HEADER_LEN_V2, PayloadLayout::Len15)
            } else if self.buffer.len() >= frame_len_15b_v1
                && Self::check_crc_slice(&self.buffer[..frame_len_15b_v1])
            {
                (frame_len_15b_v1, HEADER_LEN_V1, PayloadLayout::Len15)
            } else if self.buffer.len() >= frame_len_14b
                && Self::check_crc_slice(&self.buffer[..frame_len_14b])
            {
                (frame_len_14b, HEADER_LEN_V1, PayloadLayout::Len14)
            } else if self.buffer.len() >= frame_len_12b
                && Self::check_crc_slice(&self.buffer[..frame_len_12b])
            {
                (frame_len_12b, HEADER_LEN_V1, PayloadLayout::Len12)
            } else {
                let max_len = frame_len_15b_v2.max(frame_len_14b).max(frame_len_12b);
                if self.buffer.len() >= max_len {
                    self.buffer.drain(..1);
                    continue;
                }
                return Ok(None);
            };

            let seq_id = u16::from_le_bytes([self.buffer[2], self.buffer[3]]);
            let mut mux_mask = if header_len == HEADER_LEN_V2 { self.buffer[5] } else { 0 };

            let mut nodes = Vec::with_capacity(n_nodes);
            let payload_start = header_len;

            for i in 0..n_nodes {
                let node = match layout {
                    PayloadLayout::Len15 => {
                        let offset = payload_start + i * NODE_DATA_LEN_15B;
                        RawNodeData::from_bytes_15b(&self.buffer[offset..offset + NODE_DATA_LEN_15B])
                    }
                    PayloadLayout::Len14 => {
                        let offset = payload_start + i * NODE_DATA_LEN_14B;
                        RawNodeData::from_bytes_14b(&self.buffer[offset..offset + NODE_DATA_LEN_14B])
                    }
                    PayloadLayout::Len12 => {
                        let offset = payload_start + i * NODE_DATA_LEN_12B;
                        let def_mux = 0x70 + (i as u8 / 8);
                        let def_ch = i as u8 % 8;
                        RawNodeData::from_bytes_12b(def_mux, def_ch, 0x68, &self.buffer[offset..offset + NODE_DATA_LEN_12B])
                    }
                };

                if let Some(n) = node {
                    if header_len != HEADER_LEN_V2 && n.mux_address >= 0x70 && n.mux_address <= 0x77 {
                        mux_mask |= 1 << (n.mux_address - 0x70);
                    }
                    nodes.push(n);
                }
            }

            self.buffer.drain(..frame_len);

            return Ok(Some(RawFrame {
                sequence_id: seq_id,
                node_count,
                mux_mask,
                nodes,
            }));
        }
    }

    fn check_crc_slice(frame_slice: &[u8]) -> bool {
        let len = frame_slice.len();
        if len < HEADER_LEN_V1 + CRC_LEN {
            return false;
        }
        let expected_crc = u16::from_le_bytes([frame_slice[len - 2], frame_slice[len - 1]]);
        let actual_crc = crc16_ccitt(&frame_slice[2..len - 2]);
        expected_crc == actual_crc
    }
}