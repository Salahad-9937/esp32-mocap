//! # Высокоуровневый процессор приема, калибровки и фильтрации потока MoCap (`client`)

use crate::calibration::{BiasEstimator, Calibrator, NodeBias};
use crate::error::Result;
use crate::fusion::{create_orientation_filter, FilterAlgorithm, OrientationFilter};
use crate::mapping::MappingProfile;
use crate::protocol::{RawFrame, StreamFramer};
use crate::types::{ImuSample, Quaternion};

/// Обработанное состояние единичного измерительного узла.
#[derive(Debug, Clone, PartialEq)]
pub struct ProcessedNode {
    pub node_index: usize,
    pub mux_address: u8,
    pub channel: u8,
    pub sensor_address: u8,
    pub bone_name: Option<String>,
    pub sample: ImuSample,
    pub orientation: Quaternion,
}

/// Полный обработанный кадр физического измерения.
#[derive(Debug, Clone, PartialEq)]
pub struct ProcessedFrame {
    pub sequence_id: u16,
    pub mux_mask: u8,
    pub nodes: Vec<ProcessedNode>,
}

/// Центральный вычислительный процессор хоста для потока MoCap.
pub struct MocapProcessor {
    framer: StreamFramer,
    calibrator: Calibrator,
    bias_estimator: Option<BiasEstimator>,
    mapping: MappingProfile,
    filters: Vec<Box<dyn OrientationFilter>>,
    filter_algorithm: FilterAlgorithm,
}

impl Default for MocapProcessor {
    fn default() -> Self {
        Self::new()
    }
}

impl MocapProcessor {
    #[must_use]
    pub fn new() -> Self {
        Self {
            framer: StreamFramer::new(),
            calibrator: Calibrator::new(),
            bias_estimator: None,
            mapping: MappingProfile::default(),
            filters: Vec::new(),
            filter_algorithm: FilterAlgorithm::Madgwick,
        }
    }

    /// Инициирует процедуру калибровки смещения нуля гироскопов в покое.
    pub fn start_calibration(&mut self, target_samples: usize) {
        self.bias_estimator = Some(BiasEstimator::new(128, target_samples));
    }

    /// Проверяет активность процедуры калибровки.
    #[inline]
    #[must_use]
    pub fn is_calibrating(&self) -> bool {
        self.bias_estimator.as_ref().is_some_and(|e| !e.is_complete())
    }

    /// Возвращает прогресс калибровки в диапазоне 0.0 .. 1.0.
    #[must_use]
    pub fn calibration_progress(&self) -> Option<f32> {
        self.bias_estimator.as_ref().map(BiasEstimator::progress)
    }

    /// Задает активный профиль маппинга каналов на кости.
    pub fn set_mapping_profile(&mut self, profile: MappingProfile) {
        self.mapping = profile;
    }

    /// Считывает активный профиль маппинга.
    #[must_use]
    pub fn mapping_profile(&self) -> &MappingProfile {
        &self.mapping
    }

    /// Считывает текущие калибровочные поправки смещения нуля.
    #[must_use]
    pub fn biases(&self) -> &[NodeBias] {
        &self.calibrator.biases
    }

    /// Устанавливает сохраненные поправки смещения нуля.
    pub fn set_biases(&mut self, biases: Vec<NodeBias>) {
        self.calibrator.set_biases(biases);
    }

    /// Принимает сырые байты, выделяет кадры, производит калибровку и расчет ориентации.
    pub fn ingest_bytes(&mut self, bytes: &[u8], dt: f64) -> Result<Vec<ProcessedFrame>> {
        self.framer.push_bytes(bytes)?;
        let mut processed_frames = Vec::new();

        while let Some(raw_frame) = self.framer.poll_frame()? {
            if let Some(estimator) = &mut self.bias_estimator {
                estimator.push_sample(&raw_frame.nodes);
                if estimator.is_complete() {
                    let biases = estimator.finalize()?;
                    self.calibrator.set_biases(biases);
                    self.bias_estimator = None;
                }
            }

            let frame = self.process_raw_frame(&raw_frame, dt);
            processed_frames.push(frame);
        }

        Ok(processed_frames)
    }

    fn process_raw_frame(&mut self, raw_frame: &RawFrame, dt: f64) -> ProcessedFrame {
        let node_count = raw_frame.nodes.len();

        while self.filters.len() < node_count {
            self.filters.push(create_orientation_filter(self.filter_algorithm));
        }

        let mut processed_nodes = Vec::with_capacity(node_count);

        for (idx, raw_node) in raw_frame.nodes.iter().enumerate() {
            let sample = self.calibrator.calibrate_node(idx, raw_node);
            let orientation = self.filters[idx].update(&sample, dt);
            let bone_name = self.mapping.get_bone(raw_node.mux_address, raw_node.channel, raw_node.sensor_address).map(String::from);

            processed_nodes.push(ProcessedNode {
                node_index: idx,
                mux_address: raw_node.mux_address,
                channel: raw_node.channel,
                sensor_address: raw_node.sensor_address,
                bone_name,
                sample,
                orientation,
            });
        }

        ProcessedFrame {
            sequence_id: raw_frame.sequence_id,
            mux_mask: raw_frame.mux_mask,
            nodes: processed_nodes,
        }
    }
}