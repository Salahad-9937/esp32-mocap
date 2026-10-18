//! Высокоуровневый координатор приема, калибровки и фильтрации потока MoCap.

use crate::calibration::{BiasEstimator, Calibrator};
use crate::error::Result;
use crate::fusion::{create_orientation_filter, FilterAlgorithm, OrientationFilter};
use crate::mapping::MappingProfile;
use crate::protocol::{RawFrame, StreamFramer};
use crate::types::{ImuSample, Quaternion};

/// Обработанное состояние единичного измерительного узла.
#[derive(Debug, Clone, PartialEq)]
pub struct ProcessedNode {
    pub node_index: usize,
    pub bone_name: Option<String>,
    pub sample: ImuSample,
    pub orientation: Quaternion,
}

/// Полный обработанный кадр физического измерения.
#[derive(Debug, Clone, PartialEq)]
pub struct ProcessedFrame {
    pub sequence_id: u16,
    pub nodes: Vec<ProcessedNode>,
}

/// Центральный вычислительный процессор хоста для потока MoCap.
pub struct MocapProcessor {
    framer: StreamFramer,
    calibrator: Calibrator,
    bias_estimator: Option<BiasEstimator>,
    mapping: Option<MappingProfile>,
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
            mapping: None,
            filters: Vec::new(),
            filter_algorithm: FilterAlgorithm::Madgwick,
        }
    }

    /// Запускает процедуру статической калибровки гироскопов по выборке покоя.
    pub fn start_calibration(&mut self, node_count: usize, target_samples: usize) {
        self.bias_estimator = Some(BiasEstimator::new(node_count, target_samples));
    }

    /// Проверяет, активен ли процесс накопления калибровочной выборки.
    #[inline]
    #[must_use]
    pub fn is_calibrating(&self) -> bool {
        self.bias_estimator.as_ref().is_some_and(|e| !e.is_complete())
    }

    /// Возвращает прогресс калибровки в диапазоне 0.0 .. 1.0.
    #[must_use]
    pub fn calibration_progress(&self) -> f32 {
        self.bias_estimator.as_ref().map_or(0.0, |e| e.progress())
    }

    /// Устанавливает активный профиль привязки каналов к костям.
    pub fn set_mapping_profile(&mut self, profile: MappingProfile) {
        self.mapping = Some(profile);
    }

    /// Переключает тип используемого алгоритма ориентации.
    pub fn set_filter_algorithm(&mut self, algo: FilterAlgorithm) {
        self.filter_algorithm = algo;
        for f in &mut self.filters {
            *f = create_orientation_filter(algo);
        }
    }

    /// Сбрасывает ориентацию всех активных фильтров.
    pub fn reset_orientations(&mut self) {
        for f in &mut self.filters {
            f.reset();
        }
    }

    /// Принимает сырые байты из транспорта, декодирует кадры и производит вычисления.
    pub fn ingest_bytes(&mut self, bytes: &[u8], dt: f64) -> Result<Vec<ProcessedFrame>> {
        self.framer.push_bytes(bytes)?;
        let mut processed_frames = Vec::new();

        while let Some(raw_frame) = self.framer.poll_frame()? {
            // Обработка выборки калибровки при активности
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

        // Подстройка пула фильтров под текущее количество активных узлов
        while self.filters.len() < node_count {
            self.filters.push(create_orientation_filter(self.filter_algorithm));
        }

        let mut processed_nodes = Vec::with_capacity(node_count);

        for (idx, raw_node) in raw_frame.nodes.iter().enumerate() {
            let sample = self.calibrator.calibrate_node(idx, raw_node);
            let orientation = self.filters[idx].update(&sample, dt);
            let bone_name = self.mapping.as_ref().and_then(|m| m.find_bone(idx)).map(String::from);

            processed_nodes.push(ProcessedNode {
                node_index: idx,
                bone_name,
                sample,
                orientation,
            });
        }

        ProcessedFrame {
            sequence_id: raw_frame.sequence_id,
            nodes: processed_nodes,
        }
    }
}