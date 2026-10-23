//! # Высокоуровневый процессор приема, калибровки и фильтрации потока MoCap (`client`)

use crate::calibration::{BiasEstimator, Calibrator, NodeBias};
use crate::error::Result;
use crate::fusion::{create_orientation_filter, FilterAlgorithm, OrientationFilter};
use crate::mapping::MappingProfile;
use crate::protocol::{RawFrame, StreamFramer};
use crate::types::{ImuSample, Quaternion};

#[derive(Debug, Clone, PartialEq)]
pub struct ProcessedNode {
    pub node_index: usize,
    pub mux_address: u8,
    pub channel: u8,
    pub sensor_address: u8,
    pub bone_name: Option<String>,
    pub sample: ImuSample,
    pub orientation: Quaternion,
    pub raw_orientation: Quaternion,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ProcessedFrame {
    pub sequence_id: u16,
    pub mux_mask: u8,
    pub nodes: Vec<ProcessedNode>,
}

pub struct MocapProcessor {
    framer: StreamFramer,
    calibrator: Calibrator,
    bias_estimator: Option<BiasEstimator>,
    mapping: MappingProfile,
    filters: Vec<Box<dyn OrientationFilter>>,
    filter_algorithm: FilterAlgorithm,
    last_raw_orientations: Vec<Quaternion>,
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
            last_raw_orientations: Vec::new(),
        }
    }

    pub fn start_calibration(&mut self, target_samples: usize) {
        self.bias_estimator = Some(BiasEstimator::new(128, target_samples));
    }

    #[inline]
    #[must_use]
    pub fn is_calibrating(&self) -> bool {
        self.bias_estimator.as_ref().is_some_and(|e| !e.is_complete())
    }

    #[must_use]
    pub fn calibration_progress(&self) -> Option<f32> {
        self.bias_estimator.as_ref().map(BiasEstimator::progress)
    }

    pub fn set_mapping_profile(&mut self, profile: MappingProfile) {
        self.mapping = profile;
    }

    #[must_use]
    pub fn mapping_profile(&self) -> &MappingProfile {
        &self.mapping
    }

    #[must_use]
    pub fn biases(&self) -> &[NodeBias] {
        &self.calibrator.biases
    }

    pub fn set_biases(&mut self, biases: Vec<NodeBias>) {
        self.calibrator.set_biases(biases);
    }

    pub fn set_alignment(&mut self, key: String, alignment: Quaternion) {
        self.calibrator.set_alignment(key, alignment);
    }

    #[must_use]
    pub fn calibrator(&self) -> &Calibrator {
        &self.calibrator
    }

    pub fn capture_npose_alignment(&mut self, raw_nodes: &[crate::protocol::RawNodeData]) {
        for (i, node) in raw_nodes.iter().enumerate() {
            if let Some(&raw_q) = self.last_raw_orientations.get(i) {
                let align_q = Quaternion::new(raw_q.w, -raw_q.x, -raw_q.y, -raw_q.z).normalize();
                let key = MappingProfile::channel_key(node.mux_address, node.channel, node.sensor_address);
                self.calibrator.set_alignment(key, align_q);
            }
        }
    }

    pub fn ingest_bytes(&mut self, bytes: &[u8], dt: f64) -> Result<Vec<ProcessedFrame>> {
        self.framer.push_bytes(bytes)?;
        let mut processed_frames = Vec::new();

        while let Some(raw_frame) = self.framer.poll_frame()? {
            if let Some(estimator) = &mut self.bias_estimator {
                estimator.push_sample(&raw_frame.nodes);
                if estimator.is_complete() {
                    match estimator.finalize() {
                        Ok(biases) => {
                            self.calibrator.set_biases(biases);
                        }
                        Err(e) => {
                            eprintln!("[CALIBRATION] Предупреждение калибровки покоя: {e}");
                        }
                    }
                    // Гарантированно очищаем сборщик, чтобы исключить блокировку потока телеметрии
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
        self.last_raw_orientations.resize(node_count, Quaternion::IDENTITY);

        let mut processed_nodes = Vec::with_capacity(node_count);

        for (idx, raw_node) in raw_frame.nodes.iter().enumerate() {
            let sample = self.calibrator.calibrate_node(idx, raw_node);
            let raw_orientation = self.filters[idx].update(&sample, dt);
            self.last_raw_orientations[idx] = raw_orientation;

            let key = MappingProfile::channel_key(raw_node.mux_address, raw_node.channel, raw_node.sensor_address);
            let align_q = self.calibrator.get_alignment(&key);
            let orientation = (raw_orientation * align_q).normalize();

            let bone_name = self.mapping.get_bone(raw_node.mux_address, raw_node.channel, raw_node.sensor_address).map(String::from);

            processed_nodes.push(ProcessedNode {
                node_index: idx,
                mux_address: raw_node.mux_address,
                channel: raw_node.channel,
                sensor_address: raw_node.sensor_address,
                bone_name,
                sample,
                orientation,
                raw_orientation,
            });
        }

        ProcessedFrame {
            sequence_id: raw_frame.sequence_id,
            mux_mask: raw_frame.mux_mask,
            nodes: processed_nodes,
        }
    }
}