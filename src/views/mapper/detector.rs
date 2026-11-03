//! # Автомат накопления кинетической энергии и селекции активного канала (`detector`)

use crate::state::NodeTelemetry;
use mocap_core::mapping::MappingProfile;
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Терминальное событие завершения окна детекции кинетической активности.
pub enum DetectionEvent {
    Detected { channel_key: String, target_bone: String },
    NoMovement,
}

/// Стейт-машина скользящей интеграции энергии движений за временное окно.
pub struct KineticDetector {
    target_bone: Option<String>,
    start_time: Option<Instant>,
    accumulated: HashMap<String, f32>,
    duration: Duration,
    threshold: f32,
}

impl Default for KineticDetector {
    fn default() -> Self {
        Self::new(Duration::from_secs(2), 0.80)
    }
}

impl KineticDetector {
    #[must_use]
    pub fn new(duration: Duration, threshold: f32) -> Self {
        Self {
            target_bone: None,
            start_time: None,
            accumulated: HashMap::new(),
            duration,
            threshold,
        }
    }

    pub fn start(&mut self, target_bone: &str) {
        self.target_bone = Some(target_bone.to_string());
        self.start_time = Some(Instant::now());
        self.accumulated.clear();
    }

    pub fn cancel(&mut self) {
        self.target_bone = None;
        self.start_time = None;
        self.accumulated.clear();
    }

    #[must_use]
    pub fn target_bone(&self) -> Option<&str> {
        self.target_bone.as_deref()
    }

    #[must_use]
    pub fn progress(&self) -> f32 {
        let Some(start) = self.start_time else { return 0.0 };
        (start.elapsed().as_secs_f32() / self.duration.as_secs_f32()).clamp(0.0, 1.0)
    }

    pub fn update(&mut self, nodes: &[NodeTelemetry]) -> Option<DetectionEvent> {
        let target_bone = self.target_bone.clone()?;
        let start = self.start_time.get_or_insert_with(Instant::now);
        let elapsed = start.elapsed();

        for node in nodes {
            let key = MappingProfile::channel_key(node.mux_address, node.channel, node.sensor_address);
            *self.accumulated.entry(key).or_default() += node.peak_energy;
        }

        if elapsed < self.duration {
            return None;
        }

        let mut best_channel = None;
        let mut max_accum = self.threshold;

        for (key, &accum) in &self.accumulated {
            if accum > max_accum {
                max_accum = accum;
                best_channel = Some(key.clone());
            }
        }

        self.cancel();

        if let Some(channel_key) = best_channel {
            Some(DetectionEvent::Detected { channel_key, target_bone })
        } else {
            Some(DetectionEvent::NoMovement)
        }
    }
}