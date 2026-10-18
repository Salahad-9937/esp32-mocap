//! Модуль калибровки аппаратного нуля гироскопов и физического масштабирования.

use crate::error::{MocapError, Result};
use crate::protocol::RawNodeData;
use crate::types::{ImuSample, Vector3};

// Константы масштабных коэффициентов Bosch BMI160
pub const ACCEL_SCALE_4G: f64 = 9.80665 / 8192.0; // 8192 LSB/g -> м/с²
pub const GYRO_SCALE_1000DPS: f64 = (std::f64::consts::PI / 180.0) / 32.8; // 32.8 LSB/(°/s) -> рад/с

/// Статическое смещение нуля трехосевого гироскопа.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct NodeBias {
    pub gyro: Vector3,
}

/// Накопитель калибровочной выборки в состоянии покоя.
pub struct BiasEstimator {
    target_samples: usize,
    collected_samples: usize,
    accumulators: Vec<Vector3>,
    variance_acc: Vec<Vector3>,
}

impl BiasEstimator {
    #[must_use]
    pub fn new(node_count: usize, target_samples: usize) -> Self {
        Self {
            target_samples: target_samples.max(10),
            collected_samples: 0,
            accumulators: vec![Vector3::ZERO; node_count],
            variance_acc: vec![Vector3::ZERO; node_count],
        }
    }

    /// Добавляет срез сырых данных текущего кадра в выборку.
    pub fn push_sample(&mut self, raw_nodes: &[RawNodeData]) {
        if self.is_complete() {
            return;
        }

        for (i, node) in raw_nodes.iter().enumerate() {
            if i >= self.accumulators.len() {
                break;
            }
            let g = Vector3::new(node.gyr_x as f64, node.gyr_y as f64, node.gyr_z as f64);
            self.accumulators[i] = self.accumulators[i] + g;
            self.variance_acc[i] = self.variance_acc[i] + Vector3::new(g.x * g.x, g.y * g.y, g.z * g.z);
        }
        self.collected_samples += 1;
    }

    #[inline]
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.collected_samples >= self.target_samples
    }

    #[inline]
    #[must_use]
    pub fn progress(&self) -> f32 {
        (self.collected_samples as f32 / self.target_samples as f32).clamp(0.0, 1.0)
    }

    /// Вычисляет итоговый вектор смещений с проверкой на наличие движения оператора.
    pub fn finalize(&self) -> Result<Vec<NodeBias>> {
        if !self.is_complete() {
            return Err(MocapError::CalibrationFailed("Недостаточно накопленных выборок".into()));
        }

        let inv_n = 1.0 / (self.collected_samples as f64);
        let mut biases = Vec::with_capacity(self.accumulators.len());

        for i in 0..self.accumulators.len() {
            let mean = self.accumulators[i] * inv_n;
            let mean_sq = self.variance_acc[i] * inv_n;
            let variance = (mean_sq.x - mean.x * mean.x)
                + (mean_sq.y - mean.y * mean.y)
                + (mean_sq.z - mean.z * mean.z);

            // Если дисперсия шума превышает 400 LSB², датчик двигался во время калибровки
            if variance > 400.0 {
                return Err(MocapError::CalibrationFailed(format!(
                    "Обнаружено паразитное движение узла #{} (дисперсия {:.1})",
                    i + 1,
                    variance
                )));
            }

            biases.push(NodeBias { gyro: mean });
        }

        Ok(biases)
    }
}

/// Калибратор для нормализации сырых данных узлов в физические величины СИ.
#[derive(Debug, Clone, Default)]
pub struct Calibrator {
    pub biases: Vec<NodeBias>,
}

impl Calibrator {
    #[must_use]
    pub fn new() -> Self {
        Self { biases: Vec::new() }
    }

    pub fn set_biases(&mut self, biases: Vec<NodeBias>) {
        self.biases = biases;
    }

    /// Преобразует сырые коды в физический срез с вычитанием калибровочного нуля.
    #[must_use]
    pub fn calibrate_node(&self, node_idx: usize, raw: &RawNodeData) -> ImuSample {
        let bias = self.biases.get(node_idx).copied().unwrap_or_default();

        let raw_gx = raw.gyr_x as f64;
        let raw_gy = raw.gyr_y as f64;
        let raw_gz = raw.gyr_z as f64;

        let corrected_gx = raw_gx - bias.gyro.x;
        let corrected_gy = raw_gy - bias.gyro.y;
        let corrected_gz = raw_gz - bias.gyro.z;

        let gyro = Vector3::new(
            corrected_gx * GYRO_SCALE_1000DPS,
            corrected_gy * GYRO_SCALE_1000DPS,
            corrected_gz * GYRO_SCALE_1000DPS,
        );

        let accel = Vector3::new(
            (raw.acc_x as f64) * ACCEL_SCALE_4G,
            (raw.acc_y as f64) * ACCEL_SCALE_4G,
            (raw.acc_z as f64) * ACCEL_SCALE_4G,
        );

        ImuSample {
            accel,
            gyro,
            raw_gyro: [raw.gyr_x, raw.gyr_y, raw.gyr_z],
            raw_accel: [raw.acc_x, raw.acc_y, raw.acc_z],
        }
    }
}