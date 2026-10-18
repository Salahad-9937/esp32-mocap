//! Модульный конвейер оценивания пространственной ориентации (Sensor Fusion).

use crate::types::{ImuSample, Quaternion, Vector3};

/// Перечисление поддерживаемых алгоритмов фильтрации ориентации.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FilterAlgorithm {
    #[default]
    Madgwick,
    Mahony,
    Bypass,
}

/// Абстрактный контракт фильтра ориентации узла.
pub trait OrientationFilter: Send {
    /// Вычисляет единичный кватернион ориентации по калиброванным ускорениям и угловым скоростям.
    fn update(&mut self, sample: &ImuSample, dt: f64) -> Quaternion;

    /// Сбрасывает ориентацию фильтра к единичному кватерниону.
    fn reset(&mut self);
}

/// 64-битная реализация фильтра Маджвика с градиентным спуском по гравитации.
pub struct MadgwickFilter {
    pub beta: f64,
    pub q: Quaternion,
}

impl Default for MadgwickFilter {
    fn default() -> Self {
        Self::new(0.1)
    }
}

impl MadgwickFilter {
    #[must_use]
    pub fn new(beta: f64) -> Self {
        Self {
            beta: beta.max(0.001),
            q: Quaternion::IDENTITY,
        }
    }
}

impl OrientationFilter for MadgwickFilter {
    fn update(&mut self, sample: &ImuSample, dt: f64) -> Quaternion {
        if dt <= 0.0 {
            return self.q;
        }

        let mut q = self.q;
        let gx = sample.gyro.x;
        let gy = sample.gyro.y;
        let gz = sample.gyro.z;

        let a_len = sample.accel.length();

        if a_len > 1e-4 {
            let inv_a = 1.0 / a_len;
            let ax = sample.accel.x * inv_a;
            let ay = sample.accel.y * inv_a;
            let az = sample.accel.z * inv_a;

            // Вспомогательные переменные
            let _2q0 = 2.0 * q.w;
            let _2q1 = 2.0 * q.x;
            let _2q2 = 2.0 * q.y;
            let _2q3 = 2.0 * q.z;
            let _4q0 = 4.0 * q.w;
            let _4q1 = 4.0 * q.x;
            let _4q2 = 4.0 * q.y;
            let _8q1 = 8.0 * q.x;
            let _8q2 = 8.0 * q.y;
            let q0q0 = q.w * q.w;
            let q1q1 = q.x * q.x;
            let q2q2 = q.y * q.y;
            let q3q3 = q.z * q.z;

            // Вычисление градиента целевой функции гравитации
            let mut s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            let mut s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q.x - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            let mut s2 = 4.0 * q0q0 * q.y + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            let mut s3 = 4.0 * q1q1 * q.z - _2q1 * ax + 4.0 * q2q2 * q.z - _2q2 * ay;

            let s_len = (s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3).sqrt();
            if s_len > 1e-9 {
                let inv_s = 1.0 / s_len;
                s0 *= inv_s;
                s1 *= inv_s;
                s2 *= inv_s;
                s3 *= inv_s;

                // Интегрирование скорости изменения кватерниона
                let q_dot_w = 0.5 * (-q.x * gx - q.y * gy - q.z * gz) - self.beta * s0;
                let q_dot_x = 0.5 * (q.w * gx + q.y * gz - q.z * gy) - self.beta * s1;
                let q_dot_y = 0.5 * (q.w * gy - q.x * gz + q.z * gx) - self.beta * s2;
                let q_dot_z = 0.5 * (q.w * gz + q.x * gy - q.y * gx) - self.beta * s3;

                q.w += q_dot_w * dt;
                q.x += q_dot_x * dt;
                q.y += q_dot_y * dt;
                q.z += q_dot_z * dt;
            }
        } else {
            // При невесомости / ускорениях свободного падения чистое интегрирование гироскопа
            let q_dot_w = 0.5 * (-q.x * gx - q.y * gy - q.z * gz);
            let q_dot_x = 0.5 * (q.w * gx + q.y * gz - q.z * gy);
            let q_dot_y = 0.5 * (q.w * gy - q.x * gz + q.z * gx);
            let q_dot_z = 0.5 * (q.w * gz + q.x * gy - q.y * gx);

            q.w += q_dot_w * dt;
            q.x += q_dot_x * dt;
            q.y += q_dot_y * dt;
            q.z += q_dot_z * dt;
        }

        self.q = q.normalize();
        self.q
    }

    fn reset(&mut self) {
        self.q = Quaternion::IDENTITY;
    }
}

/// Фильтр Махони с пропорционально-интегральной обратной связью.
pub struct MahonyFilter {
    pub kp: f64,
    pub ki: f64,
    pub q: Quaternion,
    integral_fb: Vector3,
}

impl Default for MahonyFilter {
    fn default() -> Self {
        Self::new(2.5, 0.0)
    }
}

impl MahonyFilter {
    #[must_use]
    pub fn new(kp: f64, ki: f64) -> Self {
        Self {
            kp: kp.max(0.0),
            ki: ki.max(0.0),
            q: Quaternion::IDENTITY,
            integral_fb: Vector3::ZERO,
        }
    }
}

impl OrientationFilter for MahonyFilter {
    fn update(&mut self, sample: &ImuSample, dt: f64) -> Quaternion {
        if dt <= 0.0 {
            return self.q;
        }

        let mut gx = sample.gyro.x;
        let mut gy = sample.gyro.y;
        let mut gz = sample.gyro.z;

        let a_len = sample.accel.length();
        if a_len > 1e-4 {
            let inv_a = 1.0 / a_len;
            let ax = sample.accel.x * inv_a;
            let ay = sample.accel.y * inv_a;
            let az = sample.accel.z * inv_a;

            // Расчет расчетного вектора гравитации из текущего кватерниона
            let vx = 2.0 * (self.q.x * self.q.z - self.q.w * self.q.y);
            let vy = 2.0 * (self.q.w * self.q.x + self.q.y * self.q.z);
            let vz = self.q.w * self.q.w - self.q.x * self.q.x - self.q.y * self.q.y + self.q.z * self.q.z;

            // Векторная ошибка между оценкой и измерением
            let ex = ay * vz - az * vy;
            let ey = az * vx - ax * vz;
            let ez = ax * vy - ay * vx;

            if self.ki > 0.0 {
                self.integral_fb.x += self.ki * ex * dt;
                self.integral_fb.y += self.ki * ey * dt;
                self.integral_fb.z += self.ki * ez * dt;
                gx += self.integral_fb.x;
                gy += self.integral_fb.y;
                gz += self.integral_fb.z;
            }

            gx += self.kp * ex;
            gy += self.kp * ey;
            gz += self.kp * ez;
        }

        // Интегрирование скорости кватерниона
        let mut q = self.q;
        let q_dot_w = 0.5 * (-q.x * gx - q.y * gy - q.z * gz);
        let q_dot_x = 0.5 * (q.w * gx + q.y * gz - q.z * gy);
        let q_dot_y = 0.5 * (q.w * gy - q.x * gz + q.z * gx);
        let q_dot_z = 0.5 * (q.w * gz + q.x * gy - q.y * gx);

        q.w += q_dot_w * dt;
        q.x += q_dot_x * dt;
        q.y += q_dot_y * dt;
        q.z += q_dot_z * dt;

        self.q = q.normalize();
        self.q
    }

    fn reset(&mut self) {
        self.q = Quaternion::IDENTITY;
        self.integral_fb = Vector3::ZERO;
    }
}

/// Фильтр прямого пропуска (возвращает единичную ориентацию без вычислений).
#[derive(Default)]
pub struct BypassFilter;

impl OrientationFilter for BypassFilter {
    fn update(&mut self, _sample: &ImuSample, _dt: f64) -> Quaternion {
        Quaternion::IDENTITY
    }

    fn reset(&mut self) {}
}

/// Фабрика создания экземпляров фильтра ориентации.
#[must_use]
pub fn create_orientation_filter(algo: FilterAlgorithm) -> Box<dyn OrientationFilter> {
    match algo {
        FilterAlgorithm::Madgwick => Box::new(MadgwickFilter::default()),
        FilterAlgorithm::Mahony => Box::new(MahonyFilter::default()),
        FilterAlgorithm::Bypass => Box::new(BypassFilter),
    }
}