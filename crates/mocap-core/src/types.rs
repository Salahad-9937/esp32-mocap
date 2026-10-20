//! # Базовые математические примитивы и физические структуры измерений (`types`)

use std::ops::{Add, Mul, Sub};

/// Трехмерный декартов вектор вещественной точности (f64).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    #[inline]
    #[must_use]
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    #[inline]
    #[must_use]
    pub fn length_squared(self) -> f64 {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    #[inline]
    #[must_use]
    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    #[inline]
    #[must_use]
    pub fn normalize(self) -> Self {
        let len = self.length();
        if len > 1e-9 {
            let inv = 1.0 / len;
            Self { x: self.x * inv, y: self.y * inv, z: self.z * inv }
        } else {
            Self::ZERO
        }
    }

    #[inline]
    #[must_use]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }

    #[inline]
    #[must_use]
    pub fn cross(self, rhs: Self) -> Self {
        Self {
            x: self.y * rhs.z - self.z * rhs.y,
            y: self.z * rhs.x - self.x * rhs.z,
            z: self.x * rhs.y - self.y * rhs.x,
        }
    }
}

impl Add for Vector3 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Vector3 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Mul<f64> for Vector3 {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

/// Кватернион пространственной ориентации (скаляр w, вектор x, y, z).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quaternion {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Quaternion {
    pub const IDENTITY: Self = Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };

    #[inline]
    #[must_use]
    pub const fn new(w: f64, x: f64, y: f64, z: f64) -> Self {
        Self { w, x, y, z }
    }

    #[inline]
    #[must_use]
    pub fn normalize(self) -> Self {
        let len_sq = self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z;
        if len_sq > 1e-12 {
            let inv = 1.0 / len_sq.sqrt();
            Self { w: self.w * inv, x: self.x * inv, y: self.y * inv, z: self.z * inv }
        } else {
            Self::IDENTITY
        }
    }

    #[inline]
    #[must_use]
    pub fn to_f32_array(self) -> [f32; 4] {
        [self.w as f32, self.x as f32, self.y as f32, self.z as f32]
    }
}

impl Mul for Quaternion {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

/// Единичный срез измерений одного датчика в физических величинах СИ.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ImuSample {
    pub accel: Vector3,
    pub gyro: Vector3,
    pub raw_gyro: [i16; 3],
    pub raw_accel: [i16; 3],
}

impl ImuSample {
    #[inline]
    #[must_use]
    pub fn kinetic_energy(&self) -> f32 {
        self.gyro.length() as f32
    }
}