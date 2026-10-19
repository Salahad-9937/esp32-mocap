//! Минимальная векторная арифметика над `[f32; 3]` (без внешних зависимостей).

pub type V3 = [f32; 3];

#[inline]
pub fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
pub fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
pub fn scale(a: V3, s: f32) -> V3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// `a + b * s`
#[inline]
pub fn madd(a: V3, b: V3, s: f32) -> V3 {
    [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s]
}

#[inline]
pub fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
pub fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
pub fn len(a: V3) -> f32 {
    dot(a, a).sqrt()
}

#[inline]
pub fn dist(a: V3, b: V3) -> f32 {
    len(sub(a, b))
}

#[inline]
pub fn norm(a: V3) -> V3 {
    let l = len(a);
    if l < 1e-9 {
        [0.0, 1.0, 0.0]
    } else {
        scale(a, 1.0 / l)
    }
}

#[inline]
pub fn lerp(a: V3, b: V3, t: f32) -> V3 {
    [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]
}

/// Зеркалирование относительно плоскости X = 0 (`s` = +1 правая сторона, −1 левая).
#[inline]
pub fn mirror(s: f32, p: V3) -> V3 {
    [p[0] * s, p[1], p[2]]
}
