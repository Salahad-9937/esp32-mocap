//! # Векторные трансформации и поворот координат ориентации (`axes`)

/// Поворачивает трехмерный декартов вектор с помощью единичного кватерниона [w, x, y, z].
#[inline]
#[must_use]
pub fn rotate_vector_by_quat(v: [f32; 3], q: [f32; 4]) -> [f32; 3] {
    let qw = q[0];
    let qx = q[1];
    let qy = q[2];
    let qz = q[3];

    let ix = qw * v[0] + qy * v[2] - qz * v[1];
    let iy = qw * v[1] + qz * v[0] - qx * v[2];
    let iz = qw * v[2] + qx * v[1] - qy * v[0];
    let iw = -qx * v[0] - qy * v[1] - qz * v[2];

    [
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx,
    ]
}