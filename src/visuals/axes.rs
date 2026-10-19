//! Оси ориентации активных датчиков.

use super::viewport::OrbitalCamera;
use egui::{Color32, Painter, Rect, Stroke};

/// Поворачивает трехмерный вектор с помощью кватерниона [w, x, y, z].
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

/// Ориентированный базис (X — красный, Y — зелёный, Z — синий) из центра платы.
/// `opacity` (0..1) — для приглушения датчиков на обратной стороне тела.
pub fn draw_axes_gizmo(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    position: [f32; 3],
    quat: [f32; 4],
    axis_length: f32,
    opacity: f32,
) {
    let Some(center_2d) = camera.project(position, rect) else {
        return;
    };
    let width = if axis_length < 0.04 { 1.6_f32 } else { 2.4_f32 };
    let axes = [
        ([axis_length, 0.0, 0.0], Color32::from_rgb(240, 50, 50)),
        ([0.0, axis_length, 0.0], Color32::from_rgb(50, 220, 70)),
        ([0.0, 0.0, axis_length], Color32::from_rgb(45, 130, 255)),
    ];
    for (local, color) in axes {
        let r = rotate_vector_by_quat(local, quat);
        let tip = [position[0] + r[0], position[1] + r[1], position[2] + r[2]];
        if let Some(p) = camera.project(tip, rect) {
            painter.line_segment([center_2d, p], Stroke::new(width, color.gamma_multiply(opacity)));
        }
    }
}
