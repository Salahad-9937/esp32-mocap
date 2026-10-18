//! Отрисовка взаимно перпендикулярных базисных осей датчика (Axes Gizmo).

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

/// Отрисовывает ориентированный базис осей (X-красный, Y-зеленый, Z-синий) в точке узла.
pub fn draw_axes_gizmo(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    position: [f32; 3],
    quat: [f32; 4],
    axis_length: f32,
) {
    let Some(center_2d) = camera.project(position, rect) else {
        return;
    };

    let local_x = [axis_length, 0.0, 0.0];
    let local_y = [0.0, axis_length, 0.0];
    let local_z = [0.0, 0.0, axis_length];

    let rot_x = rotate_vector_by_quat(local_x, quat);
    let rot_y = rotate_vector_by_quat(local_y, quat);
    let rot_z = rotate_vector_by_quat(local_z, quat);

    let tip_x = [position[0] + rot_x[0], position[1] + rot_x[1], position[2] + rot_x[2]];
    let tip_y = [position[0] + rot_y[0], position[1] + rot_y[1], position[2] + rot_y[2]];
    let tip_z = [position[0] + rot_z[0], position[1] + rot_z[1], position[2] + rot_z[2]];

    let stroke_x = Stroke::new(2.8_f32, Color32::from_rgb(240, 50, 50));
    let stroke_y = Stroke::new(2.8_f32, Color32::from_rgb(50, 220, 70));
    let stroke_z = Stroke::new(2.8_f32, Color32::from_rgb(45, 130, 255));

    if let Some(p_x) = camera.project(tip_x, rect) {
        painter.line_segment([center_2d, p_x], stroke_x);
    }
    if let Some(p_y) = camera.project(tip_y, rect) {
        painter.line_segment([center_2d, p_y], stroke_y);
    }
    if let Some(p_z) = camera.project(tip_z, rect) {
        painter.line_segment([center_2d, p_z], stroke_z);
    }

    painter.circle_filled(center_2d, 3.0_f32, Color32::WHITE);
}