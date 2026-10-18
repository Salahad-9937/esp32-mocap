//! Схематичный проволочный анатомический каркас и координатная сетка горизонта.

use super::viewport::OrbitalCamera;
use egui::{Color32, Painter, Rect, Stroke};

/// Дескриптор сегмента проволочного скелета.
#[derive(Debug, Clone, Copy)]
pub struct SkeletonBone {
    pub name: &'static str,
    pub parent_idx: Option<usize>,
    pub local_offset: [f32; 3],
}

/// Стандартная анатомическая иерархия сегментов тела человека.
pub const STANDARD_BONES: [SkeletonBone; 17] = [
    SkeletonBone { name: "Pelvis", parent_idx: None, local_offset: [0.0, 0.95, 0.0] },
    SkeletonBone { name: "Spine", parent_idx: Some(0), local_offset: [0.0, 0.20, 0.0] },
    SkeletonBone { name: "Thorax", parent_idx: Some(1), local_offset: [0.0, 0.25, 0.0] },
    SkeletonBone { name: "Neck", parent_idx: Some(2), local_offset: [0.0, 0.12, 0.0] },
    SkeletonBone { name: "Head", parent_idx: Some(3), local_offset: [0.0, 0.16, 0.0] },
    SkeletonBone { name: "LeftShoulder", parent_idx: Some(2), local_offset: [-0.18, 0.05, 0.0] },
    SkeletonBone { name: "LeftUpperArm", parent_idx: Some(5), local_offset: [-0.05, -0.28, 0.0] },
    SkeletonBone { name: "LeftForearm", parent_idx: Some(6), local_offset: [0.0, -0.25, 0.0] },
    SkeletonBone { name: "RightShoulder", parent_idx: Some(2), local_offset: [0.18, 0.05, 0.0] },
    SkeletonBone { name: "RightUpperArm", parent_idx: Some(8), local_offset: [0.05, -0.28, 0.0] },
    SkeletonBone { name: "RightForearm", parent_idx: Some(9), local_offset: [0.0, -0.25, 0.0] },
    SkeletonBone { name: "LeftThigh", parent_idx: Some(0), local_offset: [-0.12, -0.10, 0.0] },
    SkeletonBone { name: "LeftShin", parent_idx: Some(11), local_offset: [0.0, -0.42, 0.0] },
    SkeletonBone { name: "LeftFoot", parent_idx: Some(12), local_offset: [0.0, -0.06, 0.15] },
    SkeletonBone { name: "RightThigh", parent_idx: Some(0), local_offset: [0.12, -0.10, 0.0] },
    SkeletonBone { name: "RightShin", parent_idx: Some(14), local_offset: [0.0, -0.42, 0.0] },
    SkeletonBone { name: "RightFoot", parent_idx: Some(15), local_offset: [0.0, -0.06, 0.15] },
];

/// Отрисовывает проволочный каркас человека с позициями сочленений.
pub fn draw_skeleton(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    bone_positions: &[[f32; 3]],
) {
    let bone_stroke = Stroke::new(2.5_f32, Color32::from_rgb(170, 215, 255));
    let joint_color = Color32::from_rgb(255, 185, 30);

    for (i, bone) in STANDARD_BONES.iter().enumerate() {
        if let Some(parent_idx) = bone.parent_idx {
            if let (Some(&p_pos), Some(&c_pos)) = (bone_positions.get(parent_idx), bone_positions.get(i)) {
                if let (Some(p1), Some(p2)) = (camera.project(p_pos, rect), camera.project(c_pos, rect)) {
                    painter.line_segment([p1, p2], bone_stroke);
                }
            }
        }
    }

    for &pos in bone_positions {
        if let Some(p2d) = camera.project(pos, rect) {
            painter.circle_filled(p2d, 4.0_f32, joint_color);
        }
    }
}

/// Отрисовывает горизонтальную координатную сетку пола при Y = 0.
pub fn draw_floor_grid(painter: &Painter, camera: &OrbitalCamera, rect: Rect, size: f32, step: f32) {
    let grid_stroke = Stroke::new(1.0_f32, Color32::from_rgba_unmultiplied(65, 80, 105, 110));
    let axis_stroke_x = Stroke::new(1.6_f32, Color32::from_rgba_unmultiplied(210, 60, 60, 160));
    let axis_stroke_z = Stroke::new(1.6_f32, Color32::from_rgba_unmultiplied(60, 120, 230, 160));

    let half = size * 0.5;
    let mut x = -half;
    while x <= half + 0.001 {
        let is_center = x.abs() < 0.001;
        let stroke = if is_center { axis_stroke_z } else { grid_stroke };
        if let (Some(p1), Some(p2)) = (camera.project([x, 0.0, -half], rect), camera.project([x, 0.0, half], rect)) {
            painter.line_segment([p1, p2], stroke);
        }
        x += step;
    }

    let mut z = -half;
    while z <= half + 0.001 {
        let is_center = z.abs() < 0.001;
        let stroke = if is_center { axis_stroke_x } else { grid_stroke };
        if let (Some(p1), Some(p2)) = (camera.project([-half, 0.0, z], rect), camera.project([half, 0.0, z], rect)) {
            painter.line_segment([p1, p2], stroke);
        }
        z += step;
    }
}