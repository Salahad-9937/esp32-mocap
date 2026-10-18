//! Отрисовка трехмерной сцены MoCap (координатная сетка, скелет, базисы узлов).

use super::axes::draw_axes_gizmo;
use super::skeleton::{draw_floor_grid, draw_skeleton, STANDARD_BONES};
use super::viewport::OrbitalCamera;
use crate::state::NodeTelemetry;
use egui::{Color32, Painter, Rect};

/// Отрисовывает полную трехмерную сцену с привязкой осей строго к назначенным костям.
pub fn render_3d_scene(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    nodes: &[NodeTelemetry],
) {
    draw_floor_grid(painter, camera, rect, 5.0, 0.5);

    let mut world_bone_positions = Vec::with_capacity(STANDARD_BONES.len());
    for bone in &STANDARD_BONES {
        let pos = match bone.parent_idx {
            None => bone.local_offset,
            Some(p_idx) => {
                let p = world_bone_positions.get(p_idx).copied().unwrap_or([0.0, 0.0, 0.0]);
                [
                    p[0] + bone.local_offset[0],
                    p[1] + bone.local_offset[1],
                    p[2] + bone.local_offset[2],
                ]
            }
        };
        world_bone_positions.push(pos);
    }

    draw_skeleton(painter, camera, rect, &world_bone_positions);

    for (i, node) in nodes.iter().enumerate() {
        // Поиск индекса кости по маппингу, либо fallback на базовую позицию
        let bone_idx = if let Some(ref tag) = node.bone_tag {
            STANDARD_BONES.iter().position(|b| b.name == tag).unwrap_or(i % STANDARD_BONES.len())
        } else {
            i % STANDARD_BONES.len()
        };

        let pos = world_bone_positions.get(bone_idx).copied().unwrap_or([0.0, 0.9, 0.0]);

        draw_axes_gizmo(painter, camera, rect, pos, node.orientation_quat, 0.18);

        let label_text = if let Some(ref tag) = node.bone_tag {
            format!("#{i} [{tag}]")
        } else {
            format!("#{i} (0x{:02X}:C{})", node.mux_address, node.channel)
        };

        if let Some(p2d) = camera.project(pos, rect) {
            painter.text(
                p2d + egui::vec2(8.0, -8.0),
                egui::Align2::LEFT_BOTTOM,
                label_text,
                egui::FontId::monospace(11.0),
                Color32::from_rgb(220, 240, 255),
            );
        }
    }
}