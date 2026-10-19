//! Отрисовка трехмерной сцены: сетка, анатомический скелет и 64 места датчиков.

use super::draw::{draw_floor_grid, draw_skeleton};
use super::skeleton::{skeleton, SENSOR_COUNT};
use super::viewport::OrbitalCamera;
use crate::state::NodeTelemetry;
use egui::{Painter, Rect};

/// Для каждого из 64 мест находит активный узел: по тегу маппинга,
/// иначе — немаркированный узел с тем же порядковым индексом.
fn active_quats(nodes: &[NodeTelemetry]) -> [Option<[f32; 4]>; SENSOR_COUNT] {
    let mut out = [None; SENSOR_COUNT];
    for site in &skeleton().sites {
        let node = nodes
            .iter()
            .find(|n| n.bone_tag.as_deref() == Some(site.tag))
            .or_else(|| nodes.get(site.id).filter(|n| n.bone_tag.is_none()));
        out[site.id] = node.map(|n| n.orientation_quat);
    }
    out
}

/// Отрисовывает полную 3D-сцену с 64 местами размещения датчиков.
pub fn render_3d_scene(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    nodes: &[NodeTelemetry],
) {
    draw_floor_grid(painter, camera, rect, 5.0, 0.5);
    draw_skeleton(painter, camera, rect, skeleton(), &active_quats(nodes));
}
