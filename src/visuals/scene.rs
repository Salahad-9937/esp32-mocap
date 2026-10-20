//! Отрисовка трехмерной сцены: координатная сетка, аппаратный 3D-манекен и 64 датчика.

use super::draw::draw_scene_3d;
use super::placement;
use super::skeleton::{skeleton, SENSOR_COUNT};
use super::viewport::OrbitalCamera;
use crate::state::NodeTelemetry;
use egui::{Painter, Rect};

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

/// Выполняет рендеринг 3D-сцены через аппаратный графический коллбэк.
pub fn render_3d_scene(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    nodes: &[NodeTelemetry],
) {
    let chips = placement::chips();
    draw_scene_3d(painter, camera, rect, skeleton(), &chips, &active_quats(nodes));
}