//! # Отрисовка трехмерной сцены с поддержкой режимов привязки и кинематики (`scene`)

use super::draw::draw_scene_3d;
use super::placement;
use super::viewport::OrbitalCamera;
use crate::app::ActiveTab;
use crate::state::NodeTelemetry;
use egui::{Painter, Rect};
use mocap_model::kinematics::kinematic_tree;
use mocap_model::skeleton::{skeleton, SENSOR_COUNT};
use std::collections::HashMap;

fn active_quats(nodes: &[NodeTelemetry]) -> [Option<[f32; 4]>; SENSOR_COUNT] {
    let mut out = [None; SENSOR_COUNT];
    for site in &skeleton().sites {
        let node = nodes
            .iter()
            .find(|n| n.bone_tag.as_deref() == Some(site.tag));
        out[site.id] = node.map(|n| n.orientation_quat);
    }
    out
}

/// Выполняет рендеринг сцены с жестким разделением: статический стенд в Маппинге, кинематика в Захвате.
pub fn render_3d_scene(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    nodes: &[NodeTelemetry],
    active_tab: ActiveTab,
) {
    let chips = placement::chips();
    let quats = active_quats(nodes);

    if active_tab == ActiveTab::Tracking {
        let mut orientations_map = HashMap::with_capacity(nodes.len());
        for node in nodes {
            if let Some(ref tag) = node.bone_tag {
                orientations_map.insert(tag.as_str(), node.orientation_quat);
            }
        }
        let transforms = kinematic_tree().evaluate(&orientations_map);
        draw_scene_3d(painter, camera, rect, skeleton(), &chips, &quats, Some(&transforms));
    } else {
        // В режимах Маппинга, Топологии и Калибровки манекен строго статичен (Bind Pose)
        draw_scene_3d(painter, camera, rect, skeleton(), &chips, &quats, None);
    }
}