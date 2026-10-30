//! # Аппаратный 3D-растеризатор манекена (`draw`)

pub mod geometry;
pub mod pipeline;

use super::axes::rotate_vector_by_quat;
use super::placement::{self, PlacedChip};
use super::viewport::OrbitalCamera;
use eframe::egui_glow;
use egui::{Color32, Painter, Rect, Stroke};
use geometry::{add_cage_and_grid_lines, add_capsule_geo, add_sphere_geo, tone_rgba};
use mocap_model::kinematics::BoneTransform;
use mocap_model::math::{add, cross, norm, sub, V3};
use mocap_model::skeleton::{Prim, Skeleton};
pub use pipeline::{GlSceneRenderer, GlVertex};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};

static GL_RENDERER: OnceLock<Mutex<GlSceneRenderer>> = OnceLock::new();

#[inline]
fn transform_v3(p: V3, tf: Option<&BoneTransform>) -> V3 {
    match tf {
        Some(t) => t.transform_point(p),
        None => p,
    }
}

#[inline]
fn transform_norm(n: V3, tf: Option<&BoneTransform>) -> V3 {
    match tf {
        Some(t) => t.transform_normal(n),
        None => n,
    }
}

fn joint_bone_tag(name: &str) -> &'static str {
    match name {
        n if n.contains("Pelvis") || n.contains("L5S1") => "Sacrum",
        n if n.contains("L3L4") || n.contains("SpineL3") => "SpineL3",
        n if n.contains("T12L1") || n.contains("SpineT11") => "SpineT11",
        n if n.contains("T7T8") || n.contains("SpineT7") => "SpineT7",
        n if n.contains("C7T1") || n.contains("SpineT3") => "SpineT3",
        n if n.contains("C4C5") || n.contains("Neck") => "Neck",
        n if n.contains("Occiput") || n.contains("C1C2") || n.contains("Head") => "Head",
        n if n.contains("SC") || n.contains("Clavicle") => {
            if n.starts_with("L.") || n.contains("Left") { "LeftClavicle" } else { "RightClavicle" }
        }
        n if n.contains("GH") || n.contains("Shoulder") => {
            if n.starts_with("L.") || n.contains("Left") { "LeftShoulder" } else { "RightShoulder" }
        }
        n if n.contains("Elbow") => {
            if n.starts_with("L.") || n.contains("Left") { "LeftForearmProx" } else { "RightForearmProx" }
        }
        n if n.contains("Wrist") => {
            if n.starts_with("L.") || n.contains("Left") { "LeftHand" } else { "RightHand" }
        }
        n if n.contains("MCP") || n.contains("PIP") || n.contains("DIP") || n.contains("CMC") || n.contains("IP") => {
            if n.starts_with("L.") || n.contains("Left") { "LeftHand" } else { "RightHand" }
        }
        n if n.contains("Hip") => {
            if n.starts_with("L.") || n.contains("Left") { "LeftThighProx" } else { "RightThighProx" }
        }
        n if n.contains("Knee") => {
            if n.starts_with("L.") || n.contains("Left") { "LeftShankProx" } else { "RightShankProx" }
        }
        n if n.contains("Ankle") || n.contains("Subtalar") => {
            if n.starts_with("L.") || n.contains("Left") { "LeftFoot" } else { "RightFoot" }
        }
        n if n.contains("MTP") || n.contains("Toe") => {
            if n.starts_with("L.") || n.contains("Left") { "LeftToe" } else { "RightToe" }
        }
        _ => "Sacrum",
    }
}

pub fn draw_scene_3d(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    skel: &Skeleton,
    chips: &[PlacedChip],
    quats: &[Option<[f32; 4]>],
    transforms: Option<&HashMap<&'static str, BoneTransform>>,
) {
    let mut tri_vertices: Vec<GlVertex> = Vec::with_capacity(8192);
    let mut tri_indices: Vec<u32> = Vec::with_capacity(16384);
    let mut line_vertices: Vec<GlVertex> = Vec::with_capacity(1024);
    let mut line_indices: Vec<u32> = Vec::with_capacity(1024);

    let show_mesh = placement::show_mesh();

    // 1. Отрисовка полигональной геометрии манекена
    if show_mesh {
        for (k, pr) in skel.prims.iter().enumerate() {
            let bone_tag = skel.prim_bones.get(k).copied().unwrap_or("Sacrum");
            let tf = transforms.and_then(|t| t.get(bone_tag));

            match pr {
                Prim::Capsule { a, b, ra, rb, tone } => {
                    let da = transform_v3(*a, tf);
                    let db = transform_v3(*b, tf);
                    add_capsule_geo(&mut tri_vertices, &mut tri_indices, da, db, *ra, *rb, *tone);
                }
                Prim::Sphere { c, r, tone } => {
                    let dc = transform_v3(*c, tf);
                    add_sphere_geo(&mut tri_vertices, &mut tri_indices, dc, *r, *tone);
                }
                Prim::Tri { p, tone } => {
                    let p0 = transform_v3(p[0], tf);
                    let p1 = transform_v3(p[1], tf);
                    let p2 = transform_v3(p[2], tf);
                    let edge1 = sub(p1, p0);
                    let edge2 = sub(p2, p0);
                    let normal = norm(cross(edge1, edge2));
                    let color = tone_rgba(*tone);
                    let base = tri_vertices.len() as u32;
                    for &pos in &[p0, p1, p2] {
                        tri_vertices.push(GlVertex { pos, normal, color, ao: 1.0 });
                    }
                    tri_indices.extend([base, base + 1, base + 2]);
                }
                Prim::Mesh { vertices, normals, indices, tone } => {
                    let color = tone_rgba(*tone);
                    let base = tri_vertices.len() as u32;
                    for i in 0..vertices.len() {
                        let pos = transform_v3(vertices[i], tf);
                        let normal = transform_norm(normals[i], tf);
                        tri_vertices.push(GlVertex { pos, normal, color, ao: 1.0 });
                    }
                    for &idx in indices {
                        tri_indices.push(base + idx);
                    }
                }
            }
        }
    }

    let show_sensors = placement::show_sensors();
    let highlighted = placement::highlight();
    let selected = placement::selected();

    // 2. Отрисовка сенсорных площадок IMU
    if show_sensors {
        for (st, q) in chips.iter().zip(quats) {
            let tf = transforms.and_then(|t| t.get(st.tag));
            let color = if selected == Some(st.id) {
                [1.0, 0.88, 0.20, 1.0]
            } else if highlighted == Some(st.id) {
                [0.45, 0.85, 1.0, 1.0]
            } else if q.is_some() {
                [0.18, 0.92, 0.35, 1.0]
            } else {
                [0.35, 0.40, 0.50, 1.0]
            };

            let base = tri_vertices.len() as u32;
            for k in 0..9 {
                let pos = transform_v3(st.patch[k], tf);
                let normal = transform_norm(st.patch_n[k], tf);
                tri_vertices.push(GlVertex { pos, normal, color, ao: 1.0 });
            }
            for row in 0..2u32 {
                for col in 0..2u32 {
                    let v00 = base + row * 3 + col;
                    let (v01, v10, v11) = (v00 + 1, v00 + 3, v00 + 4);
                    tri_indices.extend([v00, v11, v01, v00, v10, v11]);
                    tri_indices.extend([v00, v01, v11, v00, v11, v10]);
                }
            }
        }
    }

    // 3. Координатная сетка пола и габаритная клетка
    add_cage_and_grid_lines(&mut line_vertices, &mut line_indices);

    // 4. Локальные ортогональные базисы ориентации датчиков
    if show_sensors {
        for (st, q) in chips.iter().zip(quats) {
            if let Some(quat) = q {
                let tf = transforms.and_then(|t| t.get(st.tag));
                let chip_pos = transform_v3(st.pos, tf);
                let axis_len = st.chip.axis_len();
                let axes = [
                    ([axis_len, 0.0, 0.0], [0.95, 0.20, 0.20, 1.0]),
                    ([0.0, axis_len, 0.0], [0.20, 0.90, 0.30, 1.0]),
                    ([0.0, 0.0, axis_len], [0.20, 0.50, 1.0, 1.0]),
                ];
                for (local, col) in axes {
                    let rot = rotate_vector_by_quat(local, *quat);
                    let tip = add(chip_pos, rot);
                    let b = line_vertices.len() as u32;
                    line_vertices.push(GlVertex { pos: chip_pos, normal: [0.0, 1.0, 0.0], color: col, ao: 1.0 });
                    line_vertices.push(GlVertex { pos: tip, normal: [0.0, 1.0, 0.0], color: col, ao: 1.0 });
                    line_indices.extend([b, b + 1]);
                }
            }
        }
    }

    let aspect = rect.width() / rect.height().max(1.0);
    let mvp = camera.calc_mvp(aspect);
    let eye_pos = camera.eye_position();

    let callback = egui::PaintCallback {
        rect,
        callback: Arc::new(egui_glow::CallbackFn::new(move |_info, painter_gl| {
            let gl = painter_gl.gl();
            let renderer_lock = GL_RENDERER.get_or_init(|| {
                Mutex::new(GlSceneRenderer::new(gl).unwrap_or_else(|_| panic!("GL Init failed")))
            });
            if let Ok(mut renderer) = renderer_lock.lock() {
                renderer.paint(
                    gl,
                    &mvp,
                    eye_pos,
                    &tri_vertices,
                    &tri_indices,
                    &line_vertices,
                    &line_indices,
                );
            }
        })),
    };

    painter.add(callback);

    let show_skel_lines = placement::show_skeleton();
    let show_joint_dots = placement::show_joints();

    // 5. Отрисовка отладочного кинематического скелета (Biped Stick Skeleton)
    if show_skel_lines {
        for j in &skel.joints {
            if let Some(parent_idx) = j.parent {
                if let Some(parent_j) = skel.joints.get(parent_idx) {
                    let p_tf = transforms.and_then(|t| t.get(joint_bone_tag(&parent_j.name)));
                    let c_tf = transforms.and_then(|t| t.get(joint_bone_tag(&j.name)));

                    let p_world = transform_v3(parent_j.pos, p_tf);
                    let c_world = transform_v3(j.pos, c_tf);

                    if let (Some((p_2d, p_depth)), Some((c_2d, c_depth))) = (
                        camera.project_depth(p_world, rect),
                        camera.project_depth(c_world, rect),
                    ) {
                        let avg_depth = (p_depth + c_depth) * 0.5;
                        let line_w = (6.0 / avg_depth).clamp(1.2, 3.2);
                        let bone_color = Color32::from_rgba_premultiplied(0, 230, 255, 210);
                        painter.line_segment([p_2d, c_2d], Stroke::new(line_w, bone_color));
                    }
                }
            }
        }
    }

    // 6. Отрисовка точечных маркеров сочленений с учетом глубины
    if show_joint_dots {
        for j in &skel.joints {
            let bone_tag = joint_bone_tag(&j.name);
            let tf = transforms.and_then(|t| t.get(bone_tag));
            let world_pos = transform_v3(j.pos, tf);

            if let Some((screen_pos, depth)) = camera.project_depth(world_pos, rect) {
                let dot_r = (11.0 / depth).clamp(2.0, 4.8);
                let halo_r = dot_r * 1.6;
                let stroke_w = (3.5 / depth).clamp(0.8, 1.4);

                painter.circle_filled(screen_pos, halo_r, Color32::from_rgba_premultiplied(0, 140, 255, 120));
                painter.circle_filled(screen_pos, dot_r, Color32::from_rgb(0, 205, 255));
                painter.circle_stroke(screen_pos, halo_r, Stroke::new(stroke_w, Color32::from_rgb(190, 240, 255)));
            }
        }
    }
}