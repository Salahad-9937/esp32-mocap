//! # Аппаратный 3D-растеризатор манекена (`draw`)

pub mod geometry;
pub mod pipeline;

use super::axes::rotate_vector_by_quat;
use super::placement::{self, PlacedChip};
use super::viewport::OrbitalCamera;
use eframe::egui_glow;
use egui::{Painter, Rect};
use geometry::{add_cage_and_grid_lines, add_capsule_geo, add_sphere_geo, tone_rgba};
use mocap_model::math::{add, cross, norm, sub};
use mocap_model::skeleton::{Prim, Skeleton, Tone};
pub use pipeline::{GlSceneRenderer, GlVertex};
use std::sync::{Arc, Mutex, OnceLock};

static GL_RENDERER: OnceLock<Mutex<GlSceneRenderer>> = OnceLock::new();

pub fn draw_scene_3d(
    painter: &Painter,
    camera: &OrbitalCamera,
    rect: Rect,
    skel: &Skeleton,
    chips: &[PlacedChip],
    quats: &[Option<[f32; 4]>],
) {
    let mut tri_vertices: Vec<GlVertex> = Vec::with_capacity(8192);
    let mut tri_indices: Vec<u32> = Vec::with_capacity(16384);
    let mut line_vertices: Vec<GlVertex> = Vec::with_capacity(1024);
    let mut line_indices: Vec<u32> = Vec::with_capacity(1024);

    for pr in &skel.prims {
        match pr {
            Prim::Capsule { a, b, ra, rb, tone } => {
                add_capsule_geo(&mut tri_vertices, &mut tri_indices, *a, *b, *ra, *rb, *tone);
            }
            Prim::Sphere { c, r, tone } => {
                add_sphere_geo(&mut tri_vertices, &mut tri_indices, *c, *r, *tone);
            }
            Prim::Tri { p, tone } => {
                let edge1 = sub(p[1], p[0]);
                let edge2 = sub(p[2], p[0]);
                let normal = norm(cross(edge1, edge2));
                let color = tone_rgba(*tone);
                let base = tri_vertices.len() as u32;
                for &pos in p {
                    tri_vertices.push(GlVertex { pos, normal, color, ao: 1.0 });
                }
                tri_indices.extend([base, base + 1, base + 2]);
            }
            Prim::Mesh { vertices, normals, indices, tone } => {
                let color = tone_rgba(*tone);
                let base = tri_vertices.len() as u32;
                for i in 0..vertices.len() {
                    tri_vertices.push(GlVertex {
                        pos: vertices[i],
                        normal: normals[i],
                        color,
                        ao: 1.0,
                    });
                }
                for &idx in indices {
                    tri_indices.push(base + idx);
                }
            }
        }
    }

    for j in &skel.joints {
        let r = if j.marker_r > 0.010 { 0.007 } else { 0.005 };
        add_sphere_geo(&mut tri_vertices, &mut tri_indices, j.pos, r, Tone::Dark);
    }

    let highlighted = placement::highlight();
    let selected = placement::selected();

    for (st, q) in chips.iter().zip(quats) {
        let color = if selected == Some(st.id) {
            [1.0, 0.88, 0.20, 1.0] // Золотой: выбранный для настройки
        } else if highlighted == Some(st.id) {
            [0.45, 0.85, 1.0, 1.0] // Светло-голубой: под курсором
        } else if q.is_some() {
            [0.18, 0.92, 0.35, 1.0] // Зеленый: назначен и активен
        } else {
            [0.48, 0.52, 0.62, 0.90] // Стальной: свободный/не привязанный
        };

        let base = tri_vertices.len() as u32;
        for k in 0..9 {
            tri_vertices.push(GlVertex { pos: st.patch[k], normal: st.patch_n[k], color, ao: 1.0 });
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

    add_cage_and_grid_lines(&mut line_vertices, &mut line_indices);

    for (st, q) in chips.iter().zip(quats) {
        if let Some(quat) = q {
            let axis_len = st.chip.axis_len();
            let axes = [
                ([axis_len, 0.0, 0.0], [0.95, 0.20, 0.20, 1.0]),
                ([0.0, axis_len, 0.0], [0.20, 0.90, 0.30, 1.0]),
                ([0.0, 0.0, axis_len], [0.20, 0.50, 1.0, 1.0]),
            ];
            for (local, col) in axes {
                let rot = rotate_vector_by_quat(local, *quat);
                let tip = add(st.pos, rot);
                let b = line_vertices.len() as u32;
                line_vertices.push(GlVertex { pos: st.pos, normal: [0.0, 1.0, 0.0], color: col, ao: 1.0 });
                line_vertices.push(GlVertex { pos: tip, normal: [0.0, 1.0, 0.0], color: col, ao: 1.0 });
                line_indices.extend([b, b + 1]);
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
}