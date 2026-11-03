//! # Геометрический решатель пространственной локализации меток (`solver`)

use crate::math::{norm, V3};
use crate::skeleton::{Prim, Tone};

/// Находит точку строго на сагиттальной оси симметрии (X = 0) для позвоночника, головы и грудины.
#[must_use]
pub fn find_sagittal_point(
    prims: &[Prim],
    prim_bones: &[&'static str],
    target_tag: &str,
    target_y: f32,
    is_anterior: bool,
) -> Option<V3> {
    let mut best_score = -999.0_f32;
    let mut best_v = None;

    for (i, p) in prims.iter().enumerate() {
        if prim_bones.get(i).copied() == Some(target_tag) {
            if let Prim::Mesh { vertices, normals, .. } = p {
                for (&v, &n) in vertices.iter().zip(normals) {
                    let x_dist = v[0].abs();
                    if x_dist > 0.04 {
                        continue;
                    }
                    let y_dist = (v[1] - target_y).abs();
                    if y_dist > 0.08 {
                        continue;
                    }

                    let z_dir = if is_anterior { n[2] } else { -n[2] };
                    if z_dir < 0.10 {
                        continue;
                    }

                    let score = z_dir * 10.0 - y_dist * 25.0 - x_dist * 80.0;
                    if score > best_score {
                        best_score = score;
                        best_v = Some([0.0, v[1], v[2]]);
                    }
                }
            }
        }
    }
    best_v
}

/// Находит наружную точку на поверхности меша парных конечностей вдоль вектора нормали.
#[must_use]
pub fn find_surface_point(
    prims: &[Prim],
    prim_bones: &[&'static str],
    target_tag: &str,
    normal_dir: V3,
    y_ratio: Option<f32>,
) -> Option<V3> {
    let mut best_score = -999.0_f32;
    let mut best_v = None;

    for (i, p) in prims.iter().enumerate() {
        if prim_bones.get(i).copied() == Some(target_tag) {
            if let Prim::Mesh { vertices, normals, tone, .. } = p {
                if *tone == Tone::Plate || vertices.is_empty() {
                    continue;
                }

                let mut y_min = f32::INFINITY;
                let mut y_max = f32::NEG_INFINITY;
                let mut center = [0.0_f32; 3];

                for v in vertices {
                    y_min = y_min.min(v[1]);
                    y_max = y_max.max(v[1]);
                    center[0] += v[0];
                    center[1] += v[1];
                    center[2] += v[2];
                }
                let inv_len = 1.0 / (vertices.len() as f32).max(1.0);
                center[0] *= inv_len;
                center[1] *= inv_len;
                center[2] *= inv_len;

                let y_mid = match y_ratio {
                    Some(r) => y_min + (y_max - y_min) * r.clamp(0.0, 1.0),
                    None => (y_min + y_max) * 0.5,
                };
                let y_span = (y_max - y_min).max(0.005) * 0.5;

                for (&v, &n) in vertices.iter().zip(normals) {
                    let dot_n = n[0] * normal_dir[0] + n[1] * normal_dir[1] + n[2] * normal_dir[2];
                    if dot_n < 0.05 {
                        continue;
                    }

                    let y_dist = (v[1] - y_mid).abs();
                    let y_weight = (1.0 - (y_dist / y_span)).max(0.0);

                    let ext = (v[0] - center[0]) * normal_dir[0]
                        + (v[1] - center[1]) * normal_dir[1]
                        + (v[2] - center[2]) * normal_dir[2];

                    let z_pen = if normal_dir[2].abs() < 0.15 {
                        (v[2] - center[2]).abs() * 15.0
                    } else {
                        0.0
                    };

                    let score = ext * 12.0 + dot_n * 4.0 + y_weight * 5.0 - z_pen;
                    if score > best_score {
                        best_score = score;
                        best_v = Some(v);
                    }
                }
            }
        }
    }
    best_v
}

/// Находит точку на верхнем скате надплечья (акромион) на реальной полигональной поверхности.
#[must_use]
pub fn find_acromion_point(
    prims: &[Prim],
    prim_bones: &[&'static str],
    side_sign: f32,
) -> Option<V3> {
    let mut best_score = -999.0_f32;
    let mut best_v = None;

    let target_normal = norm([side_sign * 0.22, 0.95, -0.20]);
    let target_x = side_sign * 0.142;
    let target_y = 1.380;
    let target_z = 0.010;

    for (i, p) in prims.iter().enumerate() {
        let tag = prim_bones.get(i).copied().unwrap_or("");
        let is_target_mesh = tag == "SpineT3"
            || (side_sign > 0.0 && tag == "LeftShoulder")
            || (side_sign < 0.0 && tag == "RightShoulder");

        if is_target_mesh {
            if let Prim::Mesh { vertices, normals, .. } = p {
                for (&v, &n) in vertices.iter().zip(normals) {
                    if v[0] * side_sign < 0.08 || n[1] < 0.40 || v[1] < 1.340 || v[1] > 1.410 {
                        continue;
                    }

                    let dot_n = n[0] * target_normal[0] + n[1] * target_normal[1] + n[2] * target_normal[2];
                    if dot_n < 0.20 {
                        continue;
                    }

                    let x_dist = (v[0] - target_x).abs();
                    let y_dist = (v[1] - target_y).abs();
                    let z_dist = (v[2] - target_z).abs();

                    let score = dot_n * 10.0 + v[1] * 8.0 - x_dist * 40.0 - y_dist * 25.0 - z_dist * 30.0;
                    if score > best_score {
                        best_score = score;
                        best_v = Some(v);
                    }
                }
            }
        }
    }
    best_v
}

/// Находит точку на передней поверхности ключицы на меше грудного отдела.
#[must_use]
pub fn find_clavicle_point(
    prims: &[Prim],
    prim_bones: &[&'static str],
    side_sign: f32,
) -> Option<V3> {
    let mut best_score = -999.0_f32;
    let mut best_v = None;

    let target_normal = norm([side_sign * 0.20, 0.40, 0.89]);
    let target_x = side_sign * 0.095;
    let target_y = 1.345;
    let target_z = 0.085;

    for (i, p) in prims.iter().enumerate() {
        if prim_bones.get(i).copied() == Some("SpineT3") {
            if let Prim::Mesh { vertices, normals, .. } = p {
                for (&v, &n) in vertices.iter().zip(normals) {
                    if v[0] * side_sign < 0.04 || n[2] < 0.30 || v[1] < 1.280 || v[1] > 1.390 {
                        continue;
                    }

                    let dot_n = n[0] * target_normal[0] + n[1] * target_normal[1] + n[2] * target_normal[2];
                    if dot_n < 0.20 {
                        continue;
                    }

                    let x_dist = (v[0] - target_x).abs();
                    let y_dist = (v[1] - target_y).abs();
                    let z_dist = (v[2] - target_z).abs();

                    let score = dot_n * 10.0 + v[2] * 8.0 - x_dist * 40.0 - y_dist * 25.0 - z_dist * 30.0;
                    if score > best_score {
                        best_score = score;
                        best_v = Some(v);
                    }
                }
            }
        }
    }
    best_v
}

/// Находит точку на дорсальной поверхности стопы строго вдоль центральной продольной оси стопы.
#[must_use]
pub fn find_foot_point(
    prims: &[Prim],
    prim_bones: &[&'static str],
    target_tag: &str,
    side_sign: f32,
    target_z: f32,
) -> Option<V3> {
    let mut best_score = -999.0_f32;
    let mut best_v = None;

    for (i, p) in prims.iter().enumerate() {
        if prim_bones.get(i).copied() == Some(target_tag) {
            if let Prim::Mesh { vertices, normals, tone, .. } = p {
                if *tone == Tone::Plate || vertices.is_empty() {
                    continue;
                }

                let mut x_min = f32::INFINITY;
                let mut x_max = f32::NEG_INFINITY;
                for v in vertices {
                    x_min = x_min.min(v[0]);
                    x_max = x_max.max(v[0]);
                }
                let x_mid = if x_min.is_finite() && x_max.is_finite() {
                    (x_min + x_max) * 0.5
                } else {
                    side_sign * 0.082
                };

                for (&v, &n) in vertices.iter().zip(normals) {
                    if n[1] < 0.20 {
                        continue;
                    }

                    let x_dist = (v[0] - x_mid).abs();
                    let z_dist = (v[2] - target_z).abs();

                    let score = v[1] * 12.0 + n[1] * 6.0 - x_dist * 90.0 - z_dist * 30.0;
                    if score > best_score {
                        best_score = score;
                        best_v = Some(v);
                    }
                }
            }
        }
    }
    best_v
}