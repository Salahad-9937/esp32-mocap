//! # Загрузчик эталонной 3D-модели манекена и площадок сенсоров (`anatomy`)

use super::math::{norm, V3};
use super::skeleton::{
    Chip, Joint, JointKind, Prim, SensorSite, Side, Skeleton, Tone, SENSOR_DEFS,
};

static MANNEQUIN_BYTES: &[u8] = include_bytes!("../assets/mannequin.bin");

#[inline]
fn read_u32(bytes: &[u8], cursor: &mut usize) -> u32 {
    if *cursor + 4 > bytes.len() {
        return 0;
    }
    let val = u32::from_le_bytes([
        bytes[*cursor],
        bytes[*cursor + 1],
        bytes[*cursor + 2],
        bytes[*cursor + 3],
    ]);
    *cursor += 4;
    val
}

#[inline]
fn read_i32(bytes: &[u8], cursor: &mut usize) -> i32 {
    if *cursor + 4 > bytes.len() {
        return -1;
    }
    let val = i32::from_le_bytes([
        bytes[*cursor],
        bytes[*cursor + 1],
        bytes[*cursor + 2],
        bytes[*cursor + 3],
    ]);
    *cursor += 4;
    val
}

#[inline]
fn read_f32(bytes: &[u8], cursor: &mut usize) -> f32 {
    if *cursor + 4 > bytes.len() {
        return 0.0;
    }
    let val = f32::from_le_bytes([
        bytes[*cursor],
        bytes[*cursor + 1],
        bytes[*cursor + 2],
        bytes[*cursor + 3],
    ]);
    *cursor += 4;
    val
}

#[inline]
fn read_v3(bytes: &[u8], cursor: &mut usize) -> [f32; 3] {
    let x = read_f32(bytes, cursor);
    let y = read_f32(bytes, cursor);
    let z = read_f32(bytes, cursor);
    [x, y, z]
}

/// Находит точку строго на сагиттальной оси симметрии (X = 0) для позвоночника, головы и грудины.
fn find_sagittal_midline_point(
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

                    // Штраф за отклонение от сагиттальной плоскости X = 0
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
fn find_bone_surface_point(
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

                    let score = ext * 12.0 + dot_n * 4.0 + y_weight * 5.0;
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
fn find_shoulder_surface_point(
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
                    if v[0] * side_sign < 0.08 {
                        continue;
                    }
                    if n[1] < 0.40 {
                        continue;
                    }
                    if v[1] < 1.340 || v[1] > 1.410 {
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
fn find_clavicle_surface_point(
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
                    if v[0] * side_sign < 0.04 {
                        continue;
                    }
                    if n[2] < 0.30 {
                        continue;
                    }
                    if v[1] < 1.280 || v[1] > 1.390 {
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
fn find_foot_surface_point(
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

                    // Жесткий штраф за боковое смещение (X), исключающий снос на большой палец
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

/// Тотальная аналитическая калибровка базисов и координат всех 64 сенсорных сайтов.
fn sanitize_sensor_sites(prims: &[Prim], prim_bones: &[&'static str], sites: &mut [SensorSite]) {
    for s in sites.iter_mut() {
        let tag = s.tag;
        let side_sign = if s.side == Side::Left { 1.0_f32 } else { -1.0_f32 };

        match tag {
            // ── Осевой скелет и голова (8) — строго на оси X = 0 ────────────
            "Head" => {
                s.normal = [0.0, 0.050, -0.999];
                s.tangent = [1.0, 0.0, 0.0];
                s.chip = Chip::Medium;
                if let Some(v) = find_sagittal_midline_point(prims, prim_bones, "Head", 1.630, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.630, -0.015];
                }
            }
            "Neck" => {
                s.normal = [0.0, 0.080, -0.997];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                if let Some(v) = find_sagittal_midline_point(prims, prim_bones, "Neck", 1.490, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.490, -0.010];
                }
            }
            "SpineT3" => {
                s.normal = [0.0, 0.020, -0.999];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_midline_point(prims, prim_bones, "SpineT3", 1.370, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.370, -0.035];
                }
            }
            "SpineT7" => {
                s.normal = [0.0, 0.010, -0.999];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_midline_point(prims, prim_bones, "SpineT3", 1.270, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.270, -0.015];
                }
            }
            "SpineT11" => {
                s.normal = [0.0, -0.020, -0.999];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_midline_point(prims, prim_bones, "SpineT3", 1.170, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.170, -0.008];
                }
            }
            "SpineL3" => {
                s.normal = [0.0, -0.050, -0.998];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_midline_point(prims, prim_bones, "SpineL3", 1.070, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.070, -0.005];
                }
            }
            "Sacrum" => {
                s.normal = [0.0, -0.080, -0.996];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_midline_point(prims, prim_bones, "Sacrum", 0.960, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 0.960, -0.005];
                }
            }
            "Sternum" => {
                s.normal = [0.0, 0.0, 1.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_midline_point(prims, prim_bones, "SpineT3", 1.320, true) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.320, 0.090];
                }
            }

            // ── Плечевой пояс (4) ──────────────────────────────────────────
            "LeftClavicle" | "RightClavicle" => {
                s.normal = norm([side_sign * 0.200, 0.400, 0.894]);
                s.tangent = norm([side_sign * 0.894, 0.0, -0.200]);
                s.chip = Chip::Medium;
                if let Some(v) = find_clavicle_surface_point(prims, prim_bones, side_sign) {
                    s.pos = v;
                } else {
                    s.pos = [side_sign * 0.095, 1.345, 0.085];
                }
            }
            "LeftShoulder" | "RightShoulder" => {
                s.normal = norm([side_sign * 0.220, 0.950, -0.200]);
                s.tangent = norm([side_sign * 0.950, -0.220, 0.0]);
                s.chip = Chip::Medium;
                if let Some(v) = find_shoulder_surface_point(prims, prim_bones, side_sign) {
                    s.pos = v;
                } else {
                    s.pos = [side_sign * 0.142, 1.380, 0.010];
                }
            }

            // ── Плечо, предплечье и кисть (10) ─────────────────────────────
            "LeftUpperArmProx" | "RightUpperArmProx" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh_name = if s.side == Side::Left { "LeftUpperArmProx" } else { "RightUpperArmProx" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.75)) {
                    s.pos = v;
                }
            }
            "LeftUpperArmDist" | "RightUpperArmDist" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh_name = if s.side == Side::Left { "LeftUpperArmProx" } else { "RightUpperArmProx" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.25)) {
                    s.pos = v;
                }
            }
            "LeftForearmProx" | "RightForearmProx" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh_name = if s.side == Side::Left { "LeftForearmDist" } else { "RightForearmDist" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.75)) {
                    s.pos = v;
                }
            }
            "LeftForearmDist" | "RightForearmDist" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh_name = if s.side == Side::Left { "LeftForearmDist" } else { "RightForearmDist" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.25)) {
                    s.pos = v;
                }
            }
            "LeftHand" | "RightHand" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh_name = if s.side == Side::Left { "LeftHand" } else { "RightHand" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.50)) {
                    s.pos = v;
                }
            }

            // ── Большой палец (6) ──────────────────────────────────────────
            "LeftThumbMeta" | "RightThumbMeta" => {
                s.normal = norm([side_sign * 0.707, 0.0, 0.707]);
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                let mesh_name = if s.side == Side::Left { "LeftThumbMeta" } else { "RightThumbMeta" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.50)) {
                    s.pos = v;
                }
            }
            "LeftThumbProx" | "RightThumbProx" => {
                s.pos = [0.0, -999.0, 0.0];
                s.normal = [0.0, 1.0, 0.0];
                s.tangent = [1.0, 0.0, 0.0];
                s.chip = Chip::Small;
            }
            "LeftThumbDist" | "RightThumbDist" => {
                s.normal = norm([side_sign * 0.707, 0.0, 0.707]);
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                let mesh_name = if s.side == Side::Left { "LeftThumbDist" } else { "RightThumbDist" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.50)) {
                    s.pos = v;
                }
            }

            // ── Фаланги пальцев кистей (24) ─────────────────────────────────
            _ if tag.contains("Index") || tag.contains("Middle") || tag.contains("Ring") || tag.contains("Pinky") => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                if let Some(v) = find_bone_surface_point(prims, prim_bones, tag, s.normal, Some(0.50)) {
                    s.pos = v;
                }
            }

            // ── Нижние конечности и стопы (12) ─────────────────────────────
            "LeftThighProx" | "RightThighProx" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                let mesh_name = if s.side == Side::Left { "LeftThighDist" } else { "RightThighDist" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.80)) {
                    s.pos = v;
                }
            }
            "LeftThighDist" | "RightThighDist" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                let mesh_name = if s.side == Side::Left { "LeftThighDist" } else { "RightThighDist" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.30)) {
                    s.pos = v;
                }
            }
            "LeftShankProx" | "RightShankProx" => {
                s.normal = norm([side_sign * 0.35, 0.0, 0.93]);
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                let mesh_name = if s.side == Side::Left { "LeftShankDist" } else { "RightShankDist" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.75)) {
                    s.pos = v;
                }
            }
            "LeftShankDist" | "RightShankDist" => {
                s.normal = norm([side_sign * 0.35, 0.0, 0.93]);
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                let mesh_name = if s.side == Side::Left { "LeftShankDist" } else { "RightShankDist" };
                if let Some(v) = find_bone_surface_point(prims, prim_bones, mesh_name, s.normal, Some(0.25)) {
                    s.pos = v;
                }
            }
            "LeftFoot" | "RightFoot" => {
                s.normal = norm([0.0, 0.960, 0.280]);
                s.tangent = [0.0, 0.0, 1.0];
                s.chip = Chip::Medium;
                let mesh_name = if s.side == Side::Left { "LeftFoot" } else { "RightFoot" };
                if let Some(v) = find_foot_surface_point(prims, prim_bones, mesh_name, side_sign, 0.040) {
                    s.pos = v;
                } else {
                    s.pos = [side_sign * 0.082, 0.048, 0.040];
                }
            }
            "LeftToe" | "RightToe" => {
                s.normal = norm([0.0, 0.980, 0.200]);
                s.tangent = [0.0, 0.0, 1.0];
                s.chip = Chip::Small;
                let mesh_name = if s.side == Side::Left { "LeftToe" } else { "RightToe" };
                if let Some(v) = find_foot_surface_point(prims, prim_bones, mesh_name, side_sign, 0.095) {
                    s.pos = v;
                } else {
                    s.pos = [side_sign * 0.082, 0.024, 0.095];
                }
            }
            _ => {}
        }
    }
}

/// Выполняет быструю десериализацию запеченной 3D-модели манекена из бинарного ресурса.
#[must_use]
pub fn build() -> Skeleton {
    let bytes = MANNEQUIN_BYTES;
    assert!(bytes.len() >= 18, "Слишком короткий бинарный файл манекена");

    let is_v3 = bytes.starts_with(b"MOCAP_MESH_V3\x00");
    let is_v2 = bytes.starts_with(b"MOCAP_MESH_V2\x00");
    let is_v1 = bytes.starts_with(b"MOCAP_MESH_V1\x00");
    assert!(is_v1 || is_v2 || is_v3, "Неверный заголовок формата меша");

    let mut cursor = 14;
    let num_meshes = read_u32(bytes, &mut cursor) as usize;
    let mut prims = Vec::with_capacity(num_meshes);
    let mut prim_bones = Vec::with_capacity(num_meshes);

    for _ in 0..num_meshes {
        if cursor >= bytes.len() {
            break;
        }
        let tag_len = bytes[cursor] as usize;
        cursor += 1;
        let tag_str = if cursor + tag_len <= bytes.len() {
            std::str::from_utf8(&bytes[cursor..cursor + tag_len]).unwrap_or("Sacrum")
        } else {
            "Sacrum"
        };
        cursor += tag_len;
        let bone_tag: &'static str = Box::leak(tag_str.to_string().into_boxed_str());

        let tone_byte = if cursor < bytes.len() { bytes[cursor] } else { 0 };
        cursor += 1;
        let tone = match tone_byte {
            1 | 2 => Tone::Plate,
            _ => Tone::Cortical,
        };

        let num_verts = read_u32(bytes, &mut cursor) as usize;
        let mut vertices = Vec::with_capacity(num_verts);
        for _ in 0..num_verts {
            vertices.push(read_v3(bytes, &mut cursor));
        }

        let mut normals = Vec::with_capacity(num_verts);
        for _ in 0..num_verts {
            normals.push(read_v3(bytes, &mut cursor));
        }

        let num_indices = read_u32(bytes, &mut cursor) as usize;
        let mut indices = Vec::with_capacity(num_indices);
        for _ in 0..num_indices {
            indices.push(read_u32(bytes, &mut cursor));
        }

        prims.push(Prim::Mesh { vertices, normals, indices, tone });
        prim_bones.push(bone_tag);
    }

    let mut sites = if (is_v2 || is_v3) && cursor < bytes.len() {
        let num_sites = read_u32(bytes, &mut cursor) as usize;
        let mut sites = Vec::with_capacity(num_sites);

        for _ in 0..num_sites {
            if cursor >= bytes.len() {
                break;
            }
            let id = read_u32(bytes, &mut cursor) as usize;

            let tag_len = bytes[cursor] as usize;
            cursor += 1;
            let tag_str = if cursor + tag_len <= bytes.len() {
                std::str::from_utf8(&bytes[cursor..cursor + tag_len]).unwrap_or("Head")
            } else {
                "Head"
            };
            cursor += tag_len;
            let site_tag: &'static str = Box::leak(tag_str.to_string().into_boxed_str());

            let side_byte = if cursor < bytes.len() { bytes[cursor] } else { 0 };
            cursor += 1;
            let side = if side_byte == 0 { Side::Left } else { Side::Right };

            let chip_byte = if cursor < bytes.len() { bytes[cursor] } else { 1 };
            cursor += 1;
            let chip = match chip_byte {
                0 => Chip::Large,
                2 => Chip::Small,
                _ => Chip::Medium,
            };

            let pos = read_v3(bytes, &mut cursor);
            let normal = read_v3(bytes, &mut cursor);
            let tangent = read_v3(bytes, &mut cursor);
            let support = read_v3(bytes, &mut cursor);

            sites.push(SensorSite {
                id,
                tag: site_tag,
                side,
                pos,
                normal,
                tangent,
                support,
                chip,
            });
        }
        sites
    } else {
        build_fallback_sites(&prims, &prim_bones)
    };

    sanitize_sensor_sites(&prims, &prim_bones, &mut sites);

    let joints = if is_v3 && cursor < bytes.len() {
        let num_joints = read_u32(bytes, &mut cursor) as usize;
        let mut j_list = Vec::with_capacity(num_joints);

        for _ in 0..num_joints {
            if cursor >= bytes.len() {
                break;
            }
            let name_len = bytes[cursor] as usize;
            cursor += 1;
            let name_str = if cursor + name_len <= bytes.len() {
                std::str::from_utf8(&bytes[cursor..cursor + name_len]).unwrap_or("Joint")
            } else {
                "Joint"
            };
            cursor += name_len;

            let kind_byte = if cursor < bytes.len() { bytes[cursor] } else { 0 };
            cursor += 1;
            let kind = match kind_byte {
                1 => JointKind::Ball,
                2 => JointKind::Hinge,
                4 => JointKind::Saddle,
                5 => JointKind::Condyloid,
                7 => JointKind::Spinal,
                _ => JointKind::Root,
            };

            let pos = read_v3(bytes, &mut cursor);
            let marker_r = read_f32(bytes, &mut cursor);
            let parent_raw = read_i32(bytes, &mut cursor);
            let parent = if parent_raw >= 0 { Some(parent_raw as usize) } else { None };

            j_list.push(Joint {
                name: name_str.to_string(),
                kind,
                pos,
                parent,
                axis: None,
                marker_r,
            });
        }
        j_list
    } else {
        build_fallback_joints()
    };

    Skeleton {
        joints,
        prims,
        prim_bones,
        sites,
    }
}

fn build_fallback_sites(prims: &[Prim], prim_bones: &[&'static str]) -> Vec<SensorSite> {
    let mut sites: Vec<SensorSite> = SENSOR_DEFS
        .iter()
        .enumerate()
        .map(|(id, def)| {
            let mut center = [0.0_f32; 3];
            let mut count = 0usize;
            for (i, p) in prims.iter().enumerate() {
                if prim_bones.get(i).copied() == Some(def.tag) {
                    if let Prim::Mesh { vertices, .. } = p {
                        for v in vertices {
                            center[0] += v[0];
                            center[1] += v[1];
                            center[2] += v[2];
                            count += 1;
                        }
                    }
                }
            }
            let pos = if count > 0 {
                [
                    center[0] / count as f32,
                    center[1] / count as f32,
                    (center[2] / count as f32) + 0.02,
                ]
            } else {
                [0.0, 1.0, 0.0]
            };
            SensorSite {
                id,
                tag: def.tag,
                side: def.side,
                pos,
                normal: [0.0, 0.0, 1.0],
                tangent: [0.0, 1.0, 0.0],
                support: pos,
                chip: def.chip,
            }
        })
        .collect();

    sanitize_sensor_sites(prims, prim_bones, &mut sites);
    sites
}

fn build_fallback_joints() -> Vec<Joint> {
    vec![
        Joint { name: "Pelvis".into(), kind: JointKind::Root, pos: [0.0, 0.961, 0.041], parent: None, axis: None, marker_r: 0.007 },
        Joint { name: "L5S1".into(), kind: JointKind::Spinal, pos: [0.0, 1.050, 0.050], parent: Some(0), axis: None, marker_r: 0.004 },
        Joint { name: "T12L1".into(), kind: JointKind::Spinal, pos: [0.0, 1.113, 0.063], parent: Some(1), axis: None, marker_r: 0.004 },
        Joint { name: "T7T8".into(), kind: JointKind::Spinal, pos: [0.0, 1.250, 0.058], parent: Some(2), axis: None, marker_r: 0.004 },
        Joint { name: "C7T1".into(), kind: JointKind::Spinal, pos: [0.0, 1.300, 0.054], parent: Some(3), axis: None, marker_r: 0.004 },
        Joint { name: "Neck".into(), kind: JointKind::Spinal, pos: [0.0, 1.469, 0.021], parent: Some(4), axis: None, marker_r: 0.004 },
        Joint { name: "Head".into(), kind: JointKind::Condyloid, pos: [0.0, 1.592, 0.052], parent: Some(5), axis: None, marker_r: 0.005 },
    ]
}