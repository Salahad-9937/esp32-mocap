//! # Загрузчик эталонной 3D-модели манекена и площадок сенсоров (`anatomy`)

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

/// Находит вершину строго по двухосевому геометрическому центру тела фаланги (Y и Z)
fn find_phalanx_dorsal_point(
    prims: &[Prim],
    prim_bones: &[&'static str],
    target_tag: &str,
    side_mult: f32,
) -> Option<[f32; 3]> {
    let mut best_score = -999.0_f32;
    let mut best_v = None;

    for (i, p) in prims.iter().enumerate() {
        if prim_bones.get(i).copied() == Some(target_tag) {
            if let Prim::Mesh { vertices, normals, tone, .. } = p {
                // Исключаем суставные шарниры (Tone::Plate), берем только тело фаланги
                if *tone == Tone::Plate || vertices.is_empty() {
                    continue;
                }

                let y_min = vertices.iter().map(|v| v[1]).fold(f32::INFINITY, f32::min);
                let y_max = vertices.iter().map(|v| v[1]).fold(f32::NEG_INFINITY, f32::max);
                let y_mid = (y_min + y_max) * 0.5;
                let y_span = (y_max - y_min).max(0.004) * 0.5;

                let z_min = vertices.iter().map(|v| v[2]).fold(f32::INFINITY, f32::min);
                let z_max = vertices.iter().map(|v| v[2]).fold(f32::NEG_INFINITY, f32::max);
                let z_mid = (z_min + z_max) * 0.5;
                let z_span = (z_max - z_min).max(0.004) * 0.5;

                for (&v, &n) in vertices.iter().zip(normals) {
                    let n_lat = n[0] * side_mult;
                    if n_lat < 0.15 {
                        continue;
                    }
                    let y_dist = (v[1] - y_mid).abs();
                    let z_dist = (v[2] - z_mid).abs();

                    // Штрафуем отклонение от центра высоты (Y) и центра ширины (Z)
                    let score = n_lat * 6.0 - (y_dist / y_span) * 20.0 - (z_dist / z_span) * 20.0;
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

/// Находит вершину на задней срединной линии позвоночника или головы
fn find_mesh_posterior_point(
    prims: &[Prim],
    prim_bones: &[&'static str],
    target_tag: &str,
    target_y: f32,
) -> Option<[f32; 3]> {
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

                    let n_post = -n[2];
                    let score = n_post * 4.0 - y_dist * 20.0 - x_dist * 50.0;
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

/// Точечная корректировка сенсорных позиций по реальным мешам геометрии
fn sanitize_sensor_sites(prims: &[Prim], prim_bones: &[&'static str], sites: &mut [SensorSite]) {
    for s in sites.iter_mut() {
        match s.tag {
            // 1. Голова: затылочная область (Occiput)
            "Head" => {
                s.normal = [0.0, 0.050, -0.999];
                s.tangent = [1.0, 0.0, 0.0];
                if let Some(v) = find_mesh_posterior_point(prims, prim_bones, "Head", 1.630) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.630, -0.015];
                }
            }
            // 2. Осевой скелет: точные наружные координаты спины
            "Neck" => {
                s.normal = [0.0, 0.080, -0.997];
                s.tangent = [0.0, 1.0, 0.0];
                if let Some(v) = find_mesh_posterior_point(prims, prim_bones, "Neck", 1.490) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.490, -0.010];
                }
            }
            "SpineT3" => {
                s.normal = [0.0, 0.020, -0.999];
                s.tangent = [0.0, 1.0, 0.0];
                if let Some(v) = find_mesh_posterior_point(prims, prim_bones, "SpineT3", 1.370) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.370, -0.035];
                }
            }
            "SpineT7" => {
                s.normal = [0.0, 0.010, -0.999];
                s.tangent = [0.0, 1.0, 0.0];
                if let Some(v) = find_mesh_posterior_point(prims, prim_bones, "SpineT3", 1.270) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.270, -0.015];
                }
            }
            "SpineT11" => {
                s.normal = [0.0, -0.020, -0.999];
                s.tangent = [0.0, 1.0, 0.0];
                if let Some(v) = find_mesh_posterior_point(prims, prim_bones, "SpineT3", 1.170) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.170, -0.008];
                }
            }
            "SpineL3" => {
                s.normal = [0.0, -0.050, -0.998];
                s.tangent = [0.0, 1.0, 0.0];
                if let Some(v) = find_mesh_posterior_point(prims, prim_bones, "SpineL3", 1.070) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.070, -0.005];
                }
            }
            "Sacrum" => {
                s.normal = [0.0, -0.080, -0.996];
                s.tangent = [0.0, 1.0, 0.0];
                if let Some(v) = find_mesh_posterior_point(prims, prim_bones, "Sacrum", 0.960) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 0.960, -0.005];
                }
            }
            // 3. Плечи: акромион (плечевой скат торса)
            "LeftShoulder" => {
                s.pos = [0.140, 1.415, 0.010];
                s.normal = [0.200, 0.950, -0.220];
                s.tangent = [0.950, -0.200, 0.0];
            }
            "RightShoulder" => {
                s.pos = [-0.140, 1.415, 0.010];
                s.normal = [-0.200, 0.950, -0.220];
                s.tangent = [-0.950, -0.200, 0.0];
            }
            // 4. Ключицы
            "LeftClavicle" => {
                s.pos = [0.095, 1.350, 0.085];
                s.normal = [0.200, 0.400, 0.894];
                s.tangent = [0.894, 0.0, -0.200];
            }
            "RightClavicle" => {
                s.pos = [-0.095, 1.350, 0.085];
                s.normal = [-0.200, 0.400, 0.894];
                s.tangent = [-0.894, 0.0, -0.200];
            }
            // 5. Фаланги левого мизинца: двухосевое центрирование по геометрическому центру тела фаланги
            "LeftPinkyProx" => {
                s.normal = [1.0, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                if let Some(v) = find_phalanx_dorsal_point(prims, prim_bones, "LeftPinkyProx", 1.0) {
                    s.pos = v;
                } else {
                    s.pos = [0.205, 0.835, 0.037];
                }
            }
            "LeftPinkyMid" => {
                s.normal = [1.0, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                if let Some(v) = find_phalanx_dorsal_point(prims, prim_bones, "LeftPinkyMid", 1.0) {
                    s.pos = v;
                } else {
                    s.pos = [0.205, 0.810, 0.037];
                }
            }
            "LeftPinkyDist" => {
                s.normal = [1.0, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                if let Some(v) = find_phalanx_dorsal_point(prims, prim_bones, "LeftPinkyDist", 1.0) {
                    s.pos = v;
                } else {
                    s.pos = [0.205, 0.785, 0.037];
                }
            }
            // 6. Фаланги правого мизинца: двухосевое центрирование по геометрическому центру тела фаланги
            "RightPinkyProx" => {
                s.normal = [-1.0, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                if let Some(v) = find_phalanx_dorsal_point(prims, prim_bones, "RightPinkyProx", -1.0) {
                    s.pos = v;
                } else {
                    s.pos = [-0.205, 0.835, 0.037];
                }
            }
            "RightPinkyMid" => {
                s.normal = [-1.0, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                if let Some(v) = find_phalanx_dorsal_point(prims, prim_bones, "RightPinkyMid", -1.0) {
                    s.pos = v;
                } else {
                    s.pos = [-0.205, 0.810, 0.037];
                }
            }
            "RightPinkyDist" => {
                s.normal = [-1.0, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                if let Some(v) = find_phalanx_dorsal_point(prims, prim_bones, "RightPinkyDist", -1.0) {
                    s.pos = v;
                } else {
                    s.pos = [-0.205, 0.785, 0.037];
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