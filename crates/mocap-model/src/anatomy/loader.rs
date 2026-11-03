//! # Десериализатор бинарного контейнера мешей и скелета (`loader`)

use crate::math::V3;
use crate::skeleton::{Chip, Joint, JointKind, Prim, SensorSite, Side, Tone};

static MANNEQUIN_BYTES: &[u8] = include_bytes!("../../assets/mannequin.bin");

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
fn read_v3(bytes: &[u8], cursor: &mut usize) -> V3 {
    let x = read_f32(bytes, cursor);
    let y = read_f32(bytes, cursor);
    let z = read_f32(bytes, cursor);
    [x, y, z]
}

fn resolve_joint_bone_tag(name: &str) -> &'static str {
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

/// Выполняет плоское чтение примитивов геометрии, сайтов и суставов из бинарного ресурса.
#[must_use]
pub fn load_raw_model() -> (Vec<Prim>, Vec<&'static str>, Vec<SensorSite>, Vec<Joint>) {
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

    let mut sites = Vec::new();
    if (is_v2 || is_v3) && cursor < bytes.len() {
        let num_sites = read_u32(bytes, &mut cursor) as usize;
        sites.reserve(num_sites);

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
    }

    let mut joints = Vec::new();
    if is_v3 && cursor < bytes.len() {
        let num_joints = read_u32(bytes, &mut cursor) as usize;
        joints.reserve(num_joints);

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
            let target_bone = resolve_joint_bone_tag(&name_str);

            joints.push(Joint {
                name: name_str.to_string(),
                kind,
                pos,
                parent,
                axis: None,
                marker_r,
                target_bone,
            });
        }
    }

    (prims, prim_bones, sites, joints)
}