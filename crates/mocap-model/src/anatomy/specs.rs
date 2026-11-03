//! # Декларативная спецификация анатомической привязки датчиков (`specs`)

use super::solver::{
    find_acromion_point, find_clavicle_point, find_foot_point, find_sagittal_point,
    find_surface_point,
};
use crate::math::norm;
use crate::skeleton::{Chip, Prim, SensorSite, Side};

/// Применяет канонические анатомические параметры (нормали, тангенсы, высоты) ко всем 64 сайтам.
pub fn apply_anatomical_specs(prims: &[Prim], prim_bones: &[&'static str], sites: &mut [SensorSite]) {
    for s in sites.iter_mut() {
        let tag = s.tag;
        let side_sign = if s.side == Side::Left { 1.0_f32 } else { -1.0_f32 };

        match tag {
            // ── Осевой скелет и голова (8) — строго на сагиттальной оси X = 0 ────────────
            "Head" => {
                s.normal = [0.0, 0.050, -0.999];
                s.tangent = [1.0, 0.0, 0.0];
                s.chip = Chip::Medium;
                if let Some(v) = find_sagittal_point(prims, prim_bones, "Head", 1.630, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.630, -0.015];
                }
            }
            "Neck" => {
                s.normal = [0.0, 0.080, -0.997];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                if let Some(v) = find_sagittal_point(prims, prim_bones, "Neck", 1.490, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.490, -0.010];
                }
            }
            "SpineT3" => {
                s.normal = [0.0, 0.020, -0.999];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_point(prims, prim_bones, "SpineT3", 1.370, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.370, -0.035];
                }
            }
            "SpineT7" => {
                s.normal = [0.0, 0.010, -0.999];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_point(prims, prim_bones, "SpineT3", 1.270, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.270, -0.015];
                }
            }
            "SpineT11" => {
                s.normal = [0.0, -0.020, -0.999];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_point(prims, prim_bones, "SpineT3", 1.170, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.170, -0.008];
                }
            }
            "SpineL3" => {
                s.normal = [0.0, -0.050, -0.998];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_point(prims, prim_bones, "SpineL3", 1.070, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 1.070, -0.005];
                }
            }
            "Sacrum" => {
                s.normal = [0.0, -0.080, -0.996];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_point(prims, prim_bones, "Sacrum", 0.960, false) {
                    s.pos = v;
                } else {
                    s.pos = [0.0, 0.960, -0.005];
                }
            }
            "Sternum" => {
                s.normal = [0.0, 0.0, 1.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                if let Some(v) = find_sagittal_point(prims, prim_bones, "SpineT3", 1.320, true) {
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
                if let Some(v) = find_clavicle_point(prims, prim_bones, side_sign) {
                    s.pos = v;
                } else {
                    s.pos = [side_sign * 0.095, 1.345, 0.085];
                }
            }
            "LeftShoulder" | "RightShoulder" => {
                s.normal = norm([side_sign * 0.220, 0.950, -0.200]);
                s.tangent = norm([side_sign * 0.950, -0.220, 0.0]);
                s.chip = Chip::Medium;
                if let Some(v) = find_acromion_point(prims, prim_bones, side_sign) {
                    s.pos = v;
                } else {
                    s.pos = [side_sign * 0.142, 1.380, 0.010];
                }
            }

            // ── Конечности: плечо, предплечье и кисть (10) ─────────────────
            "LeftUpperArmProx" | "RightUpperArmProx" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh = if s.side == Side::Left { "LeftUpperArmProx" } else { "RightUpperArmProx" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.75)) {
                    s.pos = v;
                }
            }
            "LeftUpperArmDist" | "RightUpperArmDist" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh = if s.side == Side::Left { "LeftUpperArmProx" } else { "RightUpperArmProx" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.25)) {
                    s.pos = v;
                }
            }
            "LeftForearmProx" | "RightForearmProx" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh = if s.side == Side::Left { "LeftForearmDist" } else { "RightForearmDist" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.75)) {
                    s.pos = v;
                }
            }
            "LeftForearmDist" | "RightForearmDist" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh = if s.side == Side::Left { "LeftForearmDist" } else { "RightForearmDist" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.25)) {
                    s.pos = v;
                }
            }
            "LeftHand" | "RightHand" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Medium;
                let mesh = if s.side == Side::Left { "LeftHand" } else { "RightHand" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.50)) {
                    s.pos = v;
                }
            }

            // ── Большой палец (6) — подавление фантомов ThumbProx ──────────
            "LeftThumbMeta" | "RightThumbMeta" => {
                s.normal = norm([side_sign * 0.707, 0.0, 0.707]);
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                let mesh = if s.side == Side::Left { "LeftThumbMeta" } else { "RightThumbMeta" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.50)) {
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
                let mesh = if s.side == Side::Left { "LeftThumbDist" } else { "RightThumbDist" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.50)) {
                    s.pos = v;
                }
            }

            // ── Фаланги пальцев кистей (24) ─────────────────────────────────
            _ if tag.contains("Index") || tag.contains("Middle") || tag.contains("Ring") || tag.contains("Pinky") => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Small;
                if let Some(v) = find_surface_point(prims, prim_bones, tag, s.normal, Some(0.50)) {
                    s.pos = v;
                }
            }

            // ── Нижние конечности и стопы (12) ─────────────────────────────
            "LeftThighProx" | "RightThighProx" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                let mesh = if s.side == Side::Left { "LeftThighDist" } else { "RightThighDist" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.80)) {
                    s.pos = v;
                }
            }
            "LeftThighDist" | "RightThighDist" => {
                s.normal = [side_sign, 0.0, 0.0];
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                let mesh = if s.side == Side::Left { "LeftThighDist" } else { "RightThighDist" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.30)) {
                    s.pos = v;
                }
            }
            "LeftShankProx" | "RightShankProx" => {
                s.normal = norm([side_sign * 0.35, 0.0, 0.93]);
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                let mesh = if s.side == Side::Left { "LeftShankDist" } else { "RightShankDist" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.75)) {
                    s.pos = v;
                }
            }
            "LeftShankDist" | "RightShankDist" => {
                s.normal = norm([side_sign * 0.35, 0.0, 0.93]);
                s.tangent = [0.0, 1.0, 0.0];
                s.chip = Chip::Large;
                let mesh = if s.side == Side::Left { "LeftShankDist" } else { "RightShankDist" };
                if let Some(v) = find_surface_point(prims, prim_bones, mesh, s.normal, Some(0.25)) {
                    s.pos = v;
                }
            }
            "LeftFoot" | "RightFoot" => {
                s.normal = norm([0.0, 0.960, 0.280]);
                s.tangent = [0.0, 0.0, 1.0];
                s.chip = Chip::Medium;
                let mesh = if s.side == Side::Left { "LeftFoot" } else { "RightFoot" };
                if let Some(v) = find_foot_point(prims, prim_bones, mesh, side_sign, 0.040) {
                    s.pos = v;
                } else {
                    s.pos = [side_sign * 0.082, 0.048, 0.040];
                }
            }
            "LeftToe" | "RightToe" => {
                s.normal = norm([0.0, 0.980, 0.200]);
                s.tangent = [0.0, 0.0, 1.0];
                s.chip = Chip::Small;
                let mesh = if s.side == Side::Left { "LeftToe" } else { "RightToe" };
                if let Some(v) = find_foot_point(prims, prim_bones, mesh, side_sign, 0.095) {
                    s.pos = v;
                } else {
                    s.pos = [side_sign * 0.082, 0.024, 0.095];
                }
            }
            _ => {}
        }
    }
}