//! # Процедурная геометрия анатомической 5-палой стопы манекена (`foot`)

use super::super::math::{add, cross, mirror, norm, sub, V3};
use super::super::skeleton::{JointKind, Prim, Side, Tone};
use super::{cr, Builder, FRONT, UP, X};
use std::f32::consts::TAU;

#[derive(Clone, Copy)]
struct FootRing {
    z: f32,
    y: f32,
    rx: f32,
    ry: f32,
    arch: f32,
    oblique: f32,
}

const RINGS_DEF: [FootRing; 7] = [
    // 1. Пяточный бугор (Z = -26 мм)
    FootRing { z: -0.026, y: 0.016, rx: 0.014, ry: 0.013, arch: 0.000, oblique: 0.000 },
    // 2. Опорная подушка пятки (контакт с полом)
    FootRing { z: -0.012, y: 0.018, rx: 0.017, ry: 0.018, arch: 0.000, oblique: 0.000 },
    // 3. Подтаранный отдел / пред-ахиллова седловина
    FootRing { z: 0.006, y: 0.024, rx: 0.019, ry: 0.023, arch: 0.003, oblique: 0.000 },
    // 4. Ладьевидная кость / пик медиального свода (арка 15 мм)
    FootRing { z: 0.032, y: 0.023, rx: 0.021, ry: 0.020, arch: 0.015, oblique: 0.000 },
    // 5. Тела плюсневых костей
    FootRing { z: 0.060, y: 0.017, rx: 0.023, ry: 0.016, arch: 0.007, oblique: 0.006 },
    // 6. Дистальный отдел плюсны с анатомическим скосом
    FootRing { z: 0.085, y: 0.013, rx: 0.025, ry: 0.012, arch: 0.002, oblique: 0.012 },
    // 7. Подушка плюсневых головок, идущая строго по зеленой линии
    FootRing { z: 0.106, y: 0.010, rx: 0.026, ry: 0.009, arch: 0.000, oblique: 0.018 },
];

const RING_N: usize = 20;

fn refine_foot_rings(rings: &[FootRing]) -> Vec<FootRing> {
    const SUB: usize = 3;
    if rings.len() < 2 {
        return rings.to_vec();
    }
    let mut out = Vec::new();
    let z_vals: Vec<f32> = rings.iter().map(|r| r.z).collect();
    let y_vals: Vec<f32> = rings.iter().map(|r| r.y).collect();
    let rx_vals: Vec<f32> = rings.iter().map(|r| r.rx).collect();
    let ry_vals: Vec<f32> = rings.iter().map(|r| r.ry).collect();
    let arch_vals: Vec<f32> = rings.iter().map(|r| r.arch).collect();
    let obl_vals: Vec<f32> = rings.iter().map(|r| r.oblique).collect();

    for i in 0..rings.len() - 1 {
        for j in 0..SUB {
            let f = j as f32 / SUB as f32;
            out.push(FootRing {
                z: cr(&z_vals, i, f),
                y: cr(&y_vals, i, f),
                rx: cr(&rx_vals, i, f),
                ry: cr(&ry_vals, i, f),
                arch: cr(&arch_vals, i, f),
                oblique: cr(&obl_vals, i, f),
            });
        }
    }
    out.push(rings[rings.len() - 1]);
    out
}

fn foot_ring_pts(r: &FootRing, x_leg: f32, s: f32, k_scale: f32, dz: f32) -> Vec<V3> {
    (0..RING_N)
        .map(|k| {
            let th = (k as f32) * (TAU / RING_N as f32);
            let c = th.cos(); // c > 0 латерально, c < 0 медиально
            let st = th.sin();
            let x = r.rx * k_scale * c * s;
            let mut y = r.y + r.ry * k_scale * st;
            if st < 0.0 && c < 0.0 {
                y += r.arch * k_scale * (-c).powf(1.3) * (-st);
            }
            // Линейный скос по зеленой линии: латеральный край отступает назад на величину oblique
            let z_skew = if c > 0.0 {
                -r.oblique * 0.65 * c
            } else {
                r.oblique * 0.35 * (-c)
            };
            [x_leg + x, y, r.z + z_skew + dz]
        })
        .collect()
}

fn mesh_normals(v: &[V3], idx: &[u32]) -> Vec<V3> {
    let mut n = vec![[0.0_f32; 3]; v.len()];
    for t in idx.chunks_exact(3) {
        let (i0, i1, i2) = (t[0] as usize, t[1] as usize, t[2] as usize);
        let f = cross(sub(v[i1], v[i0]), sub(v[i2], v[i0]));
        for i in [i0, i1, i2] {
            n[i] = add(n[i], f);
        }
    }
    n.into_iter().map(norm).collect()
}

fn build_tarso_metatarsal_mesh(x_leg: f32, s: f32) -> (Vec<V3>, Vec<V3>, Vec<u32>) {
    let refined = refine_foot_rings(&RINGS_DEF);
    let mut layers: Vec<Vec<V3>> = Vec::new();
    let angs = [75.0_f32, 50.0, 25.0];

    // 1. Полусферический купол пятки
    let first = &refined[0];
    let h_heel = 0.006_f32;
    for th in angs {
        let (sin_a, cos_a) = th.to_radians().sin_cos();
        layers.push(foot_ring_pts(first, x_leg, s, cos_a, -h_heel * sin_a));
    }

    // 2. Сечения корпуса стопы с косым скосом плюсны (зеленая линия)
    for r in &refined {
        layers.push(foot_ring_pts(r, x_leg, s, 1.0, 0.0));
    }

    // 3. Дистальная фаска подушки плюсны, повторяющая зеленую линию
    let last = &refined[refined.len() - 1];
    let h_front = 0.004_f32;
    for th in angs.iter().rev() {
        let (sin_a, cos_a) = th.to_radians().sin_cos();
        layers.push(foot_ring_pts(last, x_leg, s, cos_a, h_front * sin_a));
    }

    let l = layers.len() as u32;
    let mut v: Vec<V3> = layers.into_iter().flatten().collect();
    let heel_apex_idx = v.len() as u32;
    v.push([x_leg, first.y, first.z - h_heel]);
    let front_apex_idx = v.len() as u32;
    v.push([x_leg - 0.005 * s, last.y, last.z + h_front]);

    let n = RING_N as u32;
    let mut idx: Vec<u32> = Vec::new();

    // Замыкание пятки
    for k in 0..n {
        let nk = (k + 1) % n;
        if s > 0.0 {
            idx.extend([heel_apex_idx, k, nk]);
        } else {
            idx.extend([heel_apex_idx, nk, k]);
        }
    }

    // Стенки между всеми кольцами (динамический обход l)
    for i in 0..(l - 1) {
        let r0 = i * n;
        let r1 = (i + 1) * n;
        for k in 0..n {
            let nk = (k + 1) % n;
            let (a0, a1, b0, b1) = (r0 + k, r0 + nk, r1 + k, r1 + nk);
            if s > 0.0 {
                idx.extend([a0, b0, a1, a1, b0, b1]);
            } else {
                idx.extend([a0, a1, b0, a1, b1, b0]);
            }
        }
    }

    // Замыкание переднего валика
    let last_r = (l - 1) * n;
    for k in 0..n {
        let nk = (k + 1) % n;
        if s > 0.0 {
            idx.extend([front_apex_idx, last_r + nk, last_r + k]);
        } else {
            idx.extend([front_apex_idx, last_r + k, last_r + nk]);
        }
    }

    let normals = mesh_normals(&v, &idx);
    (v, normals, idx)
}

/// Собирает анатомическую стопу и пальцы с правильным скосом по зеленой линии
pub fn build_foot(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let m = |v: V3| mirror(s, v);
    let x_leg = 0.082 * s;

    let foot_tag = if side == Side::Left { "LeftFoot" } else { "RightFoot" };
    let toe_tag = if side == Side::Left { "LeftToe" } else { "RightToe" };

    // 1. Голеностопный шарнир
    let ankle_pos = m([0.082, 0.056, 0.000]);
    b.ball(ankle_pos, 0.016, Tone::Plate);

    // 2. Тарсо-метатарзальный блок с анатомическим скосом
    b.set_bone(foot_tag);
    let (fv, fnm, fi) = build_tarso_metatarsal_mesh(x_leg, s);
    b.prims.push(Prim::Mesh { vertices: fv, normals: fnm, indices: fi, tone: Tone::Cortical });
    b.prim_bones.push(b.current_bone);

    // Сенсорный сегмент тыла плюсны (MT3)
    let mt3_a = m([0.084, 0.035, 0.035]);
    let mt3_b = m([0.085, 0.019, 0.075]);
    b.seg_only(format!("{p}MT3"), mt3_a, mt3_b, vec![(0.0, 0.004), (1.0, 0.004)], UP, FRONT);

    // 3. Анатомический 5-палый носок вдоль зеленой линии (без повторного умножения на s)
    b.set_bone(toe_tag);

    // Канонические координаты центров MTP вдоль скошенной дуги (зеленая линия)
    let mtp1 = [0.067, 0.0100, 0.114];
    let mtp2 = [0.077, 0.0095, 0.115];
    let mtp3 = [0.086, 0.0090, 0.111];
    let mtp4 = [0.094, 0.0085, 0.105];
    let mtp5 = [0.101, 0.0080, 0.098];

    // I луч (Hallux) — длина 26 мм
    let ip1 = [0.066, 0.0075, 0.128];
    let tip1 = [0.065, 0.0055, 0.140];
    b.ball(m(mtp1), 0.0068, Tone::Plate);
    b.part(format!("{p}HalluxPP"), m(mtp1), m(ip1), 0.0060, 0.0054, UP, FRONT);
    b.cap(m(ip1), m(tip1), 0.0054, 0.0045, Tone::Cortical);

    // II палец — длина 24 мм
    let ip2 = [0.077, 0.0070, 0.128];
    let tip2 = [0.077, 0.0045, 0.139];
    b.ball(m(mtp2), 0.0052, Tone::Plate);
    b.cap(m(mtp2), m(ip2), 0.0046, 0.0038, Tone::Cortical);
    b.cap(m(ip2), m(tip2), 0.0038, 0.0030, Tone::Cortical);

    // III палец — длина 21 мм
    let ip3 = [0.0865, 0.0065, 0.123];
    let tip3 = [0.0870, 0.0040, 0.132];
    b.ball(m(mtp3), 0.0048, Tone::Plate);
    b.cap(m(mtp3), m(ip3), 0.0042, 0.0034, Tone::Cortical);
    b.cap(m(ip3), m(tip3), 0.0034, 0.0026, Tone::Cortical);

    // IV палец — длина 18 мм
    let ip4 = [0.095, 0.0060, 0.115];
    let tip4 = [0.096, 0.0035, 0.123];
    b.ball(m(mtp4), 0.0044, Tone::Plate);
    b.cap(m(mtp4), m(ip4), 0.0038, 0.0030, Tone::Cortical);
    b.cap(m(ip4), m(tip4), 0.0030, 0.0022, Tone::Cortical);

    // V палец (мизинец) — длина 16 мм
    let ip5 = [0.102, 0.0055, 0.107];
    let tip5 = [0.103, 0.0030, 0.114];
    b.ball(m(mtp5), 0.0040, Tone::Plate);
    b.cap(m(mtp5), m(ip5), 0.0034, 0.0026, Tone::Cortical);
    b.cap(m(ip5), m(tip5), 0.0026, 0.0018, Tone::Cortical);

    // 4. Кинематические суставы
    b.joint(&format!("{p}Ankle"), JointKind::Hinge, ankle_pos, Some(&format!("{p}Knee")), 0.005, Some(X));
    b.joint(&format!("{p}Subtalar"), JointKind::Gliding, m([0.082, 0.028, -0.008]), Some(&format!("{p}Ankle")), 0.0035, None);
    b.joint(&format!("{p}MTP1"), JointKind::Hinge, m(mtp1), Some(&format!("{p}Subtalar")), 0.0030, Some(X));
}