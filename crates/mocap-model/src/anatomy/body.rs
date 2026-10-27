//! # Анатомические сегменты торса, конечностей и кистей (`body`)

use super::super::math::{add, cross, dot, madd, mirror, norm, scale, sub, V3};
use super::super::skeleton::{JointKind, Prim, Side, Tone};
use super::lofting::{loft, Ring};
use super::{cr, lin, Builder, FRONT, UP, X};

pub const WRIST: V3 = [0.216, 0.845, -0.006];
pub const ZS: f32 = 1.3;
pub const WS: f32 = 1.35;

pub const FOOT: [(f32, f32, f32, f32, f32); 11] = [
    (-0.038, 0.003, 0.036, 0.014, 0.013),
    (-0.028, 0.000, 0.047, 0.018, 0.016),
    (-0.012, 0.000, 0.060, 0.021, 0.019),
    (0.005, 0.000, 0.058, 0.022, 0.020),
    (0.022, 0.000, 0.045, 0.023, 0.021),
    (0.042, 0.000, 0.034, 0.024, 0.022),
    (0.065, 0.000, 0.026, 0.027, 0.025),
    (0.085, 0.000, 0.021, 0.027, 0.024),
    (0.105, 0.000, 0.016, 0.024, 0.021),
    (0.120, 0.000, 0.012, 0.019, 0.016),
    (0.132, 0.001, 0.007, 0.011, 0.009),
];

fn r(y: f32, cz: f32, rx: f32, rz: f32, ex: f32) -> Ring {
    Ring { c: [0.0, y, cz], rx, rz, ex }
}

fn ring_at(rings: &[Ring], y: f32) -> (f32, f32, f32) {
    let mut i = 0;
    while i + 2 < rings.len() && y > rings[i + 1].c[1] {
        i += 1;
    }
    let (r0, r1) = (&rings[i], &rings[i + 1]);
    let t = ((y - r0.c[1]) / (r1.c[1] - r0.c[1])).clamp(0.0, 1.0);
    let rx: Vec<f32> = rings.iter().map(|q| q.rx).collect();
    let rz: Vec<f32> = rings.iter().map(|q| q.rz).collect();
    (r0.c[2] + (r1.c[2] - r0.c[2]) * t, cr(&rx, i, t), cr(&rz, i, t))
}

fn surf_pt(rings: &[Ring], y: f32, front: bool) -> (V3, V3) {
    let z_at = |yy: f32| {
        let (cz, _, rz) = ring_at(rings, yy);
        if front { cz + rz } else { cz - rz }
    };
    let dz = z_at(y + 0.01) - z_at(y - 0.01);
    let n = if front { norm([0.0, -dz, 0.02]) } else { norm([0.0, dz, -0.02]) };
    ([0.0, y, z_at(y)], n)
}

#[must_use]
pub fn foot_top(z: f32) -> f32 {
    let zs: Vec<(f32, f32)> = FOOT.iter().map(|st| (st.0 * ZS, st.2)).collect();
    lin(&zs, z)
}

pub fn torso(b: &mut Builder) {
    b.set_bone("Sacrum");
    let pelvis = [
        r(0.860, -0.010, 0.070, 0.070, 2.4),
        r(0.890, -0.010, 0.112, 0.088, 2.5),
        r(0.950, -0.010, 0.140, 0.098, 2.5),
        r(1.010, -0.010, 0.142, 0.098, 2.5),
        r(1.060, -0.010, 0.126, 0.090, 2.4),
        r(1.095, -0.012, 0.114, 0.084, 2.3),
    ];
    loft(b, &pelvis, Some(0.040), None, Tone::Cortical);

    b.set_bone("SpineL3");
    let abdomen = [
        r(1.080, -0.012, 0.100, 0.080, 2.3),
        r(1.150, -0.012, 0.094, 0.074, 2.3),
        r(1.200, -0.012, 0.098, 0.076, 2.3),
        r(1.258, -0.012, 0.106, 0.082, 2.3),
    ];
    loft(b, &abdomen, None, None, Tone::Cortical);

    b.set_bone("SpineT3");
    let chest = [
        r(1.255, -0.010, 0.118, 0.088, 2.6),
        r(1.300, -0.010, 0.140, 0.096, 2.6),
        r(1.350, -0.010, 0.150, 0.100, 2.6),
        r(1.400, -0.010, 0.145, 0.094, 2.5),
        r(1.440, -0.010, 0.120, 0.085, 2.4),
        r(1.462, -0.010, 0.075, 0.060, 2.2),
    ];
    loft(b, &chest, None, Some(0.025), Tone::Cortical);

    b.set_bone("Neck");
    let neck = [
        r(1.440, -0.010, 0.050, 0.050, 2.0),
        r(1.500, -0.006, 0.043, 0.043, 2.0),
        r(1.545, 0.000, 0.038, 0.038, 2.0),
        r(1.585, 0.012, 0.035, 0.035, 2.0),
        r(1.620, 0.018, 0.034, 0.034, 2.0),
    ];
    loft(b, &neck, None, None, Tone::Cortical);

    b.set_bone("Head");
    let head: Vec<Ring> = [
        (1.585, 0.030, 0.028, 0.032),
        (1.605, 0.020, 0.052, 0.056),
        (1.635, 0.010, 0.071, 0.078),
        (1.665, 0.002, 0.077, 0.090),
        (1.695, -0.004, 0.073, 0.089),
        (1.725, -0.007, 0.056, 0.068),
    ]
    .iter()
    .map(|&(y, cz, rx, rz)| r(1.570 + (y - 1.585) * 1.1, cz * 1.1, rx * 1.1, rz * 1.1, 2.1))
    .collect();
    loft(b, &head, Some(0.0165), Some(0.035), Tone::Cortical);

    let ins = |b: &mut Builder, name: &str, (p, n): (V3, V3)| {
        b.anchors.insert(name.to_string(), (p, n));
    };
    ins(b, "Forehead", surf_pt(&head, 1.680, true));
    ins(b, "C5", surf_pt(&neck, 1.520, false));
    ins(b, "T3", surf_pt(&chest, 1.400, false));
    ins(b, "T7", surf_pt(&chest, 1.340, false));
    ins(b, "T11", surf_pt(&abdomen, 1.210, false));
    ins(b, "L3", surf_pt(&abdomen, 1.120, false));
    ins(b, "Sternum", surf_pt(&chest, 1.360, true));
    ins(b, "Sacrum", surf_pt(&pelvis, 0.975, false));

    let jr = 0.004;
    b.joint("L5S1", JointKind::Spinal, [0.0, 0.99, -0.020], Some("Pelvis"), jr, None);
    b.joint("L3L4", JointKind::Spinal, [0.0, 1.08, -0.020], Some("L5S1"), jr, None);
    b.joint("T12L1", JointKind::Spinal, [0.0, 1.18, -0.020], Some("L3L4"), jr, None);
    b.joint("T7T8", JointKind::Spinal, [0.0, 1.32, -0.020], Some("T12L1"), jr, None);
    b.joint("C7T1", JointKind::Spinal, [0.0, 1.46, -0.015], Some("T7T8"), jr, None);
    b.joint("C4C5", JointKind::Spinal, [0.0, 1.53, -0.005], Some("C7T1"), 0.0035, None);
    b.joint("C1C2", JointKind::Pivot, [0.0, 1.59, 0.005], Some("C4C5"), 0.004, None);
    b.joint("Occiput", JointKind::Condyloid, [0.0, 1.61, 0.010], Some("C1C2"), 0.005, None);
}

pub fn shoulder_and_arm(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let m = |v: V3| mirror(s, v);
    let out = [s, 0.0, 0.0];

    let clavicle_tag = if side == Side::Left { "LeftClavicle" } else { "RightClavicle" };
    let shoulder_tag = if side == Side::Left { "LeftShoulder" } else { "RightShoulder" };
    let upper_arm_tag = if side == Side::Left { "LeftUpperArmProx" } else { "RightUpperArmProx" };
    let forearm_tag = if side == Side::Left { "LeftForearmProx" } else { "RightForearmProx" };

    // 1. Плечевой пояс: Ключица и Акромиальный козырек лопатки
    b.set_bone(clavicle_tag);
    let sc_pt = m([0.038, 1.436, 0.014]);
    let ac_pt = m([0.168, 1.408, -0.014]);
    b.part(format!("{p}Clavicle"), sc_pt, ac_pt, 0.024, 0.030, UP, FRONT);

    b.set_bone(shoulder_tag);
    let acromion_hood_a = m([0.138, 1.416, -0.018]);
    let acromion_hood_b = m([0.186, 1.402, -0.020]);
    b.cap(acromion_hood_a, acromion_hood_b, 0.024, 0.026, Tone::Cortical);
    b.seg_only(format!("{p}Acromion"), acromion_hood_a, acromion_hood_b, vec![(0.0, 0.024), (1.0, 0.026)], UP, FRONT);

    // Плечевой шаровидный сустав (GH)
    let gh = m([0.178, 1.390, -0.012]);
    b.ball(gh, 0.036, Tone::Plate);

    // 2. Плечо (Brachium): дельтовидный окат + трехгранное тело + надмыщелки локтя
    b.set_bone(upper_arm_tag);
    let elbow = m([0.198, 1.108, -0.012]);

    let humerus_rings = [
        Ring { c: m([0.198, 1.118, -0.012]), rx: 0.033, rz: 0.023, ex: 2.3 },
        Ring { c: m([0.194, 1.175, -0.012]), rx: 0.031, rz: 0.027, ex: 2.2 },
        Ring { c: m([0.188, 1.245, -0.012]), rx: 0.034, rz: 0.035, ex: 2.2 },
        Ring { c: m([0.183, 1.315, -0.013]), rx: 0.038, rz: 0.037, ex: 2.1 },
        Ring { c: m([0.178, 1.378, -0.013]), rx: 0.042, rz: 0.040, ex: 2.0 },
    ];
    loft(b, &humerus_rings, Some(0.008), Some(0.016), Tone::Cortical);

    b.seg_only(
        format!("{p}UpperArm"),
        gh,
        elbow,
        vec![(0.0, 0.040), (0.30, 0.037), (0.72, 0.030), (1.0, 0.026)],
        FRONT,
        out,
    );

    // 3. Анатомический блоковидный узел локтя: поперечный валик (Hinge Spool) + Olecranon
    let ep_med = m([0.181, 1.108, -0.012]);
    let ep_lat = m([0.215, 1.108, -0.012]);
    b.cap(ep_med, ep_lat, 0.015, 0.013, Tone::Plate);

    // Локтевой отросток сзади (Olecranon) — костное ребро локтя
    let olecranon_a = m([0.198, 1.118, -0.018]);
    let olecranon_b = m([0.198, 1.096, -0.023]);
    b.cap(olecranon_a, olecranon_b, 0.012, 0.010, Tone::Plate);

    // Медиальный мыщелковый выступ внутрь
    let epicondyle_med_bump = m([0.177, 1.108, -0.012]);
    b.ball(epicondyle_med_bump, 0.0065, Tone::Plate);

    // 4. Предплечье (Antebrachium): латеральный валик плечелучевой мышцы + стройный конический сход
    b.set_bone(forearm_tag);
    let wrist = m([0.216, 0.858, -0.006]);

    let forearm_rings = [
        Ring { c: m([0.216, 0.855, -0.006]), rx: 0.0115, rz: 0.0110, ex: 2.1 },
        Ring { c: m([0.215, 0.875, -0.006]), rx: 0.0145, rz: 0.0125, ex: 2.2 },
        Ring { c: m([0.213, 0.925, -0.007]), rx: 0.0190, rz: 0.0160, ex: 2.2 },
        Ring { c: m([0.209, 0.985, -0.009]), rx: 0.0250, rz: 0.0210, ex: 2.3 },
        Ring { c: m([0.204, 1.045, -0.008]), rx: 0.0320, rz: 0.0260, ex: 2.3 },
        Ring { c: m([0.199, 1.096, -0.011]), rx: 0.0260, rz: 0.0220, ex: 2.1 },
    ];
    loft(b, &forearm_rings, Some(0.003), Some(0.008), Tone::Cortical);

    let fprof = vec![(0.0, 0.026), (0.30, 0.029), (0.72, 0.019), (1.0, 0.014)];
    b.seg_only(format!("{p}Ulna"), elbow, wrist, fprof.clone(), FRONT, out);
    b.seg_only(format!("{p}Radius"), elbow, wrist, fprof, FRONT, out);

    // 5. Лучезапястный шарнир: компактный эстетичный шаровой шарнир манекена
    let wrist_joint = m(WRIST);
    b.ball(wrist_joint, 0.0105, Tone::Plate);

    // Кинематические сочленения: прямая кинематическая цепь руки SC -> GH -> Elbow -> Wrist
    b.joint(&format!("{p}SC"), JointKind::Saddle, m([0.038, 1.436, 0.014]), Some("C7T1"), 0.005, None);
    b.joint(&format!("{p}GH"), JointKind::Ball, gh, Some(&format!("{p}SC")), 0.008, None);
    b.joint(&format!("{p}Elbow"), JointKind::Hinge, elbow, Some(&format!("{p}GH")), 0.006, Some(X));
}

pub fn hand(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let rel = |dx: f32, dy: f32, dz: f32| mirror(s, [WRIST[0] + dx, WRIST[1] + dy, WRIST[2] + dz]);
    let out = [s, 0.0, 0.0];
    let wrist_name = format!("{p}Wrist");
    let wrist_pos = mirror(s, WRIST);
    // Лучезапястный сустав соединяется напрямую с локтем (Elbow), формируя жесткое звено предплечья
    b.joint(&wrist_name, JointKind::Condyloid, wrist_pos, Some(&format!("{p}Elbow")), 0.005, None);

    let hand_tag = if side == Side::Left { "LeftHand" } else { "RightHand" };
    b.set_bone(hand_tag);

    // 1. Центральное тело пясти: плавное расширение от шарнира запястья к костяшкам
    let palm_rings = [
        Ring { c: rel(0.000, -0.062, 0.000), rx: 0.0075, rz: 0.0185, ex: 2.2 },
        Ring { c: rel(0.000, -0.054, 0.000), rx: 0.0090, rz: 0.0195, ex: 2.3 },
        Ring { c: rel(-0.001, -0.038, 0.000), rx: 0.0110, rz: 0.0185, ex: 2.3 },
        Ring { c: rel(-0.001, -0.020, 0.000), rx: 0.0105, rz: 0.0155, ex: 2.2 },
        Ring { c: rel(0.000, -0.006, -0.001), rx: 0.0095, rz: 0.0120, ex: 2.1 },
    ];
    loft(b, &palm_rings, Some(0.003), Some(0.003), Tone::Cortical);

    // 2. Тенар (каплевидный мышечный корень большого пальца)
    let thenar_a = rel(-0.003, -0.012, 0.006);
    let thenar_b = rel(-0.007, -0.038, 0.017);
    b.cap(thenar_a, thenar_b, 0.0085, 0.0070, Tone::Cortical);

    let thenar_fill_a = rel(-0.004, -0.022, 0.008);
    let thenar_fill_b = rel(-0.006, -0.040, 0.012);
    b.cap(thenar_fill_a, thenar_fill_b, 0.0075, 0.0065, Tone::Cortical);

    // 3. Гипотенар (ульнарный валик ладони со стороны мизинца)
    let hypo_a = rel(-0.003, -0.014, -0.008);
    let hypo_b = rel(-0.003, -0.048, -0.013);
    b.cap(hypo_a, hypo_b, 0.0070, 0.0060, Tone::Cortical);

    // Опорная площадка для IMU-датчика тыла кисти (MC3)
    let mc3_a = rel(0.0075, -0.016, 0.002);
    let mc3_b = rel(0.0065, -0.052, 0.003);
    b.seg_only(format!("{p}MC3"), mc3_a, mc3_b, vec![(0.0, 0.004), (1.0, 0.004)], out, FRONT);

    // 4. Пальцы II–V (MCP, фаланги и сочленения)
    let names = ["Index", "Middle", "Ring", "Pinky"];
    let mcp_coords = [
        rel(0.000, -0.063, 0.013),   // Index (II)
        rel(0.001, -0.067, 0.004),   // Middle (III, пик арки)
        rel(0.000, -0.064, -0.005),  // Ring (IV)
        rel(-0.001, -0.059, -0.013), // Pinky (V, проксимальный сдвиг)
    ];
    let knuckle_r = [0.0045, 0.0048, 0.0045, 0.0038];
    let lens = [
        [0.029, 0.018, 0.013],
        [0.033, 0.021, 0.015],
        [0.030, 0.019, 0.014],
        [0.022, 0.014, 0.011],
    ];
    let fan = [0.025_f32, 0.0, -0.016, -0.035];
    let theta = [4.0_f32, 8.0, 12.0];
    let radii = [(0.0042, 0.0036), (0.0036, 0.0031), (0.0031, 0.0024)];
    let phal = ["PP", "MP", "DP"];

    for f in 0..4 {
        let mcp = mcp_coords[f];
        let mcp_name = format!("{p}MCP{}", f + 2);
        b.ball(mcp, knuckle_r[f], Tone::Plate);
        b.joint(&mcp_name, JointKind::Condyloid, mcp, Some(&wrist_name), 0.003, None);

        let mut prev = mcp_name;
        let mut cur = mcp;
        for k in 0..3 {
            let th = theta[k].to_radians();
            let dir = norm([-s * th.sin(), -th.cos(), fan[f]]);
            let next = madd(cur, dir, lens[f][k]);
            let finger_tag = match (f, k) {
                (0, 0) => if side == Side::Left { "LeftIndexProx" } else { "RightIndexProx" },
                (0, 1) => if side == Side::Left { "LeftIndexMid" } else { "RightIndexMid" },
                (0, 2) => if side == Side::Left { "LeftIndexDist" } else { "RightIndexDist" },
                (1, 0) => if side == Side::Left { "LeftMiddleProx" } else { "RightMiddleProx" },
                (1, 1) => if side == Side::Left { "LeftMiddleMid" } else { "RightMiddleMid" },
                (1, 2) => if side == Side::Left { "LeftMiddleDist" } else { "RightMiddleDist" },
                (2, 0) => if side == Side::Left { "LeftRingProx" } else { "RightRingProx" },
                (2, 1) => if side == Side::Left { "LeftRingMid" } else { "RightRingMid" },
                (2, 2) => if side == Side::Left { "LeftRingDist" } else { "RightRingDist" },
                (3, 0) => if side == Side::Left { "LeftPinkyProx" } else { "RightPinkyProx" },
                (3, 1) => if side == Side::Left { "LeftPinkyMid" } else { "RightPinkyMid" },
                _ => if side == Side::Left { "LeftPinkyDist" } else { "RightPinkyDist" },
            };
            b.set_bone(finger_tag);
            b.part(format!("{p}{}{}", names[f], phal[k]), cur, next, radii[k].0, radii[k].1, out, FRONT);
            if k < 2 {
                let jn = format!("{p}{}{}", if k == 0 { "PIP" } else { "DIP" }, f + 2);
                let joint_r = if k == 0 { 0.0036 } else { 0.0030 };
                b.ball(next, joint_r, Tone::Plate);
                b.joint(&jn, JointKind::Hinge, next, Some(&prev), 0.002, Some(FRONT));
                prev = jn;
            }
            cur = next;
        }
    }

    // 5. Большой палец (I луч: опора в тенаре, оппозиция и суставы CMC1 / MCP1 / IP1)
    let cmc = rel(-0.005, -0.022, 0.014);
    let mcp1 = rel(-0.008, -0.042, 0.021);
    let dir_pp1 = norm([-s * 0.10, -0.82, 0.56]);
    let ip1 = madd(mcp1, dir_pp1, 0.025);
    let dir_dp1 = norm([-s * 0.12, -0.80, 0.59]);
    let tip1 = madd(ip1, dir_dp1, 0.018);

    let n0_thumb = norm([s * 0.70, 0.0, 0.70]);

    b.set_bone(if side == Side::Left { "LeftThumbMeta" } else { "RightThumbMeta" });
    b.part(format!("{p}ThumbMC"), cmc, mcp1, 0.0065, 0.0058, n0_thumb, UP);

    b.set_bone(if side == Side::Left { "LeftThumbProx" } else { "RightThumbProx" });
    b.ball(mcp1, 0.0050, Tone::Plate);
    b.part(format!("{p}ThumbPP"), mcp1, ip1, 0.0055, 0.0046, n0_thumb, UP);

    b.set_bone(if side == Side::Left { "LeftThumbDist" } else { "RightThumbDist" });
    b.ball(ip1, 0.0042, Tone::Plate);
    b.part(format!("{p}ThumbDP"), ip1, tip1, 0.0046, 0.0032, n0_thumb, UP);

    b.joint(&format!("{p}CMC1"), JointKind::Saddle, cmc, Some(&wrist_name), 0.003, None);
    b.joint(&format!("{p}MCP1"), JointKind::Hinge, mcp1, Some(&format!("{p}CMC1")), 0.0025, Some(FRONT));
    b.joint(&format!("{p}IP1"), JointKind::Hinge, ip1, Some(&format!("{p}MCP1")), 0.002, Some(FRONT));
}

pub fn foot_mesh(x_leg: f32, s: f32) -> (Vec<V3>, Vec<V3>, Vec<u32>) {
    const PTS: usize = 16;
    let mut v: Vec<V3> = vec![[x_leg, 0.018, -0.046 * ZS]];
    for &(z, y_bot, y_top, wl, wm) in &FOOT {
        let (z, wl, wm) = (z * ZS, wl * WS, wm * WS);
        let h = (y_top - y_bot).max(0.004);
        let rf = 0.004_f32.min(h * 0.25);
        for k in 0..PTS {
            let (dx, y) = match k {
                0 => (0.0, y_bot),
                1 => (wl - rf, y_bot),
                2 => (wl, y_bot + rf),
                3 => (wl, y_bot + h * 0.35),
                4 => (wl * 0.96, y_bot + h * 0.68),
                5 => (wl * 0.72, y_top - rf * 0.6),
                6 => (wl * 0.36, y_top),
                7 => (0.0, y_top),
                8 => (-wm * 0.36, y_top),
                9 => (-wm * 0.72, y_top - rf * 0.6),
                10 => (-wm * 0.96, y_bot + h * 0.68),
                11 => (-wm, y_bot + h * 0.35),
                12 => (-wm, y_bot + rf),
                13 => (-wm + rf, y_bot),
                14 => (-wm * 0.50, y_bot),
                _ => (wl * 0.50, y_bot),
            };
            v.push([x_leg + dx * s, y, z]);
        }
    }
    let front = v.len() as u32;
    v.push([x_leg, 0.004, 0.140 * ZS]);

    let mut idx: Vec<u32> = Vec::new();
    for k in 0..PTS {
        idx.extend([0, 1 + k as u32, 1 + ((k + 1) % PTS) as u32]);
    }
    for ring in 0..FOOT.len() - 1 {
        let r0 = 1 + (ring * PTS) as u32;
        let r1 = 1 + ((ring + 1) * PTS) as u32;
        for k in 0..PTS {
            let nk = ((k + 1) % PTS) as u32;
            let k = k as u32;
            idx.extend([r0 + k, r1 + k, r0 + nk, r0 + nk, r1 + k, r1 + nk]);
        }
    }
    let lr = 1 + ((FOOT.len() - 1) * PTS) as u32;
    for k in 0..PTS {
        idx.extend([front, lr + ((k + 1) % PTS) as u32, lr + k as u32]);
    }

    let (zmin, zmax) = (-0.036 * ZS, 0.132 * ZS);
    for t in idx.chunks_exact_mut(3) {
        let (a, bb, c) = (v[t[0] as usize], v[t[1] as usize], v[t[2] as usize]);
        let gn = cross(sub(bb, a), sub(c, a));
        let tc = scale(add(add(a, bb), c), 1.0 / 3.0);
        let axis_pt = [x_leg, 0.5 * foot_top(tc[2]), tc[2].clamp(zmin, zmax)];
        if dot(gn, sub(tc, axis_pt)) < 0.0 {
            t.swap(1, 2);
        }
    }
    let n = super::lofting::mesh_normals(&v, &idx);
    (v, n, idx)
}

pub fn leg(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let m = |v: V3| mirror(s, v);
    let out = [s, 0.0, 0.0];

    let thigh_tag = if side == Side::Left { "LeftThighProx" } else { "RightThighProx" };
    let shank_tag = if side == Side::Left { "LeftShankProx" } else { "RightShankProx" };
    let foot_tag = if side == Side::Left { "LeftFoot" } else { "RightFoot" };

    b.set_bone(thigh_tag);
    let hip = m([0.096, 0.905, -0.005]);
    let knee = m([0.082, 0.495, 0.000]);
    super::lofting::limb(
        b,
        Some(format!("{p}Femur")),
        m([0.096, 0.900, -0.005]),
        m([0.082, 0.505, 0.000]),
        &[(0.0, 0.057), (0.25, 0.060), (0.6, 0.048), (1.0, 0.037)],
        1.0,
        FRONT,
        out,
        true,
        false,
    );
    b.ball(knee, 0.040, Tone::Plate);

    b.set_bone(shank_tag);
    super::lofting::limb(
        b,
        Some(format!("{p}Tibia")),
        m([0.082, 0.480, 0.000]),
        m([0.082, 0.086, -0.004]),
        &[(0.0, 0.037), (0.2, 0.040), (0.55, 0.031), (1.0, 0.021)],
        1.0,
        FRONT,
        out,
        false,
        false,
    );
    let ankle = m([0.082, 0.070, -0.003]);
    b.ball(ankle, 0.024, Tone::Plate);

    b.set_bone(foot_tag);
    let (fv, fnm, fi) = foot_mesh(0.082 * s, s);
    b.prims.push(Prim::Mesh { vertices: fv, normals: fnm, indices: fi, tone: Tone::Cortical });
    b.prim_bones.push(b.current_bone);

    let (z1, z2) = (0.030 * ZS, 0.060 * ZS);
    b.seg_only(
        format!("{p}MT3"),
        m([0.082, 0.0, z1]),
        m([0.082, 0.0, z2]),
        vec![(0.0, foot_top(z1)), (1.0, foot_top(z2))],
        UP,
        out,
    );
    let (h1, h2) = (0.095 * ZS, 0.118 * ZS);
    let hx = 0.082 - 0.010 * WS;
    b.seg_only(
        format!("{p}HalluxPP"),
        m([hx, 0.0, h1]),
        m([hx, 0.0, h2]),
        vec![(0.0, foot_top(h1)), (1.0, foot_top(h2))],
        UP,
        out,
    );

    let jr = 0.005;
    b.joint(&format!("{p}Hip"), JointKind::Ball, hip, Some("Pelvis"), jr, None);
    b.joint(&format!("{p}Knee"), JointKind::Hinge, knee, Some(&format!("{p}Hip")), jr, Some(X));
    b.joint(&format!("{p}Ankle"), JointKind::Hinge, ankle, Some(&format!("{p}Knee")), 0.0045, Some(X));
    b.joint(&format!("{p}Subtalar"), JointKind::Gliding, m([0.082, 0.032, -0.010]), Some(&format!("{p}Ankle")), 0.0035, None);
    b.joint(&format!("{p}MTP1"), JointKind::Condyloid, m([hx, 0.015, 0.090 * ZS]), Some(&format!("{p}Subtalar")), 0.003, None);
}