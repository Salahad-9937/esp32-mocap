//! Упрощённая фигура-манекен (поза N, рост ≈ 1.75 м): гладкие сегменты-капсулы,
//! суставы и 64 места датчиков на поверхности сегментов.
//!
//! Правая сторона записана с положительным X, левая — зеркало. Единицы — метры.

use super::skeleton::{
    Anchor, Joint, JointKind, Prim, SensorDef, SensorSite, Side, Skeleton, Tone, SENSOR_DEFS,
    STANDOFF,
};
use super::vec3::{add, dot, lerp, madd, mirror, norm, scale, sub, V3};
use std::collections::HashMap;

/// Сегмент для привязки датчиков: ось a→b, радиусы и две опорные нормали
/// (угол вокруг оси отсчитывается от `n0` к `n90`).
#[derive(Clone, Copy)]
struct Seg {
    a: V3,
    b: V3,
    ra: f32,
    rb: f32,
    n0: V3,
    n90: V3,
}

struct Builder {
    joints: Vec<Joint>,
    jidx: HashMap<String, usize>,
    prims: Vec<Prim>,
    segs: HashMap<String, Seg>,
    /// Точки на спине для датчиков позвоночника.
    spinous: HashMap<String, V3>,
}

impl Builder {
    fn cap(&mut self, a: V3, b: V3, ra: f32, rb: f32, tone: Tone) {
        self.prims.push(Prim::Capsule { a, b, ra, rb, tone });
    }

    fn ball(&mut self, c: V3, r: f32, tone: Tone) {
        self.prims.push(Prim::Sphere { c, r, tone });
    }

    /// Капсула + регистрация сегмента для датчиков.
    fn part(&mut self, key: String, a: V3, b: V3, ra: f32, rb: f32, n0: V3, n90: V3) {
        self.cap(a, b, ra, rb, Tone::Cortical);
        self.segs.insert(key, Seg { a, b, ra, rb, n0, n90 });
    }

    fn seg_only(&mut self, key: String, a: V3, b: V3, ra: f32, rb: f32, n0: V3, n90: V3) {
        self.segs.insert(key, Seg { a, b, ra, rb, n0, n90 });
    }

    fn joint(&mut self, name: &str, kind: JointKind, pos: V3, parent: Option<&str>, r: f32, axis: Option<V3>) {
        let parent = parent.map(|p| {
            *self.jidx.get(p).unwrap_or_else(|| panic!("нет родительского сустава {p} для {name}"))
        });
        self.jidx.insert(name.to_string(), self.joints.len());
        self.joints.push(Joint { name: name.to_string(), kind, pos, parent, axis, marker_r: r });
    }
}

const UP: V3 = [0.0, 1.0, 0.0];
const FRONT: V3 = [0.0, 0.0, 1.0];
const X: V3 = [1.0, 0.0, 0.0];

// Туловище: горизонтальные «таблетки» (центр, полуширина по X, радиус).
const CHEST: (V3, f32, f32) = ([0.0, 1.36, -0.030], 0.075, 0.115);
const ABDOMEN: (V3, f32, f32) = ([0.0, 1.14, -0.030], 0.050, 0.095);
const PELVIS: (V3, f32, f32) = ([0.0, 0.96, -0.015], 0.060, 0.095);
const NECK_A: V3 = [0.0, 1.50, -0.025];
const NECK_B: V3 = [0.0, 1.60, -0.025];
const NECK_R: f32 = 0.045;
const HEAD_C: V3 = [0.0, 1.675, -0.005];
const FRONTAL_C: V3 = [0.0, 1.672, 0.028];
const FRONTAL_R: f32 = 0.062;

/// Точка на задней поверхности «таблетки» на высоте y.
fn back_pt(t: (V3, f32, f32), y: f32) -> V3 {
    let dy = y - t.0[1];
    [0.0, y, t.0[2] - (t.2 * t.2 - dy * dy).max(0.0).sqrt()]
}

fn front_pt(t: (V3, f32, f32), y: f32) -> V3 {
    let dy = y - t.0[1];
    [0.0, y, t.0[2] + (t.2 * t.2 - dy * dy).max(0.0).sqrt()]
}

fn torso(b: &mut Builder) {
    for (c, xe, r) in [CHEST, ABDOMEN, PELVIS] {
        b.cap([-xe, c[1], c[2]], [xe, c[1], c[2]], r, r, Tone::Cortical);
    }
    b.cap(NECK_A, NECK_B, NECK_R, NECK_R, Tone::Cortical);
    // голова-«яйцо»
    b.ball(HEAD_C, 0.080, Tone::Cortical);
    b.ball(FRONTAL_C, FRONTAL_R, Tone::Cortical);
    b.ball([0.0, 1.610, 0.030], 0.050, Tone::Cortical);

    for (name, y, t) in [("T3", 1.43, CHEST), ("T7", 1.36, CHEST), ("T11", 1.27, CHEST), ("L3", 1.09, ABDOMEN)] {
        b.spinous.insert(name.to_string(), back_pt(t, y));
    }
    b.spinous.insert("C5".to_string(), [0.0, 1.54, NECK_A[2] - NECK_R]);

    let jr = 0.0065;
    b.joint("L5S1", JointKind::Spinal, [0.0, 1.00, -0.030], Some("Pelvis"), jr, None);
    b.joint("L3L4", JointKind::Spinal, [0.0, 1.07, -0.030], Some("L5S1"), jr, None);
    b.joint("T12L1", JointKind::Spinal, [0.0, 1.18, -0.030], Some("L3L4"), jr, None);
    b.joint("T7T8", JointKind::Spinal, [0.0, 1.30, -0.030], Some("T12L1"), jr, None);
    b.joint("C7T1", JointKind::Spinal, [0.0, 1.47, -0.040], Some("T7T8"), jr, None);
    b.joint("C4C5", JointKind::Spinal, [0.0, 1.54, -0.030], Some("C7T1"), 0.005, None);
    b.joint("C1C2", JointKind::Pivot, [0.0, 1.59, -0.020], Some("C4C5"), 0.007, None);
    b.joint("Occiput", JointKind::Condyloid, [0.0, 1.612, -0.017], Some("C1C2"), 0.009, None);
}

fn shoulder_and_arm(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let m = |v: V3| mirror(s, v);
    let out = [s, 0.0, 0.0];

    // плечевой пояс: трапеция и дельта
    let (ca, cb, cc) = (m([0.040, 1.470, -0.020]), m([0.120, 1.465, -0.025]), m([0.190, 1.440, -0.035]));
    b.part(format!("{p}Clavicle"), ca, cb, 0.035, 0.038, UP, FRONT);
    b.part(format!("{p}Acromion"), cb, cc, 0.038, 0.045, UP, FRONT);
    let gh = m([0.175, 1.395, -0.035]);
    b.ball(gh, 0.050, Tone::Plate);

    // плечо, локоть, предплечье
    let (ha, hb) = (m([0.178, 1.385, -0.034]), m([0.195, 1.115, -0.030]));
    b.part(format!("{p}UpperArm"), ha, hb, 0.042, 0.034, FRONT, out);
    let elbow = m([0.195, 1.092, -0.030]);
    b.ball(elbow, 0.036, Tone::Plate);
    let wrist = m([0.204, 0.838, -0.008]);
    b.part(format!("{p}Ulna"), elbow, wrist, 0.034, 0.026, FRONT, out);
    b.seg_only(format!("{p}Radius"), elbow, wrist, 0.034, 0.026, FRONT, out);
    b.ball(wrist, 0.027, Tone::Plate);

    b.joint(&format!("{p}SC"), JointKind::Saddle, m([0.020, 1.432, 0.030]), Some("C7T1"), 0.008, None);
    b.joint(&format!("{p}GH"), JointKind::Ball, gh, Some(&format!("{p}SC")), 0.016, None);
    b.joint(&format!("{p}Elbow"), JointKind::Hinge, elbow, Some(&format!("{p}GH")), 0.013, Some(X));
    b.joint(&format!("{p}Radioulnar"), JointKind::Pivot, m([0.205, 1.082, -0.010]), Some(&format!("{p}Elbow")), 0.008, None);
}

const WRIST: V3 = [0.204, 0.838, -0.008];

fn hand(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    // dx — наружу, dy — вверх, dz — вперёд от центра запястья
    let rel = |dx: f32, dy: f32, dz: f32| mirror(s, [WRIST[0] + dx, WRIST[1] + dy, WRIST[2] + dz]);
    let out = [s, 0.0, 0.0];
    let wrist_name = format!("{p}Wrist");
    b.joint(&wrist_name, JointKind::Condyloid, mirror(s, WRIST), Some(&format!("{p}Radioulnar")), 0.012, None);

    // ладонь — одна капсула
    let (pa, pb) = (rel(-0.004, -0.030, -0.007), rel(-0.006, -0.056, -0.007));
    b.part(format!("{p}MC3"), pa, pb, 0.037, 0.033, out, FRONT);

    let names = ["Index", "Middle", "Ring", "Pinky"];
    let mcp_dx = [-0.004, -0.006, -0.008, -0.010];
    let mcp_dy = [-0.090, -0.093, -0.088, -0.074];
    let mcp_dz = [0.026, 0.003, -0.019, -0.040];
    let lens = [[0.040, 0.023, 0.018], [0.045, 0.028, 0.019], [0.042, 0.026, 0.019], [0.034, 0.018, 0.017]];
    let fan = [0.06_f32, 0.0, -0.05, -0.11];
    let theta = [6.0_f32, 28.0, 40.0];
    let radii = [(0.0085, 0.0075), (0.0075, 0.0068), (0.0068, 0.0055)];
    let phal = ["PP", "MP", "DP"];

    for f in 0..4 {
        let mcp = rel(mcp_dx[f], mcp_dy[f], mcp_dz[f]);
        let mcp_name = format!("{p}MCP{}", f + 2);
        b.joint(&mcp_name, JointKind::Condyloid, mcp, Some(&wrist_name), 0.0045, None);
        let mut prev = mcp_name;
        let mut cur = mcp;
        for k in 0..3 {
            let th = theta[k].to_radians();
            let dir = norm([-s * th.sin(), -th.cos(), fan[f]]);
            let next = madd(cur, dir, lens[f][k]);
            b.part(format!("{p}{}{}", names[f], phal[k]), cur, next, radii[k].0, radii[k].1, out, FRONT);
            if k < 2 {
                let jn = format!("{p}{}{}", if k == 0 { "PIP" } else { "DIP" }, f + 2);
                b.joint(&jn, JointKind::Hinge, next, Some(&prev), 0.0035, Some(FRONT));
                prev = jn;
            }
            cur = next;
        }
    }

    // большой палец
    let cmc = rel(-0.006, -0.024, 0.022);
    let mcp1 = rel(-0.016, -0.062, 0.040);
    let ip1 = rel(-0.026, -0.092, 0.054);
    let tip1 = rel(-0.034, -0.116, 0.066);
    let n0 = norm([s, 0.0, 0.5]);
    b.part(format!("{p}ThumbMC"), cmc, mcp1, 0.0120, 0.0100, n0, UP);
    b.part(format!("{p}ThumbPP"), mcp1, ip1, 0.0100, 0.0088, n0, UP);
    b.part(format!("{p}ThumbDP"), ip1, tip1, 0.0088, 0.0070, n0, UP);
    b.joint(&format!("{p}CMC1"), JointKind::Saddle, cmc, Some(&wrist_name), 0.0055, None);
    b.joint(&format!("{p}MCP1"), JointKind::Condyloid, mcp1, Some(&format!("{p}CMC1")), 0.0045, None);
    b.joint(&format!("{p}IP1"), JointKind::Hinge, ip1, Some(&format!("{p}MCP1")), 0.0035, Some(FRONT));
}

fn leg(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let m = |v: V3| mirror(s, v);
    let out = [s, 0.0, 0.0];

    let hip = m([0.090, 0.920, 0.000]);
    let knee = m([0.085, 0.495, 0.000]);
    let ankle = m([0.082, 0.082, 0.000]);
    b.ball(hip, 0.062, Tone::Cortical);
    b.part(format!("{p}Femur"), m([0.090, 0.880, 0.0]), m([0.086, 0.530, 0.0]), 0.062, 0.050, FRONT, out);
    b.ball(knee, 0.052, Tone::Plate);
    b.part(format!("{p}Tibia"), m([0.085, 0.470, 0.005]), m([0.082, 0.110, 0.0]), 0.050, 0.035, FRONT, out);
    b.ball(ankle, 0.036, Tone::Plate);

    // стопа: клин от пятки к носку + большой палец
    b.part(format!("{p}MT3"), m([0.082, 0.050, -0.030]), m([0.085, 0.035, 0.150]), 0.038, 0.030, UP, out);
    b.part(format!("{p}HalluxPP"), m([0.060, 0.030, 0.150]), m([0.057, 0.026, 0.195]), 0.017, 0.015, UP, out);

    b.joint(&format!("{p}Hip"), JointKind::Ball, hip, Some("Pelvis"), 0.016, None);
    b.joint(&format!("{p}Knee"), JointKind::Hinge, knee, Some(&format!("{p}Hip")), 0.015, Some(X));
    b.joint(&format!("{p}Ankle"), JointKind::Hinge, ankle, Some(&format!("{p}Knee")), 0.012, Some(X));
    b.joint(&format!("{p}Subtalar"), JointKind::Gliding, m([0.080, 0.050, -0.020]), Some(&format!("{p}Ankle")), 0.008, None);
    b.joint(&format!("{p}MTP1"), JointKind::Condyloid, m([0.060, 0.030, 0.150]), Some(&format!("{p}Subtalar")), 0.0065, None);
}

fn resolve(b: &Builder, id: usize, def: &SensorDef) -> SensorSite {
    let (pos, normal, tangent, support) = match def.anchor {
        Anchor::Seg(name, t, deg) => {
            let key = format!("{}{}", def.side.prefix(), name);
            let sg = b.segs.get(&key).unwrap_or_else(|| panic!("нет сегмента {key} для {}", def.tag));
            let axis = norm(sub(sg.b, sg.a));
            let n0 = norm(sub(sg.n0, scale(axis, dot(sg.n0, axis))));
            let n90 = sub(sg.n90, scale(axis, dot(sg.n90, axis)));
            let n90 = norm(sub(n90, scale(n0, dot(n90, n0))));
            let a = deg.to_radians();
            let dir = norm(add(scale(n0, a.cos()), scale(n90, a.sin())));
            let r = sg.ra + (sg.rb - sg.ra) * t;
            (madd(lerp(sg.a, sg.b, t), dir, r + STANDOFF), dir, axis, lerp(sg.a, sg.b, 0.5))
        }
        Anchor::Spinous(level) => {
            let tip = *b.spinous.get(level).unwrap_or_else(|| panic!("нет точки {level} для {}", def.tag));
            let n = [0.0, 0.0, -1.0];
            (madd(tip, n, STANDOFF), n, UP, madd(tip, n, -0.020))
        }
        Anchor::Forehead => {
            let n = norm([0.0, 0.35, 0.94]);
            let pos = madd(FRONTAL_C, n, FRONTAL_R + STANDOFF);
            (pos, n, X, madd(pos, n, -0.030))
        }
        Anchor::Sternum => {
            let n = FRONT;
            let pt = front_pt(CHEST, 1.37);
            (madd(pt, n, STANDOFF), n, UP, madd(pt, n, -0.020))
        }
        Anchor::Sacrum => {
            let n = [0.0, 0.0, -1.0];
            let pt = back_pt(PELVIS, 0.975);
            (madd(pt, n, STANDOFF), n, UP, madd(pt, n, -0.020))
        }
    };
    SensorSite { id, tag: def.tag, side: def.side, pos, normal, tangent, support, chip: def.chip }
}

/// Собирает всю фигуру.
pub fn build() -> Skeleton {
    let mut b = Builder {
        joints: Vec::new(),
        jidx: HashMap::new(),
        prims: Vec::new(),
        segs: HashMap::new(),
        spinous: HashMap::new(),
    };
    b.joint("Pelvis", JointKind::Root, [0.0, 0.935, -0.020], None, 0.018, None);
    torso(&mut b);
    for side in [Side::Right, Side::Left] {
        shoulder_and_arm(&mut b, side);
        hand(&mut b, side);
        leg(&mut b, side);
    }
    let sites = SENSOR_DEFS.iter().enumerate().map(|(i, d)| resolve(&b, i, d)).collect();
    Skeleton { joints: b.joints, prims: b.prims, sites }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::visuals::skeleton::{skeleton, SENSOR_COUNT, TARGET_BONES};
    use crate::visuals::vec3::dist;
    use std::collections::HashSet;

    #[test]
    fn exactly_64_unique_sensors() {
        let tags: HashSet<_> = TARGET_BONES.iter().collect();
        assert_eq!(tags.len(), SENSOR_COUNT);
        assert_eq!(skeleton().sites.len(), 64);
    }

    #[test]
    fn sensors_sit_on_surface() {
        let sk = skeleton();
        for s in &sk.sites {
            let mut best = f32::MAX;
            for pr in &sk.prims {
                let d = match *pr {
                    Prim::Capsule { a, b, ra, rb, .. } => {
                        let ab = sub(b, a);
                        let t = (dot(sub(s.pos, a), ab) / dot(ab, ab).max(1e-12)).clamp(0.0, 1.0);
                        dist(s.pos, madd(a, ab, t)) - (ra + (rb - ra) * t)
                    }
                    Prim::Sphere { c, r, .. } => dist(s.pos, c) - r,
                    Prim::Tri { .. } => f32::MAX,
                };
                best = best.min(d);
            }
            assert!((-0.003..=0.010).contains(&best), "{} (id {}) — {best:.4} м от поверхности", s.tag, s.id);
        }
    }

    #[test]
    fn sides_and_tree() {
        let sk = skeleton();
        for s in &sk.sites {
            if s.tag.starts_with("Left") { assert!(s.pos[0] < 0.0, "{}", s.tag); }
            if s.tag.starts_with("Right") { assert!(s.pos[0] > 0.0, "{}", s.tag); }
        }
        for (i, j) in sk.joints.iter().enumerate() {
            if let Some(p) = j.parent { assert!(p < i); }
        }
    }
}