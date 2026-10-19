//! Анатомическая модель манекена: параметрические сегменты тела
//! с естественными пропорциями, зауженной талией, V-образным торсом
//! и изолированными шарнирными сочленениями для 64 мест установки датчиков.

use super::skeleton::{
    Anchor, Joint, JointKind, Prim, SensorDef, SensorSite, Side, Skeleton, Tone, SENSOR_DEFS,
    STANDOFF,
};
use super::vec3::{add, dot, lerp, madd, mirror, norm, scale, sub, V3};
use std::collections::HashMap;

/// Сегмент тела для привязки датчиков на внешней поверхности.
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
    spinous: HashMap<String, V3>,
}

impl Builder {
    fn cap(&mut self, a: V3, b: V3, ra: f32, rb: f32, tone: Tone) {
        self.prims.push(Prim::Capsule { a, b, ra, rb, tone });
    }

    fn ball(&mut self, c: V3, r: f32, tone: Tone) {
        self.prims.push(Prim::Sphere { c, r, tone });
    }

    fn part(&mut self, key: String, a: V3, b: V3, ra: f32, rb: f32, n0: V3, n90: V3) {
        self.cap(a, b, ra, rb, Tone::Cortical);
        self.segs.insert(key, Seg { a, b, ra, rb, n0, n90 });
    }

    fn seg_only(&mut self, key: String, a: V3, b: V3, ra: f32, rb: f32, n0: V3, n90: V3) {
        self.segs.insert(key, Seg { a, b, ra, rb, n0, n90 });
    }

    fn joint(&mut self, name: &str, kind: JointKind, pos: V3, parent: Option<&str>, r: f32, axis: Option<V3>) {
        let parent = parent.map(|p| {
            *self.jidx.get(p).unwrap_or_else(|| panic!("отсутствует родительский сустав {p} для {name}"))
        });
        self.jidx.insert(name.to_string(), self.joints.len());
        self.joints.push(Joint { name: name.to_string(), kind, pos, parent, axis, marker_r: r });
    }
}

const UP: V3 = [0.0, 1.0, 0.0];
const FRONT: V3 = [0.0, 0.0, 1.0];
const X: V3 = [1.0, 0.0, 0.0];

fn torso(b: &mut Builder) {
    // 1. Грудной блок (Pectoral/Ribcage): цельный V-образный щит, сужающийся книзу
    b.cap([0.0, 1.44, -0.015], [0.0, 1.27, -0.015], 0.105, 0.080, Tone::Cortical);

    // 2. Брюшной пояс (Waist): отчетливо зауженная талия с зазором сочленения
    b.cap([0.0, 1.25, -0.015], [0.0, 1.11, -0.015], 0.076, 0.072, Tone::Cortical);

    // 3. Тазовый блок (Pelvic girdle): анатомическая чаша
    b.cap([0.0, 1.09, -0.015], [0.0, 0.93, -0.010], 0.082, 0.064, Tone::Cortical);

    // 4. Шея: конический сегмент с естественным наклоном
    b.cap([0.0, 1.47, -0.010], [0.0, 1.57, -0.005], 0.038, 0.032, Tone::Cortical);

    // 5. Голова: сферический череп и подбородок
    b.ball([0.0, 1.66, -0.005], 0.065, Tone::Cortical);
    b.cap([0.0, 1.63, 0.015], [0.0, 1.57, 0.022], 0.042, 0.026, Tone::Cortical);

    // Точки крепления датчиков позвоночника
    b.spinous.insert("T3".to_string(), [0.0, 1.40, -0.095]);
    b.spinous.insert("T7".to_string(), [0.0, 1.32, -0.090]);
    b.spinous.insert("T11".to_string(), [0.0, 1.24, -0.082]);
    b.spinous.insert("L3".to_string(), [0.0, 1.12, -0.078]);
    b.spinous.insert("C5".to_string(), [0.0, 1.53, -0.052]);

    let jr = 0.004;
    b.joint("L5S1", JointKind::Spinal, [0.0, 0.98, -0.015], Some("Pelvis"), jr, None);
    b.joint("L3L4", JointKind::Spinal, [0.0, 1.10, -0.015], Some("L5S1"), jr, None);
    b.joint("T12L1", JointKind::Spinal, [0.0, 1.20, -0.015], Some("L3L4"), jr, None);
    b.joint("T7T8", JointKind::Spinal, [0.0, 1.32, -0.015], Some("T12L1"), jr, None);
    b.joint("C7T1", JointKind::Spinal, [0.0, 1.46, -0.015], Some("T7T8"), jr, None);
    b.joint("C4C5", JointKind::Spinal, [0.0, 1.53, -0.010], Some("C7T1"), 0.0035, None);
    b.joint("C1C2", JointKind::Pivot, [0.0, 1.59, -0.005], Some("C4C5"), 0.004, None);
    b.joint("Occiput", JointKind::Condyloid, [0.0, 1.62, -0.005], Some("C1C2"), 0.005, None);
}

fn shoulder_and_arm(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let m = |v: V3| mirror(s, v);
    let out = [s, 0.0, 0.0];

    // Ключичная зона для датчиков (виртуальный сегмент, не создающий меша-пересечения)
    let c_start = m([0.035, 1.44, 0.000]);
    let c_end = m([0.165, 1.40, -0.010]);
    b.seg_only(format!("{p}Clavicle"), c_start, c_end, 0.030, 0.032, UP, FRONT);
    b.seg_only(format!("{p}Acromion"), c_start, c_end, 0.030, 0.035, UP, FRONT);

    // Плечевой сустав: изолированный шарнирный шар в боковом кармане груди
    let gh = m([0.175, 1.39, -0.015]);
    b.ball(gh, 0.035, Tone::Plate);

    // Плечо (humerus): плавный анатомический конус
    let elbow = m([0.198, 1.13, -0.015]);
    b.part(format!("{p}UpperArm"), gh, elbow, 0.036, 0.028, FRONT, out);

    // Локтевой шарнир
    let elbow_joint = m([0.198, 1.11, -0.015]);
    b.ball(elbow_joint, 0.025, Tone::Plate);

    // Предплечье: мышечный объем с сужением к запястью
    let wrist = m([0.218, 0.86, -0.008]);
    b.part(format!("{p}Ulna"), elbow_joint, wrist, 0.028, 0.020, FRONT, out);
    b.seg_only(format!("{p}Radius"), elbow_joint, wrist, 0.028, 0.020, FRONT, out);

    // Лучезапястный шарнир
    let wrist_joint = m([0.218, 0.84, -0.008]);
    b.ball(wrist_joint, 0.018, Tone::Plate);

    let jr = 0.005;
    b.joint(&format!("{p}SC"), JointKind::Saddle, m([0.035, 1.44, 0.01]), Some("C7T1"), jr, None);
    b.joint(&format!("{p}GH"), JointKind::Ball, gh, Some(&format!("{p}SC")), 0.008, None);
    b.joint(&format!("{p}Elbow"), JointKind::Hinge, elbow_joint, Some(&format!("{p}GH")), 0.007, Some(X));
    b.joint(&format!("{p}Radioulnar"), JointKind::Pivot, m([0.205, 1.10, -0.01]), Some(&format!("{p}Elbow")), 0.005, None);
}

const WRIST: V3 = [0.218, 0.84, -0.008];

fn hand(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let rel = |dx: f32, dy: f32, dz: f32| mirror(s, [WRIST[0] + dx, WRIST[1] + dy, WRIST[2] + dz]);
    let out = [s, 0.0, 0.0];
    let wrist_name = format!("{p}Wrist");
    b.joint(&wrist_name, JointKind::Condyloid, mirror(s, WRIST), Some(&format!("{p}Radioulnar")), 0.006, None);

    // Ладонь
    let pa = rel(-0.003, -0.012, -0.004);
    let pb = rel(-0.005, -0.038, -0.004);
    b.part(format!("{p}MC3"), pa, pb, 0.022, 0.018, out, FRONT);

    let names = ["Index", "Middle", "Ring", "Pinky"];
    let mcp_dx = [-0.003, -0.005, -0.007, -0.009];
    let mcp_dy = [-0.058, -0.061, -0.058, -0.052];
    let mcp_dz = [0.016, 0.002, -0.012, -0.024];
    let lens = [[0.026, 0.016, 0.012], [0.030, 0.019, 0.013], [0.027, 0.017, 0.012], [0.022, 0.013, 0.010]];
    let fan = [0.04_f32, 0.0, -0.03, -0.08];
    let theta = [5.0_f32, 20.0, 30.0];
    let radii = [(0.0055, 0.0048), (0.0048, 0.0040), (0.0040, 0.0034)];
    let phal = ["PP", "MP", "DP"];

    for f in 0..4 {
        let mcp = rel(mcp_dx[f], mcp_dy[f], mcp_dz[f]);
        let mcp_name = format!("{p}MCP{}", f + 2);
        b.joint(&mcp_name, JointKind::Condyloid, mcp, Some(&wrist_name), 0.003, None);
        let mut prev = mcp_name;
        let mut cur = mcp;
        for k in 0..3 {
            let th = theta[k].to_radians();
            let dir = norm([-s * th.sin(), -th.cos(), fan[f]]);
            let next = madd(cur, dir, lens[f][k]);
            b.part(format!("{p}{}{}", names[f], phal[k]), cur, next, radii[k].0, radii[k].1, out, FRONT);
            if k < 2 {
                let jn = format!("{p}{}{}", if k == 0 { "PIP" } else { "DIP" }, f + 2);
                b.joint(&jn, JointKind::Hinge, next, Some(&prev), 0.0022, Some(FRONT));
                prev = jn;
            }
            cur = next;
        }
    }

    // Большой палец
    let cmc = rel(-0.004, -0.012, 0.014);
    let mcp1 = rel(-0.010, -0.036, 0.024);
    let ip1 = rel(-0.015, -0.054, 0.030);
    let tip1 = rel(-0.020, -0.068, 0.036);
    let n0 = norm([s, 0.0, 0.4]);
    b.part(format!("{p}ThumbMC"), cmc, mcp1, 0.0075, 0.0062, n0, UP);
    b.part(format!("{p}ThumbPP"), mcp1, ip1, 0.0062, 0.0054, n0, UP);
    b.part(format!("{p}ThumbDP"), ip1, tip1, 0.0054, 0.0044, n0, UP);
    b.joint(&format!("{p}CMC1"), JointKind::Saddle, cmc, Some(&wrist_name), 0.003, None);
    b.joint(&format!("{p}MCP1"), JointKind::Condyloid, mcp1, Some(&format!("{p}CMC1")), 0.0025, None);
    b.joint(&format!("{p}IP1"), JointKind::Hinge, ip1, Some(&format!("{p}MCP1")), 0.002, Some(FRONT));
}

fn leg(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let m = |v: V3| mirror(s, v);
    let out = [s, 0.0, 0.0];

    // Тазобедренный шарнир
    let hip = m([0.100, 0.90, -0.005]);
    b.ball(hip, 0.036, Tone::Plate);

    // Бедро: плавный анатомический конус
    let knee = m([0.100, 0.52, 0.000]);
    b.part(format!("{p}Femur"), hip, knee, 0.052, 0.038, FRONT, out);

    // Коленный шарнир
    let knee_joint = m([0.100, 0.49, 0.000]);
    b.ball(knee_joint, 0.030, Tone::Plate);

    // Голень: выраженное утолщение икры и сужение к щиколотке
    let ankle = m([0.100, 0.12, 0.000]);
    b.part(format!("{p}Tibia"), knee_joint, ankle, 0.038, 0.025, FRONT, out);

    // Голеностопный шарнир
    let ankle_joint = m([0.100, 0.09, 0.000]);
    b.ball(ankle_joint, 0.022, Tone::Plate);

    // Стопа
    b.part(format!("{p}MT3"), m([0.100, 0.060, -0.025]), m([0.100, 0.035, 0.130]), 0.026, 0.020, UP, out);
    b.part(format!("{p}HalluxPP"), m([0.082, 0.030, 0.130]), m([0.080, 0.022, 0.170]), 0.015, 0.013, UP, out);

    let jr = 0.005;
    b.joint(&format!("{p}Hip"), JointKind::Ball, hip, Some("Pelvis"), jr, None);
    b.joint(&format!("{p}Knee"), JointKind::Hinge, knee_joint, Some(&format!("{p}Hip")), jr, Some(X));
    b.joint(&format!("{p}Ankle"), JointKind::Hinge, ankle_joint, Some(&format!("{p}Knee")), 0.0045, Some(X));
    b.joint(&format!("{p}Subtalar"), JointKind::Gliding, m([0.100, 0.050, -0.018]), Some(&format!("{p}Ankle")), 0.0035, None);
    b.joint(&format!("{p}MTP1"), JointKind::Condyloid, m([0.082, 0.030, 0.130]), Some(&format!("{p}Subtalar")), 0.003, None);
}

fn resolve(b: &Builder, id: usize, def: &SensorDef) -> SensorSite {
    let (pos, normal, tangent, support) = match def.anchor {
        Anchor::Seg(name, t, deg) => {
            let key = format!("{}{}", def.side.prefix(), name);
            let sg = b.segs.get(&key).unwrap_or_else(|| panic!("отсутствует сегмент {key} для {}", def.tag));
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
            let tip = *b.spinous.get(level).unwrap_or_else(|| panic!("отсутствует точка {level} для {}", def.tag));
            let n = [0.0, 0.0, -1.0];
            (madd(tip, n, STANDOFF), n, UP, madd(tip, n, -0.020))
        }
        Anchor::Forehead => {
            let n = norm([0.0, 0.28, 0.96]);
            let pos = madd([0.0, 1.66, 0.025], n, 0.045 + STANDOFF);
            (pos, n, X, madd(pos, n, -0.025))
        }
        Anchor::Sternum => {
            let n = FRONT;
            let pt = [0.0, 1.34, 0.082];
            (madd(pt, n, STANDOFF), n, UP, madd(pt, n, -0.020))
        }
        Anchor::Sacrum => {
            let n = [0.0, 0.0, -1.0];
            let pt = [0.0, 0.95, -0.076];
            (madd(pt, n, STANDOFF), n, UP, madd(pt, n, -0.020))
        }
    };
    SensorSite { id, tag: def.tag, side: def.side, pos, normal, tangent, support, chip: def.chip }
}

/// Собирает кинематическую фигуру манекена.
#[must_use]
pub fn build() -> Skeleton {
    let mut b = Builder {
        joints: Vec::new(),
        jidx: HashMap::new(),
        prims: Vec::new(),
        segs: HashMap::new(),
        spinous: HashMap::new(),
    };
    b.joint("Pelvis", JointKind::Root, [0.0, 0.94, -0.015], None, 0.007, None);
    torso(&mut b);
    for side in [Side::Right, Side::Left] {
        shoulder_and_arm(&mut b, side);
        hand(&mut b, side);
        leg(&mut b, side);
    }
    let sites = SENSOR_DEFS.iter().enumerate().map(|(i, d)| resolve(&b, i, d)).collect();
    Skeleton { joints: b.joints, prims: b.prims, sites }
}