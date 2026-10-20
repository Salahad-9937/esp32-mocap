//! Манекен (стиль художественного манекена / Figma): гладкие лофтинговые меши
//! с овальными сечениями — грудь, живот, таз, шея, голова, конечности, — шаровые
//! шарниры на плечах, локтях, запястьях, коленях и лодыжках, стопа-клин, кисть с
//! отдельными пальцами (на каждой фаланге стоит свой датчик).
//!
//! Система координат: Y вверх, Z вперёд (лицо в +Z), X вправо от человека
//! (левая сторона — отрицательный X). Правая сторона записана с положительным X,
//! левая — зеркало. Единицы — метры, рост ≈ 1.75 м.

use super::skeleton::{
    Anchor, Joint, JointKind, Prim, SensorDef, SensorSite, Side, Skeleton, Tone, SENSOR_DEFS,
    STANDOFF,
};
use super::vec3::{add, cross, dot, lerp, madd, mirror, norm, scale, sub, V3};
use std::collections::HashMap;
use std::f32::consts::TAU;

const UP: V3 = [0.0, 1.0, 0.0];
const FRONT: V3 = [0.0, 0.0, 1.0];
const X: V3 = [1.0, 0.0, 0.0];

/// Сегмент для привязки датчиков: ось a→b и профиль радиуса (t, r) вдоль оси.
/// Угол вокруг оси отсчитывается от `n0` (0°) к `n90` (+90°).
#[derive(Clone)]
struct Seg {
    a: V3,
    b: V3,
    prof: Vec<(f32, f32)>,
    n0: V3,
    n90: V3,
}

/// Кубическая интерполяция Катмулла–Рома по массиву значений: сегмент `i`, доля `f`.
fn cr(p: &[f32], i: usize, f: f32) -> f32 {
    let g = |k: isize| p[k.clamp(0, p.len() as isize - 1) as usize];
    let (p0, p1, p2, p3) = (g(i as isize - 1), g(i as isize), g(i as isize + 1), g(i as isize + 2));
    0.5 * (2.0 * p1
        + (p2 - p0) * f
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * f * f
        + (3.0 * p1 - p0 - 3.0 * p2 + p3) * f * f * f)
}

/// Гладкий радиус вдоль оси по профилю (t, r); тот же закон использует и меш.
fn prof_r(prof: &[(f32, f32)], t: f32) -> f32 {
    if t <= prof[0].0 {
        return prof[0].1;
    }
    let rs: Vec<f32> = prof.iter().map(|p| p.1).collect();
    for i in 0..prof.len() - 1 {
        if t <= prof[i + 1].0 {
            let f = (t - prof[i].0) / (prof[i + 1].0 - prof[i].0).max(1e-6);
            return cr(&rs, i, f);
        }
    }
    prof[prof.len() - 1].1
}

/// Кусочно-линейная интерполяция (для профиля стопы).
fn lin(p: &[(f32, f32)], t: f32) -> f32 {
    if t <= p[0].0 {
        return p[0].1;
    }
    for w in p.windows(2) {
        if t <= w[1].0 {
            return w[0].1 + (w[1].1 - w[0].1) * (t - w[0].0) / (w[1].0 - w[0].0).max(1e-6);
        }
    }
    p[p.len() - 1].1
}

struct Builder {
    joints: Vec<Joint>,
    jidx: HashMap<String, usize>,
    prims: Vec<Prim>,
    segs: HashMap<String, Seg>,
    /// Точки на теле для датчиков туловища: имя → (точка на поверхности, нормаль).
    anchors: HashMap<String, (V3, V3)>,
}

impl Builder {
    fn cap(&mut self, a: V3, b: V3, ra: f32, rb: f32, tone: Tone) {
        self.prims.push(Prim::Capsule { a, b, ra, rb, tone });
    }

    fn ball(&mut self, c: V3, r: f32, tone: Tone) {
        self.prims.push(Prim::Sphere { c, r, tone });
    }

    /// Капсула + сегмент для датчиков (пальцы, трапеция).
    fn part(&mut self, key: String, a: V3, b: V3, ra: f32, rb: f32, n0: V3, n90: V3) {
        self.cap(a, b, ra, rb, Tone::Cortical);
        self.segs.insert(key, Seg { a, b, prof: vec![(0.0, ra), (1.0, rb)], n0, n90 });
    }

    fn seg_only(&mut self, key: String, a: V3, b: V3, prof: Vec<(f32, f32)>, n0: V3, n90: V3) {
        self.segs.insert(key, Seg { a, b, prof, n0, n90 });
    }

    fn joint(&mut self, name: &str, kind: JointKind, pos: V3, parent: Option<&str>, r: f32, axis: Option<V3>) {
        let parent = parent.map(|p| {
            *self.jidx.get(p).unwrap_or_else(|| panic!("нет родительского сустава {p} для {name}"))
        });
        self.jidx.insert(name.to_string(), self.joints.len());
        self.joints.push(Joint { name: name.to_string(), kind, pos, parent, axis, marker_r: r });
    }
}

// ───────────────────────────── Лофтинг ─────────────────────────────

const RING_N: usize = 24; // кратно 4: есть вершины ровно спереди/сзади/сбоку

/// Горизонтальное сечение: центр, полуоси по X и Z, показатель суперэллипса (2 — эллипс).
#[derive(Clone, Copy)]
struct Ring {
    c: V3,
    rx: f32,
    rz: f32,
    ex: f32,
}

fn ring_pts(r: &Ring, k: f32, dy: f32) -> Vec<V3> {
    (0..RING_N)
        .map(|i| {
            let ang = TAU * i as f32 / RING_N as f32;
            let (sa, ca) = ang.sin_cos();
            let e = 2.0 / r.ex;
            let x = r.rx * k * ca.signum() * ca.abs().powf(e);
            let z = r.rz * k * sa.signum() * sa.abs().powf(e);
            [r.c[0] + x, r.c[1] + dy, r.c[2] + z]
        })
        .collect()
}

/// Подразделение сечений: промежуточные радиусы по Катмуллу–Рому (центры — линейно).
fn refine(rings: &[Ring]) -> Vec<Ring> {
    const SUB: usize = 3;
    if rings.len() < 2 {
        return rings.to_vec();
    }
    let rx: Vec<f32> = rings.iter().map(|q| q.rx).collect();
    let rz: Vec<f32> = rings.iter().map(|q| q.rz).collect();
    let mut out = Vec::new();
    for i in 0..rings.len() - 1 {
        for j in 0..SUB {
            let f = j as f32 / SUB as f32;
            out.push(Ring {
                c: lerp(rings[i].c, rings[i + 1].c, f),
                rx: cr(&rx, i, f),
                rz: cr(&rz, i, f),
                ex: rings[i].ex + (rings[i + 1].ex - rings[i].ex) * f,
            });
        }
    }
    out.push(rings[rings.len() - 1]);
    out
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

/// Гладкий меш по сечениям (снизу вверх). `bot`/`top` — высота купола на конце,
/// `None` — плоская торцевая крышка. Порядок вершин: нормаль = cross(v1−v0, v2−v0) наружу.
fn loft(b: &mut Builder, rings: &[Ring], bot: Option<f32>, top: Option<f32>, tone: Tone) {
    let refined = refine(rings);
    let rings = &refined[..];
    let n = RING_N;
    let mut layers: Vec<Vec<V3>> = Vec::new();
    let angs = [75.0_f32, 50.0, 25.0];
    if let Some(h) = bot {
        for th in angs {
            let (s, c) = th.to_radians().sin_cos();
            layers.push(ring_pts(&rings[0], c, -h * s));
        }
    }
    for r in rings {
        layers.push(ring_pts(r, 1.0, 0.0));
    }
    if let Some(h) = top {
        let last = &rings[rings.len() - 1];
        for th in angs.iter().rev() {
            let (s, c) = th.to_radians().sin_cos();
            layers.push(ring_pts(last, c, h * s));
        }
    }
    let l = layers.len();
    let first = &rings[0];
    let last = &rings[rings.len() - 1];
    let mut verts: Vec<V3> = layers.into_iter().flatten().collect();
    let apex_bot = verts.len() as u32;
    verts.push(match bot {
        Some(h) => [first.c[0], first.c[1] - h, first.c[2]],
        None => first.c,
    });
    let apex_top = verts.len() as u32;
    verts.push(match top {
        Some(h) => [last.c[0], last.c[1] + h, last.c[2]],
        None => last.c,
    });

    let mut idx: Vec<u32> = Vec::new();
    for i in 0..l - 1 {
        for k in 0..n {
            let nk = (k + 1) % n;
            let a0 = (i * n + k) as u32;
            let a1 = (i * n + nk) as u32;
            let b0 = ((i + 1) * n + k) as u32;
            let b1 = ((i + 1) * n + nk) as u32;
            idx.extend([a0, b0, a1, a1, b0, b1]);
        }
    }
    for k in 0..n {
        let nk = (k + 1) % n;
        idx.extend([k as u32, nk as u32, apex_bot]);
        idx.extend([((l - 1) * n + k) as u32, apex_top, ((l - 1) * n + nk) as u32]);
    }
    let normals = mesh_normals(&verts, &idx);
    b.prims.push(Prim::Mesh { vertices: verts, normals, indices: idx, tone });
}

/// Конечность — круглое (или сплюснутое: `aspect` = rz/rx) тело вращения вдоль a→b
/// с профилем радиуса. Регистрирует сегмент для датчиков, если задан `key`.
#[allow(clippy::too_many_arguments)]
fn limb(
    b: &mut Builder,
    key: Option<String>,
    a: V3,
    e: V3,
    prof: &[(f32, f32)],
    aspect: f32,
    n0: V3,
    n90: V3,
    round_a: bool,
    round_e: bool,
) {
    let mut rings: Vec<Ring> = prof
        .iter()
        .map(|&(t, r)| Ring { c: lerp(a, e, t), rx: r, rz: r * aspect, ex: 2.0 })
        .collect();
    let (mut bot_round, mut top_round) = (round_a, round_e);
    if a[1] > e[1] {
        rings.reverse();
        std::mem::swap(&mut bot_round, &mut top_round);
    }
    let dome = |r: &Ring| 0.55 * r.rx.min(r.rz);
    let bot = bot_round.then(|| dome(&rings[0]));
    let top = top_round.then(|| dome(&rings[rings.len() - 1]));
    loft(b, &rings, bot, top, Tone::Cortical);
    if let Some(k) = key {
        b.segs.insert(k, Seg { a, b: e, prof: prof.to_vec(), n0, n90 });
    }
}

/// (cz, rx, rz) сечения на высоте y (линейная интерполяция, как у меша).
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

/// Точка на передней/задней поверхности по оси симметрии и наружная нормаль.
fn surf_pt(rings: &[Ring], y: f32, front: bool) -> (V3, V3) {
    let z_at = |yy: f32| {
        let (cz, _, rz) = ring_at(rings, yy);
        if front { cz + rz } else { cz - rz }
    };
    let dz = z_at(y + 0.01) - z_at(y - 0.01);
    let n = if front { norm([0.0, -dz, 0.02]) } else { norm([0.0, dz, -0.02]) };
    ([0.0, y, z_at(y)], n)
}

fn r(y: f32, cz: f32, rx: f32, rz: f32, ex: f32) -> Ring {
    Ring { c: [0.0, y, cz], rx, rz, ex }
}

// ───────────────────────────── Туловище, шея, голова ─────────────────────────────

fn torso(b: &mut Builder) {
    // таз: широкие бёдра, закруглённый низ
    let pelvis = [
        r(0.860, -0.010, 0.070, 0.070, 2.4),
        r(0.890, -0.010, 0.112, 0.088, 2.5),
        r(0.950, -0.010, 0.140, 0.098, 2.5),
        r(1.010, -0.010, 0.142, 0.098, 2.5),
        r(1.060, -0.010, 0.126, 0.090, 2.4),
        r(1.095, -0.012, 0.114, 0.084, 2.3),
    ];
    loft(b, &pelvis, Some(0.040), None, Tone::Cortical);

    // живот (талия), верх чуть уже груди — получается шов, как у манекена
    let abdomen = [
        r(1.080, -0.012, 0.100, 0.080, 2.3),
        r(1.150, -0.012, 0.094, 0.074, 2.3),
        r(1.200, -0.012, 0.098, 0.076, 2.3),
        r(1.258, -0.012, 0.106, 0.082, 2.3),
    ];
    loft(b, &abdomen, None, None, Tone::Cortical);

    // грудь: широкие плечи, сужение к талии
    let chest = [
        r(1.255, -0.010, 0.118, 0.088, 2.6),
        r(1.300, -0.010, 0.140, 0.096, 2.6),
        r(1.350, -0.010, 0.150, 0.100, 2.6),
        r(1.400, -0.010, 0.145, 0.094, 2.5),
        r(1.440, -0.010, 0.120, 0.085, 2.4),
        r(1.462, -0.010, 0.075, 0.060, 2.2),
    ];
    loft(b, &chest, None, Some(0.025), Tone::Cortical);

    // шея: короткая и плотная
    let neck = [
        r(1.440, -0.010, 0.050, 0.050, 2.0),
        r(1.500, -0.006, 0.043, 0.043, 2.0),
        r(1.545, 0.000, 0.038, 0.038, 2.0),
        r(1.585, 0.012, 0.035, 0.035, 2.0),
        r(1.620, 0.018, 0.034, 0.034, 2.0),
    ];
    loft(b, &neck, None, None, Tone::Cortical);

    // голова-«яйцо» (чуть крупнее реальной — как у манекена)
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

    // точки датчиков туловища — ровно на поверхности мешей
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

// ───────────────────────────── Плечо и рука ─────────────────────────────

fn shoulder_and_arm(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let m = |v: V3| mirror(s, v);
    let out = [s, 0.0, 0.0];

    // трапеция: скат от шеи к плечу (на ней датчики ключицы и акромиона)
    let (ta, tb) = (m([0.045, 1.428, -0.012]), m([0.172, 1.395, -0.015]));
    b.part(format!("{p}Clavicle"), ta, tb, 0.034, 0.040, UP, FRONT);
    b.seg_only(format!("{p}Acromion"), ta, tb, vec![(0.0, 0.034), (1.0, 0.040)], UP, FRONT);

    // плечевой шарнир (дельта)
    let gh = m([0.178, 1.390, -0.012]);
    b.ball(gh, 0.046, Tone::Plate);

    // плечо
    let elbow = m([0.200, 1.108, -0.012]);
    limb(
        b,
        Some(format!("{p}UpperArm")),
        gh,
        elbow,
        &[(0.0, 0.040), (0.15, 0.041), (0.5, 0.034), (1.0, 0.027)],
        1.0,
        FRONT,
        out,
        true,
        false,
    );
    b.ball(elbow, 0.030, Tone::Plate);

    // предплечье
    let wrist = m([0.216, 0.858, -0.006]);
    let fprof = [(0.0, 0.029), (0.2, 0.030), (0.6, 0.024), (1.0, 0.018)];
    limb(b, Some(format!("{p}Ulna")), elbow, wrist, &fprof, 1.0, FRONT, out, false, false);
    b.seg_only(format!("{p}Radius"), elbow, wrist, fprof.to_vec(), FRONT, out);
    let wrist_joint = m(WRIST);
    b.ball(wrist_joint, 0.021, Tone::Plate);

    b.joint(&format!("{p}SC"), JointKind::Saddle, m([0.035, 1.44, 0.01]), Some("C7T1"), 0.005, None);
    b.joint(&format!("{p}GH"), JointKind::Ball, gh, Some(&format!("{p}SC")), 0.008, None);
    b.joint(&format!("{p}Elbow"), JointKind::Hinge, elbow, Some(&format!("{p}GH")), 0.007, Some(X));
    b.joint(&format!("{p}Radioulnar"), JointKind::Pivot, m([0.206, 1.08, -0.01]), Some(&format!("{p}Elbow")), 0.005, None);
}

// ───────────────────────────── Кисть ─────────────────────────────

const WRIST: V3 = [0.216, 0.845, -0.006];
/// Масштаб кисти относительно базовых размеров (кисть манекена крупнее «игрушечной»).
const HS: f32 = 1.3;

fn hand(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let rel = |dx: f32, dy: f32, dz: f32| mirror(s, [WRIST[0] + dx * HS, WRIST[1] + dy * HS, WRIST[2] + dz * HS]);
    let out = [s, 0.0, 0.0];
    let wrist_name = format!("{p}Wrist");
    b.joint(&wrist_name, JointKind::Condyloid, mirror(s, WRIST), Some(&format!("{p}Radioulnar")), 0.006, None);

    // ладонь: узкая у запястья, шире у костяшек, закруглена с обоих концов
    let (pa, pe) = (rel(-0.003, -0.010, -0.004), rel(-0.005, -0.056, -0.004));
    let pp = [(0.0_f32, 0.0125_f32, 0.020_f32), (0.35, 0.0140, 0.032), (1.0, 0.0120, 0.036)];
    let mut rings: Vec<Ring> = pp
        .iter()
        .map(|&(t, rx, rz)| Ring { c: lerp(pa, pe, t), rx, rz, ex: 2.0 })
        .collect();
    rings.reverse();
    loft(b, &rings, Some(0.0075), Some(0.010), Tone::Cortical);
    b.seg_only(format!("{p}MC3"), pa, pe, pp.iter().map(|&(t, rx, _)| (t, rx)).collect(), out, FRONT);

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
            let next = madd(cur, dir, lens[f][k] * HS);
            b.part(format!("{p}{}{}", names[f], phal[k]), cur, next, radii[k].0 * HS, radii[k].1 * HS, out, FRONT);
            if k < 2 {
                let jn = format!("{p}{}{}", if k == 0 { "PIP" } else { "DIP" }, f + 2);
                b.joint(&jn, JointKind::Hinge, next, Some(&prev), 0.0022, Some(FRONT));
                prev = jn;
            }
            cur = next;
        }
    }

    // большой палец
    let cmc = rel(-0.004, -0.012, 0.014);
    let mcp1 = rel(-0.010, -0.036, 0.024);
    let ip1 = rel(-0.015, -0.054, 0.030);
    let tip1 = rel(-0.020, -0.068, 0.036);
    let n0 = norm([s, 0.0, 0.4]);
    b.part(format!("{p}ThumbMC"), cmc, mcp1, 0.0075 * HS, 0.0062 * HS, n0, UP);
    b.part(format!("{p}ThumbPP"), mcp1, ip1, 0.0062 * HS, 0.0054 * HS, n0, UP);
    b.part(format!("{p}ThumbDP"), ip1, tip1, 0.0054 * HS, 0.0044 * HS, n0, UP);
    b.joint(&format!("{p}CMC1"), JointKind::Saddle, cmc, Some(&wrist_name), 0.003, None);
    b.joint(&format!("{p}MCP1"), JointKind::Condyloid, mcp1, Some(&format!("{p}CMC1")), 0.0025, None);
    b.joint(&format!("{p}IP1"), JointKind::Hinge, ip1, Some(&format!("{p}MCP1")), 0.002, Some(FRONT));
}

// ───────────────────────────── Нога и стопа ─────────────────────────────

/// Сечения стопы вдоль Z: (z, y_низ, y_верх, ширина наружу, ширина внутрь).
const FOOT: [(f32, f32, f32, f32, f32); 11] = [
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
const ZS: f32 = 1.3; // удлинение стопы
const WS: f32 = 1.35; // расширение стопы

/// Высота верха стопы на оси при заданном z (линейно, как у меша).
fn foot_top(z: f32) -> f32 {
    let zs: Vec<(f32, f32)> = FOOT.iter().map(|st| (st.0 * ZS, st.2)).collect();
    lin(&zs, z)
}

fn foot_mesh(x_leg: f32, s: f32) -> (Vec<V3>, Vec<V3>, Vec<u32>) {
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

    // Ориентация граней наружу: от средней линии стопы на середине высоты её сечения.
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
    let n = mesh_normals(&v, &idx);
    (v, n, idx)
}

fn leg(b: &mut Builder, side: Side) {
    let s = side.sign();
    let p = side.prefix();
    let m = |v: V3| mirror(s, v);
    let out = [s, 0.0, 0.0];

    // бедро: сверху закруглено (тазобедренный шарнир), книзу сужается к колену
    let hip = m([0.096, 0.905, -0.005]);
    let knee = m([0.082, 0.495, 0.000]);
    limb(
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

    // голень: икроножная выпуклость в верхней трети
    limb(
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

    // стопа
    let (fv, fnm, fi) = foot_mesh(0.082 * s, s);
    b.prims.push(Prim::Mesh { vertices: fv, normals: fnm, indices: fi, tone: Tone::Cortical });

    // сегменты для датчиков на поверхности стопы (высота берётся из профиля меша)
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

// ───────────────────────────── Датчики ─────────────────────────────

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
            let r = prof_r(&sg.prof, t);
            (madd(lerp(sg.a, sg.b, t), dir, r + STANDOFF), dir, axis, lerp(sg.a, sg.b, 0.5))
        }
        Anchor::Spinous(level) => anchored(b, level, def),
        Anchor::Forehead => anchored(b, "Forehead", def),
        Anchor::Sternum => anchored(b, "Sternum", def),
        Anchor::Sacrum => anchored(b, "Sacrum", def),
    };
    SensorSite { id, tag: def.tag, side: def.side, pos, normal, tangent, support, chip: def.chip }
}

fn anchored(b: &Builder, name: &str, def: &SensorDef) -> (V3, V3, V3, V3) {
    let (p, n) = *b.anchors.get(name).unwrap_or_else(|| panic!("нет точки {name} для {}", def.tag));
    let tangent = if name == "Forehead" { X } else { UP };
    (madd(p, n, STANDOFF), n, tangent, madd(p, n, -0.020))
}

/// Собирает фигуру манекена.
#[must_use]
pub fn build() -> Skeleton {
    let mut b = Builder {
        joints: Vec::new(),
        jidx: HashMap::new(),
        prims: Vec::new(),
        segs: HashMap::new(),
        anchors: HashMap::new(),
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