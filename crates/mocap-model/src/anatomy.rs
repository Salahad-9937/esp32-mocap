//! # Процедурный генератор анатомической геометрии манекена (`anatomy`)

pub mod body;
pub mod foot;
pub mod lofting;

use super::math::{dot, lerp, madd, norm, scale, sub, V3};
use super::skeleton::{
    Anchor, Joint, JointKind, Prim, SensorDef, SensorSite, Side, Skeleton, Tone, SENSOR_DEFS,
    STANDOFF,
};
use std::collections::HashMap;

pub const UP: V3 = [0.0, 1.0, 0.0];
pub const FRONT: V3 = [0.0, 0.0, 1.0];
pub const X: V3 = [1.0, 0.0, 0.0];

#[derive(Clone)]
pub struct Seg {
    pub a: V3,
    pub b: V3,
    pub prof: Vec<(f32, f32)>,
    pub n0: V3,
    pub n90: V3,
}

pub struct Builder {
    pub joints: Vec<Joint>,
    pub jidx: HashMap<String, usize>,
    pub prims: Vec<Prim>,
    pub prim_bones: Vec<&'static str>,
    pub segs: HashMap<String, Seg>,
    pub anchors: HashMap<String, (V3, V3)>,
    pub current_bone: &'static str,
}

impl Builder {
    pub fn set_bone(&mut self, bone: &'static str) {
        self.current_bone = bone;
    }

    pub fn cap(&mut self, a: V3, b: V3, ra: f32, rb: f32, tone: Tone) {
        self.prims.push(Prim::Capsule { a, b, ra, rb, tone });
        self.prim_bones.push(self.current_bone);
    }

    pub fn ball(&mut self, c: V3, r: f32, tone: Tone) {
        self.prims.push(Prim::Sphere { c, r, tone });
        self.prim_bones.push(self.current_bone);
    }

    pub fn part(&mut self, key: String, a: V3, b: V3, ra: f32, rb: f32, n0: V3, n90: V3) {
        self.cap(a, b, ra, rb, Tone::Cortical);
        self.segs.insert(key, Seg { a, b, prof: vec![(0.0, ra), (1.0, rb)], n0, n90 });
    }

    pub fn seg_only(&mut self, key: String, a: V3, b: V3, prof: Vec<(f32, f32)>, n0: V3, n90: V3) {
        self.segs.insert(key, Seg { a, b, prof, n0, n90 });
    }

    pub fn joint(&mut self, name: &str, kind: JointKind, pos: V3, parent: Option<&str>, r: f32, axis: Option<V3>) {
        let parent = parent.and_then(|p| self.jidx.get(p).copied());
        self.jidx.insert(name.to_string(), self.joints.len());
        self.joints.push(Joint { name: name.to_string(), kind, pos, parent, axis, marker_r: r });
    }
}

#[must_use]
pub fn cr(p: &[f32], i: usize, f: f32) -> f32 {
    let g = |k: isize| p[k.clamp(0, p.len() as isize - 1) as usize];
    let (p0, p1, p2, p3) = (g(i as isize - 1), g(i as isize), g(i as isize + 1), g(i as isize + 2));
    0.5 * (2.0 * p1
        + (p2 - p0) * f
        + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * f * f
        + (3.0 * p1 - p0 - 3.0 * p2 + p3) * f * f * f)
}

#[must_use]
pub fn prof_r(prof: &[(f32, f32)], t: f32) -> f32 {
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

fn resolve(b: &Builder, id: usize, def: &SensorDef) -> SensorSite {
    let (pos, normal, tangent, support) = match def.anchor {
        Anchor::Seg(name, t, deg) => {
            let key = format!("{}{}", def.side.prefix(), name);
            if let Some(sg) = b.segs.get(&key) {
                let axis = norm(sub(sg.b, sg.a));
                let n0 = norm(sub(sg.n0, scale(axis, dot(sg.n0, axis))));
                let n90 = sub(sg.n90, scale(axis, dot(sg.n90, axis)));
                let n90 = norm(sub(n90, scale(n0, dot(n90, n0))));
                let a = deg.to_radians();
                let dir = norm(super::math::add(scale(n0, a.cos()), scale(n90, a.sin())));
                let r = prof_r(&sg.prof, t);
                (madd(lerp(sg.a, sg.b, t), dir, r + STANDOFF), dir, axis, lerp(sg.a, sg.b, 0.5))
            } else {
                ([0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [0.0, 1.0, 0.0])
            }
        }
        Anchor::Spinous(level) => anchored(b, level),
        Anchor::Forehead => anchored(b, "Forehead"),
        Anchor::Sternum => anchored(b, "Sternum"),
        Anchor::Sacrum => anchored(b, "Sacrum"),
    };
    SensorSite { id, tag: def.tag, side: def.side, pos, normal, tangent, support, chip: def.chip }
}

fn anchored(b: &Builder, name: &str) -> (V3, V3, V3, V3) {
    let (p, n) = b.anchors.get(name).copied().unwrap_or(([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]));
    let tangent = if name == "Forehead" || name == "C5" { X } else { UP };
    (madd(p, n, STANDOFF), n, tangent, madd(p, n, -0.020))
}

pub fn build() -> Skeleton {
    let mut b = Builder {
        joints: Vec::new(),
        jidx: HashMap::new(),
        prims: Vec::new(),
        prim_bones: Vec::new(),
        segs: HashMap::new(),
        anchors: HashMap::new(),
        current_bone: "Sacrum",
    };
    b.joint("Pelvis", JointKind::Root, [0.0, 0.94, -0.015], None, 0.007, None);
    body::torso(&mut b);
    for side in [Side::Right, Side::Left] {
        body::shoulder_and_arm(&mut b, side);
        body::hand(&mut b, side);
        body::leg(&mut b, side);
    }
    let sites = SENSOR_DEFS.iter().enumerate().map(|(i, d)| resolve(&b, i, d)).collect();
    Skeleton {
        joints: b.joints,
        prims: b.prims,
        prim_bones: b.prim_bones,
        sites,
    }
}