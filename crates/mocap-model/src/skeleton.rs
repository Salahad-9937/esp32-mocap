//! # Анатомический скелет, суставы и реестр 64 сенсорных позиций (`skeleton`)

use super::anatomy;
use super::math::V3;
use std::sync::OnceLock;

pub const SENSOR_COUNT: usize = 64;
pub const STANDOFF: f32 = 0.0012;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

impl Side {
    #[must_use]
    pub const fn sign(self) -> f32 {
        match self {
            Side::Right => -1.0,
            Side::Left => 1.0,
        }
    }

    #[must_use]
    pub const fn prefix(self) -> &'static str {
        match self {
            Side::Right => "R.",
            Side::Left => "L.",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointKind {
    Root,
    Ball,
    Hinge,
    Pivot,
    Saddle,
    Condyloid,
    Gliding,
    Spinal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tone {
    Cortical,
    Trabecular,
    Plate,
    Cartilage,
    Tooth,
    Dark,
}

#[derive(Debug, Clone)]
pub enum Prim {
    Capsule { a: V3, b: V3, ra: f32, rb: f32, tone: Tone },
    Sphere { c: V3, r: f32, tone: Tone },
    Tri { p: [V3; 3], tone: Tone },
    Mesh {
        vertices: Vec<V3>,
        normals: Vec<V3>,
        indices: Vec<u32>,
        tone: Tone,
    },
}

#[derive(Debug, Clone)]
pub struct Joint {
    pub name: String,
    pub kind: JointKind,
    pub pos: V3,
    pub parent: Option<usize>,
    pub axis: Option<V3>,
    pub marker_r: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Chip {
    Large,
    Medium,
    Small,
}

impl Chip {
    #[must_use]
    pub const fn half(self) -> f32 {
        match self {
            Chip::Large => 0.014,
            Chip::Medium => 0.010,
            Chip::Small => 0.0055,
        }
    }

    #[must_use]
    pub const fn axis_len(self) -> f32 {
        match self {
            Chip::Large => 0.10,
            Chip::Medium => 0.065,
            Chip::Small => 0.024,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Anchor {
    Seg(&'static str, f32, f32),
    Spinous(&'static str),
    Forehead,
    Sternum,
    Sacrum,
}

#[derive(Debug, Clone, Copy)]
pub struct SensorDef {
    pub tag: &'static str,
    pub side: Side,
    pub anchor: Anchor,
    pub chip: Chip,
}

const fn d(tag: &'static str, side: Side, anchor: Anchor, chip: Chip) -> SensorDef {
    SensorDef { tag, side, anchor, chip }
}

use Anchor::{Forehead, Sacrum, Seg, Spinous, Sternum};
use Chip::{Large as L, Medium as M, Small as S};
use Side::{Left as Lt, Right as Rt};

pub const SENSOR_DEFS: [SensorDef; SENSOR_COUNT] = [
    d("Head", Rt, Forehead, M),
    d("Neck", Rt, Spinous("C5"), M),
    d("SpineT3", Rt, Spinous("T3"), L),
    d("SpineT7", Rt, Spinous("T7"), L),
    d("SpineT11", Rt, Spinous("T11"), L),
    d("SpineL3", Rt, Spinous("L3"), L),
    d("Sacrum", Rt, Sacrum, L),
    d("Sternum", Rt, Sternum, L),
    d("LeftClavicle", Lt, Seg("Clavicle", 0.5, 0.0), M),
    d("RightClavicle", Rt, Seg("Clavicle", 0.5, 0.0), M),
    d("LeftShoulder", Lt, Seg("Acromion", 0.85, 0.0), M),
    d("RightShoulder", Rt, Seg("Acromion", 0.85, 0.0), M),
    d("LeftUpperArmProx", Lt, Seg("UpperArm", 0.30, 100.0), M),
    d("LeftUpperArmDist", Lt, Seg("UpperArm", 0.72, 100.0), M),
    d("LeftForearmProx", Lt, Seg("Ulna", 0.30, 110.0), M),
    d("LeftForearmDist", Lt, Seg("Radius", 0.72, 80.0), M),
    d("LeftHand", Lt, Seg("MC3", 0.50, 0.0), M),
    d("LeftThumbMeta", Lt, Seg("ThumbMC", 0.5, 0.0), S),
    d("LeftThumbProx", Lt, Seg("ThumbPP", 0.5, 0.0), S),
    d("LeftThumbDist", Lt, Seg("ThumbDP", 0.5, 0.0), S),
    d("LeftIndexProx", Lt, Seg("IndexPP", 0.5, 0.0), S),
    d("LeftIndexMid", Lt, Seg("IndexMP", 0.5, 0.0), S),
    d("LeftIndexDist", Lt, Seg("IndexDP", 0.5, 0.0), S),
    d("LeftMiddleProx", Lt, Seg("MiddlePP", 0.5, 0.0), S),
    d("LeftMiddleMid", Lt, Seg("MiddleMP", 0.5, 0.0), S),
    d("LeftMiddleDist", Lt, Seg("MiddleDP", 0.5, 0.0), S),
    d("LeftRingProx", Lt, Seg("RingPP", 0.5, 0.0), S),
    d("LeftRingMid", Lt, Seg("RingMP", 0.5, 0.0), S),
    d("LeftRingDist", Lt, Seg("RingDP", 0.5, 0.0), S),
    d("LeftPinkyProx", Lt, Seg("PinkyPP", 0.5, 0.0), S),
    d("LeftPinkyMid", Lt, Seg("PinkyMP", 0.5, 0.0), S),
    d("LeftPinkyDist", Lt, Seg("PinkyDP", 0.5, 0.0), S),
    d("RightUpperArmProx", Rt, Seg("UpperArm", 0.30, 100.0), M),
    d("RightUpperArmDist", Rt, Seg("UpperArm", 0.72, 100.0), M),
    d("RightForearmProx", Rt, Seg("Ulna", 0.30, 110.0), M),
    d("RightForearmDist", Rt, Seg("Radius", 0.72, 80.0), M),
    d("RightHand", Rt, Seg("MC3", 0.50, 0.0), M),
    d("RightThumbMeta", Rt, Seg("ThumbMC", 0.5, 0.0), S),
    d("RightThumbProx", Rt, Seg("ThumbPP", 0.5, 0.0), S),
    d("RightThumbDist", Rt, Seg("ThumbDP", 0.5, 0.0), S),
    d("RightIndexProx", Rt, Seg("IndexPP", 0.5, 0.0), S),
    d("RightIndexMid", Rt, Seg("IndexMP", 0.5, 0.0), S),
    d("RightIndexDist", Rt, Seg("IndexDP", 0.5, 0.0), S),
    d("RightMiddleProx", Rt, Seg("MiddlePP", 0.5, 0.0), S),
    d("RightMiddleMid", Rt, Seg("MiddleMP", 0.5, 0.0), S),
    d("RightMiddleDist", Rt, Seg("MiddleDP", 0.5, 0.0), S),
    d("RightRingProx", Rt, Seg("RingPP", 0.5, 0.0), S),
    d("RightRingMid", Rt, Seg("RingMP", 0.5, 0.0), S),
    d("RightRingDist", Rt, Seg("RingDP", 0.5, 0.0), S),
    d("RightPinkyProx", Rt, Seg("PinkyPP", 0.5, 0.0), S),
    d("RightPinkyMid", Rt, Seg("PinkyMP", 0.5, 0.0), S),
    d("RightPinkyDist", Rt, Seg("PinkyDP", 0.5, 0.0), S),
    d("LeftThighProx", Lt, Seg("Femur", 0.30, 90.0), L),
    d("LeftThighDist", Lt, Seg("Femur", 0.72, 90.0), L),
    d("LeftShankProx", Lt, Seg("Tibia", 0.28, -45.0), L),
    d("LeftShankDist", Lt, Seg("Tibia", 0.70, -45.0), L),
    d("LeftFoot", Lt, Seg("MT3", 0.50, 0.0), M),
    d("LeftToe", Lt, Seg("HalluxPP", 0.50, 0.0), S),
    d("RightThighProx", Rt, Seg("Femur", 0.30, 90.0), L),
    d("RightThighDist", Rt, Seg("Femur", 0.72, 90.0), L),
    d("RightShankProx", Rt, Seg("Tibia", 0.28, -45.0), L),
    d("RightShankDist", Rt, Seg("Tibia", 0.70, -45.0), L),
    d("RightFoot", Rt, Seg("MT3", 0.50, 0.0), M),
    d("RightToe", Rt, Seg("HalluxPP", 0.50, 0.0), S),
];

pub const TARGET_BONES: [&str; SENSOR_COUNT] = {
    let mut out = [""; SENSOR_COUNT];
    let mut i = 0;
    while i < SENSOR_COUNT {
        out[i] = SENSOR_DEFS[i].tag;
        i += 1;
    }
    out
};

#[derive(Debug, Clone)]
pub struct SensorSite {
    pub id: usize,
    pub tag: &'static str,
    pub side: Side,
    pub pos: V3,
    pub normal: V3,
    pub tangent: V3,
    pub support: V3,
    pub chip: Chip,
}

pub struct Skeleton {
    pub joints: Vec<Joint>,
    pub prims: Vec<Prim>,
    pub prim_bones: Vec<&'static str>,
    pub sites: Vec<SensorSite>,
}

#[must_use]
pub fn skeleton() -> &'static Skeleton {
    static SKELETON: OnceLock<Skeleton> = OnceLock::new();
    SKELETON.get_or_init(anatomy::build)
}