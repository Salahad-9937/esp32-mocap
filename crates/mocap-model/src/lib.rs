//! # `mocap-model`
//!
//! Доменная библиотека биомеханического скелета человека, процедурной геометрии манекена,
//! прямой кинематики (Forward Kinematics) и пространственного коллидера (Raycasting).

#![forbid(unsafe_code)]

pub mod anatomy;
pub mod kinematics;
pub mod math;
pub mod placement;
pub mod skeleton;
pub mod surface;

pub use anatomy::build as build_mannequin;
pub use kinematics::{kinematic_tree, rotate_vec_by_quat, BoneTransform, KinematicTree};
pub use math::V3;
pub use placement::{MannequinPose, PlacedChip, SensorLayoutManager};
pub use skeleton::{
    skeleton, Anchor, Chip, Joint, JointKind, Prim, SensorDef, SensorSite, Side, Skeleton, Tone,
    SENSOR_COUNT, SENSOR_DEFS, STANDOFF, TARGET_BONES,
};
pub use surface::{raycast, Hit};