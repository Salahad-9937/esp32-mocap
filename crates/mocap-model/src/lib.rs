//! # `mocap-model`
//!
//! Доменная библиотека биомеханического скелета человека, процедурной геометрии манекена,
//! описания сенсорных площадок и пространственного лучевого коллидера (Raycasting).
//! Полностью изолирована от оконных систем, графических API и прикладной политики ввода.

#![forbid(unsafe_code)]

pub mod anatomy;
pub mod math;
pub mod placement;
pub mod skeleton;
pub mod surface;

pub use anatomy::build as build_mannequin;
pub use math::V3;
pub use placement::{PlacedChip, SensorLayoutManager};
pub use skeleton::{
    skeleton, Anchor, Chip, Joint, JointKind, Prim, SensorDef, SensorSite, Side, Skeleton, Tone,
    SENSOR_COUNT, SENSOR_DEFS, STANDOFF, TARGET_BONES,
};
pub use surface::{raycast, Hit};