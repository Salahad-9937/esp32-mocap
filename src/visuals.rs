//! Модуль-фасад 3D-визуализации.

pub mod axes;
pub mod scene;
pub mod skeleton;
pub mod viewport;

pub use scene::render_3d_scene;
pub use viewport::OrbitalCamera;