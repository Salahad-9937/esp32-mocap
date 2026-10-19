//! Модуль-фасад 3D-визуализации.

pub mod anatomy;
pub mod axes;
pub mod draw;
pub mod scene;
pub mod skeleton;
pub mod vec3;
pub mod viewport;

pub use scene::render_3d_scene;
pub use viewport::OrbitalCamera;
