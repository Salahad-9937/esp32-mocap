//! Модуль-фасад 3D-визуализации.

pub mod axes;
pub mod draw;
pub mod interact;
pub mod placement;
pub mod scene;
pub mod viewport;

pub use scene::render_3d_scene;
pub use viewport::OrbitalCamera;