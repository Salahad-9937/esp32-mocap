//! Модуль-фасад 3D-визуализации.

pub mod draw;
pub mod interact;
pub mod placement;
pub mod viewport;

pub use draw::render_3d_scene;
pub use viewport::OrbitalCamera;