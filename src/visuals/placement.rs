//! # Адаптер размещения меток и состояния навигации (`placement`)

use mocap_model::math::V3;
pub use mocap_model::placement::{MannequinPose, PlacedChip};
use mocap_model::placement::SensorLayoutManager;
use mocap_model::skeleton::skeleton;
use std::sync::{Mutex, OnceLock};

/// Настройки чувствительности перемещений 3D-камеры.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraSensitivity {
    pub pan_speed: f32,
    pub zoom_speed: f32,
    pub move_speed: f32,
    pub rot_speed: f32,
    pub orbit_sens: f32,
    pub free_look_sens: f32,
}

impl Default for CameraSensitivity {
    fn default() -> Self {
        Self {
            pan_speed: 1.15,
            zoom_speed: 1.0,
            move_speed: 1.0,
            rot_speed: 1.0,
            orbit_sens: 1.0,
            free_look_sens: 1.0,
        }
    }
}

struct State {
    layout: SensorLayoutManager,
    hover: Option<usize>,
    selected: Option<usize>,
    drag: Option<usize>,
    sensitivity: CameraSensitivity,
    show_mesh: bool,
    show_skeleton: bool,
    show_joints: bool,
    show_sensors: bool,
}

static PENDING_FOCUS: Mutex<Option<(V3, f32)>> = Mutex::new(None);

pub fn request_focus(pos: V3, dist: f32) {
    if let Ok(mut lock) = PENDING_FOCUS.lock() {
        *lock = Some((pos, dist));
    }
}

#[must_use]
pub fn take_pending_focus() -> Option<(V3, f32)> {
    PENDING_FOCUS.lock().ok().and_then(|mut lock| lock.take())
}

fn state() -> &'static Mutex<State> {
    static STATE: OnceLock<Mutex<State>> = OnceLock::new();
    STATE.get_or_init(|| {
        Mutex::new(State {
            layout: SensorLayoutManager::new(),
            hover: None,
            selected: None,
            drag: None,
            sensitivity: CameraSensitivity::default(),
            show_mesh: true,
            show_skeleton: false,
            show_joints: true,
            show_sensors: true,
        })
    })
}

fn with<R>(f: impl FnOnce(&mut State) -> R) -> R {
    let mut g = state().lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    f(&mut g)
}

#[must_use]
pub fn show_mesh() -> bool { with(|s| s.show_mesh) }
pub fn set_show_mesh(show: bool) { with(|s| s.show_mesh = show); }

#[must_use]
pub fn show_skeleton() -> bool { with(|s| s.show_skeleton) }
pub fn set_show_skeleton(show: bool) { with(|s| s.show_skeleton = show); }

#[must_use]
pub fn show_joints() -> bool { with(|s| s.show_joints) }
pub fn set_show_joints(show: bool) { with(|s| s.show_joints = show); }

#[must_use]
pub fn show_sensors() -> bool { with(|s| s.show_sensors) }
pub fn set_show_sensors(show: bool) { with(|s| s.show_sensors = show); }

#[must_use]
pub fn target_pose() -> MannequinPose { with(|s| s.layout.target_pose()) }
pub fn set_target_pose(pose: MannequinPose) { with(|s| s.layout.set_target_pose(pose)); }

#[must_use]
pub fn pose_blend() -> f32 { with(|s| s.layout.pose_blend()) }
pub fn update_pose_animation(dt: f32) -> bool { with(|s| s.layout.update_pose_animation(dt)) }

#[must_use]
pub fn get_sensitivity() -> CameraSensitivity { with(|s| s.sensitivity) }
pub fn set_sensitivity(sens: CameraSensitivity) { with(|s| s.sensitivity = sens); }

#[must_use]
pub fn chips() -> Vec<PlacedChip> { with(|s| s.layout.chips().to_vec()) }

#[must_use]
pub fn highlight() -> Option<usize> { with(|s| s.drag.or(s.hover)) }

#[must_use]
pub fn selected() -> Option<usize> { with(|s| s.selected) }
pub fn set_selected(sel: Option<usize>) { with(|s| s.selected = sel); }

pub fn select_by_tag(tag: &str) {
    let id = skeleton().sites.iter().position(|s| s.tag == tag);
    with(|s| s.selected = id);
}

#[must_use]
pub fn selected_tag() -> Option<&'static str> {
    with(|s| s.selected.and_then(|id| skeleton().sites.get(id).map(|site| site.tag)))
}

#[must_use]
pub fn dragging() -> Option<usize> { with(|s| s.drag) }

pub fn set_hover(h: Option<usize>) -> bool {
    with(|s| {
        let changed = s.hover != h;
        s.hover = h;
        changed
    })
}

#[must_use]
pub fn tag(id: usize) -> &'static str { skeleton().sites.get(id).map_or("", |s| s.tag) }

#[must_use]
pub fn pick(p: V3) -> Option<usize> { with(|s| s.layout.pick(p)) }

pub fn begin_drag(id: usize) { with(|s| s.drag = Some(id)); }
pub fn end_drag() { with(|s| s.drag = None); }

pub fn move_chip(id: usize, pos: V3, normal: V3) { with(|s| s.layout.move_chip(id, pos, normal)); }
pub fn reset_one(id: usize) { with(|s| s.layout.reset_one(id)); }
pub fn reset_all() { with(|s| s.layout.reset_all()); }

#[must_use]
pub fn custom_count() -> usize { with(|s| s.layout.custom_count()) }