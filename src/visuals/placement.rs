//! # Адаптер размещения меток и состояния навигации (`placement`)

use mocap_model::math::V3;
pub use mocap_model::placement::PlacedChip;
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
    drag: Option<usize>,
    sensitivity: CameraSensitivity,
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
            drag: None,
            sensitivity: CameraSensitivity::default(),
        })
    })
}

fn with<R>(f: impl FnOnce(&mut State) -> R) -> R {
    let mut g = state().lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    f(&mut g)
}

#[must_use]
pub fn get_sensitivity() -> CameraSensitivity {
    with(|s| s.sensitivity)
}

pub fn set_sensitivity(sens: CameraSensitivity) {
    with(|s| s.sensitivity = sens);
}

#[must_use]
pub fn chips() -> Vec<PlacedChip> {
    with(|s| s.layout.chips().to_vec())
}

#[must_use]
pub fn highlight() -> Option<usize> {
    with(|s| s.drag.or(s.hover))
}

#[must_use]
pub fn dragging() -> Option<usize> {
    with(|s| s.drag)
}

pub fn set_hover(h: Option<usize>) -> bool {
    with(|s| {
        let changed = s.hover != h;
        s.hover = h;
        changed
    })
}

#[must_use]
pub fn tag(id: usize) -> &'static str {
    skeleton().sites.get(id).map_or("", |s| s.tag)
}

#[must_use]
pub fn pick(p: V3) -> Option<usize> {
    with(|s| s.layout.pick(p))
}

pub fn begin_drag(id: usize) {
    with(|s| s.drag = Some(id));
}

pub fn end_drag() {
    with(|s| s.drag = None);
}

pub fn move_chip(id: usize, pos: V3, normal: V3) {
    with(|s| s.layout.move_chip(id, pos, normal));
}

pub fn reset_one(id: usize) {
    with(|s| s.layout.reset_one(id));
}

pub fn reset_all() {
    with(|s| s.layout.reset_all());
}

#[must_use]
pub fn custom_count() -> usize {
    with(|s| s.layout.custom_count())
}