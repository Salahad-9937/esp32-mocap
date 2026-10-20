//! # Размещение и фиксация меток датчиков на теле (`placement`)
//!
//! Обеспечивает приклеивание 64 сенсорных плат к кривизне поверхности манекена,
//! хранение пользовательских смещений, конфигурацию чувствительности камеры и передачу команд фокуса.

use super::skeleton::{skeleton, Chip, SENSOR_COUNT};
use super::surface::raycast;
use super::vec3::{cross, dot, madd, norm, scale, sub, V3};
use std::sync::{Mutex, OnceLock};

/// Файл локального сохранения пользовательской раскладки сенсоров
pub const LAYOUT_FILE: &str = "sensor_layout.txt";
/// Подъем плоскости метки над кожей манекена для устранения Z-файтинга (метры)
const LIFT: f32 = 0.0022;

/// Пользовательские настройки чувствительности всех видов перемещений 3D-камеры.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraSensitivity {
    /// Множитель скорости панорамирования СКМ (базово +15%)
    pub pan_speed: f32,
    /// Множитель скорости зума колесом мыши
    pub zoom_speed: f32,
    /// Множитель скорости полета WASD / Space / Ctrl
    pub move_speed: f32,
    /// Множитель скорости рыскания QE
    pub rot_speed: f32,
    /// Множитель чувствительности орбитального вращения ЛКМ
    pub orbit_sens: f32,
    /// Множитель чувствительности свободного осмотра ПКМ
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

/// Дескриптор метки датчика, зафиксированной на поверхности тела.
#[derive(Clone)]
pub struct PlacedChip {
    pub id: usize,
    pub tag: &'static str,
    pub chip: Chip,
    pub pos: V3,
    pub normal: V3,
    pub tangent: V3,
    pub patch: [V3; 9],
    pub patch_n: [V3; 9],
}

#[derive(Clone, Copy)]
struct Custom {
    pos: V3,
    normal: V3,
    tangent: V3,
}

struct State {
    defaults: Vec<PlacedChip>,
    chips: Vec<PlacedChip>,
    custom: [Option<Custom>; SENSOR_COUNT],
    hover: Option<usize>,
    drag: Option<usize>,
    sensitivity: CameraSensitivity,
}

static PENDING_FOCUS: Mutex<Option<(V3, f32)>> = Mutex::new(None);

/// Резервирует команду плавной фокусировки камеры на точке `pos`.
pub fn request_focus(pos: V3, dist: f32) {
    if let Ok(mut lock) = PENDING_FOCUS.lock() {
        *lock = Some((pos, dist));
    }
}

/// Извлекает отложенный запрос фокусировки для камеры.
#[must_use]
pub fn take_pending_focus() -> Option<(V3, f32)> {
    PENDING_FOCUS.lock().ok().and_then(|mut lock| lock.take())
}

fn state() -> &'static Mutex<State> {
    static STATE: OnceLock<Mutex<State>> = OnceLock::new();
    STATE.get_or_init(|| Mutex::new(State::new()))
}

/// Возвращает текущие настройки чувствительности камеры.
#[must_use]
pub fn get_sensitivity() -> CameraSensitivity {
    with(|s| s.sensitivity)
}

/// Обновляет параметры чувствительности камеры.
pub fn set_sensitivity(sens: CameraSensitivity) {
    with(|s| s.sensitivity = sens);
}

fn fit_tangent(hint: V3, n: V3) -> V3 {
    for cand in [hint, [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
        let t = sub(cand, scale(n, dot(cand, n)));
        if dot(t, t) > 0.09 {
            return norm(t);
        }
    }
    norm(cross(n, [1.0, 0.0, 0.0]))
}

fn snap(pos: V3, normal: V3) -> (V3, V3) {
    if let Some(h) = raycast(madd(pos, normal, 0.02), scale(normal, -1.0), 0.06) {
        return (h.pos, h.normal);
    }
    if let Some(h) = raycast(madd(pos, normal, -0.02), normal, 0.06) {
        return (h.pos, h.normal);
    }
    (pos, normal)
}

fn conform(id: usize, pos: V3, normal: V3, tangent: V3) -> PlacedChip {
    let site = &skeleton().sites[id];
    let n = normal;
    let u = fit_tangent(tangent, n);
    let v = norm(cross(n, u));
    let half = site.chip.half();
    let (hu, hv) = (half, half * 0.75);
    let mut patch = [[0.0; 3]; 9];
    let mut patch_n = [n; 9];
    for row in 0..3 {
        for col in 0..3 {
            let p0 = madd(madd(pos, u, (col as f32 - 1.0) * hu), v, (row as f32 - 1.0) * hv);
            let k = row * 3 + col;
            match raycast(madd(p0, n, 0.02), scale(n, -1.0), 0.04) {
                Some(h) if dot(h.normal, n) > 0.3 => {
                    patch[k] = madd(h.pos, h.normal, LIFT);
                    patch_n[k] = h.normal;
                }
                _ => patch[k] = madd(p0, n, LIFT),
            }
        }
    }
    PlacedChip {
        id,
        tag: site.tag,
        chip: site.chip,
        pos: patch[4],
        normal: n,
        tangent: u,
        patch,
        patch_n,
    }
}

impl State {
    fn new() -> Self {
        let defaults: Vec<PlacedChip> = skeleton()
            .sites
            .iter()
            .map(|s| {
                let (p, n) = snap(s.pos, s.normal);
                conform(s.id, p, n, s.tangent)
            })
            .collect();
        let mut st = State {
            chips: defaults.clone(),
            defaults,
            custom: [None; SENSOR_COUNT],
            hover: None,
            drag: None,
            sensitivity: CameraSensitivity::default(),
        };
        st.load();
        st
    }

    fn load(&mut self) {
        let Ok(text) = std::fs::read_to_string(LAYOUT_FILE) else {
            return;
        };
        for line in text.lines() {
            let f: Vec<f32> = line.split_whitespace().filter_map(|x| x.parse().ok()).collect();
            if f.len() != 10 || f[0] < 0.0 || f[0] as usize >= SENSOR_COUNT {
                continue;
            }
            let id = f[0] as usize;
            let (p, n) = snap([f[1], f[2], f[3]], norm([f[4], f[5], f[6]]));
            let chip = conform(id, p, n, [f[7], f[8], f[9]]);
            self.custom[id] = Some(Custom {
                pos: chip.pos,
                normal: chip.normal,
                tangent: chip.tangent,
            });
            self.chips[id] = chip;
        }
    }

    fn save(&self) {
        let mut out = String::new();
        for (id, c) in self.custom.iter().enumerate() {
            if let Some(c) = c {
                out.push_str(&format!(
                    "{id} {} {} {} {} {} {} {} {} {}\n",
                    c.pos[0], c.pos[1], c.pos[2], c.normal[0], c.normal[1], c.normal[2],
                    c.tangent[0], c.tangent[1], c.tangent[2]
                ));
            }
        }
        if out.is_empty() {
            let _ = std::fs::remove_file(LAYOUT_FILE);
        } else {
            let _ = std::fs::write(LAYOUT_FILE, out);
        }
    }
}

fn with<R>(f: impl FnOnce(&mut State) -> R) -> R {
    let mut g = state().lock().unwrap_or_else(std::sync::PoisonError::into_inner);
    f(&mut g)
}

#[must_use]
pub fn chips() -> Vec<PlacedChip> {
    with(|s| s.chips.clone())
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
    skeleton().sites[id].tag
}

#[must_use]
pub fn pick(p: V3) -> Option<usize> {
    with(|s| {
        s.chips
            .iter()
            .map(|c| (c.id, super::vec3::dist(c.pos, p), (c.chip.half() * 2.0).max(0.016)))
            .filter(|&(_, d, tol)| d < tol)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|x| x.0)
    })
}

pub fn begin_drag(id: usize) {
    with(|s| s.drag = Some(id));
}

pub fn end_drag() {
    with(|s| {
        s.drag = None;
        s.save();
    });
}

pub fn move_chip(id: usize, pos: V3, normal: V3) {
    with(|s| {
        let chip = conform(id, pos, normal, s.chips[id].tangent);
        s.custom[id] = Some(Custom {
            pos: chip.pos,
            normal: chip.normal,
            tangent: chip.tangent,
        });
        s.chips[id] = chip;
    });
}

pub fn reset_one(id: usize) {
    with(|s| {
        s.custom[id] = None;
        s.chips[id] = s.defaults[id].clone();
        s.save();
    });
}

pub fn reset_all() {
    with(|s| {
        s.custom = [None; SENSOR_COUNT];
        s.chips = s.defaults.clone();
        s.save();
    });
}

#[must_use]
pub fn custom_count() -> usize {
    with(|s| s.custom.iter().filter(|c| c.is_some()).count())
}