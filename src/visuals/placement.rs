//! Размещение 64 меток датчиков на теле: приклеивание к поверхности, перетаскивание
//! мышью, сброс (всех или одной) и сохранение раскладки в `sensor_layout.txt`.

use super::skeleton::{skeleton, Chip, SENSOR_COUNT};
use super::surface::raycast;
use super::vec3::{cross, dot, madd, norm, scale, sub, V3};
use std::sync::{Mutex, OnceLock};

/// Файл с раскладкой (рядом с исполняемым файлом / в рабочей папке).
pub const LAYOUT_FILE: &str = "sensor_layout.txt";
/// Подъём метки над поверхностью, м (чтобы не мерцала с кожей манекена).
const LIFT: f32 = 0.0008;

/// Метка, приклеенная к поверхности: центр, нормаль и сетка 3×3, повторяющая кривизну тела.
#[derive(Clone)]
pub struct PlacedChip {
    pub id: usize,
    pub tag: &'static str,
    pub chip: Chip,
    pub pos: V3,
    pub normal: V3,
    pub tangent: V3,
    /// Узлы сетки: индекс = ряд * 3 + колонка (колонка — вдоль `tangent`).
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
}

fn state() -> &'static Mutex<State> {
    static STATE: OnceLock<Mutex<State>> = OnceLock::new();
    STATE.get_or_init(|| Mutex::new(State::new()))
}

/// Тангенс, ортогональный нормали и по возможности направленный вдоль тела (вверх).
fn fit_tangent(hint: V3, n: V3) -> V3 {
    for cand in [hint, [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
        let t = sub(cand, scale(n, dot(cand, n)));
        if dot(t, t) > 0.09 {
            return norm(t);
        }
    }
    norm(cross(n, [1.0, 0.0, 0.0]))
}

/// Нормаль и точка на поверхности под заданной точкой (луч вдоль −normal).
fn snap(pos: V3, normal: V3) -> (V3, V3) {
    if let Some(h) = raycast(madd(pos, normal, 0.02), scale(normal, -1.0), 0.06) {
        return (h.pos, h.normal);
    }
    if let Some(h) = raycast(madd(pos, normal, -0.02), normal, 0.06) {
        return (h.pos, h.normal);
    }
    (pos, normal)
}

/// Строит метку: центр уже на поверхности, сетка 3×3 приклеивается лучами вдоль нормали.
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
    PlacedChip { id, tag: site.tag, chip: site.chip, pos: patch[4], normal: n, tangent: u, patch, patch_n }
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
        };
        st.load();
        st
    }

    fn load(&mut self) {
        let Ok(text) = std::fs::read_to_string(LAYOUT_FILE) else { return };
        for line in text.lines() {
            let f: Vec<f32> = line.split_whitespace().filter_map(|x| x.parse().ok()).collect();
            if f.len() != 10 || f[0] < 0.0 || f[0] as usize >= SENSOR_COUNT {
                continue;
            }
            let id = f[0] as usize;
            let (p, n) = snap([f[1], f[2], f[3]], norm([f[4], f[5], f[6]]));
            let chip = conform(id, p, n, [f[7], f[8], f[9]]);
            self.custom[id] = Some(Custom { pos: chip.pos, normal: chip.normal, tangent: chip.tangent });
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

/// Текущие метки (с учётом перемещённых пользователем).
#[must_use]
pub fn chips() -> Vec<PlacedChip> {
    with(|s| s.chips.clone())
}

/// Метка, подсвеченная курсором или перетаскиваемая.
#[must_use]
pub fn highlight() -> Option<usize> {
    with(|s| s.drag.or(s.hover))
}

#[must_use]
pub fn dragging() -> Option<usize> {
    with(|s| s.drag)
}

/// Возвращает `true`, если подсвеченная метка изменилась.
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

/// Ближайшая метка к точке на теле (в пределах размера платы).
#[must_use]
pub fn pick(p: V3) -> Option<usize> {
    with(|s| {
        s.chips
            .iter()
            .map(|c| (c.id, super::vec3::dist(c.pos, p), (c.chip.half() * 2.0).max(0.014)))
            .filter(|&(_, d, tol)| d < tol)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|x| x.0)
    })
}

pub fn begin_drag(id: usize) {
    with(|s| s.drag = Some(id));
}

/// Завершает перетаскивание и сохраняет раскладку.
pub fn end_drag() {
    with(|s| {
        s.drag = None;
        s.save();
    });
}

/// Переносит метку в точку на поверхности (`pos`, `normal` — результат трассировки).
pub fn move_chip(id: usize, pos: V3, normal: V3) {
    with(|s| {
        let chip = conform(id, pos, normal, s.chips[id].tangent);
        s.custom[id] = Some(Custom { pos: chip.pos, normal: chip.normal, tangent: chip.tangent });
        s.chips[id] = chip;
    });
}

/// Сбрасывает одну метку на положение по умолчанию.
pub fn reset_one(id: usize) {
    with(|s| {
        s.custom[id] = None;
        s.chips[id] = s.defaults[id].clone();
        s.save();
    });
}

/// Сбрасывает все метки на положения по умолчанию.
pub fn reset_all() {
    with(|s| {
        s.custom = [None; SENSOR_COUNT];
        s.chips = s.defaults.clone();
        s.save();
    });
}

/// Сколько меток сейчас перемещено пользователем.
#[must_use]
pub fn custom_count() -> usize {
    with(|s| s.custom.iter().filter(|c| c.is_some()).count())
}
