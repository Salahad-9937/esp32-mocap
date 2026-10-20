//! # Геометрическое позиционирование сенсорных площадок на меше (`placement`)

use super::math::{cross, dist, dot, madd, norm, scale, sub, V3};
use super::skeleton::{skeleton, Chip, SENSOR_COUNT};
use super::surface::raycast;
use std::fs;
use std::path::Path;

/// Имя файла пользовательской раскладки сенсоров по умолчанию
pub const DEFAULT_LAYOUT_FILE: &str = "sensor_layout.txt";
/// Подъем плоскости метки над кожей манекена (метры)
pub const LIFT: f32 = 0.0022;

/// Дескриптор сенсорной площадки с узлами двусторонней сетки 3x3
#[derive(Clone, Debug, PartialEq)]
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

/// Пользовательская поправка положения сенсора
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CustomChipPlacement {
    pub pos: V3,
    pub normal: V3,
    pub tangent: V3,
}

/// Автономный менеджер раскладки и прижатия сенсорных плат
pub struct SensorLayoutManager {
    defaults: Vec<PlacedChip>,
    chips: Vec<PlacedChip>,
    custom: [Option<CustomChipPlacement>; SENSOR_COUNT],
}

impl Default for SensorLayoutManager {
    fn default() -> Self {
        Self::new()
    }
}

impl SensorLayoutManager {
    /// Инициализирует менеджер с автоматическим прижатием дефолтных площадок.
    #[must_use]
    pub fn new() -> Self {
        let defaults: Vec<PlacedChip> = skeleton()
            .sites
            .iter()
            .map(|s| {
                let (p, n) = snap_to_surface(s.pos, s.normal);
                conform_chip(s.id, p, n, s.tangent)
            })
            .collect();

        Self {
            chips: defaults.clone(),
            defaults,
            custom: [None; SENSOR_COUNT],
        }
    }

    /// Считывает активный срез сенсорных площадок.
    #[must_use]
    pub fn chips(&self) -> &[PlacedChip] {
        &self.chips
    }

    /// Определяет идентификатор ближайшей площадки в пределах радиуса допуска.
    #[must_use]
    pub fn pick(&self, p: V3) -> Option<usize> {
        self.chips
            .iter()
            .map(|c| (c.id, dist(c.pos, p), (c.chip.half() * 2.0).max(0.016)))
            .filter(|&(_, d, tol)| d < tol)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|x| x.0)
    }

    /// Перемещает площадку с прижатием к целевой точке поверхности.
    pub fn move_chip(&mut self, id: usize, pos: V3, normal: V3) {
        if id >= SENSOR_COUNT {
            return;
        }
        let chip = conform_chip(id, pos, normal, self.chips[id].tangent);
        self.custom[id] = Some(CustomChipPlacement {
            pos: chip.pos,
            normal: chip.normal,
            tangent: chip.tangent,
        });
        self.chips[id] = chip;
    }

    /// Сбрасывает положение одной площадки к анатомическому дефолту.
    pub fn reset_one(&mut self, id: usize) {
        if id >= SENSOR_COUNT {
            return;
        }
        self.custom[id] = None;
        self.chips[id] = self.defaults[id].clone();
    }

    /// Сбрасывает положения всех 64 площадок.
    pub fn reset_all(&mut self) {
        self.custom = [None; SENSOR_COUNT];
        self.chips = self.defaults.clone();
    }

    /// Возвращает количество модифицированных пользователем площадок.
    #[must_use]
    pub fn custom_count(&self) -> usize {
        self.custom.iter().filter(|c| c.is_some()).count()
    }

    /// Считывает пользовательские смещения из файла на диске.
    pub fn load_from_file(&mut self, path: &Path) {
        let Ok(text) = fs::read_to_string(path) else { return };
        for line in text.lines() {
            let f: Vec<f32> = line.split_whitespace().filter_map(|x| x.parse().ok()).collect();
            if f.len() != 10 || f[0] < 0.0 || f[0] as usize >= SENSOR_COUNT {
                continue;
            }
            let id = f[0] as usize;
            let (p, n) = snap_to_surface([f[1], f[2], f[3]], norm([f[4], f[5], f[6]]));
            let chip = conform_chip(id, p, n, [f[7], f[8], f[9]]);
            self.custom[id] = Some(CustomChipPlacement {
                pos: chip.pos,
                normal: chip.normal,
                tangent: chip.tangent,
            });
            self.chips[id] = chip;
        }
    }

    /// Сохраняет активные пользовательские смещения в файл на диске.
    pub fn save_to_file(&self, path: &Path) {
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
            let _ = fs::remove_file(path);
        } else {
            let _ = fs::write(path, out);
        }
    }
}

/// Вычисляет касательный вектор, ортогональный внешней нормали.
#[must_use]
pub fn fit_tangent(hint: V3, n: V3) -> V3 {
    for cand in [hint, [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]] {
        let t = sub(cand, scale(n, dot(cand, n)));
        if dot(t, t) > 0.09 {
            return norm(t);
        }
    }
    norm(cross(n, [1.0, 0.0, 0.0]))
}

/// Находит ближайшую точку полигональной поверхности вдоль нормали касания.
#[must_use]
pub fn snap_to_surface(pos: V3, normal: V3) -> (V3, V3) {
    if let Some(h) = raycast(madd(pos, normal, 0.02), scale(normal, -1.0), 0.06) {
        return (h.pos, h.normal);
    }
    if let Some(h) = raycast(madd(pos, normal, -0.02), normal, 0.06) {
        return (h.pos, h.normal);
    }
    (pos, normal)
}

/// Формирует сетку 3x3 узлов для прижатия сенсорной площадки к кривизне поверхности.
#[must_use]
pub fn conform_chip(id: usize, pos: V3, normal: V3, tangent: V3) -> PlacedChip {
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