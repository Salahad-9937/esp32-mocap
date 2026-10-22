//! # Геометрическое позиционирование сенсорных площадок на меше (`placement`)

use super::math::{cross, dist, dot, madd, norm, scale, sub, V3};
use super::skeleton::{skeleton, Chip, SENSOR_COUNT};

/// Технологический зазор над кожей для исключения Z-файтинга (метры)
pub const LIFT: f32 = 0.0005;

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
    #[must_use]
    pub fn new() -> Self {
        let defaults: Vec<PlacedChip> = skeleton()
            .sites
            .iter()
            .map(|s| conform_chip(s.id, s.pos, s.normal, s.tangent))
            .collect();

        Self {
            chips: defaults.clone(),
            defaults,
            custom: [None; SENSOR_COUNT],
        }
    }

    #[must_use]
    pub fn chips(&self) -> &[PlacedChip] {
        &self.chips
    }

    #[must_use]
    pub fn pick(&self, p: V3) -> Option<usize> {
        self.chips
            .iter()
            .map(|c| (c.id, dist(c.pos, p), (c.chip.half() * 2.0).max(0.016)))
            .filter(|&(_, d, tol)| d < tol)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|x| x.0)
    }

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

    pub fn reset_one(&mut self, id: usize) {
        if id >= SENSOR_COUNT {
            return;
        }
        self.custom[id] = None;
        if let Some(def) = self.defaults.get(id) {
            self.chips[id] = def.clone();
        }
    }

    pub fn reset_all(&mut self) {
        self.custom = [None; SENSOR_COUNT];
        self.chips = self.defaults.clone();
    }

    #[must_use]
    pub fn custom_count(&self) -> usize {
        self.custom.iter().filter(|c| c.is_some()).count()
    }
}

#[must_use]
pub fn fit_tangent(hint: V3, n: V3) -> V3 {
    let t = sub(hint, scale(n, dot(hint, n)));
    if dot(t, t) > 0.04 {
        return norm(t);
    }
    for cand in [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]] {
        let tc = sub(cand, scale(n, dot(cand, n)));
        if dot(tc, tc) > 0.04 {
            return norm(tc);
        }
    }
    norm(cross(n, [1.0, 0.0, 0.0]))
}

/// Формирует чистую плоскую сетку 3x3 узлов, плотно прилегающую к поверхности.
#[must_use]
pub fn conform_chip(id: usize, pos: V3, normal: V3, tangent: V3) -> PlacedChip {
    let site = &skeleton().sites[id];
    let n = norm(normal);
    let u = fit_tangent(tangent, n);
    let v = norm(cross(n, u));
    let half = site.chip.half();
    let (hu, hv) = (half, half * 0.75);

    let mut patch = [[0.0; 3]; 9];
    let patch_n = [n; 9];

    for row in 0..3 {
        for col in 0..3 {
            let k = row * 3 + col;
            let offset_u = (col as f32 - 1.0) * hu;
            let offset_v = (row as f32 - 1.0) * hv;
            let surface_p = madd(madd(pos, u, offset_u), v, offset_v);
            patch[k] = madd(surface_p, n, LIFT);
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