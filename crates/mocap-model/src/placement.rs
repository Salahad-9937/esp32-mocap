//! # Геометрическое позиционирование сенсорных площадок на меше (`placement`)

use super::math::{cross, dist, dot, madd, norm, scale, sub, V3};
use super::skeleton::{skeleton, Chip, SENSOR_COUNT};
use super::surface::raycast;

/// Минимальный технологический зазор для исключения Z-файтинга (0.8 мм)
pub const LIFT: f32 = 0.0008;

/// Целевая поза манекена
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MannequinPose {
    #[default]
    NPose,
    TPose,
}

/// Дескриптор сенсорной площадки с конформной двусторонней сеткой 3x3
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

/// Автономный менеджер раскладки и конформного прилегания сенсорных плат
pub struct SensorLayoutManager {
    defaults: Vec<PlacedChip>,
    chips: Vec<PlacedChip>,
    custom: [Option<CustomChipPlacement>; SENSOR_COUNT],
    target_pose: MannequinPose,
    pose_blend: f32,
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
            target_pose: MannequinPose::NPose,
            pose_blend: 0.0,
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
        let site = &skeleton().sites[id];
        let chip = conform_chip(id, pos, normal, site.tangent);
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

    #[must_use]
    pub fn target_pose(&self) -> MannequinPose {
        self.target_pose
    }

    pub fn set_target_pose(&mut self, pose: MannequinPose) {
        self.target_pose = pose;
    }

    #[must_use]
    pub fn pose_blend(&self) -> f32 {
        self.pose_blend
    }

    pub fn update_pose_animation(&mut self, dt: f32) -> bool {
        let target = match self.target_pose {
            MannequinPose::NPose => 0.0_f32,
            MannequinPose::TPose => 1.0_f32,
        };

        if (self.pose_blend - target).abs() < 1e-4 {
            self.pose_blend = target;
            return false;
        }

        let speed = 1.45_f32; // Плавный переход за ~0.70 сек
        if self.pose_blend < target {
            self.pose_blend = (self.pose_blend + speed * dt).min(target);
        } else {
            self.pose_blend = (self.pose_blend - speed * dt).max(target);
        }
        true
    }
}

#[must_use]
pub fn fit_tangent(hint: V3, n: V3) -> V3 {
    let t = sub(hint, scale(n, dot(hint, n)));
    if dot(t, t) > 0.01 {
        return norm(t);
    }
    for cand in [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]] {
        let tc = sub(cand, scale(n, dot(cand, n)));
        if dot(tc, tc) > 0.01 {
            return norm(tc);
        }
    }
    norm(cross(n, [1.0, 0.0, 0.0]))
}

/// Формирует конформную сетку 3x3 узлов, точно прилегающую к поверхности меша.
#[must_use]
pub fn conform_chip(id: usize, pos: V3, normal: V3, tangent: V3) -> PlacedChip {
    let site = &skeleton().sites[id];
    let n = norm(normal);
    let u = fit_tangent(tangent, n);
    let v = norm(cross(n, u));
    let half = site.chip.half();
    let (hu, hv) = (half, half * 0.75);

    let mut patch = [[0.0; 3]; 9];
    let mut patch_n = [n; 9];

    for row in 0..3 {
        for col in 0..3 {
            let k = row * 3 + col;
            let offset_u = (col as f32 - 1.0) * hu;
            let offset_v = (row as f32 - 1.0) * hv;
            let p_nom = madd(madd(pos, u, offset_u), v, offset_v);

            // Пускаем короткий локальный луч перпендикулярно полигону (не более 10 мм)
            let ray_o = madd(p_nom, n, 0.005);
            let ray_d = scale(n, -1.0);

            if let Some(hit) = raycast(ray_o, ray_d, 0.010) {
                if dot(hit.normal, n) > 0.4 {
                    patch[k] = madd(hit.pos, hit.normal, LIFT);
                    patch_n[k] = hit.normal;
                    continue;
                }
            }

            // Плавное огибание за горизонт фаланги пальца без проникновения в меш
            let v_ratio = (offset_v.abs() / hv.max(1e-4)).clamp(0.0, 1.0);
            let curve_falloff = v_ratio * v_ratio * 0.0012;
            patch[k] = madd(madd(p_nom, n, -curve_falloff), n, LIFT);
            patch_n[k] = n;
        }
    }

    PlacedChip {
        id,
        tag: site.tag,
        chip: site.chip,
        pos: patch[4],
        normal: patch_n[4],
        tangent: u,
        patch,
        patch_n,
    }
}