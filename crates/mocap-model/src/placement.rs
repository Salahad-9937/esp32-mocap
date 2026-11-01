//! # Геометрическое позиционирование сенсорных площадок на меше (`placement`)

use super::math::{add, cross, dist, dot, madd, norm, scale, sub, V3};
use super::skeleton::{skeleton, Chip, SENSOR_COUNT};
use super::surface::raycast;

/// Толщина корпуса метки датчика (2 мм)
pub const THICKNESS: f32 = 0.002;

/// Технологический отступ основания шайбы от полигонов меша (0.4 мм)
pub const BASE_LIFT: f32 = 0.0004;

/// Технологический зазор для обратной совместимости
pub const LIFT: f32 = BASE_LIFT;

/// Количество радиальных секторов круглой метки (16 секторов)
pub const SECTORS: usize = 16;

/// Целевая поза манекена
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MannequinPose {
    #[default]
    NPose,
    TPose,
}

/// Дескриптор сенсорной площадки с жесткой недеформируемой 3D-шайбой толщиной 2 мм
#[derive(Clone, Debug, PartialEq)]
pub struct PlacedChip {
    pub id: usize,
    pub tag: &'static str,
    pub chip: Chip,
    pub pos: V3,
    pub normal: V3,
    pub tangent: V3,
    pub vertices: Vec<V3>,
    pub normals: Vec<V3>,
    pub indices: Vec<u32>,
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
            .filter(|c| !c.vertices.is_empty())
            .map(|c| (c.id, dist(c.pos, p), (c.chip.half() * 1.6).max(0.016)))
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

        let speed = 1.45_f32;
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

/// Формирует жесткую плоскую шайбу с локализованным поиском поверхности (окно 35 мм исключает пробитие на бедро)
#[must_use]
pub fn conform_chip(id: usize, pos: V3, normal: V3, tangent: V3) -> PlacedChip {
    let site = &skeleton().sites[id];

    // Подавляем только неиспользуемые фантомные промежуточные узлы ThumbProx и точки из центра брюшной полости
    let is_phantom = site.tag == "LeftThumbProx"
        || site.tag == "RightThumbProx"
        || (pos[0].abs() < 0.05 && (pos[1] - 1.0).abs() < 0.05 && pos[2].abs() < 0.05);

    if is_phantom {
        return PlacedChip {
            id,
            tag: site.tag,
            chip: site.chip,
            pos: [0.0, -999.0, 0.0],
            normal: [0.0, 1.0, 0.0],
            tangent: [1.0, 0.0, 0.0],
            vertices: Vec::new(),
            normals: Vec::new(),
            indices: Vec::new(),
            patch: [[0.0; 3]; 9],
            patch_n: [[0.0, 1.0, 0.0]; 9],
        };
    }

    let n = norm(normal);
    let u = fit_tangent(tangent, n);
    let v = norm(cross(n, u));
    let r = site.chip.half();

    // 1. Поиск поверхности строго в пределах 35 мм
    let center_surf = {
        let ray_o = madd(pos, n, 0.012);
        let ray_d = scale(n, -1.0);
        if let Some(hit) = raycast(ray_o, ray_d, 0.035) {
            if dot(hit.normal, n) > 0.15 {
                hit.pos
            } else {
                pos
            }
        } else {
            pos
        }
    };

    let c_top = madd(center_surf, n, BASE_LIFT + THICKNESS);
    let c_bot = madd(center_surf, n, BASE_LIFT);

    // 2. Генерация жесткой недеформируемой цилиндрической геометрии
    let mut vertices = Vec::with_capacity(66);
    let mut normals = Vec::with_capacity(66);
    let mut indices = Vec::with_capacity(192);

    // Верхняя крышка
    vertices.push(c_top);
    normals.push(n);
    for i in 0..SECTORS {
        let angle = (i as f32) * (std::f32::consts::TAU / SECTORS as f32);
        let (sa, ca) = angle.sin_cos();
        let radial = norm(add(scale(u, ca), scale(v, sa)));
        let p = madd(c_top, radial, r);
        vertices.push(p);
        normals.push(n);
    }
    for i in 0..SECTORS {
        let next = (i + 1) % SECTORS;
        indices.extend([0, (1 + i) as u32, (1 + next) as u32]);
    }

    // Боковая обечайка (высота 2 мм)
    let side_base = vertices.len() as u32;
    for i in 0..SECTORS {
        let angle = (i as f32) * (std::f32::consts::TAU / SECTORS as f32);
        let (sa, ca) = angle.sin_cos();
        let radial = norm(add(scale(u, ca), scale(v, sa)));
        let p_top = madd(c_top, radial, r);
        let p_bot = madd(c_bot, radial, r);

        vertices.push(p_top);
        normals.push(radial);
        vertices.push(p_bot);
        normals.push(radial);
    }
    for i in 0..SECTORS {
        let next = (i + 1) % SECTORS;
        let t_cur = side_base + (2 * i) as u32;
        let b_cur = t_cur + 1;
        let t_nxt = side_base + (2 * next) as u32;
        let b_nxt = t_nxt + 1;
        indices.extend([t_cur, b_cur, t_nxt, t_nxt, b_cur, b_nxt]);
    }

    // Нижняя крышка
    let bot_base = vertices.len() as u32;
    vertices.push(c_bot);
    normals.push(scale(n, -1.0));
    for i in 0..SECTORS {
        let angle = (i as f32) * (std::f32::consts::TAU / SECTORS as f32);
        let (sa, ca) = angle.sin_cos();
        let radial = norm(add(scale(u, ca), scale(v, sa)));
        let p = madd(c_bot, radial, r);
        vertices.push(p);
        normals.push(scale(n, -1.0));
    }
    for i in 0..SECTORS {
        let next = (i + 1) % SECTORS;
        indices.extend([bot_base, (bot_base + 1 + next as u32), (bot_base + 1 + i as u32)]);
    }

    let mut patch = [c_top; 9];
    let patch_n = [n; 9];
    patch[4] = c_top;
    for k in 0..8 {
        let idx = (k * 2) % SECTORS;
        let target = if k < 4 { k } else { k + 1 };
        patch[target] = vertices[1 + idx];
    }

    PlacedChip {
        id,
        tag: site.tag,
        chip: site.chip,
        pos: c_top,
        normal: n,
        tangent: u,
        vertices,
        normals,
        indices,
        patch,
        patch_n,
    }
}