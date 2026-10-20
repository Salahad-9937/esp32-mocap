//! # Интерактивная гибридная 3D-камера (`viewport`)
//!
//! Реализует непрерывный континуум навигации: орбитальное вращение (Orbit),
//! неинвертированный осмотр от первого лица (Free-Fly на ПКМ), настраиваемый полет по WASD / QE / Space / Ctrl,
//! гарантированную фокусировку на суставах/датчиках и границы рабочей зоны 11.25x11.25x5.78 м.

use super::placement;
use super::skeleton::skeleton;
use super::vec3::{dist, norm};
use egui::{Pos2, Rect};

/// Горизонтальный полуразмер рабочей зоны (метры)
pub const WORK_BOUND_XZ: f32 = 5.625;
/// Нижняя граница положения глаза камеры по высоте (уровень пола)
pub const WORK_BOUND_Y_MIN: f32 = 0.05;
/// Верхняя граница положения глаза камеры по высоте
pub const WORK_BOUND_Y_MAX: f32 = 5.775;
/// Минимальная дистанция приближения к точке фокуса
pub const MIN_CAMERA_DISTANCE: f32 = 0.18;
/// Максимальная дистанция отката камеры в рабочей зоне
pub const MAX_CAMERA_DISTANCE: f32 = 12.0;

/// Состояние активной плавной интерполяции фокуса камеры.
#[derive(Debug, Clone, Copy)]
struct FocusAnim {
    start_target: [f32; 3],
    goal_target: [f32; 3],
    start_dist: f32,
    goal_dist: f32,
    elapsed: f32,
    duration: f32,
}

/// Параметры гибридной интерактивной 3D-камеры.
#[derive(Debug, Clone)]
pub struct OrbitalCamera {
    /// Целевая фокальная точка в мировом пространстве [x, y, z] (Pivot)
    pub target: [f32; 3],
    /// Дистанция от точки наблюдения до целевого фокуса (Radius)
    pub distance: f32,
    /// Угол рыскания (Yaw, азимут вокруг вертикали Y) в радианах
    pub yaw: f32,
    /// Угол тангажа (Pitch, наклон взгляда) в радианах
    pub pitch: f32,
    /// Масштабный коэффициент поля зрения (FOV)
    pub fov_scale: f32,
    /// Контроллер активной плавной интерполяции фокуса
    focus_anim: Option<FocusAnim>,
}

impl Default for OrbitalCamera {
    fn default() -> Self {
        Self {
            target: [0.0, 0.88, 0.0],
            distance: 2.6,
            yaw: std::f32::consts::PI - 0.55,
            pitch: -0.10,
            fov_scale: 1.0,
            focus_anim: None,
        }
    }
}

impl OrbitalCamera {
    /// Единичный вектор направления взгляда камеры в мировых координатах.
    #[must_use]
    pub fn forward(&self) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        [cp * sy, sp, cp * cy]
    }

    /// Единичный вектор «вправо» локального видового базиса камеры.
    #[must_use]
    pub fn right(&self) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        [cy, 0.0, -sy]
    }

    /// Единичный вектор «вверх» локального видового базиса камеры.
    #[must_use]
    pub fn up(&self) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        [-sp * sy, cp, -sp * cy]
    }

    /// Мировые координаты точки наблюдения (глаза) камеры.
    #[must_use]
    pub fn eye_position(&self) -> [f32; 3] {
        let fwd = self.forward();
        [
            self.target[0] - fwd[0] * self.distance,
            self.target[1] - fwd[1] * self.distance,
            self.target[2] - fwd[2] * self.distance,
        ]
    }

    /// Перемещает точку наблюдения (глаз) камеры и целевой фокус на вектор `delta`,
    /// останавливая движение ровно у границы рабочей зоны без паразитных сползаний вниз/вбок.
    pub fn apply_eye_translation(&mut self, delta: [f32; 3]) -> bool {
        let len_sq = delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
        if len_sq < 1e-9 {
            return false;
        }

        let eye = self.eye_position();
        let mut t_max = 1.0_f32;

        for (curr, d, min_b, max_b) in [
            (eye[0], delta[0], -WORK_BOUND_XZ, WORK_BOUND_XZ),
            (eye[1], delta[1], WORK_BOUND_Y_MIN, WORK_BOUND_Y_MAX),
            (eye[2], delta[2], -WORK_BOUND_XZ, WORK_BOUND_XZ),
        ] {
            if d > 1e-6 {
                let t = (max_b - curr) / d;
                t_max = t_max.min(t.max(0.0));
            } else if d < -1e-6 {
                let t = (min_b - curr) / d;
                t_max = t_max.min(t.max(0.0));
            }
        }

        if t_max > 1e-4 {
            let actual = [delta[0] * t_max, delta[1] * t_max, delta[2] * t_max];
            self.target[0] += actual[0];
            self.target[1] += actual[1];
            self.target[2] += actual[2];
            true
        } else {
            false
        }
    }

    /// Удерживает точку наблюдения камеры строго внутри рабочей зоны.
    pub fn clamp_bounds(&mut self) {
        self.distance = self.distance.clamp(MIN_CAMERA_DISTANCE, MAX_CAMERA_DISTANCE);

        let eye = self.eye_position();
        let clamped_eye = [
            eye[0].clamp(-WORK_BOUND_XZ, WORK_BOUND_XZ),
            eye[1].clamp(WORK_BOUND_Y_MIN, WORK_BOUND_Y_MAX),
            eye[2].clamp(-WORK_BOUND_XZ, WORK_BOUND_XZ),
        ];

        let shift = [
            clamped_eye[0] - eye[0],
            clamped_eye[1] - eye[1],
            clamped_eye[2] - eye[2],
        ];

        if shift[0].abs() > 1e-5 || shift[1].abs() > 1e-5 || shift[2].abs() > 1e-5 {
            self.target[0] += shift[0];
            self.target[1] += shift[1];
            self.target[2] += shift[2];
        }
    }

    /// Инициирует плавный переход камеры к заданной 3D-точке фокусировки.
    pub fn focus_target(&mut self, point: [f32; 3], target_distance: f32) {
        let clamped_dist = target_distance.clamp(MIN_CAMERA_DISTANCE, MAX_CAMERA_DISTANCE);
        self.focus_anim = Some(FocusAnim {
            start_target: self.target,
            goal_target: point,
            start_dist: self.distance,
            goal_dist: clamped_dist,
            elapsed: 0.0,
            duration: 0.28,
        });
    }

    /// Базовый обработчик ввода для сохранения обратной совместимости.
    pub fn handle_input(&mut self, ui_response: &egui::Response, scroll_delta: f32) {
        self.handle_input_ex(ui_response, scroll_delta, true);
    }

    /// Расширенный гибридный обработчик навигации с поддержкой пользовательской чувствительности.
    pub fn handle_input_ex(
        &mut self,
        ui_response: &egui::Response,
        scroll_delta: f32,
        allow_drag: bool,
    ) {
        let ctx = &ui_response.ctx;
        let rect = ui_response.rect;
        let sens = placement::get_sensitivity();
        let mut changed = false;

        // 1. Исполнение плавной интерполяции фокусировки (Smoothstep)
        let dt = ctx.input(|i| i.stable_dt).min(0.05);
        if let Some(mut anim) = self.focus_anim {
            anim.elapsed += dt;
            let progress = (anim.elapsed / anim.duration).clamp(0.0, 1.0);
            let t = progress * progress * (3.0 - 2.0 * progress);

            self.target[0] = anim.start_target[0] + (anim.goal_target[0] - anim.start_target[0]) * t;
            self.target[1] = anim.start_target[1] + (anim.goal_target[1] - anim.start_target[1]) * t;
            self.target[2] = anim.start_target[2] + (anim.goal_target[2] - anim.start_target[2]) * t;
            self.distance = anim.start_dist + (anim.goal_dist - anim.start_dist) * t;

            if progress >= 1.0 {
                self.target = anim.goal_target;
                self.distance = anim.goal_dist;
                self.focus_anim = None;
            } else {
                self.focus_anim = Some(anim);
            }

            self.clamp_bounds();
            ctx.request_repaint();
        }

        // 2. Применение запроса быстрой фокусировки из оверлейных кнопок
        if let Some((target_pos, target_dist)) = placement::take_pending_focus() {
            self.focus_target(target_pos, target_dist);
            changed = true;
        }

        let is_shift = ctx.input(|i| i.modifiers.shift);

        // 3. Надежное определение 2x-ЛКМ на базе прямого опроса контекстных событий
        let is_double_click = ui_response.hovered()
            && allow_drag
            && !is_shift
            && ctx.input(|i| i.pointer.button_double_clicked(egui::PointerButton::Primary));

        if is_double_click {
            let click_pos = ctx
                .input(|i| i.pointer.latest_pos())
                .or_else(|| ui_response.interact_pointer_pos());

            if let Some(pos) = click_pos {
                let (origin, dir) = self.ray_from_screen(pos, rect);

                let mut best_target: Option<([f32; 3], f32)> = None;
                let mut best_dist_to_ray = 0.12_f32;

                // А. Проверка чипов сенсоров в 3D пространстве вдоль луча
                for chip in &placement::chips() {
                    let w = [chip.pos[0] - origin[0], chip.pos[1] - origin[1], chip.pos[2] - origin[2]];
                    let t_ray = w[0] * dir[0] + w[1] * dir[1] + w[2] * dir[2];
                    if t_ray > 0.05 {
                        let proj = [origin[0] + dir[0] * t_ray, origin[1] + dir[1] * t_ray, origin[2] + dir[2] * t_ray];
                        let d_perp = dist(chip.pos, proj);
                        let screen_d = self.project(chip.pos, rect).map_or(999.0, |sp| (sp.x - pos.x).hypot(sp.y - pos.y));
                        if d_perp < best_dist_to_ray || screen_d <= 30.0 {
                            best_dist_to_ray = d_perp;
                            best_target = Some((chip.pos, 0.30));
                        }
                    }
                }

                // Б. Проверка анатомических суставов скелета в 3D пространстве
                for joint in &skeleton().joints {
                    let w = [joint.pos[0] - origin[0], joint.pos[1] - origin[1], joint.pos[2] - origin[2]];
                    let t_ray = w[0] * dir[0] + w[1] * dir[1] + w[2] * dir[2];
                    if t_ray > 0.05 {
                        let proj = [origin[0] + dir[0] * t_ray, origin[1] + dir[1] * t_ray, origin[2] + dir[2] * t_ray];
                        let d_perp = dist(joint.pos, proj);
                        let screen_d = self.project(joint.pos, rect).map_or(999.0, |sp| (sp.x - pos.x).hypot(sp.y - pos.y));
                        if d_perp < best_dist_to_ray || screen_d <= 30.0 {
                            best_dist_to_ray = d_perp;
                            best_target = Some((joint.pos, 0.35));
                        }
                    }
                }

                // В. Проверка полигонального меша тела манекена
                if best_target.is_none() {
                    if let Some(hit) = crate::visuals::surface::raycast(origin, dir, 30.0) {
                        best_target = Some((hit.pos, 0.45));
                    }
                }

                if let Some((target_pos, target_dist)) = best_target {
                    self.focus_target(target_pos, target_dist);
                } else {
                    self.focus_target([0.0, 0.88, 0.0], 2.6);
                }
                changed = true;
            }
        }

        // 4. Клавиатурная навигация с масштабируемой скоростью
        let has_kb_focus = ui_response.hovered() || ui_response.has_focus() || ui_response.dragged();
        if has_kb_focus && !ctx.wants_keyboard_input() {
            let fwd = self.forward();
            let r = self.right();
            let move_speed = (self.distance * 0.022).clamp(0.012, 0.22) * sens.move_speed;
            let rot_speed = 0.030_f32 * sens.rot_speed;

            if ctx.input(|i| i.key_down(egui::Key::W)) {
                self.focus_anim = None;
                let delta = [fwd[0] * move_speed, fwd[1] * move_speed, fwd[2] * move_speed];
                if self.apply_eye_translation(delta) {
                    changed = true;
                }
            }
            if ctx.input(|i| i.key_down(egui::Key::S)) {
                self.focus_anim = None;
                let delta = [-fwd[0] * move_speed, -fwd[1] * move_speed, -fwd[2] * move_speed];
                if self.apply_eye_translation(delta) {
                    changed = true;
                }
            }
            if ctx.input(|i| i.key_down(egui::Key::A)) {
                self.focus_anim = None;
                let delta = [-r[0] * move_speed, -r[1] * move_speed, -r[2] * move_speed];
                if self.apply_eye_translation(delta) {
                    changed = true;
                }
            }
            if ctx.input(|i| i.key_down(egui::Key::D)) {
                self.focus_anim = None;
                let delta = [r[0] * move_speed, r[1] * move_speed, r[2] * move_speed];
                if self.apply_eye_translation(delta) {
                    changed = true;
                }
            }
            if ctx.input(|i| i.key_down(egui::Key::Q)) {
                self.focus_anim = None;
                self.yaw -= rot_speed;
                changed = true;
            }
            if ctx.input(|i| i.key_down(egui::Key::E)) {
                self.focus_anim = None;
                self.yaw += rot_speed;
                changed = true;
            }
            if ctx.input(|i| i.key_down(egui::Key::Space)) {
                self.focus_anim = None;
                let delta = [0.0, move_speed, 0.0];
                if self.apply_eye_translation(delta) {
                    changed = true;
                }
            }
            if ctx.input(|i| i.modifiers.ctrl || i.modifiers.command) {
                self.focus_anim = None;
                let delta = [0.0, -move_speed, 0.0];
                if self.apply_eye_translation(delta) {
                    changed = true;
                }
            }
        }

        // 5. Опрос мыши с настраиваемыми множителями
        let is_rmb = ctx.input(|i| i.pointer.button_down(egui::PointerButton::Secondary));
        let is_mmb = ctx.input(|i| i.pointer.button_down(egui::PointerButton::Middle));
        let is_lmb = ctx.input(|i| i.pointer.button_down(egui::PointerButton::Primary));
        let delta = ctx.input(|i| i.pointer.delta());

        if (ui_response.hovered() || ui_response.dragged()) && (delta.x != 0.0 || delta.y != 0.0) {
            if is_mmb {
                self.focus_anim = None;
                let fov_rad = 0.785398_f32 / self.fov_scale.clamp(0.1, 4.0);
                let f_proj = 1.0 / (fov_rad * 0.5).tan().max(1e-4);
                let pan_factor = ((2.0 * self.distance) / (f_proj * rect.height().max(1.0))) * sens.pan_speed;

                let r = self.right();
                let u = self.up();
                let dx = delta.x * pan_factor;
                let dy = delta.y * pan_factor;

                let pan_delta = [-(r[0] * dx - u[0] * dy), -(r[1] * dx - u[1] * dy), -(r[2] * dx - u[2] * dy)];
                if self.apply_eye_translation(pan_delta) {
                    changed = true;
                }
            } else if is_rmb {
                let eye = self.eye_position();
                self.yaw += delta.x * 0.004 * sens.free_look_sens;
                self.pitch = (self.pitch - delta.y * 0.004 * sens.free_look_sens).clamp(-1.52, 1.52);

                let fwd = self.forward();
                self.target = [
                    eye[0] + fwd[0] * self.distance,
                    eye[1] + fwd[1] * self.distance,
                    eye[2] + fwd[2] * self.distance,
                ];
                self.clamp_bounds();
                changed = true;
            } else if is_lmb && allow_drag && !is_shift {
                self.yaw -= delta.x * 0.006 * sens.orbit_sens;
                self.pitch = (self.pitch + delta.y * 0.006 * sens.orbit_sens).clamp(-1.52, 1.52);
                self.clamp_bounds();
                changed = true;
            }
        }

        // 6. Зум колесом мыши с прямым множителем чувствительности
        if ui_response.hovered() && scroll_delta != 0.0 {
            let zoom_factor = 1.0 - scroll_delta * 0.0035 * sens.zoom_speed;
            let next_dist = self.distance * zoom_factor;
            if next_dist < MIN_CAMERA_DISTANCE {
                let push = (MIN_CAMERA_DISTANCE - next_dist).clamp(0.01, 0.45);
                let fwd = self.forward();
                let push_delta = [fwd[0] * push, fwd[1] * push, fwd[2] * push];
                self.apply_eye_translation(push_delta);
                self.distance = MIN_CAMERA_DISTANCE;
            } else {
                self.distance = next_dist.min(MAX_CAMERA_DISTANCE);
                self.clamp_bounds();
            }
            changed = true;
        }

        if changed {
            ctx.request_repaint();
        }
    }

    /// Преобразует трехмерную мировую координату в двумерную экранную точку холста egui.
    #[must_use]
    pub fn project(&self, p: [f32; 3], rect: Rect) -> Option<Pos2> {
        self.project_depth(p, rect).map(|(pos, _)| pos)
    }

    /// Проекция мировой точки с вычислением оптической глубины вдоль луча камеры.
    #[must_use]
    pub fn project_depth(&self, p: [f32; 3], rect: Rect) -> Option<(Pos2, f32)> {
        let eye = self.eye_position();
        let dx = p[0] - eye[0];
        let dy = p[1] - eye[1];
        let dz = p[2] - eye[2];

        let fwd = self.forward();
        let z_depth = dx * fwd[0] + dy * fwd[1] + dz * fwd[2];
        if z_depth <= 0.05 {
            return None;
        }

        let r = self.right();
        let u = self.up();
        let x_cam = dx * r[0] + dy * r[1] + dz * r[2];
        let y_cam = dx * u[0] + dy * u[1] + dz * u[2];

        let fov_rad = 0.785398_f32 / self.fov_scale.clamp(0.1, 4.0);
        let f_proj = 1.0 / (fov_rad * 0.5).tan().max(1e-4);
        let scale = (f_proj * rect.height() * 0.5) / z_depth;

        let screen_x = rect.center().x + x_cam * scale;
        let screen_y = rect.center().y - y_cam * scale;

        Some((Pos2::new(screen_x, screen_y), z_depth))
    }

    /// Масштаб проекции в пикселях на метр на единичной глубине.
    #[must_use]
    pub fn pixels_per_meter_at_unit_depth(&self, rect: Rect) -> f32 {
        let fov_rad = 0.785398_f32 / self.fov_scale.clamp(0.1, 4.0);
        let f_proj = 1.0 / (fov_rad * 0.5).tan().max(1e-4);
        f_proj * rect.height() * 0.5
    }

    /// Формирует нормализованный 3D-луч из точки наблюдения через экранный пиксель.
    #[must_use]
    pub fn ray_from_screen(&self, p: Pos2, rect: Rect) -> ([f32; 3], [f32; 3]) {
        let fov_rad = 0.785398_f32 / self.fov_scale.clamp(0.1, 4.0);
        let f_proj = 1.0 / (fov_rad * 0.5).tan().max(1e-4);
        let scale = f_proj * rect.height().max(1.0) * 0.5;

        let x_cam = (p.x - rect.center().x) / scale;
        let y_cam = -(p.y - rect.center().y) / scale;

        let fwd = self.forward();
        let r = self.right();
        let u = self.up();

        let dir_unnorm = [
            fwd[0] + x_cam * r[0] + y_cam * u[0],
            fwd[1] + x_cam * r[1] + y_cam * u[1],
            fwd[2] + x_cam * r[2] + y_cam * u[2],
        ];

        (self.eye_position(), norm(dir_unnorm))
    }

    /// Рассчитывает каноническую матрицу Model-View-Projection (column-major) для OpenGL.
    #[must_use]
    pub fn calc_mvp(&self, aspect: f32) -> [f32; 16] {
        let eye = self.eye_position();
        let r = self.right();
        let u = self.up();
        let f = self.forward();

        let fov_rad = 0.785398_f32 / self.fov_scale.clamp(0.1, 4.0);
        let f_proj = 1.0 / (fov_rad * 0.5).tan().max(1e-4);
        let a = aspect.max(0.001);

        let near = 0.1_f32;
        let far = 50.0_f32;
        let range_inv = 1.0 / (near - far);
        let p22 = (far + near) * range_inv;
        let p23 = 2.0 * far * near * range_inv;

        let v03 = -(r[0] * eye[0] + r[1] * eye[1] + r[2] * eye[2]);
        let v13 = -(u[0] * eye[0] + u[1] * eye[1] + u[2] * eye[2]);
        let v23 = f[0] * eye[0] + f[1] * eye[1] + f[2] * eye[2];

        let sx = f_proj / a;
        let sy = f_proj;

        [
            sx * r[0],
            sy * u[0],
            p22 * -f[0],
            f[0],
            sx * r[1],
            sy * u[1],
            p22 * -f[1],
            f[1],
            sx * r[2],
            sy * u[2],
            p22 * -f[2],
            f[2],
            sx * v03,
            sy * v13,
            p22 * v23 + p23,
            -v23,
        ]
    }
}