//! Интерактивная орбитальная 3D-камера и расчет матриц преобразования.

use super::vec3::{add, cross, dot, norm, scale, sub};
use egui::{Pos2, Rect};

/// Параметры сферической орбитальной камеры вокруг целевой точки.
#[derive(Debug, Clone)]
pub struct OrbitalCamera {
    /// Целевая точка фокусировки [x, y, z]
    pub target: [f32; 3],
    /// Дистанция удаления камеры от цели
    pub distance: f32,
    /// Угол рыскания (азимут) в радианах
    pub yaw: f32,
    /// Угол тангажа (высота) в радианах
    pub pitch: f32,
    /// Коэффициент перспективы фокусного расстояния
    pub fov_scale: f32,
}

impl Default for OrbitalCamera {
    fn default() -> Self {
        Self {
            target: [0.0, 0.88, 0.0],
            distance: 2.6,
            yaw: std::f32::consts::PI - 0.55,
            pitch: -0.10,
            fov_scale: 1.0,
        }
    }
}

impl OrbitalCamera {
    /// Обрабатывает взаимодействие с мышью.
    pub fn handle_input(&mut self, ui_response: &egui::Response, scroll_delta: f32) {
        self.handle_input_ex(ui_response, scroll_delta, true);
    }

    /// То же, но `allow_drag = false` отключает вращение/панорамирование
    /// (мышь занята перетаскиванием метки); масштаб колесом работает всегда.
    pub fn handle_input_ex(&mut self, ui_response: &egui::Response, scroll_delta: f32, allow_drag: bool) {
        if !allow_drag {
            // перетаскивание занято меткой датчика
        } else if ui_response.dragged_by(egui::PointerButton::Primary) {
            let delta = ui_response.drag_delta();
            self.yaw -= delta.x * 0.008;
            self.pitch = (self.pitch + delta.y * 0.008).clamp(-1.45, 1.45);
        } else if ui_response.dragged_by(egui::PointerButton::Middle)
            || ui_response.dragged_by(egui::PointerButton::Secondary)
        {
            let delta = ui_response.drag_delta();
            let cos_y = self.yaw.cos();
            let sin_y = self.yaw.sin();
            let pan_speed = self.distance * 0.0016;
            self.target[0] -= (cos_y * delta.x + sin_y * delta.y) * pan_speed;
            self.target[1] += delta.y * pan_speed;
            self.target[2] -= (-sin_y * delta.x + cos_y * delta.y) * pan_speed;
        }

        if ui_response.hovered() && scroll_delta != 0.0 {
            let zoom_factor = 1.0 - scroll_delta * 0.002;
            self.distance = (self.distance * zoom_factor).clamp(0.4, 12.0);
        }
    }

    /// Преобразует трехмерную мировую координату в двумерную координату холста egui.
    #[must_use]
    pub fn project(&self, p: [f32; 3], rect: Rect) -> Option<Pos2> {
        self.project_depth(p, rect).map(|(pos, _)| pos)
    }

    /// Проекция вместе с глубиной точки вдоль взгляда камеры.
    #[must_use]
    pub fn project_depth(&self, p: [f32; 3], rect: Rect) -> Option<(Pos2, f32)> {
        let dx = p[0] - self.target[0];
        let dy = p[1] - self.target[1];
        let dz = p[2] - self.target[2];

        let cos_y = self.yaw.cos();
        let sin_y = self.yaw.sin();
        let x1 = cos_y * dx - sin_y * dz;
        let z1 = sin_y * dx + cos_y * dz;

        let cos_p = self.pitch.cos();
        let sin_p = self.pitch.sin();
        let y2 = cos_p * dy - sin_p * z1;
        let z2 = sin_p * dy + cos_p * z1;

        let depth = z2 + self.distance;
        if depth <= 0.10 {
            return None;
        }

        let scale = self.pixels_per_meter_at_unit_depth(rect) / depth;
        let screen_x = rect.center().x + x1 * scale;
        let screen_y = rect.center().y - y2 * scale;

        Some((Pos2::new(screen_x, screen_y), depth))
    }

    /// Масштаб проекции в пикселях на метр.
    #[must_use]
    pub fn pixels_per_meter_at_unit_depth(&self, rect: Rect) -> f32 {
        rect.height() * 0.88 * self.fov_scale
    }

    /// Единичный вектор направления взгляда камеры.
    #[must_use]
    pub fn forward(&self) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        [cp * sy, sp, cp * cy]
    }

    /// Луч из камеры через точку экрана: (начало, единичное направление) в мировых координатах.
    #[must_use]
    pub fn ray_from_screen(&self, p: Pos2, rect: Rect) -> ([f32; 3], [f32; 3]) {
        let aspect = rect.width() / rect.height().max(1.0);
        let nx = (p.x - rect.min.x) / rect.width().max(1.0) * 2.0 - 1.0;
        let ny = 1.0 - (p.y - rect.min.y) / rect.height().max(1.0) * 2.0;
        let eye = self.eye_position();
        let f = norm(sub(self.target, eye));
        let r = norm(cross(f, [0.0, 1.0, 0.0]));
        let u = cross(r, f);
        let t = (0.785398_f32 / self.fov_scale.clamp(0.1, 4.0) * 0.5).tan();
        let dir = norm(add(add(f, scale(r, nx * aspect * t)), scale(u, ny * t)));
        (eye, dir)
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

    /// Вычисляет однородную $4 \times 4$ матрицу Model-View-Projection (column-major) для OpenGL.
    #[must_use]
    pub fn calc_mvp(&self, aspect: f32) -> [f32; 16] {
        let eye = self.eye_position();
        let target = self.target;

        let f = norm(sub(target, eye));
        let up_ref = [0.0_f32, 1.0, 0.0];
        let r = norm(cross(f, up_ref));
        let u = cross(r, f);

        let view = [
            [r[0], u[0], -f[0], 0.0],
            [r[1], u[1], -f[1], 0.0],
            [r[2], u[2], -f[2], 0.0],
            [-dot(r, eye), -dot(u, eye), dot(f, eye), 1.0],
        ];

        let fov_rad = 0.785398_f32 / self.fov_scale.clamp(0.1, 4.0);
        let f_proj = 1.0 / (fov_rad * 0.5).tan();
        let near = 0.1_f32;
        let far = 50.0_f32;
        let range_inv = 1.0 / (near - far);

        let proj = [
            [f_proj / aspect.max(0.001), 0.0, 0.0, 0.0],
            [0.0, f_proj, 0.0, 0.0],
            [0.0, 0.0, (far + near) * range_inv, -1.0],
            [0.0, 0.0, 2.0 * far * near * range_inv, 0.0],
        ];

        let mut mvp = [0.0_f32; 16];
        for col in 0..4 {
            for row in 0..4 {
                let mut sum = 0.0_f32;
                for k in 0..4 {
                    sum += proj[k][row] * view[col][k];
                }
                mvp[col * 4 + row] = sum;
            }
        }
        mvp
    }
}