//! Интерактивная орбитальная 3D-камера и аналитическая проекция на 2D-холст.

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
            // Человек смотрит в +Z, поэтому камера с yaw ≈ π видит его спереди.
            yaw: std::f32::consts::PI - 0.55,
            pitch: -0.10,
            fov_scale: 1.0,
        }
    }
}

impl OrbitalCamera {
    /// Обрабатывает взаимодействие с мышью без захвата глобальных блокировок контекста.
    pub fn handle_input(&mut self, ui_response: &egui::Response, scroll_delta: f32) {
        if ui_response.dragged_by(egui::PointerButton::Primary) {
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

    /// Проекция вместе с глубиной точки вдоль взгляда камеры (для сортировки по глубине).
    #[must_use]
    pub fn project_depth(&self, p: [f32; 3], rect: Rect) -> Option<(Pos2, f32)> {
        let dx = p[0] - self.target[0];
        let dy = p[1] - self.target[1];
        let dz = p[2] - self.target[2];

        // Поворот вокруг оси Y (yaw)
        let cos_y = self.yaw.cos();
        let sin_y = self.yaw.sin();
        let x1 = cos_y * dx - sin_y * dz;
        let z1 = sin_y * dx + cos_y * dz;

        // Поворот вокруг поперечной оси X (pitch)
        let cos_p = self.pitch.cos();
        let sin_p = self.pitch.sin();
        let y2 = cos_p * dy - sin_p * z1;
        let z2 = sin_p * dy + cos_p * z1;

        let depth = z2 + self.distance;
        if depth <= 0.10 {
            return None; // Точка за плоскостью отсечения камеры
        }

        let scale = self.pixels_per_meter_at_unit_depth(rect) / depth;
        let screen_x = rect.center().x + x1 * scale;
        let screen_y = rect.center().y - y2 * scale;

        Some((Pos2::new(screen_x, screen_y), depth))
    }

    /// Масштаб проекции: пикселей на метр при глубине 1 м (на глубине `d` — делить на `d`).
    #[must_use]
    pub fn pixels_per_meter_at_unit_depth(&self, rect: Rect) -> f32 {
        rect.height() * 0.88 * self.fov_scale
    }

    /// Единичный вектор направления взгляда камеры в мировых координатах.
    /// Поверхность смотрит на камеру, если `dot(normal, forward) < 0`.
    #[must_use]
    pub fn forward(&self) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        [cp * sy, sp, cp * cy]
    }
}
