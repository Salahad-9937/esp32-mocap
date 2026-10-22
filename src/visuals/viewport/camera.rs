//! # Проекционная геометрия орбитальной камеры (`camera`)

use egui::{Pos2, Rect};
use mocap_model::math::norm;

pub(crate) const WORK_BOUND_XZ: f32 = 5.625;
pub(crate) const WORK_BOUND_Y_MIN: f32 = 0.05;
pub(crate) const WORK_BOUND_Y_MAX: f32 = 5.775;
pub(crate) const MIN_CAMERA_DISTANCE: f32 = 0.18;
pub(crate) const MAX_CAMERA_DISTANCE: f32 = 12.0;

#[derive(Debug, Clone, Copy)]
pub struct FocusAnim {
    pub start_target: [f32; 3],
    pub goal_target: [f32; 3],
    pub start_dist: f32,
    pub goal_dist: f32,
    pub elapsed: f32,
    pub duration: f32,
}

#[derive(Debug, Clone)]
pub struct OrbitalCamera {
    pub target: [f32; 3],
    pub distance: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub fov_scale: f32,
    pub(crate) focus_anim: Option<FocusAnim>,
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
    #[must_use]
    pub fn forward(&self) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        [cp * sy, sp, cp * cy]
    }

    #[must_use]
    pub fn right(&self) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        [cy, 0.0, -sy]
    }

    #[must_use]
    pub fn up(&self) -> [f32; 3] {
        let (sy, cy) = self.yaw.sin_cos();
        let (sp, cp) = self.pitch.sin_cos();
        [-sp * sy, cp, -sp * cy]
    }

    #[must_use]
    pub fn eye_position(&self) -> [f32; 3] {
        let fwd = self.forward();
        [
            self.target[0] - fwd[0] * self.distance,
            self.target[1] - fwd[1] * self.distance,
            self.target[2] - fwd[2] * self.distance,
        ]
    }

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
            self.target[0] += delta[0] * t_max;
            self.target[1] += delta[1] * t_max;
            self.target[2] += delta[2] * t_max;
            true
        } else {
            false
        }
    }

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

    #[must_use]
    pub fn project(&self, p: [f32; 3], rect: Rect) -> Option<Pos2> {
        self.project_depth(p, rect).map(|(pos, _)| pos)
    }

    #[must_use]
    pub fn project_depth(&self, p: [f32; 3], rect: Rect) -> Option<(Pos2, f32)> {
        let eye = self.eye_position();
        let (dx, dy, dz) = (p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]);
        let fwd = self.forward();
        let z_depth = dx * fwd[0] + dy * fwd[1] + dz * fwd[2];
        if z_depth <= 0.05 {
            return None;
        }

        let (r, u) = (self.right(), self.up());
        let x_cam = dx * r[0] + dy * r[1] + dz * r[2];
        let y_cam = dx * u[0] + dy * u[1] + dz * u[2];

        let fov_rad = 0.785398_f32 / self.fov_scale.clamp(0.1, 4.0);
        let f_proj = 1.0 / (fov_rad * 0.5).tan().max(1e-4);
        let scale = (f_proj * rect.height() * 0.5) / z_depth;

        Some((
            Pos2::new(rect.center().x + x_cam * scale, rect.center().y - y_cam * scale),
            z_depth,
        ))
    }

    #[must_use]
    pub fn ray_from_screen(&self, p: Pos2, rect: Rect) -> ([f32; 3], [f32; 3]) {
        let fov_rad = 0.785398_f32 / self.fov_scale.clamp(0.1, 4.0);
        let f_proj = 1.0 / (fov_rad * 0.5).tan().max(1e-4);
        let scale = f_proj * rect.height().max(1.0) * 0.5;

        let x_cam = (p.x - rect.center().x) / scale;
        let y_cam = -(p.y - rect.center().y) / scale;

        let (fwd, r, u) = (self.forward(), self.right(), self.up());
        let dir = [
            fwd[0] + x_cam * r[0] + y_cam * u[0],
            fwd[1] + x_cam * r[1] + y_cam * u[1],
            fwd[2] + x_cam * r[2] + y_cam * u[2],
        ];
        (self.eye_position(), norm(dir))
    }

    #[must_use]
    pub fn calc_mvp(&self, aspect: f32) -> [f32; 16] {
        let eye = self.eye_position();
        let (r, u, f) = (self.right(), self.up(), self.forward());

        let fov_rad = 0.785398_f32 / self.fov_scale.clamp(0.1, 4.0);
        let f_proj = 1.0 / (fov_rad * 0.5).tan().max(1e-4);
        let a = aspect.max(0.001);

        let (near, far) = (0.1_f32, 50.0_f32);
        let range_inv = 1.0 / (near - far);
        let p22 = (far + near) * range_inv;
        let p23 = 2.0 * far * near * range_inv;

        let v03 = -(r[0] * eye[0] + r[1] * eye[1] + r[2] * eye[2]);
        let v13 = -(u[0] * eye[0] + u[1] * eye[1] + u[2] * eye[2]);
        let v23 = f[0] * eye[0] + f[1] * eye[1] + f[2] * eye[2];
        let (sx, sy) = (f_proj / a, f_proj);

        [
            sx * r[0], sy * u[0], p22 * -f[0], f[0],
            sx * r[1], sy * u[1], p22 * -f[1], f[1],
            sx * r[2], sy * u[2], p22 * -f[2], f[2],
            sx * v03, sy * v13, p22 * v23 + p23, -v23,
        ]
    }
}