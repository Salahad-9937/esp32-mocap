//! # Интерактивное управление перемещением и фокусом 3D-камеры (`controller`)

use super::camera::{FocusAnim, OrbitalCamera, MAX_CAMERA_DISTANCE, MIN_CAMERA_DISTANCE};
use crate::visuals::placement;
use egui::Response;
use mocap_model::math::dist;
use mocap_model::skeleton::skeleton;

impl OrbitalCamera {
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

    pub fn handle_input_ex(&mut self, ui_response: &Response, scroll_delta: f32, allow_drag: bool) {
        let ctx = &ui_response.ctx;
        let rect = ui_response.rect;
        let sens = placement::get_sensitivity();
        let mut changed = false;

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

        if let Some((target_pos, target_dist)) = placement::take_pending_focus() {
            self.focus_target(target_pos, target_dist);
            changed = true;
        }

        let is_shift = ctx.input(|i| i.modifiers.shift);
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
                let mut best_d = 0.12_f32;

                for chip in &placement::chips() {
                    let w = [
                        chip.pos[0] - origin[0],
                        chip.pos[1] - origin[1],
                        chip.pos[2] - origin[2],
                    ];
                    let t_ray = w[0] * dir[0] + w[1] * dir[1] + w[2] * dir[2];
                    if t_ray > 0.05 {
                        let proj = [
                            origin[0] + dir[0] * t_ray,
                            origin[1] + dir[1] * t_ray,
                            origin[2] + dir[2] * t_ray,
                        ];
                        let screen_d = self
                            .project(chip.pos, rect)
                            .map_or(999.0, |sp| (sp.x - pos.x).hypot(sp.y - pos.y));
                        if dist(chip.pos, proj) < best_d || screen_d <= 30.0 {
                            best_d = dist(chip.pos, proj);
                            best_target = Some((chip.pos, 0.30));
                        }
                    }
                }

                for joint in &skeleton().joints {
                    let w = [
                        joint.pos[0] - origin[0],
                        joint.pos[1] - origin[1],
                        joint.pos[2] - origin[2],
                    ];
                    let t_ray = w[0] * dir[0] + w[1] * dir[1] + w[2] * dir[2];
                    if t_ray > 0.05 {
                        let proj = [
                            origin[0] + dir[0] * t_ray,
                            origin[1] + dir[1] * t_ray,
                            origin[2] + dir[2] * t_ray,
                        ];
                        let screen_d = self
                            .project(joint.pos, rect)
                            .map_or(999.0, |sp| (sp.x - pos.x).hypot(sp.y - pos.y));
                        if dist(joint.pos, proj) < best_d || screen_d <= 30.0 {
                            best_d = dist(joint.pos, proj);
                            best_target = Some((joint.pos, 0.35));
                        }
                    }
                }

                let target_and_dist = best_target
                    .or_else(|| {
                        mocap_model::surface::raycast(origin, dir, 30.0).map(|hit| (hit.pos, 0.45))
                    })
                    .unwrap_or(([0.0, 0.88, 0.0], 2.6));

                self.focus_target(target_and_dist.0, target_and_dist.1);
                changed = true;
            }
        }

        let has_focus = ui_response.hovered() || ui_response.has_focus() || ui_response.dragged();
        if has_focus && !ctx.wants_keyboard_input() {
            let (fwd, r) = (self.forward(), self.right());
            let move_speed = (self.distance * 0.022).clamp(0.012, 0.22) * sens.move_speed;
            let rot_speed = 0.030_f32 * sens.rot_speed;

            if ctx.input(|i| i.key_down(egui::Key::W)) {
                self.focus_anim = None;
                if self.apply_eye_translation([fwd[0] * move_speed, fwd[1] * move_speed, fwd[2] * move_speed]) {
                    changed = true;
                }
            }
            if ctx.input(|i| i.key_down(egui::Key::S)) {
                self.focus_anim = None;
                if self.apply_eye_translation([-fwd[0] * move_speed, -fwd[1] * move_speed, -fwd[2] * move_speed]) {
                    changed = true;
                }
            }
            if ctx.input(|i| i.key_down(egui::Key::A)) {
                self.focus_anim = None;
                if self.apply_eye_translation([-r[0] * move_speed, -r[1] * move_speed, -r[2] * move_speed]) {
                    changed = true;
                }
            }
            if ctx.input(|i| i.key_down(egui::Key::D)) {
                self.focus_anim = None;
                if self.apply_eye_translation([r[0] * move_speed, r[1] * move_speed, r[2] * move_speed]) {
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
                if self.apply_eye_translation([0.0, move_speed, 0.0]) {
                    changed = true;
                }
            }
            if ctx.input(|i| i.modifiers.ctrl || i.modifiers.command) {
                self.focus_anim = None;
                if self.apply_eye_translation([0.0, -move_speed, 0.0]) {
                    changed = true;
                }
            }
        }

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
                let (r, u) = (self.right(), self.up());
                let (dx, dy) = (delta.x * pan_factor, delta.y * pan_factor);
                if self.apply_eye_translation([-(r[0] * dx - u[0] * dy), -(r[1] * dx - u[1] * dy), -(r[2] * dx - u[2] * dy)]) {
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

        if ui_response.hovered() && scroll_delta != 0.0 {
            let next_dist = self.distance * (1.0 - scroll_delta * 0.0035 * sens.zoom_speed);
            if next_dist < MIN_CAMERA_DISTANCE {
                let push = (MIN_CAMERA_DISTANCE - next_dist).clamp(0.01, 0.45);
                let fwd = self.forward();
                self.apply_eye_translation([fwd[0] * push, fwd[1] * push, fwd[2] * push]);
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
}