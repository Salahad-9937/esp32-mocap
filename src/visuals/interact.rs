//! # Интерактивный манипулятор датчиков, меню чувствительности и фокуса (`interact`)

use super::placement::{self, MannequinPose};
use super::viewport::OrbitalCamera;
use egui::{
    Area, Button, Context, CursorIcon, Frame, Id, Order, PointerButton, Rect, Response, RichText,
    Slider, Ui,
};
use mocap_model::skeleton::skeleton;
use mocap_model::surface::raycast;

fn show_tools(ctx: &Context, rect: Rect) {
    Area::new(Id::new("sensor_layout_tools"))
        .order(Order::Foreground)
        .fixed_pos(rect.right_top() + egui::vec2(-295.0, 10.0))
        .show(ctx, |ui| {
            Frame::popup(ui.style()).show(ui, |ui| {
                ui.set_width(275.0);
                ui.label(RichText::new("Навигация 3D-камеры").strong());
                ui.label("Вращение: ЛКМ  |  Осмотр: ПКМ\nПанорамирование: СКМ\nПолет: W/A/S/D  |  Поворот: Q/E\nВверх/Вниз: Space / Ctrl\nЗум: Колесо мыши (СКМ)");

                ui.separator();
                ui.label(RichText::new("Фокусировка").strong());

                ui.horizontal_wrapped(|ui| {
                    if ui.button("🎯 Тело").on_hover_text("Общий вид тела в целом").clicked() {
                        placement::request_focus([0.0, 0.88, 0.0], 2.6);
                    }
                    if ui.button("Голова").on_hover_text("Фокус на голове").clicked() {
                        placement::request_focus([0.0, 1.68, 0.0], 0.45);
                    }
                    if ui.button("Торс").on_hover_text("Верхняя часть туловища и грудь").clicked() {
                        placement::request_focus([0.0, 1.35, 0.0], 0.65);
                    }
                    if ui.button("Таз").on_hover_text("Тазобедренная область").clicked() {
                        placement::request_focus([0.0, 0.94, -0.015], 0.60);
                    }
                    if ui.button("Ноги").on_hover_text("Нижние конечности").clicked() {
                        placement::request_focus([0.0, 0.45, 0.0], 1.10);
                    }
                });

                ui.add_space(2.0);
                ui.separator();

                // Переключение позы манекена (N-поза <-> T-поза)
                ui.label(RichText::new("Поза манекена").strong());
                ui.horizontal(|ui| {
                    let cur_pose = placement::target_pose();
                    if ui.selectable_label(cur_pose == MannequinPose::NPose, "🧍 N-поза").on_hover_text("Руки опущены вдоль туловища").clicked() {
                        placement::set_target_pose(MannequinPose::NPose);
                    }
                    if ui.selectable_label(cur_pose == MannequinPose::TPose, "🤸 T-поза").on_hover_text("Руки разведены горизонтально в стороны").clicked() {
                        placement::set_target_pose(MannequinPose::TPose);
                    }
                });

                ui.add_space(2.0);
                ui.separator();

                // Матрица переключателей видимости элементов сцены
                ui.label(RichText::new("Режимы отображения").strong());
                ui.horizontal_wrapped(|ui| {
                    let mut show_mesh = placement::show_mesh();
                    if ui.checkbox(&mut show_mesh, "🧍 Манекен").on_hover_text("Показать/скрыть 3D-тело манекена").changed() {
                        placement::set_show_mesh(show_mesh);
                        ctx.request_repaint();
                    }

                    let mut show_skel = placement::show_skeleton();
                    if ui.checkbox(&mut show_skel, "🦴 Скелет").on_hover_text("Показать отладочный каркас кинематических связей").changed() {
                        placement::set_show_skeleton(show_skel);
                        ctx.request_repaint();
                    }

                    let mut show_joints = placement::show_joints();
                    if ui.checkbox(&mut show_joints, "🔵 Суставы").on_hover_text("Показать маркеры шарниров сочленений").changed() {
                        placement::set_show_joints(show_joints);
                        ctx.request_repaint();
                    }

                    let mut show_sensors = placement::show_sensors();
                    if ui.checkbox(&mut show_sensors, "🏷 Датчики").on_hover_text("Показать сенсорные площадки IMU и оси").changed() {
                        placement::set_show_sensors(show_sensors);
                        ctx.request_repaint();
                    }
                });

                if placement::show_sensors() {
                    ui.label(RichText::new("ЛКМ по площадке — выбрать сегмент для маппинга.\n2x-ЛКМ — сфокусировать камеру на объекте.").italics().size(11.0));
                }

                ui.separator();
                egui::CollapsingHeader::new(RichText::new("⚙ Чувствительность управления").strong())
                    .default_open(false)
                    .show(ui, |ui| {
                        let mut sens = placement::get_sensitivity();
                        let mut modified = false;

                        ui.label("Панорамирование (СКМ):");
                        if ui.add(Slider::new(&mut sens.pan_speed, 0.2..=3.0).step_by(0.05)).changed() {
                            modified = true;
                        }

                        ui.label("Зум (Колесо мыши):");
                        if ui.add(Slider::new(&mut sens.zoom_speed, 0.2..=3.0).step_by(0.05)).changed() {
                            modified = true;
                        }

                        ui.label("Полет (WASD / Space / Ctrl):");
                        if ui.add(Slider::new(&mut sens.move_speed, 0.1..=3.0).step_by(0.05)).changed() {
                            modified = true;
                        }

                        ui.label("Поворот (Q / E):");
                        if ui.add(Slider::new(&mut sens.rot_speed, 0.1..=3.0).step_by(0.05)).changed() {
                            modified = true;
                        }

                        ui.label("Орбитальное вращение (ЛКМ):");
                        if ui.add(Slider::new(&mut sens.orbit_sens, 0.1..=3.0).step_by(0.05)).changed() {
                            modified = true;
                        }

                        ui.label("Свободный осмотр (ПКМ):");
                        if ui.add(Slider::new(&mut sens.free_look_sens, 0.1..=3.0).step_by(0.05)).changed() {
                            modified = true;
                        }

                        if ui.button("Сброс к базовым").clicked() {
                            sens = placement::CameraSensitivity::default();
                            modified = true;
                        }

                        if modified {
                            placement::set_sensitivity(sens);
                        }
                    });

                if placement::show_sensors() {
                    ui.separator();
                    ui.label(RichText::new("Позиционирование меток").strong());
                    ui.label("Перемещение: Shift + перетаскивание ЛКМ\nСброс одной: Shift + 2x-ЛКМ");
                    let n = placement::custom_count();
                    if ui.add_enabled(n > 0, Button::new(format!("Сбросить все ({n})"))).clicked() {
                        placement::reset_all();
                    }
                }
            });
        });
}

pub fn handle(ui: &Ui, response: &Response, camera: &OrbitalCamera) -> bool {
    let rect = response.rect;
    let ctx = ui.ctx();
    show_tools(ctx, rect);

    let cast = |p: egui::Pos2| {
        let (o, d) = camera.ray_from_screen(p, rect);
        raycast(o, d, 30.0)
    };

    let is_shift_active = ui.input(|i| i.modifiers.shift);
    let sensors_visible = placement::show_sensors();
    let joints_visible = placement::show_joints();

    if sensors_visible {
        if let Some(id) = placement::dragging() {
            if response.dragged_by(PointerButton::Primary) {
                if let Some(h) = response.interact_pointer_pos().and_then(cast) {
                    placement::move_chip(id, h.pos, h.normal);
                }
                ctx.set_cursor_icon(CursorIcon::Grabbing);
                ctx.request_repaint();
                return true;
            }
            placement::end_drag();
            ctx.request_repaint();
        }
    }

    let hover_chip = if sensors_visible && response.hovered() {
        response.hover_pos().and_then(cast).and_then(|h| placement::pick(h.pos))
    } else {
        None
    };

    if placement::set_hover(hover_chip) {
        ctx.request_repaint();
    }

    if sensors_visible && response.clicked_by(PointerButton::Primary) && !is_shift_active && hover_chip.is_some() {
        placement::set_selected(hover_chip);
        ctx.request_repaint();
    }

    let hover_joint = if joints_visible && hover_chip.is_none() && response.hovered() {
        response.hover_pos().and_then(|pos| {
            skeleton().joints.iter().find(|j| {
                camera.project(j.pos, rect).map_or(false, |sp| {
                    (sp.x - pos.x).hypot(sp.y - pos.y) <= 18.0
                })
            })
        })
    } else {
        None
    };

    if let Some(id) = hover_chip {
        if is_shift_active {
            ctx.set_cursor_icon(CursorIcon::Grab);
            response
                .clone()
                .on_hover_text(format!("{} (Shift+перетаскивание для сдвига)", placement::tag(id)));

            if ui.input(|i| i.pointer.button_double_clicked(PointerButton::Primary)) {
                placement::reset_one(id);
                ctx.request_repaint();
                return true;
            }

            if response.drag_started_by(PointerButton::Primary) {
                placement::begin_drag(id);
                return true;
            }
        } else {
            ctx.set_cursor_icon(CursorIcon::PointingHand);
            let is_sel = placement::selected() == Some(id);
            let sel_suffix = if is_sel { " [ВЫБРАН ДЛЯ МАППИНГА]" } else { " [ЛКМ: выбрать]" };
            response.clone().on_hover_text(format!(
                "Датчик: {}{sel_suffix}\n2x-ЛКМ: фокус камеры",
                placement::tag(id)
            ));
        }
    } else if let Some(j) = hover_joint {
        ctx.set_cursor_icon(CursorIcon::PointingHand);
        response.clone().on_hover_text(format!(
            "Сустав: {} [2x-ЛКМ: плавный фокус]",
            j.name
        ));
    }

    false
}