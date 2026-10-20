//! Мышь в 3D-сцене: наведение на метку, перетаскивание по телу, сброс двойным кликом
//! и небольшая панель «Метки датчиков».

use super::placement;
use super::surface::raycast;
use super::viewport::OrbitalCamera;
use egui::{Area, Button, Context, CursorIcon, Frame, Id, Order, PointerButton, Rect, Response, RichText, Ui};

/// Панель в правом верхнем углу сцены.
fn show_tools(ctx: &Context, rect: Rect) {
    Area::new(Id::new("sensor_layout_tools"))
        .order(Order::Foreground)
        .fixed_pos(rect.right_top() + egui::vec2(-232.0, 10.0))
        .show(ctx, |ui| {
            Frame::popup(ui.style()).show(ui, |ui| {
                ui.set_width(210.0);
                ui.label(RichText::new("Метки датчиков").strong());
                ui.label("Тяните метку мышью по телу.\nДвойной клик по метке — сброс одной.");
                let n = placement::custom_count();
                if ui.add_enabled(n > 0, Button::new(format!("Сбросить все ({n})"))).clicked() {
                    placement::reset_all();
                }
            });
        });
}

/// Обрабатывает взаимодействие с метками. Возвращает `true`, если мышь занята меткой
/// (тогда камеру вращать не нужно).
pub fn handle(ui: &Ui, response: &Response, camera: &OrbitalCamera) -> bool {
    let rect = response.rect;
    let ctx = ui.ctx();
    show_tools(ctx, rect);

    let cast = |p: egui::Pos2| {
        let (o, d) = camera.ray_from_screen(p, rect);
        raycast(o, d, 30.0)
    };

    // уже тащим метку
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

    // наведение
    let hover = if response.hovered() {
        response.hover_pos().and_then(cast).and_then(|h| placement::pick(h.pos))
    } else {
        None
    };
    if placement::set_hover(hover) {
        ctx.request_repaint();
    }
    let Some(id) = hover else { return false };

    ctx.set_cursor_icon(CursorIcon::Grab);
    response.clone().on_hover_text(placement::tag(id));

    if ui.input(|i| i.pointer.button_double_clicked(PointerButton::Primary)) {
        placement::reset_one(id);
        ctx.request_repaint();
        return true;
    }
    if response.drag_started_by(PointerButton::Primary) {
        placement::begin_drag(id);
        return true;
    }
    false
}
