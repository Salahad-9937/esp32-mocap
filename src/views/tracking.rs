//! # Режим живого захвата движений и инспекции поз скелета (`tracking`)

use crate::state::SharedStateHandle;
use egui::{Color32, ProgressBar, RichText, Ui};
use std::sync::atomic::Ordering;

#[derive(Debug, Default)]
pub struct TrackingView;

impl TrackingView {
    pub fn show(&mut self, ui: &mut Ui, shared: &SharedStateHandle) {
        ui.heading("Живой захват движений (Live Motion)");
        ui.label("В этом режиме манекен деформируется в реальном времени прямой кинематикой.");
        ui.add_space(4.0);

        let nodes = shared.nodes.lock().map(|g| g.clone()).unwrap_or_default();
        let fps = shared.calculated_fps.lock().map(|f| *f).unwrap_or(0.0);
        let active_count = nodes.len();
        let mapped_count = nodes.iter().filter(|n| n.bone_tag.is_some()).count();

        ui.group(|ui| {
            ui.horizontal(|ui| {
                ui.label("Частота потока:");
                ui.label(RichText::new(format!("{fps:.1} Гц")).color(Color32::GREEN).strong());
                ui.separator();
                ui.label("Датчиков на теле:");
                ui.label(RichText::new(format!("{active_count}")).strong());
            });
            ui.horizontal(|ui| {
                ui.label("Назначено на скелет:");
                let col = if mapped_count > 0 { Color32::GREEN } else { Color32::YELLOW };
                ui.label(RichText::new(format!("{mapped_count} из {active_count}")).color(col).strong());
            });
        });

        ui.add_space(6.0);
        ui.group(|ui| {
            ui.label(RichText::new("Быстрое выравнивание позы (Boresight):").strong());
            ui.label("Если манекен стоит криво относительно оператора, встаньте в N-позу и нажмите выравнивание:");
            ui.add_space(2.0);

            let is_running = shared.user_wants_connected.load(Ordering::Relaxed);
            if ui.add_enabled(is_running, egui::Button::new(RichText::new("🎯 СБРОСИТЬ В N-ПОЗУ").strong())).clicked() {
                shared.request_npose_alignment.store(true, Ordering::SeqCst);
            }
        });

        ui.add_space(8.0);
        ui.heading("Текущая кинематическая активность звеньев");

        ui.group(|ui| {
            if nodes.is_empty() {
                ui.label(RichText::new("Нет активных сигналов в потоке").color(Color32::GRAY));
            } else {
                egui::ScrollArea::vertical().max_height(320.0).show(ui, |ui| {
                    egui::Grid::new("tracking_nodes_grid")
                        .num_columns(3)
                        .spacing([12.0, 4.0])
                        .show(ui, |ui| {
                            ui.label(RichText::new("Сегмент").strong());
                            ui.label(RichText::new("Канал").strong());
                            ui.label(RichText::new("Энергия").strong());
                            ui.end_row();

                            for node in &nodes {
                                let bone_name = node.bone_tag.as_deref().unwrap_or("Не назначен");
                                let key = format!("0x{:02X}:{:X}:0x{:02X}", node.mux_address, node.channel, node.sensor_address);

                                ui.label(RichText::new(bone_name).color(if node.bone_tag.is_some() { Color32::LIGHT_BLUE } else { Color32::GRAY }));
                                ui.label(RichText::new(key).monospace());

                                let energy_norm = (node.peak_energy / 8.0).clamp(0.0, 1.0);
                                let col = if energy_norm > 0.4 { Color32::GREEN } else { Color32::DARK_GRAY };
                                ui.add(ProgressBar::new(energy_norm).fill(col).desired_width(70.0));
                                ui.end_row();
                            }
                        });
                });
            }
        });
    }
}