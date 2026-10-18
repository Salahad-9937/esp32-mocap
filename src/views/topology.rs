//! Интерактивная матрица оборудования шины I2C (мультиплексоры 0x70..0x77).

use crate::state::SharedStateHandle;
use egui::{Color32, RichText, Stroke, Ui};

/// Матричный виджет шины I2C (8 коммутаторов x 8 подканалов = 64 точки).
#[derive(Debug, Default)]
pub struct TopologyView;

impl TopologyView {
    pub fn show(&mut self, ui: &mut Ui, shared: &SharedStateHandle, endpoint: &str) {
        ui.heading("Топология оборудования I2C");
        ui.label("Автосканирование шины:\n8 коммутаторов PCA9548A × 8 подканалов (0x68)");
        ui.add_space(4.0);

        let active_nodes = shared.nodes.lock().map(|g| g.clone()).unwrap_or_default();
        let total_active = active_nodes.len();
        let frame_size = 7 + total_active * 12;
        let fps = shared.calculated_fps.lock().map(|f| *f).unwrap_or(0.0);

        let matrix_w = ui
            .data(|d| d.get_temp::<f32>(egui::Id::new("i2c_matrix_group_width")))
            .unwrap_or(340.0);

        ui.group(|ui| {
            ui.set_width(matrix_w);
            ui.horizontal(|ui| {
                ui.label("Эндпоинт:");
                ui.label(RichText::new(endpoint).monospace().strong());
                ui.separator();
                ui.label("Частота:");
                ui.label(RichText::new(format!("{fps:.1} Гц")).color(Color32::GREEN).strong());
            });
            ui.horizontal(|ui| {
                ui.label("Активных датчиков:");
                ui.label(RichText::new(format!("{total_active}")).color(Color32::GREEN).strong());
                ui.separator();
                ui.label("Размер кадра:");
                ui.label(RichText::new(format!("{frame_size} Б")).monospace());
            });
        });

        ui.add_space(8.0);

        // Исходный полноразмерный формат ячеек: все 8 каналов (C0..C7) гарантированно умещаются при ширине панели 380px
        let cell_size = egui::vec2(34.0_f32, 26.0_f32);

        let matrix_resp = ui.group(|ui| {
            egui::Grid::new("i2c_topology_matrix")
                .spacing([4.0_f32, 4.0_f32])
                .show(ui, |ui| {
                    ui.label(RichText::new("MUX").monospace().strong());
                    for ch in 0..8 {
                        ui.label(RichText::new(format!("C{ch}")).monospace().strong());
                    }
                    ui.end_row();

                    for m in 0..8 {
                        let mux_addr = 0x70 + m;
                        ui.label(RichText::new(format!("0x{mux_addr:02X}")).monospace());

                        for ch in 0..8 {
                            let is_active = active_nodes
                                .iter()
                                .any(|n| n.mux_address == mux_addr && n.channel == ch);

                            let (bg_color, stroke_color, text) = if is_active {
                                (
                                    Color32::from_rgb(25, 95, 45),
                                    Color32::from_rgb(50, 210, 85),
                                    "OK",
                                )
                            } else {
                                (
                                    Color32::from_rgb(32, 35, 44),
                                    Color32::from_rgb(55, 60, 72),
                                    "·",
                                )
                            };

                            let (rect, resp) = ui.allocate_exact_size(cell_size, egui::Sense::hover());
                            ui.painter().rect(rect, 3.0_f32, bg_color, Stroke::new(1.0_f32, stroke_color));
                            ui.painter().text(
                                rect.center(),
                                egui::Align2::CENTER_CENTER,
                                text,
                                egui::FontId::monospace(10.5),
                                if is_active { Color32::WHITE } else { Color32::GRAY },
                            );

                            resp.on_hover_ui(|ui| {
                                ui.label(format!("Коммутатор: 0x{mux_addr:02X}"));
                                ui.label(format!("Подканал: CH{ch}"));
                                ui.label(format!("Статус: {}", if is_active { "BMI160 обнаружен" } else { "Пусто" }));
                            });
                        }
                        ui.end_row();
                    }
                });
            ui.min_rect().width()
        });
        ui.data_mut(|d| d.insert_temp(egui::Id::new("i2c_matrix_group_width"), matrix_resp.inner));
    }
}