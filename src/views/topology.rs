//! Интерактивная матрица оборудования шины I2C (мультиплексоры 0x70..0x77, адреса 0x68 и 0x69).

use crate::state::SharedStateHandle;
use egui::{Color32, RichText, Stroke, Ui};
use std::sync::atomic::Ordering;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TopologyAddressTab {
    #[default]
    All,
    Addr0x68,
    Addr0x69,
}

/// Матричный виджет шины I2C (8 коммутаторов x 8 подканалов x 2 адреса = 128 точек).
#[derive(Debug, Default)]
pub struct TopologyView {
    pub address_tab: TopologyAddressTab,
}

impl TopologyView {
    pub fn show(&mut self, ui: &mut Ui, shared: &SharedStateHandle, endpoint: &str) {
        ui.heading("Топология оборудования I2C");
        ui.label("Шина I2C: 8 коммутаторов PCA9548A × 8 подканалов × 2 адреса (0x68/0x69)");
        ui.add_space(4.0);

        let active_nodes = shared.nodes.lock().map(|g| g.clone()).unwrap_or_default();
        let total_active = active_nodes.len();
        let frame_size = 8 + total_active * 15;
        let fps = shared.calculated_fps.lock().map(|f| *f).unwrap_or(0.0);
        let mux_mask = shared.detected_mux_mask.load(Ordering::Relaxed);

        let count_0x68 = active_nodes.iter().filter(|n| n.sensor_address == 0x68).count();
        let count_0x69 = active_nodes.iter().filter(|n| n.sensor_address == 0x69).count();

        let mut found_mux_count = 0u8;
        for m in 0..8 {
            let mux_addr = 0x70 + m;
            let exists = (mux_mask & (1 << m)) != 0 || active_nodes.iter().any(|n| n.mux_address == mux_addr);
            if exists {
                found_mux_count += 1;
            }
        }

        let matrix_w = ui
            .data(|d| d.get_temp::<f32>(egui::Id::new("i2c_matrix_group_width")))
            .unwrap_or(360.0);

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
                ui.label("Коммутаторов:");
                ui.label(RichText::new(format!("{found_mux_count} из 8")).color(Color32::GREEN).strong());
                ui.separator();
                ui.label("Размер кадра:");
                ui.label(RichText::new(format!("{frame_size} Б")).monospace());
            });
            ui.horizontal(|ui| {
                ui.label("Активных датчиков:");
                ui.label(RichText::new(format!("{total_active}")).color(Color32::GREEN).strong());
                ui.label(RichText::new(format!("(0x68: {count_0x68} | 0x69: {count_0x69})")).size(11.0).color(Color32::GRAY));
            });
        });

        ui.add_space(6.0);
        ui.heading("Коммутаторы шины (PCA9548A 0x70 .. 0x77)");

        // Индикатор статуса обнаружения мультиплексоров, стилизованный аналогично датчикам
        let mux_bar_resp = ui.group(|ui| {
            ui.horizontal(|ui| {
                for m in 0..8 {
                    let mux_addr = 0x70 + m;
                    let exists = (mux_mask & (1 << m)) != 0 || active_nodes.iter().any(|n| n.mux_address == mux_addr);

                    let (bg_color, stroke_color, text_color, status_text) = if exists {
                        (
                            Color32::from_rgb(25, 95, 45),
                            Color32::from_rgb(50, 210, 85),
                            Color32::WHITE,
                            "Подключен (ACK)",
                        )
                    } else {
                        (
                            Color32::from_rgb(32, 35, 44),
                            Color32::from_rgb(55, 60, 72),
                            Color32::GRAY,
                            "Не обнаружен",
                        )
                    };

                    let (rect, resp) = ui.allocate_exact_size(egui::vec2(38.0, 24.0), egui::Sense::hover());
                    ui.painter().rect(rect, 3.0_f32, bg_color, Stroke::new(1.0_f32, stroke_color));
                    ui.painter().text(
                        rect.center(),
                        egui::Align2::CENTER_CENTER,
                        format!("{mux_addr:02X}"),
                        egui::FontId::monospace(10.5),
                        text_color,
                    );

                    resp.on_hover_ui(|ui| {
                        ui.label(RichText::new(format!("Коммутатор: 0x{mux_addr:02X}")).strong());
                        ui.label(format!("Битовый индекс: MUX #{m}"));
                        ui.label(format!("Статус: {status_text}"));
                    });
                }
            });
            ui.min_rect().width()
        });
        ui.data_mut(|d| d.insert_temp(egui::Id::new("i2c_matrix_group_width"), mux_bar_resp.inner));

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.label(RichText::new("Адрес SA0:").strong());
            for (tab, label) in [
                (TopologyAddressTab::All, "Все адреса"),
                (TopologyAddressTab::Addr0x68, "SA0 = GND (0x68)"),
                (TopologyAddressTab::Addr0x69, "SA0 = VIN (0x69)"),
            ] {
                if ui.selectable_label(self.address_tab == tab, label).clicked() {
                    self.address_tab = tab;
                }
            }
        });

        ui.add_space(6.0);

        match self.address_tab {
            TopologyAddressTab::All => {
                ui.label(RichText::new("1. Адресное пространство SA0 = GND (0x68)").color(Color32::KHAKI).strong());
                self.render_sensor_matrix(ui, &active_nodes, mux_mask, 0x68);

                ui.add_space(10.0);
                ui.label(RichText::new("2. Адресное пространство SA0 = VIN (0x69)").color(Color32::KHAKI).strong());
                self.render_sensor_matrix(ui, &active_nodes, mux_mask, 0x69);
            }
            TopologyAddressTab::Addr0x68 => {
                ui.label(RichText::new("Адресное пространство SA0 = GND (0x68)").color(Color32::KHAKI).strong());
                self.render_sensor_matrix(ui, &active_nodes, mux_mask, 0x68);
            }
            TopologyAddressTab::Addr0x69 => {
                ui.label(RichText::new("Адресное пространство SA0 = VIN (0x69)").color(Color32::KHAKI).strong());
                self.render_sensor_matrix(ui, &active_nodes, mux_mask, 0x69);
            }
        }
    }

    fn render_sensor_matrix(
        &self,
        ui: &mut Ui,
        active_nodes: &[crate::state::NodeTelemetry],
        mux_mask: u8,
        target_sensor_addr: u8,
    ) {
        let cell_size = egui::vec2(34.0_f32, 26.0_f32);
        let mux_cell_size = egui::vec2(42.0_f32, 26.0_f32);

        ui.group(|ui| {
            egui::Grid::new(format!("i2c_matrix_grid_0x{target_sensor_addr:02X}"))
                .spacing([4.0_f32, 4.0_f32])
                .show(ui, |ui| {
                    ui.label(RichText::new("MUX").monospace().strong());
                    for ch in 0..8 {
                        ui.label(RichText::new(format!("C{ch}")).monospace().strong());
                    }
                    ui.end_row();

                    for m in 0..8 {
                        let mux_addr = 0x70 + m;
                        let mux_exists = (mux_mask & (1 << m)) != 0 || active_nodes.iter().any(|n| n.mux_address == mux_addr);

                        // Отображение коммутатора в виде визуального статусного блока
                        let (m_bg, m_stroke, m_fg, m_stat) = if mux_exists {
                            (
                                Color32::from_rgb(25, 95, 45),
                                Color32::from_rgb(50, 210, 85),
                                Color32::WHITE,
                                "Подключен (ACK)",
                            )
                        } else {
                            (
                                Color32::from_rgb(32, 35, 44),
                                Color32::from_rgb(55, 60, 72),
                                Color32::GRAY,
                                "Не обнаружен",
                            )
                        };

                        let (m_rect, m_resp) = ui.allocate_exact_size(mux_cell_size, egui::Sense::hover());
                        ui.painter().rect(m_rect, 3.0_f32, m_bg, Stroke::new(1.0_f32, m_stroke));
                        ui.painter().text(
                            m_rect.center(),
                            egui::Align2::CENTER_CENTER,
                            format!("0x{mux_addr:02X}"),
                            egui::FontId::monospace(10.0),
                            m_fg,
                        );

                        m_resp.on_hover_ui(|ui| {
                            ui.label(RichText::new(format!("Коммутатор: 0x{mux_addr:02X}")).strong());
                            ui.label(format!("Статус: {m_stat}"));
                        });

                        for ch in 0..8 {
                            let is_active = active_nodes.iter().any(|n| {
                                n.mux_address == mux_addr && n.channel == ch && n.sensor_address == target_sensor_addr
                            });

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
                                ui.label(format!("Адрес BMI160: 0x{target_sensor_addr:02X} ({})", if target_sensor_addr == 0x68 { "SA0=GND" } else { "SA0=VIN" }));
                                ui.label(format!("Статус: {}", if is_active { "BMI160 обнаружен и активен" } else { "Пусто" }));
                            });
                        }
                        ui.end_row();
                    }
                });
        });
    }
}