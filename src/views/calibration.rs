//! # Двухфазный мастер статической и позной калибровки (`calibration`)

use crate::state::SharedStateHandle;
use egui::{Color32, ProgressBar, RichText, Ui};
use std::sync::atomic::Ordering;

#[derive(Debug, Default)]
pub struct CalibrationView {
    pub status_text: String,
}

impl CalibrationView {
    pub fn show(&mut self, ui: &mut Ui, shared: &SharedStateHandle) {
        ui.heading("Мастер комплексной калибровки костюма");
        ui.label("Двухфазный регламент ISB: устранение дрейфа гироскопов и выравнивание соосности позы.");
        ui.add_space(4.0);

        let is_running = shared.user_wants_connected.load(Ordering::Relaxed);
        let progress_opt = shared.calibration_progress.lock().map(|p| *p).unwrap_or(None);
        let biases = shared.active_biases.lock().map(|b| b.clone()).unwrap_or_default();

        // ── Фаза 1: Статическая калибровка нуля гироскопов ──────────────────
        ui.group(|ui| {
            ui.label(RichText::new("ФАЗА 1: Смещение нуля гироскопов (Zero-Rate Bias)").strong().color(Color32::from_rgb(100, 200, 255)));
            ui.label("1. Сохраняйте полную неподвижность датчиков на столе или теле.");
            ui.label("2. Нажмите запуск: ядро соберет 200 калиброванных срезов шума.");
            ui.add_space(4.0);

            if let Some(progress) = progress_opt {
                ui.add(ProgressBar::new(progress).show_percentage().animate(true));
                ui.colored_label(Color32::YELLOW, "Накопление выборки покоя и оценка дисперсии...");
            } else {
                let btn = ui.add_enabled(
                    is_running,
                    egui::Button::new(RichText::new("▶ 1. ЗАПУСТИТЬ КАЛИБРОВКУ ПОКОЯ").strong()),
                );
                if btn.clicked() {
                    shared.request_calibration.store(true, Ordering::SeqCst);
                    self.status_text = "Запрос сбора шума покоя отправлен в ядро.".to_string();
                }
            }
        });

        ui.add_space(8.0);

        // ── Фаза 2: Выравнивание позы скелета (Boresight Alignment) ────────
        ui.group(|ui| {
            ui.label(RichText::new("ФАЗА 2: Выравнивание соосности сенсоров (Boresight N-Pose)").strong().color(Color32::from_rgb(100, 255, 180)));
            ui.label("1. Встаньте прямо в N-позу: руки вдоль тела, ладони к бедрам, взгляд строго вперед.");
            ui.label("2. Нажмите фиксацию: ядро рассчитает кватернион выравнивания q_align = q_sensor⁻¹ для всех костей.");
            ui.add_space(4.0);

            let btn_npose = ui.add_enabled(
                is_running && progress_opt.is_none(),
                egui::Button::new(RichText::new("🎯 2. ЗАФИКСИРОВАТЬ N-ПОЗУ (ВЫРАВНИВАНИЕ)").strong()),
            );
            if btn_npose.clicked() {
                shared.request_npose_alignment.store(true, Ordering::SeqCst);
                self.status_text = "Поза N-Pose зафиксирована. Кватернионы выравнивания применены к скелету.".to_string();
            }
        });

        if !self.status_text.is_empty() {
            ui.add_space(4.0);
            ui.colored_label(Color32::YELLOW, &self.status_text);
        }

        ui.add_space(8.0);
        ui.heading("Активные смещения нуля гироскопов ядра (LSB)");

        ui.group(|ui| {
            if biases.is_empty() {
                ui.label(RichText::new("Смещения еще не рассчитывались (по умолчанию: 0.0)").color(Color32::GRAY));
            } else {
                egui::ScrollArea::vertical().max_height(220.0).show(ui, |ui| {
                    egui::Grid::new("core_bias_table_grid")
                        .num_columns(4)
                        .spacing([18.0, 4.0])
                        .show(ui, |ui| {
                            ui.label(RichText::new("Узел #").strong());
                            ui.label(RichText::new("Bias X").strong());
                            ui.label(RichText::new("Bias Y").strong());
                            ui.label(RichText::new("Bias Z").strong());
                            ui.end_row();

                            for (i, bias) in biases.iter().enumerate() {
                                ui.label(RichText::new(format!("#{i}")).monospace());
                                ui.label(format!("{:+.1}", bias.gyro.x));
                                ui.label(format!("{:+.1}", bias.gyro.y));
                                ui.label(format!("{:+.1}", bias.gyro.z));
                                ui.end_row();
                            }
                        });
                });
            }
        });
    }
}