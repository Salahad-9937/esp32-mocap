//! # Мастер пошаговой статической калибровки гироскопов (`calibration`)

use crate::state::SharedStateHandle;
use egui::{Color32, ProgressBar, RichText, Ui};
use std::sync::atomic::Ordering;

/// Представление мастера калибровки смещения нуля гироскопов.
#[derive(Debug, Default)]
pub struct CalibrationView {
    pub status_text: String,
}

impl CalibrationView {
    pub fn show(&mut self, ui: &mut Ui, shared: &SharedStateHandle) {
        ui.heading("Калибровка смещения нуля (Zero-Rate Bias)");
        ui.add_space(4.0);

        let is_running = shared.user_wants_connected.load(Ordering::Relaxed);
        let progress_opt = shared.calibration_progress.lock().map(|p| *p).unwrap_or(None);
        let biases = shared.active_biases.lock().map(|b| b.clone()).unwrap_or_default();

        ui.group(|ui| {
            ui.label(RichText::new("Регламент калибровки:").strong());
            ui.label("1. Зафиксируйте датчики в неподвижном положении.");
            ui.label("2. Запустите сбор 200 калибровочных кадров через MocapProcessor.");
            ui.label("3. Смещения нуля будут автоматически вычитаться ядром из сигналов.");
        });

        ui.add_space(8.0);

        if let Some(progress) = progress_opt {
            ui.add(ProgressBar::new(progress).show_percentage().animate(true));
            ui.label("Сбор калибровочной выборки и расчет дисперсии шума...");
            self.status_text = "Выполняется калибровка покоя...".to_string();
        } else {
            let btn = ui.add_enabled(
                is_running,
                egui::Button::new(RichText::new("▶ ЗАПУСТИТЬ КАЛИБРОВКУ").strong()),
            );

            if btn.clicked() {
                shared.request_calibration.store(true, Ordering::SeqCst);
                self.status_text = "Запрос калибровки отправлен в вычислительное ядро.".to_string();
            }
        }

        ui.add_space(4.0);
        ui.colored_label(Color32::YELLOW, &self.status_text);

        ui.add_space(8.0);
        ui.heading("Активные смещения нуля гироскопов ядра (LSB)");

        ui.group(|ui| {
            if biases.is_empty() {
                ui.label(RichText::new("Калибровка еще не проводилась (смещения: 0.0)").color(Color32::GRAY));
            } else {
                egui::Grid::new("core_bias_table_grid")
                    .num_columns(4)
                    .spacing([20.0, 4.0])
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
            }
        });
    }
}