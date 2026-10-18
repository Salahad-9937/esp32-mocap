//! Мастер пошаговой статической калибровки гироскопов (Zero-Rate Bias).

use crate::state::{NodeTelemetry, SharedStateHandle};
use egui::{Color32, ProgressBar, RichText, Ui};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Профиль смещений нуля для всех узлов.
#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct CalibrationProfile {
    pub gyro_bias: HashMap<String, [f32; 3]>,
}

/// Мастер калибровки статического дрейфа нуля гироскопов.
pub struct CalibrationView {
    pub profile: CalibrationProfile,
    pub is_calibrating: bool,
    pub collected_samples: usize,
    pub target_samples: usize,
    pub accumulator: HashMap<String, Vec<[f32; 3]>>,
    pub status_text: String,
}

impl Default for CalibrationView {
    fn default() -> Self {
        Self {
            profile: CalibrationProfile::default(),
            is_calibrating: false,
            collected_samples: 0,
            target_samples: 200,
            accumulator: HashMap::new(),
            status_text: "Готов к запуску калибровки.".to_string(),
        }
    }
}

impl CalibrationView {
    pub fn update(&mut self, nodes: &[NodeTelemetry]) {
        if !self.is_calibrating || nodes.is_empty() {
            return;
        }

        for node in nodes {
            let key = format!("0x{:02X}:{}", node.mux_address, node.channel);
            let gyro_raw = [
                node.raw.gx as f32,
                node.raw.gy as f32,
                node.raw.gz as f32,
            ];
            self.accumulator.entry(key).or_default().push(gyro_raw);
        }

        self.collected_samples += 1;

        if self.collected_samples >= self.target_samples {
            self.finalize_calibration();
        }
    }

    fn finalize_calibration(&mut self) {
        self.is_calibrating = false;
        let mut success_count = 0;

        for (key, samples) in &self.accumulator {
            if samples.is_empty() {
                continue;
            }
            let n = samples.len() as f32;
            let mut mean = [0.0f32; 3];
            for s in samples {
                mean[0] += s[0];
                mean[1] += s[1];
                mean[2] += s[2];
            }
            mean[0] /= n;
            mean[1] /= n;
            mean[2] /= n;

            let mut var = [0.0f32; 3];
            for s in samples {
                var[0] += (s[0] - mean[0]).powi(2);
                var[1] += (s[1] - mean[1]).powi(2);
                var[2] += (s[2] - mean[2]).powi(2);
            }
            let max_std_dev = (var[0].max(var[1]).max(var[2]) / n).sqrt();

            if max_std_dev < 120.0 {
                self.profile.gyro_bias.insert(key.clone(), mean);
                success_count += 1;
            }
        }

        self.status_text = format!("Калибровка завершена. Откалибровано узлов: {success_count}.");
    }

    pub fn show(&mut self, ui: &mut Ui, shared: &SharedStateHandle) {
        ui.heading("Калибровка смещения нуля (Zero-Rate Bias)");
        ui.add_space(4.0);

        let nodes = shared.nodes.lock().map(|g| g.clone()).unwrap_or_default();
        self.update(&nodes);

        ui.group(|ui| {
            ui.label(RichText::new("Регламент калибровки:").strong());
            ui.label("1. Зафиксируйте датчики в неподвижном положении.");
            ui.label("2. Запустите сбор 200 калибровочных кадров.");
            ui.label("3. Смещения нуля будут автоматически учтены в расчетах.");
        });

        ui.add_space(8.0);

        if self.is_calibrating {
            let progress = (self.collected_samples as f32) / (self.target_samples as f32);
            ui.add(ProgressBar::new(progress).show_percentage().animate(true));
            ui.label(format!("Сбор: {} / {}", self.collected_samples, self.target_samples));
        } else {
            let btn = ui.add_enabled(
                !nodes.is_empty(),
                egui::Button::new(RichText::new("▶ ЗАПУСТИТЬ КАЛИБРОВКУ").strong()),
            );

            if btn.clicked() {
                self.is_calibrating = true;
                self.collected_samples = 0;
                self.accumulator.clear();
                self.status_text = "Сбор калибровочной выборки...".to_string();
            }
        }

        ui.add_space(4.0);
        ui.colored_label(Color32::YELLOW, &self.status_text);

        ui.add_space(8.0);
        ui.heading("Рассчитанные смещения нуля покоя (LSB)");

        ui.group(|ui| {
            egui::Grid::new("bias_table_grid")
                .num_columns(4)
                .spacing([20.0, 4.0])
                .show(ui, |ui| {
                    ui.label(RichText::new("Канал").strong());
                    ui.label(RichText::new("Bias X").strong());
                    ui.label(RichText::new("Bias Y").strong());
                    ui.label(RichText::new("Bias Z").strong());
                    ui.end_row();

                    for (key, bias) in &self.profile.gyro_bias {
                        ui.label(RichText::new(key).monospace());
                        ui.label(format!("{:+.1}", bias[0]));
                        ui.label(format!("{:+.1}", bias[1]));
                        ui.label(format!("{:+.1}", bias[2]));
                        ui.end_row();
                    }
                });
        });
    }
}