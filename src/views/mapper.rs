//! Интерактивный конфигуратор привязки физических каналов к сегментам скелета.

use crate::state::SharedStateHandle;
use egui::{Color32, ProgressBar, RichText, Ui};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::Path;
use std::time::{Duration, Instant};

pub const TARGET_BONES: [&str; 17] = [
    "Pelvis", "Spine", "Thorax", "Neck", "Head",
    "LeftShoulder", "LeftUpperArm", "LeftForearm",
    "RightShoulder", "RightUpperArm", "RightForearm",
    "LeftThigh", "LeftShin", "LeftFoot",
    "RightThigh", "RightShin", "RightFoot",
];

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct MappingProfile {
    pub mappings: HashMap<String, String>,
}

pub struct MapperView {
    pub profile: MappingProfile,
    pub status_message: String,
    pub detecting_for_bone: Option<String>,
    pub detection_start: Option<Instant>,
    pub accumulated_energies: HashMap<String, f32>,
}

impl Default for MapperView {
    fn default() -> Self {
        Self {
            profile: MappingProfile::default(),
            status_message: String::new(),
            detecting_for_bone: None,
            detection_start: None,
            accumulated_energies: HashMap::new(),
        }
    }
}

impl MapperView {
    pub fn load_profile(&mut self, path: &Path) {
        if let Ok(content) = fs::read_to_string(path) {
            if let Ok(deserialized) = toml::from_str::<MappingProfile>(&content) {
                self.profile = deserialized;
            }
        }
    }

    pub fn show(&mut self, ui: &mut Ui, shared: &SharedStateHandle, profile_path: &Path) {
        ui.heading("Маппинг датчиков на скелет");
        ui.label("Назначьте датчик через меню выбора или используйте накопление движения.");
        ui.add_space(4.0);

        let nodes = shared.nodes.lock().map(|g| g.clone()).unwrap_or_default();

        if let Some(target_bone) = &self.detecting_for_bone {
            let start = self.detection_start.get_or_insert_with(Instant::now);
            let elapsed = start.elapsed();
            let total_dur = Duration::from_secs(2);

            for node in &nodes {
                let key = format!("0x{:02X}:{}", node.mux_address, node.channel);
                *self.accumulated_energies.entry(key).or_default() += node.peak_energy;
            }

            ui.group(|ui| {
                ui.label(RichText::new(format!("Пошевелите сегментом: {target_bone}")).color(Color32::YELLOW).strong());
                let progress = (elapsed.as_secs_f32() / total_dur.as_secs_f32()).clamp(0.0, 1.0);
                ui.add(ProgressBar::new(progress).show_percentage());
            });

            if elapsed >= total_dur {
                let mut best_channel = None;
                let mut max_accum = 0.80f32;

                for (key, &accum) in &self.accumulated_energies {
                    if accum > max_accum {
                        max_accum = accum;
                        best_channel = Some(key.clone());
                    }
                }

                if let Some(ch) = best_channel {
                    self.profile.mappings.insert(ch.clone(), target_bone.clone());
                    self.status_message = format!("Успешно назначен узел {ch} -> {target_bone}");
                } else {
                    self.status_message = "Движение не обнаружено. Повторите попытку.".to_string();
                }

                self.detecting_for_bone = None;
                self.detection_start = None;
                self.accumulated_energies.clear();
            }
        }

        ui.add_space(6.0);
        ui.heading("Обнаруженные активные датчики");

        ui.group(|ui| {
            if nodes.is_empty() {
                ui.label(RichText::new("Нет активных датчиков в потоке").color(Color32::GRAY));
            } else {
                egui::Grid::new("active_sensors_mapper_grid")
                    .num_columns(4)
                    .spacing([8.0, 6.0])
                    .show(ui, |ui| {
                        ui.label(RichText::new("Канал").strong());
                        ui.label(RichText::new("Активность").strong());
                        ui.label(RichText::new("Назначенная кость").strong());
                        ui.label("Действие");
                        ui.end_row();

                        for (i, node) in nodes.iter().enumerate() {
                            let key = format!("0x{:02X}:{}", node.mux_address, node.channel);
                            ui.label(RichText::new(format!("#{i} [{key}]")).monospace());

                            let norm_energy = (node.peak_energy / 8.0).clamp(0.0, 1.0);
                            let bar_color = if norm_energy > 0.4 { Color32::GREEN } else { Color32::DARK_GRAY };
                            ui.add(ProgressBar::new(norm_energy).fill(bar_color).desired_width(55.0));

                            let current_bone = self.profile.mappings.get(&key).cloned().unwrap_or_else(|| "Не назначен".to_string());
                            ui.label(RichText::new(&current_bone).color(if current_bone == "Не назначен" { Color32::GRAY } else { Color32::LIGHT_BLUE }));

                            ui.menu_button("Выбрать ▼", |ui| {
                                for &bone in &TARGET_BONES {
                                    if ui.button(bone).clicked() {
                                        self.profile.mappings.insert(key.clone(), bone.to_string());
                                        ui.close_menu();
                                    }
                                }
                                ui.separator();
                                if ui.button("Снять привязку").clicked() {
                                    self.profile.mappings.remove(&key);
                                    ui.close_menu();
                                }
                            });

                            ui.end_row();
                        }
                    });
            }
        });

        ui.add_space(8.0);
        ui.heading("Интерактивный список сегментов тела");

        ui.group(|ui| {
            egui::Grid::new("bones_assignment_grid")
                .num_columns(3)
                .spacing([8.0, 5.0])
                .show(ui, |ui| {
                    for &bone in &TARGET_BONES {
                        let assigned_key = self.profile.mappings.iter().find_map(|(k, v)| if v == bone { Some(k) } else { None });

                        ui.label(RichText::new(bone).strong());

                        if let Some(key) = assigned_key {
                            ui.colored_label(Color32::GREEN, format!("[{key}]"));
                        } else {
                            ui.colored_label(Color32::GRAY, "—");
                        }

                        if self.detecting_for_bone.is_none() {
                            if ui.button("🎯 Засечь").clicked() {
                                self.detecting_for_bone = Some(bone.to_string());
                                self.detection_start = Some(Instant::now());
                                self.accumulated_energies.clear();
                            }
                        } else {
                            ui.label("");
                        }

                        ui.end_row();
                    }
                });
        });

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            if ui.button("💾 Сохранить в TOML").clicked() {
                if let Ok(serialized) = toml::to_string_pretty(&self.profile) {
                    let _ = fs::write(profile_path, serialized);
                    self.status_message = "Профиль сохранен.".to_string();
                }
            }

            if ui.button("📂 Загрузить").clicked() {
                self.load_profile(profile_path);
                self.status_message = "Профиль загружен.".to_string();
            }

            if !self.status_message.is_empty() {
                ui.colored_label(Color32::YELLOW, &self.status_message);
            }
        });
    }
}