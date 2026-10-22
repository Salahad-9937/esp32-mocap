//! # Интерактивный конфигуратор привязки физических каналов к скелету (`mapper`)

use crate::state::SharedStateHandle;
use egui::{Color32, ProgressBar, RichText, Ui};
use mocap_core::mapping::MappingProfile;
pub use mocap_model::TARGET_BONES;
use std::collections::HashMap;
use std::path::Path;
use std::time::{Duration, Instant};

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
    fn migrate_legacy_bone_name(name: &str) -> Option<&'static str> {
        if let Some(canonical) = TARGET_BONES.iter().copied().find(|&b| b == name) {
            return Some(canonical);
        }
        match name {
            "HeadTop" | "HeadFront" | "HeadOcciput" => Some("Head"),
            "Neck" | "SpineCervical" => Some("Neck"),
            "ThoraxUpper" => Some("SpineT3"),
            "Thorax" => Some("Sternum"),
            "SpineThoracic" => Some("SpineT7"),
            "Spine" => Some("SpineL3"),
            "Pelvis" | "PelvisLeft" | "PelvisRight" => Some("Sacrum"),
            "LeftShoulder" => Some("LeftShoulder"),
            "RightShoulder" => Some("RightShoulder"),
            "LeftUpperArm" => Some("LeftUpperArmProx"),
            "LeftUpperArmLower" => Some("LeftUpperArmDist"),
            "LeftForearm" => Some("LeftForearmProx"),
            "LeftForearmWrist" => Some("LeftForearmDist"),
            "RightUpperArm" => Some("RightUpperArmProx"),
            "RightUpperArmLower" => Some("RightUpperArmDist"),
            "RightForearm" => Some("RightForearmProx"),
            "RightForearmWrist" => Some("RightForearmDist"),
            "LeftHand" => Some("LeftHand"),
            "RightHand" => Some("RightHand"),
            "LeftThigh" => Some("LeftThighProx"),
            "LeftThighLower" | "LeftKneeSensor" => Some("LeftThighDist"),
            "LeftShin" => Some("LeftShankProx"),
            "LeftShinLower" | "LeftAnkleSensor" => Some("LeftShankDist"),
            "RightThigh" => Some("RightThighProx"),
            "RightThighLower" | "RightKneeSensor" => Some("RightThighDist"),
            "RightShin" => Some("RightShankProx"),
            "RightShinLower" | "RightAnkleSensor" => Some("RightShankDist"),
            "LeftFoot" => Some("LeftFoot"),
            "LeftToe" => Some("LeftToe"),
            "RightFoot" => Some("RightFoot"),
            "RightToe" => Some("RightToe"),
            "LeftThumbProximal" => Some("LeftThumbProx"),
            "LeftThumbDistal" => Some("LeftThumbDist"),
            "LeftIndexProximal" => Some("LeftIndexProx"),
            "LeftIndexDistal" | "LeftIndexTip" => Some("LeftIndexDist"),
            "LeftMiddleProximal" => Some("LeftMiddleProx"),
            "LeftMiddleDistal" | "LeftMiddleTip" => Some("LeftMiddleDist"),
            "LeftRingProximal" => Some("LeftRingProx"),
            "LeftRingDistal" => Some("LeftRingDist"),
            "LeftPinkyProximal" => Some("LeftPinkyProx"),
            "LeftPinkyDistal" => Some("LeftPinkyDist"),
            "RightThumbProximal" => Some("RightThumbProx"),
            "RightThumbDistal" => Some("RightThumbDist"),
            "RightIndexProximal" => Some("RightIndexProx"),
            "RightIndexDistal" | "RightIndexTip" => Some("RightIndexDist"),
            "RightMiddleProximal" => Some("RightMiddleProx"),
            "RightMiddleDistal" | "RightMiddleTip" => Some("RightMiddleDist"),
            "RightRingProximal" => Some("RightRingProx"),
            "RightRingDistal" => Some("RightRingDist"),
            "RightPinkyProximal" => Some("RightPinkyProx"),
            "RightPinkyDistal" => Some("RightPinkyDist"),
            _ => None,
        }
    }

    pub fn load_profile(&mut self, path: &Path) {
        if let Ok(p) = MappingProfile::load_from_file(path) {
            let mut migrated = HashMap::new();
            for (channel, legacy_bone) in p.mappings {
                if let Some(canonical) = Self::migrate_legacy_bone_name(&legacy_bone) {
                    migrated.insert(channel, canonical.to_string());
                }
            }
            self.profile = MappingProfile {
                name: p.name,
                mappings: migrated,
            };
        }
    }

    pub fn show(&mut self, ui: &mut Ui, shared: &SharedStateHandle, profile_path: &Path) {
        ui.heading("Маппинг датчиков на скелет (64 узла)");
        ui.label("Назначьте физический канал коммутатора на сегмент скелета.");
        ui.add_space(4.0);

        let nodes = shared.nodes.lock().map(|g| g.clone()).unwrap_or_default();

        if let Some(target_bone) = &self.detecting_for_bone {
            let start = self.detection_start.get_or_insert_with(Instant::now);
            let elapsed = start.elapsed();
            let total_dur = Duration::from_secs(2);

            for node in &nodes {
                let key = MappingProfile::channel_key(node.mux_address, node.channel);
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
                    if let Ok(mut prof) = shared.mapping_profile.lock() {
                        *prof = self.profile.clone();
                    }
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
                        ui.label(RichText::new("Назначенный сегмент").strong());
                        ui.label("Действие");
                        ui.end_row();

                        for (i, node) in nodes.iter().enumerate() {
                            let key = MappingProfile::channel_key(node.mux_address, node.channel);
                            ui.label(RichText::new(format!("#{i} [{key}]")).monospace());

                            let norm_energy = (node.peak_energy / 8.0).clamp(0.0, 1.0);
                            let bar_color = if norm_energy > 0.4 { Color32::GREEN } else { Color32::DARK_GRAY };
                            ui.add(ProgressBar::new(norm_energy).fill(bar_color).desired_width(55.0));

                            let current_bone = self.profile.mappings.get(&key).cloned().unwrap_or_else(|| "Не назначен".to_string());
                            ui.label(RichText::new(&current_bone).color(if current_bone == "Не назначен" { Color32::GRAY } else { Color32::LIGHT_BLUE }));

                            ui.menu_button("Выбрать ▼", |ui| {
                                egui::ScrollArea::vertical().max_height(240.0).show(ui, |ui| {
                                    for &bone in &TARGET_BONES {
                                        if ui.button(bone).clicked() {
                                            self.profile.mappings.insert(key.clone(), bone.to_string());
                                            if let Ok(mut prof) = shared.mapping_profile.lock() {
                                                *prof = self.profile.clone();
                                            }
                                            ui.close_menu();
                                        }
                                    }
                                });
                                ui.separator();
                                if ui.button("Снять привязку").clicked() {
                                    self.profile.mappings.remove(&key);
                                    if let Ok(mut prof) = shared.mapping_profile.lock() {
                                        *prof = self.profile.clone();
                                    }
                                    ui.close_menu();
                                }
                            });

                            ui.end_row();
                        }
                    });
            }
        });

        ui.add_space(8.0);
        ui.heading("Реестр 64 сегментов скелета");

        ui.group(|ui| {
            egui::ScrollArea::vertical().max_height(260.0).show(ui, |ui| {
                egui::Grid::new("bones_assignment_grid")
                    .num_columns(3)
                    .spacing([8.0, 4.0])
                    .show(ui, |ui| {
                        for &bone in &TARGET_BONES {
                            let assigned_key = self.profile.mappings.iter().find_map(|(k, v)| if v == bone { Some(k) } else { None });

                            ui.label(RichText::new(bone).monospace());

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
        });

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            if ui.button("💾 Сохранить в TOML").clicked() {
                if self.profile.save_to_file(profile_path).is_ok() {
                    self.status_message = "Профиль сохранен.".to_string();
                }
            }

            if ui.button("📂 Загрузить").clicked() {
                self.load_profile(profile_path);
                if let Ok(mut prof) = shared.mapping_profile.lock() {
                    *prof = self.profile.clone();
                }
                self.status_message = "Профиль загружен.".to_string();
            }

            if !self.status_message.is_empty() {
                ui.colored_label(Color32::YELLOW, &self.status_message);
            }
        });
    }
}