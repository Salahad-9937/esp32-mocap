//! # Интерактивный 3D-центричный конфигуратор привязки каналов (`mapper`)

pub mod detector;
pub mod groups;

use crate::state::SharedStateHandle;
use crate::visuals::placement;
use detector::{DetectionEvent, KineticDetector};
use egui::{Color32, Frame, Margin, ProgressBar, RichText, Stroke, TextWrapMode, Ui};
use groups::{friendly_bone_name, ANATOMY_CLUSTERS};
use mocap_core::mapping::MappingProfile;
use std::collections::HashMap;
use std::path::Path;

pub struct MapperView {
    pub profile: MappingProfile,
    pub status_message: String,
    pub detector: KineticDetector,
}

impl Default for MapperView {
    fn default() -> Self {
        Self {
            profile: MappingProfile::default(),
            status_message: String::new(),
            detector: KineticDetector::default(),
        }
    }
}

impl MapperView {
    pub fn load_profile(&mut self, path: &Path) {
        if let Ok(p) = MappingProfile::load_from_file(path) {
            let mut migrated = HashMap::new();
            for (raw_channel, bone) in p.mappings {
                let canonical_channel = if raw_channel.matches(':').count() == 1 {
                    format!("{raw_channel}:0x68")
                } else {
                    raw_channel
                };
                migrated.insert(canonical_channel, bone);
            }
            self.profile = MappingProfile { name: p.name, mappings: migrated };
        }
    }

    pub fn show(&mut self, ui: &mut Ui, shared: &SharedStateHandle, profile_path: &Path) {
        let nodes = shared.telemetry.nodes.lock().map(|g| g.clone()).unwrap_or_default();
        let selected_tag = placement::selected_tag().map(String::from);

        ui.heading("Маппинг датчиков на скелет");
        ui.label("Кликните на сегмент в 3D-сцене или выберите из списка ниже.");
        ui.add_space(4.0);

        self.poll_detector(&nodes, shared);
        self.render_context_binding_card(ui, selected_tag.as_deref(), &nodes, shared);

        ui.add_space(8.0);
        self.render_active_hardware_list(ui, &nodes, selected_tag.as_deref(), shared);

        ui.add_space(8.0);
        self.render_anatomical_tree(ui, selected_tag.as_deref());

        ui.add_space(10.0);
        ui.horizontal(|ui| {
            if ui.button("💾 Сохранить").clicked() {
                if self.profile.save_to_file(profile_path).is_ok() {
                    self.status_message = "Профиль сохранен в TOML.".to_string();
                }
            }
            if ui.button("📂 Загрузить").clicked() {
                self.load_profile(profile_path);
                if let Ok(mut prof) = shared.mapping.profile.lock() {
                    *prof = self.profile.clone();
                }
                self.status_message = "Профиль загружен.".to_string();
            }
            if !self.status_message.is_empty() {
                ui.colored_label(Color32::YELLOW, &self.status_message);
            }
        });
    }

    fn poll_detector(&mut self, nodes: &[crate::state::NodeTelemetry], shared: &SharedStateHandle) {
        if let Some(event) = self.detector.update(nodes) {
            match event {
                DetectionEvent::Detected { channel_key, target_bone } => {
                    self.profile.mappings.insert(channel_key.clone(), target_bone.clone());
                    self.status_message = format!("Успешно назначен узел {channel_key} -> {target_bone}");
                    if let Ok(mut prof) = shared.mapping.profile.lock() {
                        *prof = self.profile.clone();
                    }
                }
                DetectionEvent::NoMovement => {
                    self.status_message = "Движение не обнаружено. Повторите попытку.".to_string();
                }
            }
        }
    }

    fn render_context_binding_card(
        &mut self,
        ui: &mut Ui,
        selected_bone: Option<&str>,
        nodes: &[crate::state::NodeTelemetry],
        shared: &SharedStateHandle,
    ) {
        ui.group(|ui| {
            ui.set_width(ui.available_width());
            let name = match selected_bone {
                Some(b) => friendly_bone_name(b),
                None => "Сегмент не выбран (кликните в 3D)",
            };

            ui.horizontal(|ui| {
                ui.label(RichText::new("🎯 ЦЕЛЕВОЙ СЕГМЕНТ:").strong().color(Color32::from_rgb(100, 200, 255)));
                ui.label(RichText::new(name).strong());
            });

            if let Some(bone_tag) = selected_bone {
                let assigned_key = self.profile.mappings.iter().find_map(|(k, v)| if v == bone_tag { Some(k.clone()) } else { None });

                ui.horizontal(|ui| {
                    ui.label("Назначенный канал:");
                    if let Some(ref key) = assigned_key {
                        ui.colored_label(Color32::GREEN, RichText::new(key).monospace().strong());
                        if ui.button("Снять привязку").clicked() {
                            self.profile.mappings.remove(key);
                            if let Ok(mut prof) = shared.mapping.profile.lock() {
                                *prof = self.profile.clone();
                            }
                        }
                    } else {
                        ui.colored_label(Color32::GRAY, "Не назначен");
                    }
                });

                ui.add_space(2.0);
                if self.detector.target_bone() == Some(bone_tag) {
                    ui.colored_label(Color32::YELLOW, "Пошевелите этим сегментом...");
                    ui.add(ProgressBar::new(self.detector.progress()).show_percentage());
                } else {
                    ui.horizontal(|ui| {
                        if ui.button(RichText::new("🎯 Засечь движение").strong()).clicked() {
                            self.detector.start(bone_tag);
                        }

                        ui.menu_button("Назначить вручную ▼", |ui| {
                            egui::ScrollArea::vertical().max_height(200.0).show(ui, |ui| {
                                for node in nodes {
                                    let key = MappingProfile::channel_key(node.mux_address, node.channel, node.sensor_address);
                                    let btn_text = format!("{key} (Энергия: {:.1})", node.peak_energy);
                                    if ui.button(btn_text).clicked() {
                                        self.profile.set_mapping(node.mux_address, node.channel, node.sensor_address, bone_tag);
                                        if let Ok(mut prof) = shared.mapping.profile.lock() {
                                            *prof = self.profile.clone();
                                        }
                                        ui.close_menu();
                                    }
                                }
                            });
                        });
                    });
                }
            }
        });
    }

    fn render_active_hardware_list(
        &mut self,
        ui: &mut Ui,
        nodes: &[crate::state::NodeTelemetry],
        selected_bone: Option<&str>,
        shared: &SharedStateHandle,
    ) {
        egui::CollapsingHeader::new(RichText::new(format!("Активные датчики на шине ({})", nodes.len())).strong())
            .default_open(true)
            .show(ui, |ui| {
                if nodes.is_empty() {
                    ui.label(RichText::new("Нет активных сигналов в потоке").color(Color32::GRAY));
                    return;
                }

                for node in nodes {
                    let key = MappingProfile::channel_key(node.mux_address, node.channel, node.sensor_address);
                    let current_bone = self.profile.mappings.get(&key).cloned();

                    ui.horizontal(|ui| {
                        ui.add(egui::Label::new(RichText::new(&key).monospace().strong()).wrap_mode(TextWrapMode::Extend));
                        let energy_norm = (node.peak_energy / 8.0).clamp(0.0, 1.0);
                        let bar_color = if energy_norm > 0.35 { Color32::GREEN } else { Color32::DARK_GRAY };
                        ui.add(ProgressBar::new(energy_norm).fill(bar_color).desired_width(45.0));

                        if let Some(ref target) = current_bone {
                            ui.colored_label(Color32::LIGHT_BLUE, friendly_bone_name(target));
                        } else {
                            ui.colored_label(Color32::GRAY, "— Свободен —");
                        }

                        if let Some(target_tag) = selected_bone {
                            if current_bone.as_deref() != Some(target_tag) && ui.button("Привязать к цели").clicked() {
                                self.profile.set_mapping(node.mux_address, node.channel, node.sensor_address, target_tag);
                                if let Ok(mut prof) = shared.mapping.profile.lock() {
                                    *prof = self.profile.clone();
                                }
                            }
                        }
                    });
                }
            });
    }

    fn render_anatomical_tree(
        &mut self,
        ui: &mut Ui,
        selected_bone: Option<&str>,
    ) {
        ui.heading("Целевые сегменты скелета");

        for cluster in ANATOMY_CLUSTERS {
            egui::CollapsingHeader::new(RichText::new(cluster.title).strong())
                .default_open(false)
                .show(ui, |ui| {
                    for &bone in cluster.bones {
                        let is_selected = selected_bone == Some(bone);
                        let assigned_key = self.profile.mappings.iter().find_map(|(k, v)| if v == bone { Some(k.clone()) } else { None });

                        let row_frame = Frame::none()
                            .inner_margin(Margin::symmetric(4.0, 2.0))
                            .fill(if is_selected { Color32::from_rgb(30, 45, 65) } else { Color32::TRANSPARENT })
                            .stroke(if is_selected { Stroke::new(1.0_f32, Color32::from_rgb(100, 200, 255)) } else { Stroke::NONE });

                        row_frame.show(ui, |ui| {
                            ui.horizontal(|ui| {
                                let label_text = format!("{} [{bone}]", friendly_bone_name(bone));
                                if ui.selectable_label(is_selected, label_text).clicked() {
                                    placement::select_by_tag(bone);
                                }

                                if let Some(ref key) = assigned_key {
                                    ui.colored_label(Color32::GREEN, RichText::new(key.as_str()).monospace());
                                } else {
                                    ui.colored_label(Color32::GRAY, "—");
                                }

                                if self.detector.target_bone().is_none()
                                    && ui.button("🎯").on_hover_text("Засечь движение для этой кости").clicked()
                                {
                                    placement::select_by_tag(bone);
                                    self.detector.start(bone);
                                }
                            });
                        });
                    }
                });
        }
    }
}