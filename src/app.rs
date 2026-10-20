//! # Главная оконная стейт-машина MoCapApp (`app`)

use crate::config::AppConfig;
use crate::ingestion::spawn_background_ingestion;
use crate::state::{ConnectionStatus, SharedStateHandle, SharedTelemetryState};
use crate::views::{CalibrationView, MapperView, TopologyView};
use crate::visuals::{render_3d_scene, OrbitalCamera};
use egui::{Color32, Frame, Margin, RichText, Stroke, TextWrapMode, Visuals};
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveTab {
    Topology,
    Mapper,
    Calibration,
}

pub struct MocapApp {
    pub config: AppConfig,
    pub shared: SharedStateHandle,
    pub connection_status: ConnectionStatus,
    pub active_tab: ActiveTab,
    pub camera: OrbitalCamera,
    pub topology_view: TopologyView,
    pub mapper_view: MapperView,
    pub calibration_view: CalibrationView,
}

impl MocapApp {
    #[must_use]
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        let config = AppConfig::default();
        let shared = Arc::new(SharedTelemetryState::default());

        shared.is_running.store(true, Ordering::SeqCst);
        shared.user_wants_connected.store(true, Ordering::SeqCst);
        shared.auto_reconnect.store(config.auto_reconnect, Ordering::SeqCst);
        shared.reconnect_interval_ms.store(500, Ordering::SeqCst);

        let mut mapper_view = MapperView::default();
        mapper_view.load_profile(&config.mapping_profile_path);

        if let Ok(mut prof) = shared.mapping_profile.lock() {
            *prof = mapper_view.profile.clone();
        }

        spawn_background_ingestion(
            Arc::clone(&shared),
            config.default_port.clone(),
            config.baud_rate,
        );

        Self {
            config,
            shared,
            connection_status: ConnectionStatus::Connecting,
            active_tab: ActiveTab::Topology,
            camera: OrbitalCamera::default(),
            topology_view: TopologyView::default(),
            mapper_view,
            calibration_view: CalibrationView::default(),
        }
    }

    fn update_connection_status(&mut self) {
        if let Ok(inst_guard) = self.shared.last_frame_instant.lock() {
            if let Some(instant) = *inst_guard {
                if instant.elapsed() > Duration::from_millis(500) {
                    if let Ok(mut nodes_guard) = self.shared.nodes.lock() {
                        nodes_guard.clear();
                    }
                    if let Ok(mut fps_guard) = self.shared.calculated_fps.lock() {
                        *fps_guard = 0.0;
                    }
                    if let Ok(mut err_guard) = self.shared.connection_error.lock() {
                        if err_guard.is_none() || err_guard.as_deref() == Some("") {
                            *err_guard = Some("Поиск устройства (таймаут данных)...".to_string());
                        }
                    }
                }
            }
        }

        let fps = self.shared.calculated_fps.lock().map(|f| *f).unwrap_or(0.0);
        let wants_connected = self.shared.user_wants_connected.load(Ordering::Relaxed);

        if let Ok(err_guard) = self.shared.connection_error.lock() {
            if !wants_connected {
                self.connection_status = ConnectionStatus::Disconnected;
            } else if fps > 0.0 {
                self.connection_status = ConnectionStatus::Connected;
            } else if let Some(ref err_msg) = *err_guard {
                if err_msg.starts_with("Поиск")
                    || err_msg.starts_with("Ожидание")
                    || err_msg.contains("таймаут")
                    || err_msg.contains("отключилось")
                {
                    self.connection_status = ConnectionStatus::Connecting;
                } else {
                    self.connection_status = ConnectionStatus::Error(err_msg.clone());
                }
            } else {
                self.connection_status = ConnectionStatus::Connecting;
            }
        }
    }
}

impl eframe::App for MocapApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let bg_color = Color32::from_rgb(16, 20, 28);
        ctx.set_visuals(Visuals {
            dark_mode: true,
            panel_fill: bg_color,
            window_fill: bg_color,
            extreme_bg_color: Color32::from_rgb(10, 14, 20),
            faint_bg_color: Color32::from_rgb(22, 27, 36),
            ..Visuals::dark()
        });

        self.update_connection_status();

        if let Ok(mut nodes_guard) = self.shared.nodes.lock() {
            for node in nodes_guard.iter_mut() {
                let key = format!("0x{:02X}:{}", node.mux_address, node.channel);
                node.bone_tag = self.mapper_view.profile.mappings.get(&key).cloned();
            }
        }

        egui::SidePanel::left("mocap_side_panel_static")
            .exact_width(425.0)
            .resizable(false)
            .frame(Frame {
                inner_margin: Margin::same(8.0),
                fill: bg_color,
                stroke: Stroke::new(1.0_f32, Color32::from_rgb(38, 46, 62)),
                ..Default::default()
            })
            .show(ctx, |ui| {
                ui.style_mut().wrap_mode = Some(TextWrapMode::Wrap);

                ui.horizontal(|ui| {
                    ui.heading("ESP32 MoCap");
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        let (status_text, status_color) = match &self.connection_status {
                            ConnectionStatus::Connected => ("ПОТОК АКТИВЕН", Color32::GREEN),
                            ConnectionStatus::Connecting => ("ОЖИДАНИЕ", Color32::YELLOW),
                            ConnectionStatus::Paused => ("ПАУЗА", Color32::KHAKI),
                            ConnectionStatus::Disconnected => ("ОТКЛЮЧЕНО", Color32::GRAY),
                            ConnectionStatus::Error(err) => (err.as_str(), Color32::RED),
                        };
                        ui.label(RichText::new(status_text).color(status_color).strong());
                    });
                });

                ui.add_space(4.0);

                let conn_row_id = ui.id().with("conn_row_width");
                let conn_row_w = ui.data(|d| d.get_temp::<f32>(conn_row_id)).unwrap_or(270.0);
                let conn_offset = (ui.available_width() - conn_row_w).max(0.0) * 0.5;

                let conn_resp = ui.horizontal(|ui| {
                    ui.add_space(conn_offset);
                    let start_x = ui.cursor().min.x;

                    let btn_text = match self.connection_status {
                        ConnectionStatus::Connected => "Отключиться",
                        _ => "Подключиться",
                    };

                    let btn = egui::Button::new(
                        RichText::new(btn_text)
                            .size(13.5)
                            .color(Color32::WHITE)
                            .strong(),
                    )
                    .min_size(egui::vec2(125.0, 28.0));

                    if ui.add(btn).clicked() {
                        match self.connection_status {
                            ConnectionStatus::Connected | ConnectionStatus::Connecting => {
                                self.shared.user_wants_connected.store(false, Ordering::SeqCst);
                            }
                            _ => {
                                self.shared.user_wants_connected.store(true, Ordering::SeqCst);
                                if let Ok(mut err_guard) = self.shared.connection_error.lock() {
                                    *err_guard = None;
                                }
                            }
                        }
                    }

                    ui.add_space(10.0);

                    let mut auto_rec = self.shared.auto_reconnect.load(Ordering::Relaxed);
                    if ui.checkbox(&mut auto_rec, RichText::new("Авто-реконнект").size(13.5)).changed() {
                        self.shared.auto_reconnect.store(auto_rec, Ordering::SeqCst);
                    }

                    ui.min_rect().max.x - start_x
                });
                ui.data_mut(|d| d.insert_temp(conn_row_id, conn_resp.inner));

                ui.add_space(4.0);

                let tabs_row_id = ui.id().with("tabs_row_width");
                let tabs_row_w = ui.data(|d| d.get_temp::<f32>(tabs_row_id)).unwrap_or(342.0);
                let tabs_offset = (ui.available_width() - tabs_row_w).max(0.0) * 0.5;

                let mut clicked_tab = None;
                let tabs_resp = ui.horizontal(|ui| {
                    ui.add_space(tabs_offset);
                    let start_x = ui.cursor().min.x;

                    for (tab, text) in [
                        (ActiveTab::Topology, "Топология"),
                        (ActiveTab::Mapper, "Маппинг"),
                        (ActiveTab::Calibration, "Калибровка"),
                    ] {
                        let btn = egui::Button::new(RichText::new(text).size(13.5).strong())
                            .selected(self.active_tab == tab)
                            .min_size(egui::vec2(108.0, 28.0));
                        if ui.add(btn).clicked() {
                            clicked_tab = Some(tab);
                        }
                    }

                    ui.min_rect().max.x - start_x
                });
                ui.data_mut(|d| d.insert_temp(tabs_row_id, tabs_resp.inner));

                if let Some(tab) = clicked_tab {
                    self.active_tab = tab;
                }

                ui.add_space(2.0);
                ui.separator();

                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| {
                        match self.active_tab {
                            ActiveTab::Topology => {
                                self.topology_view.show(ui, &self.shared, &self.config.default_port);
                            }
                            ActiveTab::Mapper => {
                                self.mapper_view.show(ui, &self.shared, &self.config.mapping_profile_path);
                            }
                            ActiveTab::Calibration => {
                                self.calibration_view.show(ui, &self.shared);
                            }
                        }
                    });
            });

        egui::CentralPanel::default()
            .frame(Frame {
                inner_margin: Margin::same(0.0),
                fill: bg_color,
                ..Default::default()
            })
            .show(ctx, |ui| {
                let (response, painter) = ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());
                let scroll_delta = ui.input(|i| i.raw_scroll_delta.y);
                let chip_drag = crate::visuals::interact::handle(ui, &response, &self.camera);
                self.camera.handle_input_ex(&response, scroll_delta, !chip_drag);

                painter.rect_filled(response.rect, 0.0_f32, bg_color);

                let nodes = self.shared.nodes.lock().map(|g| g.clone()).unwrap_or_default();
                render_3d_scene(&painter, &self.camera, response.rect, &nodes);
            });

        if self.connection_status == ConnectionStatus::Connected {
            ctx.request_repaint_after(Duration::from_millis(14));
        }
    }
}