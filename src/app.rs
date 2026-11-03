//! # Главная оконная стейт-машина MoCapApp (`app`)

use crate::config::AppConfig;
use crate::ingestion::spawn_background_ingestion;
use crate::state::{ConnectionStatus, SharedStateHandle, SharedTelemetryState};
use crate::views::{CalibrationView, MapperView, TopologyView, TrackingView};
use crate::visuals::{placement, render_3d_scene, OrbitalCamera};
use egui::{Color32, Frame, Margin, RichText, Stroke, TextWrapMode, Visuals};
use std::path::PathBuf;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActiveTab {
    Topology,
    Mapper,
    Calibration,
    Tracking,
}

pub struct MocapApp {
    pub config: AppConfig,
    pub config_path: PathBuf,
    pub shared: SharedStateHandle,
    pub active_tab: ActiveTab,
    pub camera: OrbitalCamera,
    pub topology_view: TopologyView,
    pub mapper_view: MapperView,
    pub calibration_view: CalibrationView,
    pub tracking_view: TrackingView,
}

impl MocapApp {
    #[must_use]
    pub fn new(_cc: &eframe::CreationContext<'_>, config: AppConfig, config_path: PathBuf) -> Self {
        let shared = Arc::new(SharedTelemetryState::default());

        shared.connection.is_running.store(true, Ordering::SeqCst);
        shared.connection.user_wants_connected.store(true, Ordering::SeqCst);
        shared.connection.auto_reconnect.store(config.auto_reconnect, Ordering::SeqCst);
        shared.connection.reconnect_interval_ms.store(500, Ordering::SeqCst);

        let mut mapper_view = MapperView::default();
        mapper_view.load_profile(&config.mapping_profile_path);

        if let Ok(mut prof) = shared.mapping.profile.lock() {
            *prof = mapper_view.profile.clone();
        }

        spawn_background_ingestion(
            Arc::clone(&shared),
            config.default_port.clone(),
            config.baud_rate,
        );

        Self {
            config,
            config_path,
            shared,
            active_tab: ActiveTab::Topology,
            camera: OrbitalCamera::default(),
            topology_view: TopologyView::default(),
            mapper_view,
            calibration_view: CalibrationView::default(),
            tracking_view: TrackingView::default(),
        }
    }
}

impl eframe::App for MocapApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let dt = ctx.input(|i| i.stable_dt).min(0.05);
        if placement::update_pose_animation(dt) {
            ctx.request_repaint();
        }

        let bg_color = Color32::from_rgb(16, 20, 28);
        ctx.set_visuals(Visuals {
            dark_mode: true,
            panel_fill: bg_color,
            window_fill: bg_color,
            extreme_bg_color: Color32::from_rgb(10, 14, 20),
            faint_bg_color: Color32::from_rgb(22, 27, 36),
            ..Visuals::dark()
        });

        let conn_status = self.shared.connection_status();

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
                        let (status_text, status_color) = match &conn_status {
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

                    let btn_text = match conn_status {
                        ConnectionStatus::Connected | ConnectionStatus::Paused | ConnectionStatus::Connecting => "Отключиться",
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
                        match conn_status {
                            ConnectionStatus::Connected | ConnectionStatus::Connecting | ConnectionStatus::Paused => {
                                self.shared.connection.user_wants_connected.store(false, Ordering::SeqCst);
                                self.shared.connection.is_paused.store(false, Ordering::SeqCst);
                                self.shared.set_connection_status(ConnectionStatus::Disconnected);
                            }
                            ConnectionStatus::Disconnected | ConnectionStatus::Error(_) => {
                                self.shared.connection.user_wants_connected.store(true, Ordering::SeqCst);
                                self.shared.connection.is_paused.store(false, Ordering::SeqCst);
                                self.shared.set_connection_status(ConnectionStatus::Connecting);
                            }
                        }
                    }

                    ui.add_space(10.0);

                    let mut auto_rec = self.shared.connection.auto_reconnect.load(Ordering::Relaxed);
                    if ui.checkbox(&mut auto_rec, RichText::new("Авто-реконнект").size(13.5)).changed() {
                        self.shared.connection.auto_reconnect.store(auto_rec, Ordering::SeqCst);
                        self.config.auto_reconnect = auto_rec;
                        let _ = self.config.save_to_file(&self.config_path);
                    }

                    if conn_status == ConnectionStatus::Connected || conn_status == ConnectionStatus::Paused {
                        ui.add_space(8.0);
                        let mut paused = self.shared.connection.is_paused.load(Ordering::Relaxed);
                        if ui.checkbox(&mut paused, RichText::new("Пауза").size(13.5)).changed() {
                            self.shared.connection.is_paused.store(paused, Ordering::SeqCst);
                        }
                    }

                    ui.min_rect().max.x - start_x
                });
                ui.data_mut(|d| d.insert_temp(conn_row_id, conn_resp.inner));

                ui.add_space(4.0);

                let mut clicked_tab = None;
                ui.horizontal(|ui| {
                    for (tab, text) in [
                        (ActiveTab::Topology, "Топология"),
                        (ActiveTab::Mapper, "Маппинг"),
                        (ActiveTab::Calibration, "Калибровка"),
                        (ActiveTab::Tracking, "🎬 Захват"),
                    ] {
                        let btn = egui::Button::new(RichText::new(text).size(12.5).strong())
                            .selected(self.active_tab == tab)
                            .min_size(egui::vec2(95.0, 28.0));
                        if ui.add(btn).clicked() {
                            clicked_tab = Some(tab);
                        }
                    }
                });

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
                            ActiveTab::Tracking => {
                                self.tracking_view.show(ui, &self.shared);
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

                let nodes = self.shared.snapshot_nodes();
                render_3d_scene(&painter, &self.camera, response.rect, &nodes, self.active_tab);
            });

        if conn_status == ConnectionStatus::Connected {
            ctx.request_repaint_after(Duration::from_millis(14));
        }
    }
}