//! Точка входа эталонного графического приложения `mocap-app`.

mod app;
mod config;
mod state;
mod transport;
mod views;
mod visuals;

use app::MocapApp;
use config::AppConfig;
use eframe::NativeOptions;
use std::path::Path;

fn main() -> Result<(), eframe::Error> {
    let config_path = Path::new("mocap_config.toml");
    let config = AppConfig::load_from_file(config_path);

    let native_options = NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(&config.window_title)
            .with_inner_size([config.window_width, config.window_height])
            .with_min_inner_size([1024.0, 640.0]),
        ..Default::default()
    };

    eframe::run_native(
        "esp32_mocap_app",
        native_options,
        Box::new(|cc| Ok(Box::new(MocapApp::new(cc)))),
    )
}