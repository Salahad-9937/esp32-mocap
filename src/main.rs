//! # Точка входа эталонного графического приложения `mocap-app`

mod app;
mod config;
mod ingestion;
mod state;
mod views;
mod visuals;

use app::MocapApp;
use config::AppConfig;
use eframe::NativeOptions;
use std::path::Path;

#[cfg(unix)]
fn setup_shutdown_signal_handler() {
    unsafe {
        let mut set: libc::sigset_t = std::mem::zeroed();
        libc::sigemptyset(&mut set);
        libc::sigaddset(&mut set, libc::SIGINT);
        libc::sigaddset(&mut set, libc::SIGTERM);
        libc::pthread_sigmask(libc::SIG_BLOCK, &set, std::ptr::null_mut());

        std::thread::spawn(move || {
            let mut sig: libc::c_int = 0;
            if libc::sigwait(&set, &mut sig) == 0 {
                std::process::exit(0);
            }
        });
    }
}

#[cfg(not(unix))]
fn setup_shutdown_signal_handler() {}

fn main() -> Result<(), eframe::Error> {
    setup_shutdown_signal_handler();

    let config_path = Path::new("mocap_config.toml");
    let config = if config_path.exists() {
        AppConfig::load_from_file(config_path)
    } else {
        let default_cfg = AppConfig::default();
        let _ = default_cfg.save_to_file(config_path);
        default_cfg
    };

    let native_options = NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(&config.window_title)
            .with_inner_size([config.window_width, config.window_height])
            .with_min_inner_size([1024.0, 640.0]),
        depth_buffer: 24,
        ..Default::default()
    };

    let app_config_path = config_path.to_path_buf();
    eframe::run_native(
        "esp32_mocap_app",
        native_options,
        Box::new(move |cc| Ok(Box::new(MocapApp::new(cc, config, app_config_path)))),
    )
}