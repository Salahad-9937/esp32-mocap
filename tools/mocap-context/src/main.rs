//! Точка входа TUI-менеджера контекста `mocap-context`.

mod config;
mod extractor;
mod generator;
mod scanner;
mod ui;

use config::{ContextConfig, SessionState};
use scanner::default_components;
use std::fs;
use std::path::{Path, PathBuf};

fn find_project_root() -> PathBuf {
    let mut current = std::env::current_dir().unwrap_or_else(|_| PathBuf::from("."));
    for _ in 0..10 {
        if current.join("firmware").exists() || current.join("docs").exists() {
            return current;
        }
        if let Some(parent) = current.parent() {
            current = parent.to_path_buf();
        } else {
            break;
        }
    }
    PathBuf::from(".")
}

fn load_context_config(root: &Path) -> ContextConfig {
    let config_path = root.join("context.toml");
    if config_path.exists()
        && let Ok(content) = fs::read_to_string(&config_path)
        && let Ok(cfg) = toml::from_str::<ContextConfig>(&content)
    {
        return cfg;
    }
    ContextConfig::default()
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let root_dir = find_project_root();
    let config = load_context_config(&root_dir);
    let mut components = default_components();

    let session = SessionState::load_from_file(&root_dir);
    session.apply_to_components(&mut components);

    ui::run_tui(&root_dir, components, config, session.selected_index)?;

    Ok(())
}