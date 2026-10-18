//! Модуль конфигурационных параметров приложения и путей к профилям.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

/// Конфигурационные параметры графического интерфейса и подключения.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AppConfig {
    /// Заголовок главного окна
    pub window_title: String,
    /// Начальная ширина окна в пикселях
    pub window_width: f32,
    /// Начальная высота окна в пикселях
    pub window_height: f32,
    /// Целевой MAC-адрес Bluetooth или имя порта
    pub default_port: String,
    /// Скорость передачи данных (бод)
    pub baud_rate: u32,
    /// Целевая частота цикла опроса (Гц)
    pub target_fps: u32,
    /// Флаг разрешения автоподключения при старте
    pub auto_reconnect: bool,
    /// Интервал повторного автоподключения в миллисекундах
    pub reconnect_interval_ms: u64,
    /// Путь к файлу профиля маппинга каналов
    pub mapping_profile_path: PathBuf,
    /// Путь к файлу калибровочного профиля
    pub calibration_profile_path: PathBuf,
}

impl Default for AppConfig {
    fn default() -> Self {
        Self {
            window_title: "ESP32 IMU MoCap :: Reference Station".to_string(),
            window_width: 1400.0,
            window_height: 880.0,
            default_port: "20:50:0D:2A:85:96".to_string(),
            baud_rate: 115200,
            target_fps: 70,
            auto_reconnect: true,
            reconnect_interval_ms: 500,
            mapping_profile_path: PathBuf::from("mapping_profile.toml"),
            calibration_profile_path: PathBuf::from("calibration_profile.toml"),
        }
    }
}

impl AppConfig {
    /// Загружает конфигурацию из файла или возвращает значения по умолчанию.
    #[must_use]
    pub fn load_from_file(path: &Path) -> Self {
        if path.exists() {
            if let Ok(content) = fs::read_to_string(path) {
                if let Ok(cfg) = toml::from_str::<Self>(&content) {
                    return cfg;
                }
            }
        }
        Self::default()
    }

    /// Сохраняет текущую конфигурацию в файл формата TOML.
    pub fn save_to_file(&self, path: &Path) -> Result<(), String> {
        let content = toml::to_string_pretty(self)
            .map_err(|e| format!("Ошибка сериализации конфигурации: {e}"))?;
        fs::write(path, content)
            .map_err(|e| format!("Ошибка записи файла конфигурации: {e}"))?;
        Ok(())
    }
}