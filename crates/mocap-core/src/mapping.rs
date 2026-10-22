//! # Логическая привязка каналов к сегментам анатомического скелета (`mapping`)

use crate::error::{MocapError, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

/// Физический идентификатор аппаратного канала коммутатора и адреса датчика.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct HardwareChannel {
    pub mux_address: u8,
    pub channel_index: u8,
    pub sensor_address: u8,
}

/// Конфигурационный профиль маппинга датчиков на сегменты скелета.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct MappingProfile {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub mappings: HashMap<String, String>,
}

impl MappingProfile {
    #[must_use]
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            mappings: HashMap::new(),
        }
    }

    /// Формирует строковый ключ канала в формате `0x70:0:0x68`.
    #[must_use]
    pub fn channel_key(mux_address: u8, channel: u8, sensor_address: u8) -> String {
        format!("0x{mux_address:02X}:{channel}:0x{sensor_address:02X}")
    }

    /// Устанавливает привязку аппаратного канала к сегменту скелета.
    pub fn set_mapping(&mut self, mux_address: u8, channel: u8, sensor_address: u8, bone_name: &str) {
        let key = Self::channel_key(mux_address, channel, sensor_address);
        self.mappings.insert(key, bone_name.to_string());
    }

    /// Считывает имя сегмента скелета для заданного физического канала с поддержкой легаси-ключей.
    #[must_use]
    pub fn get_bone(&self, mux_address: u8, channel: u8, sensor_address: u8) -> Option<&str> {
        let key = Self::channel_key(mux_address, channel, sensor_address);
        if let Some(name) = self.mappings.get(&key) {
            return Some(name.as_str());
        }
        if sensor_address == 0x68 {
            let legacy_key = format!("0x{mux_address:02X}:{channel}");
            if let Some(name) = self.mappings.get(&legacy_key) {
                return Some(name.as_str());
            }
        }
        None
    }

    /// Удаляет привязку физического канала.
    pub fn remove_mapping(&mut self, mux_address: u8, channel: u8, sensor_address: u8) {
        let key = Self::channel_key(mux_address, channel, sensor_address);
        self.mappings.remove(&key);
        if sensor_address == 0x68 {
            let legacy_key = format!("0x{mux_address:02X}:{channel}");
            self.mappings.remove(&legacy_key);
        }
    }

    /// Загружает профиль из TOML файла на диске.
    pub fn load_from_file(path: &Path) -> Result<Self> {
        if !path.exists() {
            return Ok(Self::default());
        }
        let content = std::fs::read_to_string(path)
            .map_err(|e| MocapError::MappingError(e.to_string()))?;
        let profile = toml::from_str(&content)?;
        Ok(profile)
    }

    /// Сохраняет профиль в TOML файл на диске.
    pub fn save_to_file(&self, path: &Path) -> Result<()> {
        let serialized = toml::to_string_pretty(self)?;
        std::fs::write(path, serialized)
            .map_err(|e| MocapError::MappingError(e.to_string()))?;
        Ok(())
    }
}