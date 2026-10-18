//! Модуль логической привязки аппаратных каналов к сегментам анатомического скелета.

use crate::error::{MocapError, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

/// Физический идентификатор аппаратного канала.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct HardwareChannel {
    pub mux_index: u8,
    pub channel_index: u8,
}

/// Запись ассоциативного сопоставления узла.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MappingEntry {
    pub node_index: usize,
    pub hardware: HardwareChannel,
    pub bone_name: String,
}

/// Сохраняемый конфигурационный профиль маппинга.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize, Default)]
pub struct MappingProfile {
    pub name: String,
    pub entries: Vec<MappingEntry>,
}

impl MappingProfile {
    #[must_use]
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            entries: Vec::new(),
        }
    }

    pub fn add_mapping(&mut self, node_index: usize, hardware: HardwareChannel, bone_name: &str) {
        self.entries.retain(|e| e.node_index != node_index && e.bone_name != bone_name);
        self.entries.push(MappingEntry {
            node_index,
            hardware,
            bone_name: bone_name.to_string(),
        });
    }

    #[must_use]
    pub fn find_bone(&self, node_index: usize) -> Option<&str> {
        self.entries
            .iter()
            .find(|e| e.node_index == node_index)
            .map(|e| e.bone_name.as_str())
    }

    /// Сохраняет профиль в текстовый файл формата TOML.
    pub fn save_to_file(&self, path: &Path) -> Result<()> {
        let serialized = toml::to_string_pretty(self)?;
        std::fs::write(path, serialized).map_err(|e| MocapError::MappingError(e.to_string()))?;
        Ok(())
    }

    /// Загружает профиль из текстового файла формата TOML.
    pub fn load_from_file(path: &Path) -> Result<Self> {
        let content = std::fs::read_to_string(path).map_err(|e| MocapError::MappingError(e.to_string()))?;
        let profile = toml::from_str(&content)?;
        Ok(profile)
    }

    /// Формирует быструю хеш-карту соответствия индексов узлов именам костей.
    #[must_use]
    pub fn to_lookup_map(&self) -> HashMap<usize, String> {
        let mut map = HashMap::new();
        for entry in &self.entries {
            map.insert(entry.node_index, entry.bone_name.clone());
        }
        map
    }
}