//! Модели конфигурации и сохраненного состояния менеджера контекста MoCap.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DetailLevel {
    Full,
    NoTests,
    TestsOnly,
    ContractOnly,
    ManifestOnly,
    Skip,
}

impl DetailLevel {
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::Full => "FULL",
            Self::NoTests => "NO_TESTS",
            Self::TestsOnly => "TESTS_ONLY",
            Self::ContractOnly => "CONTRACTS",
            Self::ManifestOnly => "MANIFEST",
            Self::Skip => "SKIP",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComponentKind {
    CodeCrate,
    Directory,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProjectComponent {
    pub name: String,
    pub path: String,
    pub description: String,
    pub level: DetailLevel,
    pub kind: ComponentKind,
}

impl ProjectComponent {
    pub fn cycle_next(&mut self) {
        match self.kind {
            ComponentKind::CodeCrate => {
                self.level = match self.level {
                    DetailLevel::Full => DetailLevel::NoTests,
                    DetailLevel::NoTests => DetailLevel::TestsOnly,
                    DetailLevel::TestsOnly => DetailLevel::ContractOnly,
                    DetailLevel::ContractOnly => DetailLevel::ManifestOnly,
                    DetailLevel::ManifestOnly => DetailLevel::Skip,
                    DetailLevel::Skip => DetailLevel::Full,
                };
            }
            ComponentKind::Directory => {
                self.level = match self.level {
                    DetailLevel::Full => DetailLevel::ManifestOnly,
                    DetailLevel::ManifestOnly => DetailLevel::Skip,
                    _ => DetailLevel::Full,
                };
            }
        }
    }

    pub fn cycle_prev(&mut self) {
        match self.kind {
            ComponentKind::CodeCrate => {
                self.level = match self.level {
                    DetailLevel::Full => DetailLevel::Skip,
                    DetailLevel::NoTests => DetailLevel::Full,
                    DetailLevel::TestsOnly => DetailLevel::NoTests,
                    DetailLevel::ContractOnly => DetailLevel::TestsOnly,
                    DetailLevel::ManifestOnly => DetailLevel::ContractOnly,
                    DetailLevel::Skip => DetailLevel::ManifestOnly,
                };
            }
            ComponentKind::Directory => {
                self.level = match self.level {
                    DetailLevel::Full => DetailLevel::Skip,
                    DetailLevel::Skip => DetailLevel::ManifestOnly,
                    _ => DetailLevel::Full,
                };
            }
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct SessionState {
    pub selected_index: usize,
    pub levels: HashMap<String, DetailLevel>,
}

impl SessionState {
    #[must_use]
    pub fn load_from_file(root_dir: &Path) -> Self {
        let state_path = root_dir.join("tools/mocap-context/.context_state.toml");
        if state_path.exists()
            && let Ok(content) = std::fs::read_to_string(&state_path)
            && let Ok(state) = toml::from_str::<Self>(&content)
        {
            state
        } else {
            Self::default()
        }
    }

    pub fn save_to_file(&self, root_dir: &Path) {
        let tools_dir = root_dir.join("tools/mocap-context");
        if !tools_dir.exists() {
            let _ = std::fs::create_dir_all(&tools_dir);
        }
        let state_path = tools_dir.join(".context_state.toml");
        if let Ok(serialized) = toml::to_string_pretty(self) {
            let _ = std::fs::write(state_path, serialized);
        }
    }

    pub fn apply_to_components(&self, components: &mut [ProjectComponent]) {
        for comp in components {
            if let Some(&level) = self.levels.get(&comp.name) {
                comp.level = level;
            }
        }
    }

    #[must_use]
    pub fn from_components(selected_index: usize, components: &[ProjectComponent]) -> Self {
        let mut levels = HashMap::new();
        for comp in components {
            levels.insert(comp.name.clone(), comp.level);
        }
        Self {
            selected_index,
            levels,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContextConfig {
    pub output_file: String,
    pub supplement_prefix: String,
    pub ignore_patterns: Vec<String>,
}

impl Default for ContextConfig {
    fn default() -> Self {
        Self {
            output_file: "project_context.txt".to_string(),
            supplement_prefix: "supplement_".to_string(),
            ignore_patterns: vec![
                "target".to_string(),
                ".git".to_string(),
                "logs".to_string(),
                "dist".to_string(),
                "Cargo.lock".to_string(),
                "changes.txt".to_string(),
                ".context_state.toml".to_string(),
                "*.ttf".to_string(),
                "*.otf".to_string(),
                "project_context.txt".to_string(),
                "supplement_*.txt".to_string(),
            ],
        }
    }
}