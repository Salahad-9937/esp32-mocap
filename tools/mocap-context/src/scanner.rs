//! Обход файловой системы и компоненты проекта ESP32 MoCap.

use crate::config::{ComponentKind, DetailLevel, ProjectComponent};
use std::fs;
use std::path::Path;
use walkdir::WalkDir;

#[must_use]
pub fn default_components() -> Vec<ProjectComponent> {
    vec![
        ProjectComponent {
            name: "firmware".to_string(),
            path: "firmware".to_string(),
            description: "Прошивка ESP32 (I2C опрос 24 датчиков, Bluetooth SPP поток)".to_string(),
            level: DetailLevel::Full,
            kind: ComponentKind::Directory,
        },
        ProjectComponent {
            name: "software".to_string(),
            path: "software".to_string(),
            description: "ПК-приемник (чтение COM-порта, парсинг кадров, фильтрация)".to_string(),
            level: DetailLevel::Full,
            kind: ComponentKind::Directory,
        },
        ProjectComponent {
            name: "docs".to_string(),
            path: "docs".to_string(),
            description: "Спецификации, регламенты и границы проекта MoCap".to_string(),
            level: DetailLevel::Full,
            kind: ComponentKind::Directory,
        },
        ProjectComponent {
            name: "scripts".to_string(),
            path: "scripts".to_string(),
            description: "Скрипты сборки документации Pandoc/XeLaTeX".to_string(),
            level: DetailLevel::Full,
            kind: ComponentKind::Directory,
        },
        ProjectComponent {
            name: "tools".to_string(),
            path: "tools".to_string(),
            description: "Инструменты разработки (mocap-context)".to_string(),
            level: DetailLevel::ManifestOnly,
            kind: ComponentKind::Directory,
        },
        ProjectComponent {
            name: "root".to_string(),
            path: ".".to_string(),
            description: "Корневой justfile, context.toml и конфигурации".to_string(),
            level: DetailLevel::Full,
            kind: ComponentKind::Directory,
        },
    ]
}

#[must_use]
pub fn scan_component_files(
    root_dir: &Path,
    comp: &ProjectComponent,
    ignore_patterns: &[String],
) -> Vec<String> {
    if comp.level == DetailLevel::Skip {
        return Vec::new();
    }

    let target_path = if comp.path == "." {
        root_dir.to_path_buf()
    } else {
        root_dir.join(&comp.path)
    };

    if !target_path.exists() {
        return Vec::new();
    }

    let mut files = Vec::new();

    if comp.path == "." {
        if let Ok(entries) = fs::read_dir(root_dir) {
            for entry in entries.flatten() {
                let p = entry.path();
                if p.is_file() {
                    let rel = p
                        .strip_prefix(root_dir)
                        .unwrap_or(&p)
                        .to_string_lossy()
                        .to_string();
                    if !is_ignored(&rel, ignore_patterns) {
                        files.push(rel);
                    }
                }
            }
        }
        files.sort();
        return files;
    }

    for entry in WalkDir::new(&target_path)
        .into_iter()
        .filter_map(Result::ok)
    {
        let p = entry.path();
        if p.is_file() {
            let rel = p
                .strip_prefix(root_dir)
                .unwrap_or(p)
                .to_string_lossy()
                .to_string();
            if !is_ignored(&rel, ignore_patterns) {
                if comp.level == DetailLevel::NoTests
                    && (rel.contains("/tests/") || rel.contains("/test/"))
                {
                    continue;
                }

                if comp.level == DetailLevel::TestsOnly {
                    let is_test = rel.contains("/tests/") || rel.ends_with("tests.rs");
                    if !is_test {
                        if let Ok(content) = fs::read_to_string(p) {
                            if !content.contains("#[cfg(test)]") && !content.contains("#[test]") {
                                continue;
                            }
                        } else {
                            continue;
                        }
                    }
                }

                files.push(rel);
            }
        }
    }

    files.sort();
    files
}

fn is_ignored(rel_path: &str, ignore_patterns: &[String]) -> bool {
    let path_obj = Path::new(rel_path);
    let file_name = path_obj.file_name().and_then(|n| n.to_str()).unwrap_or("");

    if file_name == "Cargo.lock"
        || file_name == "changes.txt"
        || file_name == ".context_state.toml"
        || file_name.starts_with(".context_state.")
        || file_name.ends_with(".lock")
        || file_name.ends_with(".log")
        || file_name.ends_with(".ttf")
        || file_name.ends_with(".otf")
        || file_name.ends_with(".pdf")
        || file_name == "project_context.txt"
        || file_name.starts_with("supplement_")
    {
        return true;
    }

    for component in path_obj.components() {
        let seg = component.as_os_str().to_string_lossy();
        if matches!(seg.as_ref(), "target" | ".git" | "logs" | "dist") {
            return true;
        }
    }

    for pat in ignore_patterns {
        if pat.starts_with("*.") {
            let ext = &pat[1..];
            if file_name.ends_with(ext) || rel_path.ends_with(ext) {
                return true;
            }
        } else if file_name == pat || rel_path == pat || rel_path.starts_with(pat) {
            return true;
        }
    }

    false
}