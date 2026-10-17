//! Сборка и генерация файлов project_context.txt, changes.txt и срезов MoCap-проекта.

use crate::config::{ContextConfig, DetailLevel, ProjectComponent};
use crate::extractor::{extract_rust_contracts, extract_rust_tests_only, strip_rust_tests};
use crate::scanner::scan_component_files;
use std::fs;
use std::path::Path;
use std::process::Command;

pub fn generate_context_file(
    root_dir: &Path,
    components: &[ProjectComponent],
    config: &ContextConfig,
    override_filename: Option<&str>,
) -> Result<(String, usize), String> {
    let output_filename = override_filename.unwrap_or(&config.output_file);
    let output_path = root_dir.join(output_filename);

    let mut output = String::new();

    // 1. Аппаратный манифест хоста
    output.push_str("--- START OF FILE host_hardware_info.txt ---\n");
    output.push_str("Host System Hardware Environment:\n");
    output.push_str(&collect_hardware_info());
    output.push_str("\n--- END OF FILE host_hardware_info.txt ---\n\n");

    // 2. Манифест структуры компонентов
    output.push_str("--- START OF FILE project_structure_manifest.txt ---\n");
    output.push_str("PROJECT STRUCTURE & CONTEXT PROFILE MANIFEST:\n");
    for comp in components {
        if comp.level == DetailLevel::Skip {
            continue;
        }

        if comp.level == DetailLevel::ManifestOnly {
            let files = scan_component_files(root_dir, comp, &config.ignore_patterns);
            output.push_str(&format!(
                "  - [{:<10}] {:<20} : {} (файлов: {})\n",
                comp.level.label(),
                comp.name,
                comp.description,
                files.len()
            ));
            for f in files {
                output.push_str(&format!("      * {f}\n"));
            }
        } else {
            output.push_str(&format!(
                "  - [{:<10}] {:<20} : {}\n",
                comp.level.label(),
                comp.name,
                comp.description
            ));
        }
    }
    output.push_str("--- END OF FILE project_structure_manifest.txt ---\n\n");

    let mut total_files = 0;

    // 3. Содержимое файлов
    for comp in components {
        if comp.level == DetailLevel::Skip || comp.level == DetailLevel::ManifestOnly {
            continue;
        }

        let files = scan_component_files(root_dir, comp, &config.ignore_patterns);

        for rel_path in files {
            let full_path = root_dir.join(&rel_path);
            let Ok(raw_content) = fs::read_to_string(&full_path) else {
                continue;
            };

            let processed = match comp.level {
                DetailLevel::Full => Some(raw_content),
                DetailLevel::NoTests => {
                    if rel_path.ends_with(".rs") {
                        Some(strip_rust_tests(&raw_content))
                    } else {
                        Some(raw_content)
                    }
                }
                DetailLevel::TestsOnly => extract_rust_tests_only(&raw_content, &rel_path),
                DetailLevel::ContractOnly => {
                    if rel_path.ends_with(".rs") {
                        Some(extract_rust_contracts(&raw_content))
                    } else {
                        Some(raw_content)
                    }
                }
                DetailLevel::ManifestOnly | DetailLevel::Skip => None,
            };

            if let Some(content) = processed {
                if !content.trim().is_empty() {
                    output.push_str(&format!("--- START OF FILE {rel_path} ---\n"));
                    output.push_str(&content);
                    output.push_str(&format!("\n--- END OF FILE {rel_path} ---\n\n"));
                    total_files += 1;
                }
            }
        }
    }

    fs::write(&output_path, output).map_err(|e| e.to_string())?;

    Ok((output_filename.to_string(), total_files))
}

pub fn generate_supplement_drop(
    root_dir: &Path,
    comp: &ProjectComponent,
    config: &ContextConfig,
) -> Result<(String, usize), String> {
    let sanitized_name = comp.name.replace('/', "_");
    let filename = format!("{}{}.txt", config.supplement_prefix, sanitized_name);
    let output_path = root_dir.join(&filename);

    let mut drop_comp = comp.clone();
    if drop_comp.level == DetailLevel::Skip {
        drop_comp.level = DetailLevel::Full;
    }

    let files = scan_component_files(root_dir, &drop_comp, &config.ignore_patterns);
    let mut output = String::new();

    output.push_str(&format!(
        "=== SUPPLEMENTAL CONTEXT DROP: {} [{}] ({}) ===\n\n",
        drop_comp.name,
        drop_comp.level.label(),
        drop_comp.description
    ));

    let mut total_files = 0;

    for rel_path in files {
        let full_path = root_dir.join(&rel_path);
        let Ok(raw_content) = fs::read_to_string(&full_path) else {
            continue;
        };

        let processed = match drop_comp.level {
            DetailLevel::Full => Some(raw_content),
            DetailLevel::NoTests => {
                if rel_path.ends_with(".rs") {
                    Some(strip_rust_tests(&raw_content))
                } else {
                    Some(raw_content)
                }
            }
            DetailLevel::TestsOnly => extract_rust_tests_only(&raw_content, &rel_path),
            DetailLevel::ContractOnly => {
                if rel_path.ends_with(".rs") {
                    Some(extract_rust_contracts(&raw_content))
                } else {
                    Some(raw_content)
                }
            }
            DetailLevel::ManifestOnly => Some(format!("// Manifest only: {}\n", rel_path)),
            DetailLevel::Skip => None,
        };

        if let Some(content) = processed {
            if !content.trim().is_empty() {
                output.push_str(&format!("--- START OF FILE {rel_path} ---\n"));
                output.push_str(&content);
                output.push_str(&format!("\n--- END OF FILE {rel_path} ---\n\n"));
                total_files += 1;
            }
        }
    }

    fs::write(&output_path, output).map_err(|e| e.to_string())?;

    Ok((filename, total_files))
}

pub fn generate_changes_report(
    root_dir: &Path,
    _config: &ContextConfig,
) -> Result<(String, usize), String> {
    let output_filename = "changes.txt";
    let output_path = root_dir.join(output_filename);

    let mut output = String::new();

    // 1. Статус Git
    output.push_str("=== GIT STATUS ===\n");
    if let Ok(res) = Command::new("git").arg("status").current_dir(root_dir).output() {
        output.push_str(&String::from_utf8_lossy(&res.stdout));
        output.push_str(&String::from_utf8_lossy(&res.stderr));
    }
    output.push_str("\n");

    // 2. Непроиндексированные диффы
    output.push_str("=== UNSTAGED DIFF ===\n");
    if let Ok(res) = Command::new("git").arg("diff").current_dir(root_dir).output() {
        output.push_str(&String::from_utf8_lossy(&res.stdout));
    }
    output.push_str("\n");

    // 3. Проиндексированные диффы
    output.push_str("=== STAGED DIFF ===\n");
    if let Ok(res) = Command::new("git").args(["diff", "--cached"]).current_dir(root_dir).output() {
        output.push_str(&String::from_utf8_lossy(&res.stdout));
    }
    output.push_str("\n");

    // 4. Неотслеживаемые новые файлы
    output.push_str("=== UNTRACKED NEW FILES CONTENT ===\n");
    if let Ok(res) = Command::new("git").args(["ls-files", "--others", "--exclude-standard"]).current_dir(root_dir).output() {
        let files_str = String::from_utf8_lossy(&res.stdout);
        for line in files_str.lines() {
            let rel_path = line.trim();
            if rel_path.is_empty()
                || rel_path == "changes.txt"
                || rel_path.ends_with(".lock")
                || rel_path.ends_with(".log")
                || rel_path.ends_with(".ttf")
                || rel_path.ends_with(".otf")
                || rel_path.starts_with(".context_state.")
            {
                continue;
            }

            let full_path = root_dir.join(rel_path);
            if let Ok(content) = fs::read_to_string(&full_path) {
                output.push_str(&format!("\n--- START OF UNTRACKED FILE: {rel_path} ---\n"));
                output.push_str(&content);
                output.push_str(&format!("\n--- END OF UNTRACKED FILE: {rel_path} ---\n"));
            }
        }
    }
    output.push_str("\n");

    fs::write(&output_path, &output).map_err(|e| e.to_string())?;

    Ok((output_filename.to_string(), output.len()))
}

fn collect_hardware_info() -> String {
    if let Ok(output) = Command::new("inxi").args(["-b", "-c", "0"]).output()
        && output.status.success()
    {
        return String::from_utf8_lossy(&output.stdout).to_string();
    }

    let uname = Command::new("uname")
        .arg("-a")
        .output()
        .map(|o| String::from_utf8_lossy(&o.stdout).to_string())
        .unwrap_or_else(|_| "Linux".to_string());

    format!("System Info (fallback): {uname}")
}