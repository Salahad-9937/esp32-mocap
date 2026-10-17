//! Фильтрация тестов и извлечение сигнатур интерфейсов Rust.

use quote::ToTokens;
use syn::{Item, Visibility};

#[must_use]
pub fn extract_rust_contracts(content: &str) -> String {
    let Ok(syntax_tree) = syn::parse_file(content) else {
        return strip_rust_tests(content);
    };

    let mut lines = Vec::new();

    for line in content.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("//!") {
            lines.push(line.to_string());
        } else if !trimmed.is_empty() && !trimmed.starts_with("//") {
            break;
        }
    }
    if !lines.is_empty() {
        lines.push(String::new());
    }

    for item in &syntax_tree.items {
        match item {
            Item::Use(u) => {
                let text = u.to_token_stream().to_string();
                lines.push(format!("{text};"));
            }
            Item::Struct(s) if matches!(s.vis, Visibility::Public(_)) => {
                let mut s_clean = s.clone();
                s_clean.attrs.clear();
                lines.push(s_clean.to_token_stream().to_string());
                lines.push(String::new());
            }
            Item::Enum(e) if matches!(e.vis, Visibility::Public(_)) => {
                let mut e_clean = e.clone();
                e_clean.attrs.clear();
                lines.push(e_clean.to_token_stream().to_string());
                lines.push(String::new());
            }
            Item::Trait(t) if matches!(t.vis, Visibility::Public(_)) => {
                let mut t_clean = t.clone();
                t_clean.attrs.clear();
                lines.push(t_clean.to_token_stream().to_string());
                lines.push(String::new());
            }
            Item::Fn(f) if matches!(f.vis, Visibility::Public(_)) => {
                let mut f_stub = f.clone();
                f_stub.attrs.clear();
                f_stub.block = syn::parse_quote!({ /* ... */ });
                lines.push(f_stub.to_token_stream().to_string());
                lines.push(String::new());
            }
            Item::Impl(imp) => {
                let is_trait = imp.trait_.is_some();
                let has_pub = imp.items.iter().any(|it| match it {
                    syn::ImplItem::Fn(m) => matches!(m.vis, Visibility::Public(_)) || is_trait,
                    _ => false,
                });

                if has_pub {
                    let mut imp_clean = imp.clone();
                    imp_clean.items.retain_mut(|impl_item| match impl_item {
                        syn::ImplItem::Fn(m) => {
                            if matches!(m.vis, Visibility::Public(_)) || is_trait {
                                m.block = syn::parse_quote!({ /* ... */ });
                                true
                            } else {
                                false
                            }
                        }
                        _ => true,
                    });
                    lines.push(imp_clean.to_token_stream().to_string());
                    lines.push(String::new());
                }
            }
            _ => {}
        }
    }

    if lines.is_empty() {
        strip_rust_tests(content)
    } else {
        lines.join("\n")
    }
}

#[must_use]
pub fn strip_rust_tests(content: &str) -> String {
    let mut result = Vec::new();
    let mut in_test_block = false;
    let mut brace_depth: usize = 0;
    let mut pending_cfg_test = false;

    for line in content.lines() {
        let trimmed = line.trim();

        if trimmed.starts_with("#[cfg(test)]") || trimmed.starts_with("#![cfg(test)]") {
            pending_cfg_test = true;
            continue;
        }

        if pending_cfg_test {
            if trimmed.starts_with("mod ") || trimmed.starts_with("pub mod ") {
                in_test_block = true;
                pending_cfg_test = false;
                let open_braces = line.chars().filter(|&c| c == '{').count();
                let close_braces = line.chars().filter(|&c| c == '}').count();
                brace_depth = brace_depth.saturating_add(open_braces).saturating_sub(close_braces);
                if brace_depth == 0 && open_braces > 0 {
                    in_test_block = false;
                }
                continue;
            } else if !trimmed.starts_with('#') {
                pending_cfg_test = false;
            }
        }

        if in_test_block {
            let open_braces = line.chars().filter(|&c| c == '{').count();
            let close_braces = line.chars().filter(|&c| c == '}').count();
            brace_depth = brace_depth.saturating_add(open_braces).saturating_sub(close_braces);
            if brace_depth == 0 {
                in_test_block = false;
            }
            continue;
        }

        result.push(line);
    }

    while result.last().is_some_and(|l| l.trim().is_empty()) {
        result.pop();
    }

    result.join("\n")
}

#[must_use]
pub fn extract_rust_tests_only(content: &str, rel_path: &str) -> Option<String> {
    if rel_path.contains("/tests/") || rel_path.ends_with("tests.rs") {
        return Some(content.to_string());
    }
    None
}