//! Интерактивный терминальный интерфейс TUI для проекта MoCap.

use crate::config::{ContextConfig, DetailLevel, ProjectComponent, SessionState};
use crate::generator::{generate_changes_report, generate_context_file, generate_supplement_drop};
use crossterm::event::{self, Event, KeyCode, KeyEventKind};
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Alignment, Constraint, Direction, Layout};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{
    Block, Borders, List, ListItem, ListState, Paragraph, Scrollbar, ScrollbarOrientation,
    ScrollbarState,
};
use ratatui::Terminal;
use std::io::{self, stdout};
use std::path::Path;

pub fn run_tui(
    root_dir: &Path,
    mut components: Vec<ProjectComponent>,
    config: ContextConfig,
    initial_selected_idx: usize,
) -> io::Result<()> {
    enable_raw_mode()?;
    let mut stdout = stdout();
    crossterm::execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;

    let mut selected_idx = if initial_selected_idx < components.len() {
        initial_selected_idx
    } else {
        0
    };
    let mut list_state = ListState::default();
    list_state.select(Some(selected_idx));

    let mut status_msg = String::from("Готов к формированию контекста MoCap-проекта.");

    loop {
        list_state.select(Some(selected_idx));
        terminal.draw(|f| {
            render_ui(f, &components, &mut list_state, selected_idx, &status_msg);
        })?;

        if let Event::Key(key) = event::read()? {
            if key.kind != KeyEventKind::Press {
                continue;
            }

            match key.code {
                KeyCode::Char('q') | KeyCode::Char('Q') | KeyCode::Esc => break,
                KeyCode::Up | KeyCode::Char('k') => {
                    if selected_idx > 0 {
                        selected_idx -= 1;
                    }
                }
                KeyCode::Down | KeyCode::Char('j') => {
                    if selected_idx + 1 < components.len() {
                        selected_idx += 1;
                    }
                }
                KeyCode::Right | KeyCode::Char(' ') => {
                    if let Some(comp) = components.get_mut(selected_idx) {
                        comp.cycle_next();
                    }
                }
                KeyCode::Left => {
                    if let Some(comp) = components.get_mut(selected_idx) {
                        comp.cycle_prev();
                    }
                }
                // Пресет 1: Полный дамп всего репозитория
                KeyCode::Char('1') => {
                    for c in &mut components {
                        c.level = DetailLevel::Full;
                    }
                    status_msg = String::from("Пресет 1: Полный дамп всего проекта (FULL)");
                }
                // Пресет 2: Только прошивка и софт
                KeyCode::Char('2') => {
                    for comp in &mut components {
                        match comp.name.as_str() {
                            "firmware" | "software" | "root" => comp.level = DetailLevel::Full,
                            _ => comp.level = DetailLevel::ManifestOnly,
                        }
                    }
                    status_msg = String::from("Пресет 2: Прошивка и ПО верхнего уровня");
                }
                // Пресет 3: Только документация
                KeyCode::Char('3') => {
                    for comp in &mut components {
                        match comp.name.as_str() {
                            "docs" | "scripts" | "root" => comp.level = DetailLevel::Full,
                            _ => comp.level = DetailLevel::ManifestOnly,
                        }
                    }
                    status_msg = String::from("Пресет 3: Документация и спецификации");
                }
                // Отчет изменений Git (changes.txt)
                KeyCode::Char('c') | KeyCode::Char('C') => {
                    match generate_changes_report(root_dir, &config) {
                        Ok((filename, bytes)) => {
                            status_msg = format!("✔ Сформирован отчет: {} ({} байт)", filename, bytes);
                        }
                        Err(e) => status_msg = format!("ОШИБКА: {e}"),
                    }
                }
                // Срез для выбранного компонента
                KeyCode::Char('d') | KeyCode::Char('D') => {
                    if let Some(comp) = components.get(selected_idx) {
                        match generate_supplement_drop(root_dir, comp, &config) {
                            Ok((filename, count)) => {
                                status_msg = format!("✔ Срез [{}] (файлов: {}): {}", comp.level.label(), count, filename);
                            }
                            Err(e) => status_msg = format!("ОШИБКА: {e}"),
                        }
                    }
                }
                // Сборка итогового project_context.txt
                KeyCode::Enter => {
                    match generate_context_file(root_dir, &components, &config, None) {
                        Ok((filename, count)) => {
                            status_msg = format!("✔ УСПЕШНО: Контекст сохранен в {} (файлов: {}). Выход...", filename, count);
                            let _ = terminal.draw(|f| {
                                render_ui(f, &components, &mut list_state, selected_idx, &status_msg);
                            });
                            std::thread::sleep(std::time::Duration::from_millis(1000));
                            break;
                        }
                        Err(e) => status_msg = format!("ОШИБКА: {e}"),
                    }
                }
                _ => {}
            }
        }
    }

    let session = SessionState::from_components(selected_idx, &components);
    session.save_to_file(root_dir);

    disable_raw_mode()?;
    crossterm::execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;

    Ok(())
}

fn render_ui(
    f: &mut ratatui::Frame,
    components: &[ProjectComponent],
    list_state: &mut ListState,
    selected_idx: usize,
    status_msg: &str,
) {
    let size = f.area();

    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Min(4),
            Constraint::Length(2),
        ])
        .split(size);

    let title_text = Paragraph::new("MOCAP-CONTEXT MANAGER :: Менеджер сборки контекста ESP32 MoCap")
        .alignment(Alignment::Center)
        .style(Style::default().add_modifier(Modifier::BOLD).fg(Color::Cyan))
        .block(Block::default().borders(Borders::ALL).style(Style::default().fg(Color::Cyan)));
    f.render_widget(title_text, chunks[0]);

    let main_chunks = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(28), Constraint::Percentage(72)])
        .split(chunks[1]);

    let presets = vec![
        ListItem::new(Line::from(vec![
            Span::styled("[1] ", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
            Span::raw("Полный дамп (FULL)"),
        ])),
        ListItem::new(Line::from(vec![
            Span::styled("[2] ", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
            Span::raw("Прошивка и ПО (Core)"),
        ])),
        ListItem::new(Line::from(vec![
            Span::styled("[3] ", Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
            Span::raw("Документация (Docs)"),
        ])),
    ];

    let presets_list = List::new(presets)
        .block(Block::default().borders(Borders::ALL).title(" Пресеты [Клавиши 1-3] "))
        .style(Style::default().fg(Color::White));
    f.render_widget(presets_list, main_chunks[0]);

    let inner_height = main_chunks[1].height.saturating_sub(2) as usize;
    if inner_height > 0 {
        if inner_height >= components.len() {
            *list_state.offset_mut() = 0;
        } else {
            let mut offset = list_state.offset();
            if selected_idx < offset {
                offset = selected_idx;
            } else if selected_idx >= offset + inner_height {
                offset = selected_idx.saturating_sub(inner_height - 1);
            }
            if offset + inner_height > components.len() {
                offset = components.len().saturating_sub(inner_height);
            }
            *list_state.offset_mut() = offset;
        }
    }

    let items: Vec<ListItem> = components
        .iter()
        .enumerate()
        .map(|(idx, comp)| {
            let is_selected = idx == selected_idx;
            let level_color = match comp.level {
                DetailLevel::Full => Color::Green,
                DetailLevel::NoTests => Color::Cyan,
                DetailLevel::TestsOnly => Color::LightBlue,
                DetailLevel::ContractOnly => Color::Magenta,
                DetailLevel::ManifestOnly => Color::Yellow,
                DetailLevel::Skip => Color::DarkGray,
            };

            let marker = if is_selected { "> " } else { "  " };

            let line = Line::from(vec![
                Span::styled(marker, Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD)),
                Span::styled(
                    format!("{:<16}", comp.name),
                    if is_selected {
                        Style::default().fg(Color::White).add_modifier(Modifier::BOLD)
                    } else {
                        Style::default().fg(Color::Gray)
                    },
                ),
                Span::raw(" ["),
                Span::styled(format!("{:^10}", comp.level.label()), Style::default().fg(level_color).add_modifier(Modifier::BOLD)),
                Span::raw("] "),
                Span::styled(format!("- {}", comp.description), Style::default().fg(Color::DarkGray)),
            ]);

            ListItem::new(line)
        })
        .collect();

    let comp_list = List::new(items)
        .block(Block::default().borders(Borders::ALL).title(" Компоненты MoCap [Пробел: переключить] "))
        .highlight_style(Style::default().bg(Color::Rgb(20, 30, 45)));

    f.render_stateful_widget(comp_list, main_chunks[1], list_state);

    if components.len() > inner_height {
        let mut scrollbar_state = ScrollbarState::default()
            .content_length(components.len())
            .position(selected_idx);
        f.render_stateful_widget(
            Scrollbar::default()
                .orientation(ScrollbarOrientation::VerticalRight)
                .begin_symbol(Some("▲"))
                .end_symbol(Some("▼")),
            main_chunks[1],
            &mut scrollbar_state,
        );
    }

    let status_span = if status_msg.starts_with('✔') {
        Span::styled(status_msg, Style::default().fg(Color::Green).add_modifier(Modifier::BOLD))
    } else if status_msg.starts_with("ОШИБКА") {
        Span::styled(status_msg, Style::default().fg(Color::Red).add_modifier(Modifier::BOLD))
    } else {
        Span::styled(format!("● {status_msg}"), Style::default().fg(Color::Yellow))
    };

    let footer_lines = vec![
        Line::from(status_span),
        Line::from(Span::styled(
            "[Enter] Собрать context.txt  |  [C] Отчет changes.txt  |  [D] Срез выбранного  |  [1-3] Пресеты  |  [Пробел/←/→] Уровень  |  [Q/Esc] Выход",
            Style::default().fg(Color::DarkGray),
        )),
    ];

    let footer_para = Paragraph::new(footer_lines).alignment(Alignment::Left);
    f.render_widget(footer_para, chunks[2]);
}