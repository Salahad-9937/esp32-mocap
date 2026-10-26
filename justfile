default: run

# Run MoCap UI application
[no-exit-message]
run:
	cargo run

# Run interactive TUI context manager for MoCap project
ctx:
	cargo run --manifest-path tools/mocap-context/Cargo.toml

# Compile MoCap boundaries specification PDF via Pandoc
doc:
	bash scripts/build_doc.sh