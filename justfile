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

# Export mannequin 3D mesh from Blender to binary engine container
export-mesh:
	blender -b assets/mannequin.blend -P tools/export_mannequin.py