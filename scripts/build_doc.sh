#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Read project version from VERSION file or set default
if [ -f "VERSION" ]; then
  APP_VERSION=$(cat VERSION | tr -d '\r\n')
else
  APP_VERSION="0.1.0"
fi

# Directories setup
DOCS_BASE_DIR="docs"
DIST_DIR="docs/dist"
LUA_FILTER="scripts/fix_pdf_links.lua"

# Ensure output distribution folder exists
mkdir -p "$DIST_DIR"

# Function to build a single document folder
build_doc() {
  local doc_name="$1"
  local doc_dir="${DOCS_BASE_DIR}/${doc_name}"

  if [ ! -d "$doc_dir" ]; then
    echo "Error: Directory '$doc_dir' does not exist."
    exit 1
  fi

  local output_pdf="${DIST_DIR}/MoCap_${doc_name}_v${APP_VERSION}.pdf"
  local doc_files=()

  # Option A: Read custom file list if files.txt exists in document directory
  if [ -f "${doc_dir}/files.txt" ]; then
    echo "Using custom file order from ${doc_dir}/files.txt..."
    while IFS= read -r line || [ -n "$line" ]; do
      # Ignore empty lines and comments starting with #
      [[ -z "$line" || "$line" =~ ^[[:space:]]*# ]] && continue
      doc_files+=("$line")
    done < "${doc_dir}/files.txt"
  else
    # Option B: Automatic discovery of markdown files
    echo "Auto-discovering markdown files in ${doc_dir}..."

    # Always put 00-title.md FIRST if it exists in the document root
    if [ -f "${doc_dir}/00-title.md" ]; then
      doc_files+=("${doc_dir}/00-title.md")
    fi

    # Append all remaining markdown files in alphabetical order (excluding 00-title.md and index.md)
    while IFS= read -r -d '' file; do
      doc_files+=("$file")
    done < <(find "$doc_dir" -type f -name "*.md" ! -name "00-title.md" ! -name "index.md" -print0 | sort -z)
  fi

  if [ ${#doc_files[@]} -eq 0 ]; then
    echo "Warning: No markdown files found in '$doc_dir'. Skipping."
    return 0
  fi

  echo "Compiling '${doc_name}' (${#doc_files[@]} files) -> ${output_pdf}..."

  # Pandoc compilation options
  local pandoc_options=(
    --pdf-engine=xelatex
    --lua-filter="$LUA_FILTER"
    -V mainfont="DejaVu Sans"
    -V sansfont="DejaVu Sans"
    -V monofont="DejaVu Sans Mono"
    -V geometry:margin=2cm
    -V colorlinks=true
    -V linkcolor=blue
    -V urlcolor=blue
    -V lang=ru-RU
    --toc-depth=3
  )

  pandoc "${doc_files[@]}" \
    -o "$output_pdf" \
    "${pandoc_options[@]}"

  echo "Successfully built: ${output_pdf}"
}

# Target specified as command-line argument, or build all
TARGET="${1:-all}"

if [ "$TARGET" = "all" ]; then
  echo "Building all documentation in '${DOCS_BASE_DIR}'..."
  for dir in "${DOCS_BASE_DIR}"/*; do
    if [ -d "$dir" ] && [ "$(basename "$dir")" != "dist" ]; then
      build_doc "$(basename "$dir")"
    fi
  done
else
  build_doc "$TARGET"
fi