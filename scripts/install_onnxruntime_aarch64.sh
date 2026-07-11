#!/usr/bin/env bash
set -euo pipefail

ONNXRUNTIME_VERSION="${ONNXRUNTIME_VERSION:-1.22.0}"
ARCH="$(uname -m)"
INSTALL_PARENT="${ONNXRUNTIME_PARENT:-$HOME/libs}"
INSTALL_DIR="${ONNXRUNTIME_DIR:-$INSTALL_PARENT/onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}}"
ARCHIVE_NAME="onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz"
DOWNLOAD_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ARCHIVE_NAME}"

if [[ "$ARCH" != "aarch64" && "$ARCH" != "arm64" ]]; then
  echo "ERROR: This installer is for Linux aarch64/arm64, but uname -m returned '$ARCH'." >&2
  exit 5
fi

if [[ -f "$INSTALL_DIR/include/onnxruntime_cxx_api.h" && -f "$INSTALL_DIR/lib/libonnxruntime.so" ]]; then
  echo "ONNX Runtime ${ONNXRUNTIME_VERSION} is already installed at:"
  echo "  $INSTALL_DIR"
  exit 0
fi

mkdir -p "$INSTALL_PARENT"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT
ARCHIVE_PATH="$TMP_DIR/$ARCHIVE_NAME"

if [[ -n "${ONNXRUNTIME_ARCHIVE:-}" ]]; then
  if [[ ! -f "$ONNXRUNTIME_ARCHIVE" ]]; then
    echo "ERROR: ONNXRUNTIME_ARCHIVE does not exist: $ONNXRUNTIME_ARCHIVE" >&2
    exit 5
  fi
  cp "$ONNXRUNTIME_ARCHIVE" "$ARCHIVE_PATH"
else
  command -v curl >/dev/null 2>&1 || {
    echo "ERROR: curl is required to download ONNX Runtime." >&2
    exit 4
  }
  echo "Downloading ONNX Runtime ${ONNXRUNTIME_VERSION} for aarch64..."
  if ! curl -fL --retry 3 --retry-delay 2 --connect-timeout 20 \
      -o "$ARCHIVE_PATH" "$DOWNLOAD_URL"; then
    cat >&2 <<EOF
ERROR: ONNX Runtime download failed.

Expected asset:
  $DOWNLOAD_URL

For an offline install, download the archive on another computer and run:
  ONNXRUNTIME_ARCHIVE=/path/to/$ARCHIVE_NAME $0
EOF
    exit 4
  fi
fi

tar -xzf "$ARCHIVE_PATH" -C "$INSTALL_PARENT"

if [[ ! -f "$INSTALL_DIR/include/onnxruntime_cxx_api.h" || ! -f "$INSTALL_DIR/lib/libonnxruntime.so" ]]; then
  echo "ERROR: Extracted ONNX Runtime layout is incomplete at $INSTALL_DIR" >&2
  exit 6
fi

sha256sum "$ARCHIVE_PATH" > "$INSTALL_DIR/source_archive.sha256"
printf '%s\n' "$DOWNLOAD_URL" > "$INSTALL_DIR/source_url.txt"

echo "Installed ONNX Runtime ${ONNXRUNTIME_VERSION}:"
echo "  $INSTALL_DIR"
