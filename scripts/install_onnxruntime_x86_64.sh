#!/usr/bin/env bash
set -euo pipefail

ONNXRUNTIME_VERSION="${ONNXRUNTIME_VERSION:-1.22.0}"
ARCHIVE_NAME="onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}.tgz"
DEFAULT_DIR="$HOME/libs/onnxruntime-linux-x64-${ONNXRUNTIME_VERSION}"
ONNXRUNTIME_DIR="${ONNXRUNTIME_DIR:-$DEFAULT_DIR}"
DOWNLOAD_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ARCHIVE_NAME}"
LIBS_DIR="$(dirname "$ONNXRUNTIME_DIR")"
TMP_ARCHIVE="/tmp/${ARCHIVE_NAME}"

case "$(uname -m)" in
  x86_64|amd64) ;;
  *) echo "ERROR: Expected x86_64/amd64; found $(uname -m)." >&2; exit 5 ;;
esac

if [[ -f "$ONNXRUNTIME_DIR/include/onnxruntime_cxx_api.h" ]] \
    && compgen -G "$ONNXRUNTIME_DIR/lib/libonnxruntime.so*" >/dev/null; then
  echo "==> ONNX Runtime ${ONNXRUNTIME_VERSION} x64 is already installed:"
  echo "    $ONNXRUNTIME_DIR"
  exit 0
fi

mkdir -p "$LIBS_DIR"

echo "==> Downloading ONNX Runtime ${ONNXRUNTIME_VERSION} for Linux x86_64"
curl -fL --retry 3 --retry-delay 2 --connect-timeout 20 \
  -o "$TMP_ARCHIVE" "$DOWNLOAD_URL"

echo "==> Extracting ONNX Runtime into $LIBS_DIR"
tar -xzf "$TMP_ARCHIVE" -C "$LIBS_DIR"
rm -f "$TMP_ARCHIVE"

if [[ ! -f "$ONNXRUNTIME_DIR/include/onnxruntime_cxx_api.h" ]]; then
  echo "ERROR: ONNX Runtime header was not installed at:" >&2
  echo "       $ONNXRUNTIME_DIR/include/onnxruntime_cxx_api.h" >&2
  exit 6
fi

if ! compgen -G "$ONNXRUNTIME_DIR/lib/libonnxruntime.so*" >/dev/null; then
  echo "ERROR: ONNX Runtime shared library was not installed under:" >&2
  echo "       $ONNXRUNTIME_DIR/lib" >&2
  exit 6
fi

echo "==> ONNX Runtime installation complete"
echo "    $ONNXRUNTIME_DIR"
