#!/usr/bin/env bash

set -e

NANOPB_VERSION="0.4.9.1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRMWARE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "Setting up nanopb ${NANOPB_VERSION}..."

# Detect OS and architecture
OS=$(uname -s)
ARCH=$(uname -m)

# Determine the correct nanopb package
if [ "$OS" = "Linux" ]; then
    if [ "$ARCH" = "x86_64" ]; then
        NANOPB_PACKAGE="nanopb-${NANOPB_VERSION}-linux-x86"
    elif [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
        NANOPB_PACKAGE="nanopb-${NANOPB_VERSION}-linux-aarch64"
    else
        echo "Unsupported Linux architecture: $ARCH"
        exit 1
    fi
elif [ "$OS" = "Darwin" ]; then
    if [ "$ARCH" = "x86_64" ]; then
        NANOPB_PACKAGE="nanopb-${NANOPB_VERSION}-macosx-x86"
    elif [ "$ARCH" = "arm64" ]; then
        NANOPB_PACKAGE="nanopb-${NANOPB_VERSION}-macosx-arm64"
    else
        echo "Unsupported macOS architecture: $ARCH"
        exit 1
    fi
elif [[ "$OS" == MINGW* ]] || [[ "$OS" == MSYS* ]] || [[ "$OS" == CYGWIN* ]]; then
    NANOPB_PACKAGE="nanopb-${NANOPB_VERSION}-windows-x86"
else
    echo "Unsupported operating system: $OS"
    exit 1
fi

NANOPB_URL="https://jpa.kapsi.fi/nanopb/download/${NANOPB_PACKAGE}.tar.gz"
NANOPB_DIR="${FIRMWARE_ROOT}/nanopb-${NANOPB_VERSION}"

# Check if nanopb is already set up correctly
if [ -d "$NANOPB_DIR" ] && [ -f "$NANOPB_DIR/generator-bin/protoc" ]; then
    echo "nanopb ${NANOPB_VERSION} is already set up correctly at $NANOPB_DIR"
    exit 0
fi

# Backup existing directory if it exists but doesn't have generator-bin
if [ -d "$NANOPB_DIR" ] && [ ! -f "$NANOPB_DIR/generator-bin/protoc" ]; then
    echo "Backing up existing nanopb directory (source version) to ${NANOPB_DIR}-source"
    rm -rf "${NANOPB_DIR}-source"
    mv "$NANOPB_DIR" "${NANOPB_DIR}-source"
fi

# Download nanopb
echo "Downloading ${NANOPB_PACKAGE}..."
cd "$FIRMWARE_ROOT"
wget -q --show-progress "$NANOPB_URL" -O "${NANOPB_PACKAGE}.tar.gz"

# Extract nanopb
echo "Extracting ${NANOPB_PACKAGE}..."
tar -xzf "${NANOPB_PACKAGE}.tar.gz"

# Rename to standard directory name if needed
if [ "$NANOPB_PACKAGE" != "nanopb-${NANOPB_VERSION}" ]; then
    mv "$NANOPB_PACKAGE" "nanopb-${NANOPB_VERSION}"
fi

# Clean up archive
rm "${NANOPB_PACKAGE}.tar.gz"

# Verify installation
if [ -f "$NANOPB_DIR/generator-bin/protoc" ]; then
    echo "✓ nanopb ${NANOPB_VERSION} successfully installed at $NANOPB_DIR"
    echo "You can now run ./bin/regen-protos.sh"
else
    echo "✗ Installation failed: protoc binary not found"
    exit 1
fi
