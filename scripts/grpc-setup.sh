#!/usr/bin/env bash
set -euo pipefail

# ──────────────────────────────────────────────────────────────────
# grpc-setup.sh — Install gRPC / Protocol Buffers toolchain for the
# Go Cyberia Server on RHEL / Rocky Linux / AlmaLinux / Fedora.
#
# Installs:
#   1. protobuf-compiler (protoc) + protobuf-devel via dnf
#   2. grpc system packages (grpc, grpc-devel, grpc-plugins)
#   3. Go protoc code-gen plugins (protoc-gen-go, protoc-gen-go-grpc)
#   4. Go module dependencies (google.golang.org/grpc, protobuf)
#   5. Generates Go stubs from cyberia.proto
#
# Usage:
#   ./cyberia-server/scripts/grpc-setup.sh
#   ./cyberia-server/scripts/grpc-setup.sh --skip-dnf   # skip system packages
#   ./cyberia-server/scripts/grpc-setup.sh --proto-only  # only regenerate stubs
# ──────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROTO_DIR="${PROJECT_ROOT}/proto"

SKIP_DNF=0
PROTO_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-dnf)
      SKIP_DNF=1
      shift
      ;;
    --proto-only)
      PROTO_ONLY=1
      shift
      ;;
    -h|--help)
      echo "Usage: $0 [--skip-dnf] [--proto-only]"
      echo "  --skip-dnf    Skip dnf system package installation"
      echo "  --proto-only  Only regenerate Go stubs from cyberia.proto"
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

# ── 1. Install system packages via dnf ────────────────────────────
if [[ "$PROTO_ONLY" -eq 0 && "$SKIP_DNF" -eq 0 ]]; then
  echo ">>> Enabling EPEL repository..."
  sudo dnf install -y epel-release 2>/dev/null || true
  sudo dnf install -y dnf-plugins-core 2>/dev/null || true

  # Enable CRB/PowerTools for devel headers
  if grep -qi 'Red Hat Enterprise' /etc/os-release 2>/dev/null; then
    sudo dnf config-manager --set-enabled \
      "codeready-builder-for-rhel-$(rpm -E %rhel)-$(uname -m)-rpms" 2>/dev/null || true
  elif grep -qi 'Rocky\|AlmaLinux' /etc/os-release 2>/dev/null; then
    sudo dnf config-manager --set-enabled crb 2>/dev/null || \
      sudo dnf config-manager --set-enabled powertools 2>/dev/null || true
  fi

  echo ">>> Installing protobuf-compiler, grpc, grpc-devel, grpc-plugins..."
  sudo dnf install -y \
    protobuf-compiler \
    protobuf-devel \
    grpc \
    grpc-devel \
    grpc-plugins
fi

# ── 2. Verify Go is available ─────────────────────────────────────
if ! command -v go &>/dev/null; then
  echo "ERROR: Go is not installed. Install Go >= 1.23 first." >&2
  exit 1
fi
echo ">>> Go version: $(go version)"

# ── 3. Install Go protoc plugins ─────────────────────────────────
if [[ "$PROTO_ONLY" -eq 0 ]]; then
  echo ">>> Installing protoc-gen-go and protoc-gen-go-grpc..."
  go install google.golang.org/protobuf/cmd/protoc-gen-go@latest
  go install google.golang.org/grpc/cmd/protoc-gen-go-grpc@latest
fi

# Ensure GOPATH/bin is in PATH for this session
GOBIN="$(go env GOPATH)/bin"
if [[ ":$PATH:" != *":${GOBIN}:"* ]]; then
  export PATH="$PATH:${GOBIN}"
fi

# ── 4. Verify protoc + plugins ───────────────────────────────────
if ! command -v protoc &>/dev/null; then
  echo "ERROR: protoc not found in PATH." >&2
  exit 1
fi
echo ">>> protoc: $(protoc --version)"

if ! command -v protoc-gen-go &>/dev/null; then
  echo "ERROR: protoc-gen-go not found. Check \$GOPATH/bin is in PATH." >&2
  exit 1
fi

if ! command -v protoc-gen-go-grpc &>/dev/null; then
  echo "ERROR: protoc-gen-go-grpc not found. Check \$GOPATH/bin is in PATH." >&2
  exit 1
fi

# ── 5. Install Go module dependencies ────────────────────────────
if [[ "$PROTO_ONLY" -eq 0 ]]; then
  echo ">>> Running go mod tidy in ${PROJECT_ROOT}..."
  cd "${PROJECT_ROOT}"
  go mod tidy
fi

# ── 6. Generate Go stubs from cyberia.proto ──────────────────────
if [[ ! -f "${PROTO_DIR}/cyberia.proto" ]]; then
  echo "ERROR: ${PROTO_DIR}/cyberia.proto not found." >&2
  exit 1
fi

echo ">>> Generating Go protobuf + gRPC stubs..."
protoc \
  --proto_path="${PROTO_DIR}" \
  --go_out="${PROTO_DIR}" --go_opt=paths=source_relative \
  --go-grpc_out="${PROTO_DIR}" --go-grpc_opt=paths=source_relative \
  "${PROTO_DIR}/cyberia.proto"

echo ">>> Generated:"
echo "      ${PROTO_DIR}/cyberia.pb.go"
echo "      ${PROTO_DIR}/cyberia_grpc.pb.go"

echo ">>> gRPC setup complete for cyberia-server."
