#!/usr/bin/env bash
set -euo pipefail

# ──────────────────────────────────────────────────────────────────
# grpc-setup.sh — DEV ONLY. Install protoc and regenerate the Go
# gRPC/protobuf stubs after editing gen/proto/cyberia.proto. Not used by the
# build or the image: `go build` compiles the committed gen/proto/*.pb.go, and
# gRPC is a pure-Go module. Run go-setup.sh first for the Go code-gen plugins.
#
# Usage:
#   ./scripts/grpc-setup.sh
#   ./scripts/grpc-setup.sh --skip-dnf   # protoc already installed
# ──────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROTO_DIR="${PROJECT_ROOT}/gen/proto"

SKIP_DNF=0
[[ "${1:-}" == "--skip-dnf" ]] && SKIP_DNF=1

# ── protoc (compiler only; RHEL/Rocky/Alma/Fedora) ────────────────
if [[ "$SKIP_DNF" -eq 0 ]]; then
  echo ">>> Installing protobuf-compiler (protoc)..."
  sudo dnf install -y protobuf-compiler
fi

# ── Verify protoc + Go plugins (plugins come from go-setup.sh) ─────
GOBIN="$(go env GOPATH 2>/dev/null)/bin"
[[ ":$PATH:" == *":${GOBIN}:"* ]] || export PATH="$PATH:${GOBIN}"
for bin in protoc protoc-gen-go protoc-gen-go-grpc; do
  command -v "$bin" &>/dev/null || { echo "ERROR: $bin not found. Run ./scripts/go-setup.sh first." >&2; exit 1; }
done
echo ">>> protoc: $(protoc --version)"

# ── Generate Go stubs from cyberia.proto ──────────────────────────
[[ -f "${PROTO_DIR}/cyberia.proto" ]] || { echo "ERROR: ${PROTO_DIR}/cyberia.proto not found." >&2; exit 1; }

echo ">>> Generating Go protobuf + gRPC stubs..."
protoc \
  --proto_path="${PROTO_DIR}" \
  --go_out="${PROJECT_ROOT}" --go_opt=module=cyberia-server \
  --go-grpc_out="${PROJECT_ROOT}" --go-grpc_opt=module=cyberia-server \
  "${PROTO_DIR}/cyberia.proto"

echo ">>> Generated ${PROTO_DIR}/cyberia.pb.go + cyberia_grpc.pb.go — commit them."
