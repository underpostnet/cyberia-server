#!/usr/bin/env bash
set -euo pipefail

# ──────────────────────────────────────────────────────────────────
# go-setup.sh — DEV ONLY. Install the Go protoc code-gen plugins used to
# regenerate the gRPC/protobuf stubs. Run this before grpc-setup.sh (which
# invokes protoc against these plugins). Requires Go >= 1.23.
# ──────────────────────────────────────────────────────────────────

command -v go &>/dev/null || { echo "ERROR: Go is not installed (need >= 1.23)." >&2; exit 1; }
echo ">>> Go version: $(go version)"

echo ">>> Installing protoc-gen-go and protoc-gen-go-grpc..."
go install google.golang.org/protobuf/cmd/protoc-gen-go@latest
go install google.golang.org/grpc/cmd/protoc-gen-go-grpc@latest

echo ">>> Installed to $(go env GOPATH)/bin — ensure it is on PATH."
