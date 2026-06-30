#!/usr/bin/env bash
# Start the Sweetie Bot LLM provider container (ollama, OpenAI-compatible).
#
# We deliberately use the official, self-contained `ollama/ollama` image rather than building a
# custom CUDA image: it bundles its own CUDA runtime (no duplicated NVIDIA layers) and wraps
# llama.cpp, giving OpenAI-compatible chat, streaming, JSON-schema constrained decoding and
# native tool-call parsing out of the box — minimal footprint. (If we later switch to a built
# llama.cpp `llama-server`, share the vision container's CUDA base per the design doc.)
#
# Host prerequisite (this Proxmox/LXC host): /etc/nvidia-container-runtime/config.toml has
# `no-cgroups = true` (already set for the vision containers).
set -euo pipefail

NAME="${SBLLM_NAME:-sbllm}"
PORT="${SBLLM_PORT:-11434}"
MODEL="${SBLLM_MODEL:-qwen2.5:14b}"          # 7b fallback: qwen2.5:7b
CTX="${SBLLM_CTX:-8192}"                      # bounded KV cache -> fits 16GB shared GPU
VOL="${SBLLM_VOL:-sbllm-models}"

docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run -d --gpus all \
  -v "$VOL":/root/.ollama \
  -p "$PORT":11434 \
  -e OLLAMA_CONTEXT_LENGTH="$CTX" \
  -e OLLAMA_KEEP_ALIVE=-1 \
  --restart unless-stopped \
  --name "$NAME" ollama/ollama

echo "waiting for ollama..."
for _ in $(seq 1 30); do
  curl -sf "http://localhost:${PORT}/api/version" >/dev/null 2>&1 && break || sleep 1
done

echo "pulling ${MODEL} (first run only)..."
docker exec "$NAME" ollama pull "$MODEL"

echo "warming ${MODEL}@${CTX}..."
curl -s "http://localhost:${PORT}/v1/chat/completions" \
  -H 'Content-Type: application/json' \
  -d "{\"model\":\"${MODEL}\",\"messages\":[{\"role\":\"user\",\"content\":\"hi\"}]}" >/dev/null

docker exec "$NAME" ollama ps
echo "LLM provider ready at http://localhost:${PORT}/v1 (model ${MODEL}, ctx ${CTX})"
