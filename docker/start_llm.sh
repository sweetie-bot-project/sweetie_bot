#!/usr/bin/env bash
set -euo pipefail

cd /opt/text-generation-webui

PORT="${LLM_PORT:-5001}"

DEFAULT_MODEL_REPO="TheBloke/LLaMA-13b-GGUF"
DEFAULT_SPECIFIC_FILE="llama-13b.Q2_K.gguf"
DEFAULT_LOADER="llama.cpp"

MODEL_REPO="${LLM_MODEL_REPO:-}"
MODEL_DIR="${LLM_MODEL_DIR:-}"
MODEL_LOCAL="${LLM_MODEL_LOCAL:-}"
SPECIFIC_FILE="${LLM_SPECIFIC_FILE:-}"
LOADER="${LLM_LOADER:-}"

# Default mode if neither repo nor dir is provided
if [ -z "$MODEL_REPO" ] && [ -z "$MODEL_DIR" ]; then
  MODEL_REPO="$DEFAULT_MODEL_REPO"
  SPECIFIC_FILE="$DEFAULT_SPECIFIC_FILE"
  LOADER="$DEFAULT_LOADER"
fi

# Default loader if still empty
LOADER="${LOADER:-llama.cpp}"

resolve_repo_local_name() {
  if [ -n "$MODEL_LOCAL" ]; then
    printf '%s\n' "$MODEL_LOCAL"
  else
    printf '%s\n' "$MODEL_REPO" | tr '/' '_'
  fi
}

run_download_model() {
  if [ -n "$SPECIFIC_FILE" ]; then
    /opt/venv-webui/bin/python download-model.py \
      "$MODEL_REPO" \
      --specific-file "$SPECIFIC_FILE" \
      --output "user_data/models/$MODEL_LOCAL" \
      "$@"
  else
    /opt/venv-webui/bin/python download-model.py "$MODEL_REPO" "$@"
  fi
}

download_text_only() {
  echo "Downloading text-only files for: $MODEL_REPO"
  /opt/venv-webui/bin/python download-model.py "$MODEL_REPO" --text-only
}

download_specific_file() {
  echo "Downloading specific file: $SPECIFIC_FILE"
  /opt/venv-webui/bin/python download-model.py \
    "$MODEL_REPO" \
    --specific-file "$SPECIFIC_FILE" \
    --output "user_data/models/$MODEL_LOCAL"
}

check_specific_file() {
  echo "Checking specific file: $SPECIFIC_FILE"
  local out
  out="$(
    /opt/venv-webui/bin/python download-model.py \
      "$MODEL_REPO" \
      --specific-file "$SPECIFIC_FILE" \
      --output "user_data/models/$MODEL_LOCAL" \
      --check 2>&1 || true
  )"
  printf '%s\n' "$out"

  if grep -qE 'Invalid checksums|Checksum failed|The following file is missing:|\[-\]' <<<"$out"; then
    return 1
  fi
  return 0
}

download_full_model() {
  echo "Downloading full model repo: $MODEL_REPO"
  /opt/venv-webui/bin/python download-model.py "$MODEL_REPO"
}

check_full_model() {
  echo "Checking full model repo: $MODEL_REPO"
  local out
  out="$(/opt/venv-webui/bin/python download-model.py "$MODEL_REPO" --check 2>&1 || true)"
  printf '%s\n' "$out"

  if grep -qE 'Invalid checksums|Checksum failed|The following file is missing:|\[-\]' <<<"$out"; then
    return 1
  fi
  return 0
}

resolve_runtime_target_local_dir() {
  # Local directory mode
  if [ -n "$SPECIFIC_FILE" ]; then
    if [ -f "$MODEL_PATH/$SPECIFIC_FILE" ]; then
      RUNTIME_MODEL_DIR="$MODEL_PATH"
      RUNTIME_MODEL="$SPECIFIC_FILE"
      return 0
    fi
    return 1
  else
    if [ -d "$MODEL_PATH" ]; then
      RUNTIME_MODEL_DIR="$MODEL_DIR"
      RUNTIME_MODEL="$MODEL_LOCAL"
      return 0
    fi
    return 1
  fi
}

resolve_runtime_target_downloaded() {
  # Downloaded-to-user_data mode
  if [ -n "$SPECIFIC_FILE" ]; then
    if [ -f "$MODEL_PATH/$SPECIFIC_FILE" ]; then
      RUNTIME_MODEL_DIR="$MODEL_PATH"
      RUNTIME_MODEL="$SPECIFIC_FILE"
      return 0
    fi
    return 1
  else
    if [ -d "$MODEL_PATH" ]; then
      RUNTIME_MODEL_DIR="$MODEL_DIR"
      RUNTIME_MODEL="$MODEL_LOCAL"
      return 0
    fi
    return 1
  fi
}

# ----------------------------
# Mode 1: local model directory
# ----------------------------
if [ -n "$MODEL_DIR" ]; then
  MODEL_LOCAL="${MODEL_LOCAL:-$(basename "$MODEL_DIR")}"
  MODEL_PATH="$MODEL_DIR"

  echo "MODE=local-dir"
  echo "MODEL_DIR=$MODEL_DIR"
  echo "MODEL_PATH=$MODEL_PATH"
  echo "MODEL_LOCAL=$MODEL_LOCAL"
  echo "PORT=$PORT"
  echo "LOADER=$LOADER"
  echo "SPECIFIC_FILE=$SPECIFIC_FILE"

  if ! resolve_runtime_target_local_dir; then
    echo "Local model is missing in: $MODEL_PATH" >&2
    ls -lah "$MODEL_PATH" >&2 || true
    exit 1
  fi

# ----------------------------
# Mode 2: downloaded model
# ----------------------------
else
  MODEL_LOCAL="$(resolve_repo_local_name)"
  MODEL_DIR="/opt/text-generation-webui/user_data/models"
  MODEL_PATH="$MODEL_DIR/$MODEL_LOCAL"

  echo "MODE=download"
  echo "MODEL_REPO=$MODEL_REPO"
  echo "MODEL_LOCAL=$MODEL_LOCAL"
  echo "MODEL_DIR=$MODEL_DIR"
  echo "MODEL_PATH=$MODEL_PATH"
  echo "PORT=$PORT"
  echo "LOADER=$LOADER"
  echo "SPECIFIC_FILE=$SPECIFIC_FILE"

  mkdir -p "$MODEL_DIR"

  if [ -n "$SPECIFIC_FILE" ]; then
    mkdir -p "$MODEL_PATH"

    if [ ! -f "$MODEL_PATH/$SPECIFIC_FILE" ]; then
      download_text_only
      download_specific_file
    fi

    if ! check_specific_file; then
      echo "Specific file check failed, redownloading..."
      rm -f "$MODEL_PATH/$SPECIFIC_FILE"
      download_text_only
      download_specific_file
    fi

    if ! resolve_runtime_target_downloaded; then
      echo "Specific file is still missing after redownload: $MODEL_PATH/$SPECIFIC_FILE" >&2
      ls -lah "$MODEL_DIR" >&2 || true
      ls -lah "$MODEL_PATH" >&2 || true
      exit 1
    fi

    if ! check_specific_file; then
      echo "Specific file is still invalid after redownload: $MODEL_REPO / $SPECIFIC_FILE" >&2
      exit 1
    fi
  else
    if ! resolve_runtime_target_downloaded; then
      download_full_model
    fi

    if ! check_full_model; then
      echo "Model check failed, redownloading full repo..."
      rm -rf "$MODEL_PATH"
      download_full_model
    fi

    if ! resolve_runtime_target_downloaded; then
      echo "Model is still missing after redownload" >&2
      ls -lah "$MODEL_DIR" >&2 || true
      [ -d "$MODEL_PATH" ] && ls -lah "$MODEL_PATH" >&2 || true
      exit 1
    fi

    if ! check_full_model; then
      echo "Model is still invalid after redownload: $MODEL_REPO" >&2
      exit 1
    fi
  fi
fi

echo "Starting webui with model $RUNTIME_MODEL from $RUNTIME_MODEL_DIR"

exec /opt/venv-webui/bin/python -u server.py \
  --listen \
  --api \
  --listen-port "$PORT" \
  --model-dir "$RUNTIME_MODEL_DIR" \
  --model "$RUNTIME_MODEL" \
  --loader "$LOADER"
