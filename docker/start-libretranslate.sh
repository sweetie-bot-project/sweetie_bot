#!/bin/sh
set -eu

LOAD_ONLY="${LT_LOAD_ONLY:-${LIBRETRANSLATE_LOAD_ONLY:-}}"

set -- /opt/venv-main/bin/libretranslate --host 0.0.0.0 --port 5002

if [ "${LT_UPDATE_MODELS:-}" = "true" ] || [ "${LT_UPDATE_MODELS:-}" = "1" ]; then
  set -- "$@" --update-models
fi

if [ -n "$LOAD_ONLY" ]; then
  set -- "$@" --load-only "$LOAD_ONLY"
fi

echo "Starting: $*"
exec "$@"
