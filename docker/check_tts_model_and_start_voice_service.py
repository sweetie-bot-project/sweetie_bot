#!/usr/bin/env python3
from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

from TTS.api import TTS


def env(name: str, default: str) -> str:
    value = os.environ.get(name, default)
    return value.strip() if isinstance(value, str) else default


def model_dir_from_name(model_name: str) -> Path:
    return Path("/root/.local/share/tts") / model_name.replace("/", "--")


def is_complete(model_dir: Path, model_file: str, config_file: str, speakers_file: str) -> bool:
    required = [
        model_dir / config_file,
        model_dir / model_file,
    ]
    if speakers_file:
        required.append(model_dir / speakers_file)
    return all(p.is_file() and p.stat().st_size > 0 for p in required)


def debug_state(model_dir: Path, model_file: str, config_file: str, speakers_file: str) -> None:
    for name in filter(None, [config_file, model_file, speakers_file]):
        p = model_dir / name
        print(
            f"  {p}: exists={p.exists()} size={p.stat().st_size if p.exists() else 'n/a'}",
            file=sys.stderr,
            flush=True,
        )


def main() -> int:
    python_bin = env("TTS_PYTHON_BIN", "/opt/venv-tts/bin/python")
    server_py = env(
        "TTS_SERVER_PY",
        "/opt/venv-tts/lib/python3.10/site-packages/TTS/server/server.py",
    )

    model_name = env("TTS_MODEL_NAME", "tts_models/en/vctk/vits")
    port = env("TTS_PORT", "5003")
    use_cuda = env("TTS_USE_CUDA", "true")

    model_file = env("TTS_MODEL_FILE", "")
    config_file = env("TTS_CONFIG_FILE", "")
    speakers_file = env("TTS_SPEAKERS_FILE", "")

    model_dir = Path(env("TTS_MODEL_DIR", str(model_dir_from_name(model_name))))

    print(f"[TTS] PORT={port}", flush=True)
    print(f"[TTS] USE_CUDA={use_cuda}", flush=True)
    print(f"[TTS] MODEL_NAME={model_name}", flush=True)
    print(f"[TTS] MODEL_DIR={model_dir}", flush=True)
    print(f"[TTS] MODEL_FILE={model_file}", flush=True)
    print(f"[TTS] CONFIG_FILE={config_file}", flush=True)
    print(f"[TTS] SPEAKERS_FILE={speakers_file}", flush=True)

    explicit_mode = bool(model_file and config_file)
    print(f"[TTS] EXPLICIT_MODE={explicit_mode}", flush=True)

    if not explicit_mode:
        try:
            TTS(model_name=model_name)
        except Exception as e:
            print(f"[TTS] failed to initialize standard model: {e}", file=sys.stderr, flush=True)
            return 1

        cmd = [
            python_bin,
            server_py,
            "--model_name",
            model_name,
            "--port",
            port,
            "--use_cuda",
            use_cuda,
        ]
        print("[TTS] starting standard model server", flush=True)
        return subprocess.run(cmd).returncode

    if not is_complete(model_dir, model_file, config_file, speakers_file):
        print("[TTS] model missing or incomplete, redownloading", flush=True)
        shutil.rmtree(model_dir, ignore_errors=True)
        model_dir.parent.mkdir(parents=True, exist_ok=True)

        try:
            TTS(model_name=model_name)
        except Exception as e:
            print(f"[TTS] failed to redownload explicit-file model: {e}", file=sys.stderr, flush=True)
            return 1

    if not is_complete(model_dir, model_file, config_file, speakers_file):
        print("[TTS] model still broken after redownload", file=sys.stderr, flush=True)
        debug_state(model_dir, model_file, config_file, speakers_file)
        return 1

    cmd = [
        python_bin,
        server_py,
        "--model_path",
        str(model_dir / model_file),
        "--config_path",
        str(model_dir / config_file),
        "--port",
        port,
        "--use_cuda",
        use_cuda,
    ]

    if speakers_file:
        cmd.extend(["--speakers_file_path", str(model_dir / speakers_file)])

    print("[TTS] starting explicit-file model server", flush=True)
    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    raise SystemExit(main())
