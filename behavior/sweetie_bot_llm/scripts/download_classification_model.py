#!/usr/bin/env python3
import json
import os
import sys
from huggingface_hub import snapshot_download

MODEL_DIR = os.environ.get(
    "MODEL_DIR",
    "/root/models/classification/Bert-Base-Emotion-Sentiment-Analysis",
)
MODEL_REPO = os.environ.get(
    "CLASSIFICATION_MODEL_REPO",
    "Thamognya/Bert-Base-Emotion-Sentiment-Analysis",
)

REQUIRED_FILES = [
    "config.json",
]

def fix_config_json(model_dir: str) -> None:
    config_path = os.path.join(model_dir, "config.json")
    if not os.path.isfile(config_path):
        return

    with open(config_path, "r", encoding="utf-8") as f:
        config = json.load(f)

    changed = False

    id2label = config.get("id2label")
    label2id = config.get("label2id")

    # If id2label keys are label names instead of numeric ids, fix it.
    if isinstance(id2label, dict):
        bad_id2label = False
        for key in id2label.keys():
            try:
                int(key)
            except Exception:
                bad_id2label = True
                break

        if bad_id2label:
            fixed_id2label = {}
            fixed_label2id = {}

            # Case: id2label is actually label -> id
            for label, idx in id2label.items():
                fixed_id2label[str(idx)] = label
                fixed_label2id[label] = int(idx)

            config["id2label"] = fixed_id2label
            config["label2id"] = fixed_label2id
            changed = True

    # If label2id exists but id2label does not, rebuild id2label.
    if not changed and isinstance(label2id, dict) and isinstance(id2label, dict):
        # keep as-is
        pass

    if changed:
        with open(config_path, "w", encoding="utf-8") as f:
            json.dump(config, f, ensure_ascii=False, indent=2)
        print(f"fixed config.json label mappings: {config_path}")

def main() -> int:
    missing = [name for name in REQUIRED_FILES if not os.path.isfile(os.path.join(MODEL_DIR, name))]
    if missing:
        os.makedirs(MODEL_DIR, exist_ok=True)
        print(f"downloading classification model {MODEL_REPO} -> {MODEL_DIR}")
        snapshot_download(
            repo_id=MODEL_REPO,
            local_dir=MODEL_DIR,
        )
    else:
        print(f"classification model already present: {MODEL_DIR}")

    fix_config_json(MODEL_DIR)

    missing_after = [name for name in REQUIRED_FILES if not os.path.isfile(os.path.join(MODEL_DIR, name))]
    if missing_after:
        print(f"required files are missing: {missing_after}", file=sys.stderr)
        return 1

    return 0

if __name__ == "__main__":
    raise SystemExit(main())
