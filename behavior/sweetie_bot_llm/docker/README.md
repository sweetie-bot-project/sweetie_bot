# Sweetie Bot LLM provider (container)

OpenAI-compatible local LLM, served by **ollama** (wraps llama.cpp). Start with
`./run_llm_provider.sh`. The native ROS agent (`sweetie_bot_llm`) talks to it via the
`sweetie_bot_ai_core` provider registry (local first, cloud `ai.sweetie.bot` fallback).

## Why ollama (not a hand-built llama-server image)
The official `ollama/ollama` image is self-contained (bundles its own CUDA runtime — no
duplicated NVIDIA layers, minimal footprint) and already provides everything we need:
OpenAI `/v1/chat/completions`, streaming, **JSON-schema constrained decoding** (`response_format`)
and **native tool-call parsing**. It does **not** pre-grab VRAM and unloads on idle, which suits a
*shared* GPU. (If we ever need finer grammar/KV control we can switch to a built llama.cpp
`llama-server` sharing the vision container's CUDA base — same OpenAI API, registry unchanged.)

## Model decision (Phase 0, measured on swai RTX 4090)
| Model | ctx | runner VRAM | emotion accuracy |
|-------|-----|-------------|------------------|
| qwen2.5:7b  | 32k | 6.6 GB | weak (mislabels compliments) |
| qwen2.5:14b | 32k | 15 GB  | strong |
| **qwen2.5:14b** | **8k** | **10 GB** | **strong** ✅ default |

**Decision:** `qwen2.5:14b` at `OLLAMA_CONTEXT_LENGTH=8192` (≈10 GB) is the default — it correctly
handles emotion, tool calls and native Russian. On the robot's 16 GB *shared* GPU it fits next to
TTS (~2.8 GB) once the BERT sentiment service is retired (frees ~2.5 GB). `qwen2.5:7b` (~6.6 GB) is
the fallback when VRAM is contested. `OLLAMA_KEEP_ALIVE=-1` keeps the model resident (no cold-load).

Validated end-to-end through the agent: structured `{response_text, emotion, sentence_type}`,
`get_robot_state` tool dispatch, persona in-character, and native EN/RU. No `"\n"` stop token.
