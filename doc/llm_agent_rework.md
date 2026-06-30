# Sweetie Bot — LLM Subsystem Refactor Plan

## Context

The robot's conversational "agent" runs an aging **llama-13b-4bit-128g (GPTQ)** behind a
text-generation-webui `/v1/completions` server, fronted by a dumb ROS proxy
(`sweetie_bot_llm/completion.py`, service `/llm_request`, srv `CompleteRaw`). All real LLM
logic — prompt building, multi-call regex/keyword/BERT parsing — is baked into SOAR's
`output_modules/lang.py` + `soar.yaml`. Accumulated problems:

- **`"\n"` is a stop token everywhere** (`ai.yaml`, every profile, per-request `stop_list`) →
  any newline truncates the reply. Should stop only on the model's real end token.
- **No structured output** — the `My emotion: X | Sweetie Bot: …End!` format is coaxed by prompt
  text and scraped back with brittle regex + spaCy keyword maps + a separate **BERT sentiment
  service** (`/classification`, ~2.5 GB GPU).
- **No tool calling**, **no dynamic robot-state in the prompt**, static persona files, and
  **on-the-fly persona ("character") switching is a hack** strapped onto the SOAR quest system.
- Old model: weak instruction-following, not multilingual-robust, no native function calling.

We are replacing the whole AI side with a modern, structured, tool-capable LLM agent while
**leaving SOAR's production rules untouched**. Outcome: deterministic JSON output (emotion as a
field, BERT retired), a better multilingual model, dynamic state + tools, clean persona
management, a shared container/provider deployment mirroring the vision system, and a ROS-light
code shape ready for future streaming/barge-in and eventual de-ROSification.

This is the LLM-side refactor only (plus translation routing). No agentic frameworks
(Hermes/OpenCLAW); SOAR core untouched; VLM/MCP/skills are future expansion the design *accommodates*.

## Research reconciliation (deep-research pass)

The deep-research corroborated the stack: the **Gorilla/BFCL tool-calling leaderboard**, the
**Qwen2.5-14B-Instruct-AWQ** model card, and **llama.cpp's function-calling + GBNF/JSON-schema
docs** support the Qwen + llama.cpp + native-grammar/tool-call choice; the **llama.cpp-vs-vLLM**
comparisons confirm vLLM/SGLang pre-allocate VRAM (`gpu_memory_utilization`) and assume a near-
exclusive GPU — ill-suited to our *shared* laptop GPU. (Two stray claims were refuted and are not
load-bearing.) The serving design is model-agnostic, so a final model pick is config-only.

## Constraints & decisions (confirmed with user)

- **Do not change SOAR production rules.** `output_modules/lang.py` + its `soar.yaml` `lang-model`
  block ARE in scope to fully rewrite.
- **Deployment:** model inference server in a **Docker container** (OpenAI-compatible API); agent
  logic as a **native ROS node**. **Share a minimal CUDA base image with the vision container** —
  do not duplicate the NVIDIA/CUDA layers; keep the footprint minimal.
- **Keep local→remote failover** (`ai.sweetie.bot`); the balancer is in scope to improve, but it
  is **shared infra → lives in `lib/`**, not in the LLM package (see below).
- **Multilingual = per-language routing policy:** some languages fed to the model natively, others
  translated to a pivot (EN); configurable. No assumption any one model is great at Russian.
- **Generalize the container/provider pattern** so STT and TTS can be added as providers later
  under the same principle as the LLM.
- **VLM:** not now; design the request contract so a multimodal provider can slot in later
  (optional image input, future `assess_scene` request type).
- **Scope now:** fully implement the structured conversational-reply path; *architect* (don't
  build) extra request types (`classify`, `assess_scene`).
- **Persona switching** must be reworked properly (clean, data-driven, runtime-switchable),
  replacing the current quest-coupled hack.
- **Conversation history** must be saved properly: keep the last N dialogue turns verbatim **plus**
  a rolling summary block for older context (and leverage KV/prefix caching for speed).
- **Design for (don't build):** streaming STT→MT→LLM→MT→TTS with barge-in; LLM skills/MCP/web;
  de-ROSified standalone agent (sim, messengers, TUI); smart-home.
- **Implement + verify on swai** `~/c/sweetie_bot`, branch **`llm-agent-rework`**, in **ROS
  simulation** (`autonomous_control.launch run_real:=false`, virtual motion). **Commit autonomously,
  do NOT push.**

## The preserved SOAR boundary (must not break)

From `soar/unified/verbolize-llm.soar` (rules — **unchanged**):

- SOAR puts an output command on `io.output-link.lang-model <cmd>` carrying `^request <name>` +
  `^event <E>{name,text,emotion,initiated-at}…` + `^predicate <P>…` + `^text`.
- Rules **wait** for the output module to write back onto `<cmd>`: `^status (succeed|error)`,
  `^result <text>`, `^emotion <e>`, `^sentence-type (question|statement)`. Event-driven across
  decision cycles → **async output already supported** (no same-cycle assumption). `error` triggers
  the request-retry chain.
- **Request names are hardcoded in rules** with a priority fallback chain:
  `simple-en` (3) → `failsafe-en` (2) → `complex-en` (1), each tried once, error → next. The new
  output module **must still accept all three names** and honor error→retry. (In practice today
  `complex-en` is the main path; `simple-en`/`failsafe-en` are largely unused — see profiles below.)
- `^emotion` is reused downstream as **`^animation-tag`** → the emotion enum must stay within the
  values the animation/eye system understands: **`love, joy, surprise, neutral, sadness, fear,
  anger`**.
- `additional_emotion` is **not read by any rule** → safe to drop.

Net obligation of the rewritten output module: write `status/result/emotion/sentence-type` onto the
command WME for request names `simple-en|failsafe-en|complex-en`. Everything between that boundary
and the model is ours.

## Target architecture

### Model + serving
- **Model (recommended):** **Qwen2.5-14B-Instruct** (or **Qwen3-14B**) at **Q4_K_M GGUF (~9 GB)** —
  strongest native tool-call template among open models this size + solid Russian/CJK + reliable
  JSON. **Qwen2.5-7B / Qwen3-8B** as a lighter, possibly-*default* variant if latency or VRAM
  coexistence demands it (decided in Phase 0). Model-agnostic serving → swap is config-only.
- **Serving runtime:** **llama.cpp `llama-server`** in a container. For a *shared* 16 GB GPU, GGUF
  uses only what model+KV need (no big pre-grab), unlike vLLM/SGLang. Gives OpenAI-compatible
  `/v1/chat/completions`, native streaming, **native GBNF + JSON-schema constrained decoding**
  (`response_format`), and **native tool-call parsing** via the model's jinja template (`--jinja`).
- **Constrained decoding:** llama.cpp native JSON-schema/grammar for the *content* schema; the
  model's native tool-call path for tools. **Drop `"\n"` stops entirely** — stop on EOS + schema
  completion.

### Container / provider pattern (generalized, shared base)
A uniform **"AI provider container"** abstraction, manifest-described
(`{name, model, port, protocol, capabilities}`):
- **Shared minimal base image** with the vision system: factor the common **CUDA runtime** layer so
  vision (`docker/sweetie-vision.Dockerfile`) and the LLM image share it; the LLM image adds only
  the llama.cpp CUDA server binary + model mount (no pytorch). Minimal footprint.
- **LLM provider** = llama-server container (first concrete provider).
- **Translation provider** = LibreTranslate container (second provider — validates generality).
- **STT / TTS providers** = future containers, same pattern.

### Shared lib vs ROS package (resolves the registry-placement worry)
ROS-free code that STT/TTS will also need must NOT live inside the LLM package. Therefore:
- **`lib/sweetie_bot_ai_core/`** — NEW shared, **ROS-free** Python module (the de-ROSification
  seam). Holds everything reusable headless and by other providers:
  ```
  schema.py       # Pydantic v2: AgentRequest, AgentReply (discriminated union), Emotion enum,
                  #   SentenceType, ToolCall, ToolResult, RobotState
  client.py       # async OpenAI-compatible client (chat, response_format, tools, streaming-ready)
  registry.py     # provider registry: health-check + circuit-breaker + failover (evolves the
                  #   current sweetie_bot_load_balancer). SHARED by LLM / STT / TTS — that's why it's
                  #   in lib/, not in the LLM package.
  prompt.py       # system-prompt builder: active persona + dynamic RobotState block (PURE)
  persona.py      # persona registry + active-persona state (replaces the quest hack)
  tools.py        # tool declarations + per-tool dispatch_mode (execute|propose|disabled)
  translation.py  # per-language routing policy + TranslationProvider interface
  history.py      # conversation memory: N verbatim turns + rolling summary block
  agent.py        # orchestrator: handle(AgentRequest)->AgentReply; injected providers/ports
  ```
  The current `lib/sweetie_bot_load_balancer` is folded into / depended on by this module so the
  registry is shared infra, not a ROS package wedged into a non-ROS pipeline.
- **`behavior/sweetie_bot_llm/`** — thin **ROS glue only**, depends on `sweetie_bot_ai_core`:
  ```
  agent_node.py        # exposes GenerateReply action; single-flight queue + cancel; asyncio bridge
  state_collector.py   # subs battery_state, servo_states, joint_states, clock + system date/time
                       #   -> RobotState (RobotStateProvider impl)
  tool_adapters.py     # EffectorPort impl: info-tools execute; actuator-tools propose/disabled
  ```

**De-ROSification seam:** core exposes `agent.handle(AgentRequest) -> AgentReply` (history/text are
plain data in the request, never pulled from ROS inside core) plus two injected Protocols —
`RobotStateProvider.snapshot() -> RobotState` and `EffectorPort.dispatch(tool_call) -> ToolResult`.
ROS impls live in the ROS package; sim/TUI/messenger impls are stubs. `prompt.py` reads state only
via the provider.

### Output schema (discriminated union — robust with local models)
```
action="reply": { response_text: str, emotion: Emotion, sentence_type: SentenceType }
action="tool" : { tool_calls: [ToolCall], response_text: str|null, emotion: Emotion }
```
- `Emotion` enum = the 7 sanctioned values; **validate & default to `neutral`** on any surprise
  (now the single source for an existing WME).
- Put `response_text` **first** so future streaming can forward spoken text while metadata
  (emotion/sentence_type/tool_calls) resolves at completion.
- Replaces all regex + spaCy + BERT parsing; `sentence_type` replaces the regex `?`-test.

### System prompt + personas (proper rework)
- **Static persona = data, not code.** `config/persona/<name>.yaml`: identity/character, traits,
  behavioral guidelines, emotion-vocabulary usage, tool-usage guidance, default language, voice/TTS
  hints. `core/persona.py` is a **PersonaRegistry** with an **active-persona** state.
- **Runtime switching, clean:** a ROS service (`~set_persona`) and/or an `AgentRequest.persona`
  field selects the active persona; `prompt.py` renders `active_persona + dynamic state`. This
  **replaces the current quest/`character`-coupled hack** — persona becomes a first-class,
  self-contained capability. (The quest system is **deprecated**; the new persona module does not
  depend on or design for it.)
- **Dynamic state block** injected each turn: **current date/time**, battery %/state, servo
  faults/overtemp, current pose, mood — built by `prompt.py` from `RobotStateProvider.snapshot()`.
- **On-demand:** also expose `get_robot_state` info tools so the model can pull detail not in the block.

### Conversation history (saved properly)
- `core/history.py` keeps per-session memory = **last N turns verbatim** + a **rolling summary
  block** that compresses older turns into a short "previously relevant context" paragraph
  (LLM-summarized on overflow; cheap heuristic fallback). The summary block sits atop the verbatim
  window in the prompt.
- Reconcile with SOAR: the request's `^event`s are the recent verbatim turns (SOAR's window); the
  agent's summary covers context beyond that window. Stored history is kept in the **canonical
  language (EN)** to avoid re-translation.
- **Speed:** rely on **llama.cpp slot/prefix KV-cache** so the stable persona + summary prefix isn't
  recomputed each turn; cap turns/tokens to bound KV-cache VRAM.

### Request-name profiles (give simple-en a real job)
The output module accepts the three rule-hardcoded names as **agent profiles**:
- **`complex-en`** = full agent: rich context + dynamic state + tools + history summary (primary path).
- **`simple-en`** = fast lightweight reply: reduced context, no tools, smaller token budget — revived
  as the low-latency quick path.
- **`failsafe-en`** = minimal degraded path: tiny context / canned-style fallback (used when the
  retry chain falls through or the provider is degraded).
- Error-retry chain preserved. Future (rules-changing, out of scope): migrate to a clean
  `request_type` (`reply`/`classify`/`assess_scene`) and deprecate the `*-en` triplet.

### Tool calling — arbitration-safe (critical)
The LLM is a **mind that speaks and feels, not a second pair of hands.** SOAR is the sole arbiter of
shared actuators (eyes/animations/gaze) — it already drives expression from the `emotion` WME.
Direct LLM dispatch would race SOAR (preemption, WM desync, priority inversion). Therefore:
- Per-tool **`dispatch_mode`**: `execute | propose | disabled`.
  - **`execute`** only for non-contended info/side-effect-free tools (`get_robot_state`, future
    `web_search`, smart-home query) — run by `agent_node`.
  - Actuator/expressive intent (animations, look_at, eye expressions) defaults to **`disabled`/
    `propose`** now; expression flows the sanctioned way via **`emotion` WME → animation-tag**. Build
    the `propose` path so it can be enabled later *once rules are extended to consume proposals*
    (future, rules-changing).
- Tool execution is an explicit **allowlist** (a security control vs prompt injection).

### Translation routing — canonical-language invariant
- Policy in `core/translation.py`; execution via a **`TranslationProvider`** interface (default
  LibreTranslate = 2nd provider container).
- `languages.yaml`: `native_languages` the model handles directly vs others translated to `pivot`
  (EN). `AgentRequest` carries explicit `text_language` / `reply_language` so we know what STT did.
- **Invariant (prevents double-translation):** STT/TTS own *user-facing* speech translation; the
  agent only does **pivot translation for model competence** and returns `result` in a **fixed
  canonical language (EN)**; TTS does final localization. Documented as a hard invariant.

### Transport: ROS action, sync-first
- New **`GenerateReply.action`** (in `sweetie_bot_text_msgs`):
  - goal: `request_type` (`reply` now; `classify`/`assess_scene` reserved), `text`, `history`
    (events), `language`, `persona` (optional), optional `sensor_msgs/Image` (future VLM).
  - result: `response_text`, `emotion`, `sentence_type`, `tool_calls`, `error_code`.
  - feedback: reserved for future streaming partial text.
- **Phase 1 calls it synchronously** from `lang.py` (`send_goal`→`wait_for_result` w/ timeout→write
  WMEs) — observably identical to today. The action (not service) gives a **cancel path** (barge-in
  foundation) and a later async upgrade — painful to retrofit onto a service.
- `CompleteRaw`/`completion.py` **stay alive** during migration for parallel-run + rollback.

## Phased migration (parallel-run, low-risk, ROS1)

- **Phase 0 — container + model bring-up (no robot impact).** Build the llama-server container on
  the **shared CUDA base**; **measure real VRAM peak incl. KV cache** alongside TTS (retire BERT
  early if needed); verify chat/stream/JSON-schema and — explicitly — **tool-call × grammar
  coexistence** in llama.cpp (the known rough edge). Set a **latency target** (e.g. first-audio
  < ~1.5 s) that may make 7–8B the default. curl-only.
- **Phase 1 — `lib/sweetie_bot_ai_core`, offline.** Build schema/client/registry/prompt/persona/
  tools/translation/history/agent as pure Python with a **CLI/TUI harness** (doubles as the future
  TUI seam). Golden test: recorded SOAR commands → valid emotion/result/sentence-type; persona
  switch; EN+RU routing; history summary on overflow. `pytest` in the swai venv.
- **Phase 2 — ROS node, synchronous, behind a flag, parallel to CompleteRaw.** Add `agent_node` +
  `GenerateReply.action` + `state_collector`. `lang.py` supports **both** paths via a `soar.yaml`
  `backend: legacy|agent` flag (default legacy); honor `simple-en|failsafe-en|complex-en` +
  error-retry. All actuator tools `disabled`. Verify in **sim** (`run_real:=false`) on swai.
- **Phase 3 — failover + translation hardening.** Exercise local-down→remote via the registry;
  multilingual routing with the canonical-language invariant; confirm no double-translation with
  STT/TTS in the loop.
- **Phase 4 — cutover.** Default `agent`; keep `legacy` one release for rollback. Then **retire
  BERT** (free ~2.5 GB), keeping the `/classification` interface stub for future use.
- **Phase 5+ (design-only now):** async WME-deferral + feedback streaming + barge-in cancel; tool
  `propose` once rules opt in; STT/TTS provider containers; `assess_scene` multimodal/VLM;
  de-ROSified TUI/sim/messenger via the `core/` seam; MCP/skills/web.

## Files to be modified / created (on swai `~/c/sweetie_bot`, branch `llm-agent-rework`)

- **New shared lib** `lib/sweetie_bot_ai_core/` (ROS-free core listed above); **fold in/evolve**
  `lib/sweetie_bot_load_balancer` → `registry.py`.
- **Rewrite** `behavior/sweetie_bot_llm/` to thin ROS glue (`agent_node.py`, `state_collector.py`,
  `tool_adapters.py`); keep `completion.py`/`CompleteRaw` alive until Phase 4; retire
  `classification.py` runtime in Phase 4 (keep `/classification` srv stub).
- **Rewrite** SOAR `behavior/sweetie_bot_soar/src/sweetie_bot_soar/output_modules/lang.py` → thin
  client of `GenerateReply` writing the same `status/result/emotion/sentence-type` WMEs.
- **Edit** `config/sweetie_bot_proto3_deploy/default/soar.yaml` `lang-model` block → `backend` flag
  + request-name→profile mapping (drop the AttribRequest regex/map/classification chain).
- **Edit** `config/sweetie_bot_proto3_deploy/default/ai.yaml` → providers/registry config; remove
  `"\n"` stops; point at the new container.
- **New** `behavior/sweetie_bot_text_msgs/action/GenerateReply.action`.
- **New** `config/persona/*.yaml`, `tools.yaml`, `languages.yaml`, `providers.yaml`; `docker/`
  llama-server Dockerfile (on shared CUDA base) + run scripts/manifests.

## Risks / open items

- **VRAM coexistence + KV cache** and **tool-call × grammar coexistence in llama.cpp** — retire in
  Phase 0; either can force a model/approach change.
- **Latency** of 14B Q4 on a shared GPU vs the old 13B — may make 7–8B the default.
- **Concurrency:** single-flight queue + cancel in `agent_node` (one GPU, `--parallel 1`).
- **History/KV budget:** cap turns/tokens; summary on overflow.
- **Observability:** log raw model output + parsed schema + language route + failover + active persona.
- **Model/template pinning:** pin model + jinja template together in the container manifest; tool-call
  format can change silently on swap — test on swap.
- **Prompt-injection:** tool-execution allowlist as a security control.

## Verification (swai, ROS simulation)

All on swai `~/c/sweetie_bot`, branch `llm-agent-rework`; **commit autonomously, do not push**.

- **Phase 0:** `curl` the container for chat/stream/JSON-schema/tool-call; `nvidia-smi` peak VRAM
  with TTS-class load; latency timing. (Mind the 4090 is shared — check free VRAM first.)
- **Phase 1:** `pytest` on `sweetie_bot_ai_core` with golden recorded SOAR commands → valid
  `Emotion`/`result`/`sentence_type`; CLI/TUI manual chat (EN + RU routing, persona switch, history
  summary).
- **Phase 2 (ROS sim):** launch `config/sweetie_bot_deploy/highlevel/autonomous_control.launch
  run_real:=false` (virtual motion) in a byobu window. `backend: legacy` → diff WMEs vs today on a
  recorded conversation (must be identical). `backend: agent` → watch the `lang-model` command WME
  get `status succeed/result/emotion/sentence-type`; confirm expression driven only via
  emotion→animation-tag (no double-actuation); kill the local container → remote failover; measure
  turn latency within SOAR's cycle.
- **Phase 3:** recorded RU input → inspect language tag at STT→agent→TTS hops for no double-
  translation; force provider failures → registry recovers.
