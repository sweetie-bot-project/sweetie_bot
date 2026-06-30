# LLM rework — implementation status & verification

Branch `llm-agent-rework`. Developed on swai `~/c/sweetie_bot` (5 commits), mirrored to and
**verified on proto3** in the working checkout there. Nothing pushed.

## Done & verified
- **Phase 0** — ollama LLM provider; `qwen2.5:7b` on proto3 (16 GB shared GPU) / `14b` validated on
  swai. Structured JSON output, native tool calling, streaming, EN/RU.
- **Phase 1** — `lib/sweetie_bot_ai_core` (ROS-free). 12 offline tests pass on swai (py3.12) AND
  proto3 (py3.8 + pydantic 2.9).
- **Phase 2** — `GenerateReply.action`; `llm_agent` node (action server, single-flight + preempt,
  `~set_persona`); `state_collector`; `tool_adapters`; SOAR `lang.py` rewritten as thin adapter
  (rules UNCHANGED; legacy preserved as `lang_legacy.py`; `backend: legacy|agent` flag).
- **Phase 3** — provider failover + circuit-breaker (offline + live dead→real).
- **Phase 4** — old LLM retired: the `sweetie_bot_llm.service` (text-gen-webui llama-13b) inside the
  `sweetie_bot_services` container was stopped, freeing ~9 GB. (BERT classification retirement still
  to do when its consumer is removed; `/classification` stub kept.)

## proto3 live verification (real ROS1 + SOAR/SML)
Reached proto3 via the swai jump (`ssh -o ProxyJump=swai mutr@raider`). `catkin build` OK
(`GenerateReply` generated). Runtime split: SOAR process = system py3.8 + pydantic-v1 + spaCy + SML;
agent node = `~/sbllm-venv` (py3.8 + pydantic 2.9, `--system-site` for rospy). Provider = ollama
container `sbllm` (`OLLAMA_CONTEXT_LENGTH=8192`, `keep_alive=-1`).
- **Node-level** (`test/integration/test_generate_reply_client.py`): GenerateReply action → valid
  structured replies, `get_robot_state` tool dispatch, `context_facts`, native RU; all `err=0`.
- **SOAR adapter** (`test/integration/test_soar_adapter.py`): real SML command WMEs → `lang.py`
  `AgentLangModel` → live agent → wrote `result`/`emotion`/`sentence-type` WMEs, `status=succeed`;
  3/3 cases (simple/complex/failsafe-en), valid emotion enum + sentence-type.

Full path proven: SOAR command → rewritten lang.py → GenerateReply → ai_core agent (7B) → structured
reply + tools → contract WMEs back (consumed by the unchanged SOAR rules).

## Not done (by design / environment)
- Full operator-GUI sim (`autonomous_control.launch`) isn't headless-runnable (rviz/joystick/moveit/
  orocos) — the **dialogue path** it would exercise is instead verified directly via the SOAR-adapter
  SML test above, which is the faithful equivalent for the LLM rework.
- Production launch wiring into the deployed system (adding `agent.launch` to the autonomous launch,
  pointing the system at ollama) and BERT removal are deployment follow-ups.

## Current proto3 state
Old LLM service stopped (`inactive`). ollama `sbllm` container up with `qwen2.5:7b` (the new
provider). No stray rosmaster. Branch committed, not pushed.
