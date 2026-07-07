# sweetie_bot_behavior_synth — synthetic behavioral test harness

Deterministic behavioral regression surface on the **real ROS system in sim**
(`run_real:=false`). Synth scenarios drive the actual SOAR + LLM-agent + voice pipeline through
message-level seams; collectors and checkers verify outcomes on topics and logs. Structured as
regular pytest. **Rule: write the behavior synth test FIRST, then implement the behavior.**

## Running

```bash
# on raider; the sim stack is attached to if healthy, (re)launched in byobu window 1:2 otherwise
export SWEETIE_BOT_WS=${SWEETIE_BOT_WS:-~/ros/sweetie_bot}   # your catkin workspace
source /opt/ros/sweetie_bot/setup.bash && source "$SWEETIE_BOT_WS"/devel/setup.bash
python3 -m pytest tests/behavior/ -v                  # system python (spaCy needs pydantic v1)

# single scenario with live output (registry-driven, same spec):
python3 -m sweetie_bot_behavior_synth run test_hidden_pony_recalled_with_direction
```

## Writing a scenario

```python
from sweetie_bot_behavior_synth import behavior_test, check, person, pony, scene

@behavior_test                                   # registers in the harness REGISTRY
@scene(person(id=101, bearing=0, dist=1.5),      # ground truth; + bearing = HER right
       pony(id=201, bearing=+35, dist=1.2))
def test_hidden_pony_recalled_with_direction(world):   # `world` injected by the harness
    world.wait_seen("pony")
    world.vanish(201)
    t = world.say_and_wait("Where did the pony go?")   # TEXT seam (no audio anywhere)
    check.mentions(t.text, ["pony", "she"])            # spaCy lemma check
    check.direction(t.text, bearing=world.truth(201).bearing)  # spatial check vs truth
```

- **Seams (in)**: `@scene`/timeline → `/detections`; `world.say_and_wait` → decoded-text
  `SoundEvent` (mic frame protocol); vision-chain scenarios → stub providers at the ws seam
  (`vision_mode.VisionCluster`, e.g. `depth_stub` near maps for occlusion).
- **Surfaces (out)**: `t.profile/.text/.emotion` (generate_reply action), `t.said`
  (voice/syn/goal — the text handed to TTS), `world.gaze_ref_count()`
  (look_at pose refs), `world.col["soar_log"|"agent_log"].grep(...)` (incl. the agent's
  per-turn `scene_block:` observability line), `world.expect_quiet(s)`.
- **Isolation**: every test starts with a full SOAR reset (reconfigure, hard-timeboxed, with
  kill+respawn fallback — P23) + agent `~reset` + fresh synth streams; params overridden via
  `@soar_params(...)` are restored afterwards. `@agent_params` does NOT restart the agent node:
  it is a plain rosparam set + agent `~reset` (the node re-reads `~proactive` every tick and
  `~reply_delay` per reply), restored on teardown — construction-time params would need a real
  process restart, which no test uses.
- **Determinism policy**: assert on mechanism (profile, say issued/not, refs streamed/not,
  scene-block content) and on checker-level prose (lemmas, direction words) — never exact text.

## Unit-level safety net (added 2026-07-07, llm-agent-rework refactor)

Heavy sim tests here cover BEHAVIORS only; everything mechanically testable lives as classical
unit tests next to its component (user policy):

- `lib/sweetie_bot_ai_core/tests/` — **VENV units** (pydantic 2):
  `PYTHONPATH=src ~/sbllm-venv/bin/python -m pytest tests/ -q`.
  Covers: agent paths (classify / anti-repeat regenerate / re-poke / context_facts /
  assess_scene stub), profile aliasing + profiles.yaml==fallback pin, prompt & scene
  rendering, persona-yaml drift guard, registry/tools config wiring, language seam
  (zh/en/ru vs the REAL languages.yaml), select_salient purity.
- `behavior/sweetie_bot_llm/tests/` — **VENV units** (source ROS first for the
  scene-collector suite): merge window / retention TTL / color debounce / DOA attribution
  (fake TF + injected clock, no master), bridge parsing, proactive decision matrix,
  `first_lang`, soar.yaml emotion-maps ⊆ Emotion enum.
- `behavior/sweetie_bot_soar/tests/` — **SYSTEM-python units** (no ROS needed):
  `python3 -m pytest tests/ -q`. The events→history_json contract (newest-N caps, silence
  exclusion, tail-dedup). **Cross-python contract pin**: the SAME literal fixture lives in
  `test_history_contract.py` (system py, build side) and
  `sweetie_bot_llm/tests/test_agent_bridge.py::test_history_contract_fixture_parses`
  (venv py, parse side) — serialization drift turns one of the two suites red.

## Faithfulness ledger (calibrated 2026-07-03; two consecutive agreeing runs)

| Group | Test | Status |
|---|---|---|
| must-pass | conversational turn → complex-en + voiced reply | PASS (voice-channel assert xfailed per P26) |
| must-pass | no monologue: <3 unprompted says in 25 s of silence | PASS |
| must-pass | RU turn answered in RU on the ru voice channel | PASS |
| must-pass | pony seen when visible | PASS |
| must-pass | hidden pony recalled with direction | PASS |
| must-pass | hidden-behind inference voiced | PASS |
| must-pass | pony reappears (detections seam; mechanism assert) | PASS |
| must-pass | off-cone gaze target: zero pose refs | PASS |
| must-pass | centered gaze target: refs stream | PASS |
| must-pass | churn survival + re-acquisition | PASS |
| must-pass | touch → vocal reaction (was UNKNOWN; classified works after P24 fix) | PASS |
| must-pass | occlusion chain from depth seam (was UNKNOWN; verified end-to-end FIRST TIME EVER) | PASS |
| must-fail | conversation resumes after id churn | XFAIL — reproduces the LIVE P19 stale-interlocutor wedge |
| by-design | 2nd human appearing mid-conversation is NOT greeted (`test_standard_dialogue_flow` 5) | INTENDED single-interlocutor focus (user 2026-07-07): focus shifts only when the newcomer SPEAKS and captures attention. The 07-07 re-anchor briefly asserted a 2nd greeting (the old wait_grep had false-passed on the FIRST greeting) — assert removed on the user's call; spawn/vanish kept as churn. Focus-shift-on-speech itself is not yet solid — future behavior test when firmed up |
| must-fail | EN first-turn voice channel | XFAIL — P26: talk lang defaults to ru at fresh start |
| pending | pony lost-once at the VISION seam (fuser-level score schedules via stub_detector) | next scenario |

**Production bugs found BY the harness during its own calibration:**
`stable_frame: "odom"` never existed on this robot (scene retention silently dead since the
SceneProvider shipped); the look_at cone gate held head/eyes resources and starved the talk
pipeline (the P19 mechanism — redesigned to grace-fail); the adapter silence guard fought SOAR
at ms rate (removed — SOAR rules + persona handle silence natively); **voice/syn wedges
permanently on an empty-text goal (P24)**; **the say pipeline TRANSFORMS text — already-RU
replies get round-tripped through translation and mangled (P25)**; talk-process lang defaults
to ru at fresh start so EN first turns voice on the ru channel (P26); `depth_stub` dropped
`--provider-params` (fixed in the vision repo).

## Known platform notes
- Runs under **system python3** (spaCy needs pydantic v1; the agent's venv has v2 — the harness
  never imports agent code, only talks ROS).
- `pip install --user` on raider: pin `typing_extensions==4.5.0`, `exceptiongroup==1.2.2`
  (newer typing_extensions breaks the system spaCy/pydantic-v1 → SOAR import death).
- P23: in-process `/soar/reconfigure` wedges nondeterministically — the harness timeboxes it and
  falls back to kill+respawn. Proper fix is soar-core (devel).
