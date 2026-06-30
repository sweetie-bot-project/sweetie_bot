# LLM rework — implementation status & proto3 verification runbook

Branch `llm-agent-rework` (base `88248c48`). Commits: ai_core+provider (Phase 0/1), action+node+
SOAR adapter (Phase 2), launch/deps wiring. **Committed, not pushed.**

## What is done & verified (on swai)
- **Phase 0** — ollama LLM provider; `qwen2.5:14b @ ctx 8192` (~10 GB). Structured JSON output,
  native tool calling, streaming, EN/RU all verified by curl + through the agent.
- **Phase 1** — `lib/sweetie_bot_ai_core` (ROS-free). 12 offline tests + live e2e through 14B.
- **Phase 2 (code)** — `GenerateReply.action`; `llm_agent` node (action server + state collector +
  tool adapters); SOAR `lang.py` rewritten as thin adapter (rules unchanged; legacy preserved as
  `lang_legacy.py`; `backend: legacy|agent` flag, soar.yaml set to `agent`); `agent.launch`.
  `py_compile` clean; 6 bridge tests; live e2e (context_facts, profiles).
- **Phase 3 (failover)** — live failover dead→real provider + circuit-breaker (offline + live).

## Blocked: live ROS/SOAR-sim verification
swai has **no ROS1 Noetic / SOAR (SML)** — only ROS2 + orocos rt_control. The `rospy` node and the
SOAR module cannot run there. Verification target = **proto3_host** (real ROS1 brain), which was
**unreachable** during this session (ZeroTier/VPN flap, `192.168.192.201` no route). Recover the
link (reconnect AmneziaVPN → restart local zerotier → restart proto3 zerotier via swai jump), then:

## proto3 verification runbook (when reachable)
Repo on proto3: `~/repos/sweetie/sweetie_bot_mike`. Bring it to the same state without pull/push:
```bash
# on swai: export the three commits as patches
cd ~/c/sweetie_bot && git format-patch 88248c48..HEAD -o /tmp/llm_rework_patches
scp /tmp/llm_rework_patches/*.patch proto3_host:/tmp/llm_rework_patches/   # via your routing
# on proto3 (confirm it has base 88248c48 first):
cd ~/repos/sweetie/sweetie_bot_mike
git checkout 88248c48 -b llm-agent-rework
git am /tmp/llm_rework_patches/*.patch
```
Build + run (in a byobu window — ask which):
```bash
cd ~/ros/sweetie_bot && catkin build sweetie_bot_text_msgs sweetie_bot_ai_core sweetie_bot_llm sweetie_bot_soar
# install python deps into the ROS env: pip3 install --user "pydantic>=2" requests pyyaml
./behavior/.../docker/run_llm_provider.sh      # or point providers.yaml local url at swai:11434
roslaunch sweetie_bot_llm agent.launch          # start the agent node
roslaunch sweetie_bot_deploy autonomous_control.launch run_real:=false   # sim
```
Checks:
1. `backend: legacy` (flip soar.yaml) → behaviour identical to today (regression baseline).
2. `backend: agent` → talk to Sweetie; watch the `lang-model` cmd WME get
   `status succeed / result / emotion / sentence-type`; eyes/animation still driven only via
   emotion→animation-tag (no double-actuation).
3. Kill local provider → confirm failover to remote.
4. Then Phase 4: retire BERT classification service (free ~2.5 GB); keep `/classification` stub.

## Notes / decisions
- Provider = official `ollama/ollama` image (self-contained CUDA, minimal footprint; no custom base).
- LLM does NOT actuate: emotion WME is the only expressive channel; actuator tools `disabled`.
- Canonical reply language = English; TTS localizes (prevents double-translation).
- `additional_emotion` confirmed unused by rules → dropped.
