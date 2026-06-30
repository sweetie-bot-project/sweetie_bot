# LLM agent — live ROS integration tests (proto3)

Manual integration tests run against a live `llm_agent` node + ollama provider on proto3.
They are NOT part of the offline pytest suite (they need a ROS master, the action server, and a
running model).

## Prerequisites
```bash
# SWEETIE_BOT_WS = the catkin workspace holding this checkout (adjust to yours)
export SWEETIE_BOT_WS=${SWEETIE_BOT_WS:-~/ros/sweetie_bot}
# provider (ollama) — see ../../docker/run_llm_provider.sh
# agent node (pydantic-v2 venv):
source /opt/ros/sweetie_bot/setup.bash
source "$SWEETIE_BOT_WS"/devel/setup.bash
source ~/sbllm-venv/bin/activate          # py3.8 + pydantic 2.9 + rospy
roslaunch sweetie_bot_llm agent.launch
```

## test_generate_reply_client.py — node-level
Plain `actionlib` client that sends `GenerateReply` goals (run in the venv). Verifies the node:
structured reply (emotion/sentence_type), `get_robot_state` tool dispatch, `context_facts`,
multilingual routing. Run: `python test_generate_reply_client.py`.

## test_soar_adapter.py — SOAR adapter (SML)
Drives the rewritten SOAR `output_modules/lang.py` (`AgentLangModel`) with real SML command WMEs
(`^request`/`^event`/`^text`) and checks it writes the contract WMEs (`result`/`emotion`/
`sentence-type`) by calling the live agent. Run in the SOAR **system** python (pydantic v1 + spaCy
+ SML), NOT the venv:
```bash
source /opt/ros/sweetie_bot/setup.bash
source "$SWEETIE_BOT_WS"/devel/setup.bash
export PYTHONPATH=/opt/soar:$PYTHONPATH
export LD_LIBRARY_PATH=/opt/soar:$LD_LIBRARY_PATH
python3 test_soar_adapter.py
```

Last run (2026-06-30, proto3, qwen2.5:7b): both tests green — node-level 4/4 calls err=0;
SOAR adapter 3/3 cases (simple/complex/failsafe-en) `succeed` with valid emotion + sentence-type
and `get_robot_state` tool dispatch.
