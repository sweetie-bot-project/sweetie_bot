#!/usr/bin/env python3
"""Integration test of the SOAR lang.py adapter against the live llm_agent node.

Builds a real SOAR 'lang-model' command WME structure via SML (^request, ^event{talk-heard,text,
initiated-at}, ^text), runs the AgentLangModel hooks (startHook -> sends GenerateReply goal;
updateHook -> polls + writes result/emotion/sentence-type WMEs), and checks the contract WMEs.

Runs in the SOAR system python (pydantic v1 + SML + spaCy). The agent node runs separately in the
pydantic-v2 venv; they communicate only over the ROS action — exactly the production topology.
"""
import time
import rospy
import Python_sml_ClientInterface as sml

rospy.init_node("soar_adapter_test")
from sweetie_bot_soar.output_modules.lang import AgentLangModel  # noqa: E402


def build_cmd(agent, request, human_text, history=None):
    il = agent.GetInputLink()
    cmd = agent.CreateIdWME(il, "lang-model")
    agent.CreateStringWME(cmd, "request", request)
    agent.CreateStringWME(cmd, "text", human_text)
    stamp = 0.0
    for turn in (history or []):
        ev = agent.CreateIdWME(cmd, "event")
        agent.CreateStringWME(ev, "name", turn["name"])
        agent.CreateStringWME(ev, "text", turn["text"])
        if "emotion" in turn:
            agent.CreateStringWME(ev, "emotion", turn["emotion"])
        agent.CreateFloatWME(ev, "initiated-at", stamp)
        stamp += 1.0
    # the current utterance as the last talk-heard event
    ev = agent.CreateIdWME(cmd, "event")
    agent.CreateStringWME(ev, "name", "talk-heard")
    agent.CreateStringWME(ev, "text", human_text)
    agent.CreateFloatWME(ev, "initiated-at", stamp)
    agent.Commit()
    return cmd


def gv(agent, cmd, attr):
    w = cmd.FindByAttribute(attr, 0)
    return w.GetValueAsString() if w is not None else None


def run_case(agent, mod, request, human_text, history=None):
    cmd = build_cmd(agent, request, human_text, history)
    status = mod.startHook(cmd)
    assert status is None, f"startHook returned {status} (expected None / running)"
    final = None
    deadline = time.time() + 75
    while time.time() < deadline:
        final = mod.updateHook(cmd, False)
        if final is not None:
            break
        time.sleep(0.25)
    agent.Commit()
    result = gv(agent, cmd, "result")
    emotion = gv(agent, cmd, "emotion")
    stype = gv(agent, cmd, "sentence-type")
    print(f"[{request}] status={final} emotion={emotion} sentence-type={stype}")
    print(f"   result={result!r}")
    ok = (final == "succeed" and result and
          emotion in ("love", "joy", "surprise", "neutral", "sadness", "fear", "anger") and
          stype in ("question", "statement"))
    print("   -> PASS" if ok else "   -> FAIL")
    # clean the command for the next case
    agent.DestroyWME(cmd)
    agent.Commit()
    return ok


def main():
    kernel = sml.Kernel.CreateKernelInNewThread()
    agent = kernel.CreateAgent("adapter_test")
    mod = AgentLangModel({"action_ns": "generate_reply", "requests": {}, "timeout": 70})
    results = []
    results.append(run_case(agent, mod, "simple-en", "I really love your sparkly mane, Sweetie!"))
    results.append(run_case(agent, mod, "complex-en", "What is your battery level right now?",
                            history=[{"name": "talk-said", "text": "Hi! I am Sweetie Bot!",
                                      "emotion": "joy"}]))
    results.append(run_case(agent, mod, "failsafe-en", "Are you a real pony?"))
    kernel.Shutdown()
    print(f"\nADAPTER TEST: {sum(results)}/{len(results)} cases passed")


if __name__ == "__main__":
    main()
