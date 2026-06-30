#!/usr/bin/env python3
"""Headless chat harness for sweetie_bot_ai_core (no ROS).

Doubles as the future TUI seam: it builds the same Agent the ROS node will, but with stub
state/effector. Use it to smoke-test personas, tools, structured output and translation routing.

Examples:
  python3 sbai_chat.py --once "I love your mane!"
  python3 sbai_chat.py --model qwen2.5:14b
  python3 sbai_chat.py --url http://localhost:11434/v1 --persona sweetie
"""
import argparse
import sys

from sweetie_bot_ai_core import (Agent, AgentRequest, LanguagePolicy, PersonaRegistry,
                                 ProviderRegistry, RobotState, ToolRegistry, build_llm_registry)
from sweetie_bot_ai_core.schema import TalkTurn


class DemoState:
    def snapshot(self) -> RobotState:
        return RobotState(datetime_iso="2026-06-30T16:00:00", weekday="Tuesday",
                          battery_percent=82.0, battery_status="discharging",
                          pose="body_nominal", moving=False, mood="cheerful")


class DemoEffector:
    def __init__(self, state: RobotState):
        self.state = state

    def dispatch(self, tool_call):
        from sweetie_bot_ai_core.schema import ToolResult
        if tool_call.name == "get_robot_state":
            return ToolResult(name=tool_call.name, content=self.state.model_dump_json(),
                              id=tool_call.id)
        return ToolResult(name=tool_call.name, content="(stub)", ok=False, id=tool_call.id)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:11434/v1")
    ap.add_argument("--model", default="qwen2.5:14b")
    ap.add_argument("--persona", default=None)
    ap.add_argument("--profile", default="complex-en")
    ap.add_argument("--persona-dir", default=None)
    ap.add_argument("--once", default=None, help="single prompt then exit")
    args = ap.parse_args()

    registry: ProviderRegistry = build_llm_registry(
        {"providers": {"local": {"url": args.url, "model": args.model, "priority": 10}}},
        logger=lambda m: print("[reg]", m, file=sys.stderr))
    personas = (PersonaRegistry.from_dir(args.persona_dir) if args.persona_dir
                else PersonaRegistry())
    state = DemoState()
    agent = Agent(registry, personas=personas, tools=ToolRegistry(),
                  state_provider=state, effector=DemoEffector(state.snapshot()),
                  language_policy=LanguagePolicy(native_languages=["en", "ru"]),
                  logger=lambda m: print("[agent]", m, file=sys.stderr))

    history = []

    def turn(text: str):
        req = AgentRequest(text=text, history=list(history), persona=args.persona,
                           profile=args.profile)
        reply = agent.handle(req)
        print(f"Sweetie [{reply.emotion.value}/{reply.sentence_type.value}]: {reply.response_text}")
        if reply.tool_calls:
            print("  proposed tools:", [t.name for t in reply.tool_calls])
        if reply.error_code.value:
            print("  ERROR:", reply.error_code.name, reply.error_desc)
        history.append(TalkTurn(speaker="human", text=text))
        history.append(TalkTurn(speaker="sweetie", text=reply.response_text, emotion=reply.emotion))

    if args.once is not None:
        turn(args.once)
        return
    print("Chat with Sweetie (Ctrl-D to exit).")
    try:
        while True:
            text = input("You: ").strip()
            if text:
                turn(text)
    except (EOFError, KeyboardInterrupt):
        print()


if __name__ == "__main__":
    main()
