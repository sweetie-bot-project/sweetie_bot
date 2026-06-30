#!/usr/bin/env python3
"""Standalone GenerateReply action client to verify the llm_agent node in real ROS."""
import json
import rospy
import actionlib
from sweetie_bot_text_msgs.msg import GenerateReplyAction, GenerateReplyGoal

rospy.init_node("gr_test_client")
c = actionlib.SimpleActionClient("generate_reply", GenerateReplyAction)
print("waiting for action server...")
assert c.wait_for_server(rospy.Duration(15)), "no generate_reply server"
print("server up")


def ask(text, profile="simple-en", history=None, ctx=None, lang="en", persona=""):
    g = GenerateReplyGoal()
    g.request_type = "reply"
    g.profile = profile
    g.text = text
    g.history_json = json.dumps(history or [])
    g.context_json = json.dumps(ctx or [])
    g.text_language = lang
    g.reply_language = lang
    g.persona = persona
    c.send_goal(g)
    c.wait_for_result(rospy.Duration(90))
    r = c.get_result()
    print(f"[{profile}/{lang}] err={r.error_code} emo={r.emotion} st={r.sentence_type} :: {r.response_text}")
    if r.tool_calls_json and r.tool_calls_json not in ("", "[]"):
        print("   tool_calls:", r.tool_calls_json)


ask("I love your sparkly mane, Sweetie!")
ask("What is your battery level right now?", profile="complex-en")
ask("Do you remember my dog?", ctx=["The human has a dog named Rex."])
ask("Привет! Как у тебя дела сегодня?", lang="ru")
print("DONE")
