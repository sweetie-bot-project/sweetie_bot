#!/usr/bin/env python3
"""Bench harness for the LLM-agent rework (run_real:=false, no camera/mic).

Modes:
  inject <scenario>   loop-publish synthetic DetectionArray on /detections (7 Hz) until killed.
                      scenarios: person | person_pony | pony | empty | reenter
                      (reenter: 8s person id=101, 6s empty, then person id=105 - the P12 case)
  say <lang> <text>   publish one decoded-speech SoundEvent on /sound_event (full SOAR loop)
  ask <profile> <text> [lang]  call /generate_reply directly and print the result
"""
import sys
import json
import math
import time

import rospy
import actionlib
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sweetie_bot_text_msgs.msg import (DetectionArray, Detection, SoundEvent,
                                       GenerateReplyAction, GenerateReplyGoal)

FRAME = "base_link"


def det(id_, type_, x, y, label="", attrs=None):
    d = Detection()
    d.header = Header(stamp=rospy.Time.now(), frame_id=FRAME)
    d.id = id_
    d.label = label or "%s_%d" % (type_, id_)
    d.type = type_
    d.score = 0.9
    d.pose = Pose(position=Point(x=x, y=y, z=0.4), orientation=Quaternion(w=1.0))
    d.box = Vector3(0.2, 0.2, 0.2)
    for k, v in (attrs or {}).items():
        d.attribute.append(k)
        d.value.append(v)
    return d


SCENARIOS = {
    "person":      lambda t: [det(101, "human", 1.5, 0.2, attrs={"emotion": "happy"})],
    "person_pony": lambda t: [det(101, "human", 1.5, 0.2, attrs={"emotion": "happy"}),
                              det(201, "pony", 1.2, -0.5)],
    "pony":        lambda t: [det(201, "pony", 1.2, -0.5)],
    "empty":       lambda t: [],
    # P12 leave/re-enter: person, gone, back with a NEW track id
    "reenter":     lambda t: ([det(101, "human", 1.5, 0.2)] if t % 20 < 8 else
                              [] if t % 20 < 14 else
                              [det(105, "human", 1.4, -0.3)]),
}


def inject(scenario):
    rospy.init_node("sb_bench_inject", anonymous=True)
    pub = rospy.Publisher("detections", DetectionArray, queue_size=2)
    make = SCENARIOS[scenario]
    r = rospy.Rate(7)
    t0 = time.time()
    print("injecting scenario '%s' on /detections (Ctrl-C to stop)" % scenario, flush=True)
    while not rospy.is_shutdown():
        msg = DetectionArray()
        msg.detections = make(time.time() - t0)
        pub.publish(msg)
        r.sleep()


def say(lang, text):
    rospy.init_node("sb_bench_say", anonymous=True)
    pub = rospy.Publisher("sound_event", SoundEvent, queue_size=1, latch=True)
    # Mimic the real mic frame sequence: DETECTING frames while talking (no text), then ONE
    # DECODED-only frame with text (the decode branch is an elif - DETECTING|DECODED together
    # never reaches it), then silence frames so speech_timeout can progress.
    az = [float(math.atan2(0.1, 1.0))]
    rospy.sleep(0.3)
    r = rospy.Rate(5)
    m = SoundEvent()
    m.sound_flags = SoundEvent.SOUND_DETECTING | SoundEvent.SPEECH_DETECTING
    m.intensity = 0.7
    m.speech_probability = 0.95
    m.doa = Vector3(x=1.0, y=0.1, z=0.0)   # roughly ahead, matches the injected face bearing
    m.doa_azimuth = az
    m.doa_values = [1.0]
    for _ in range(5):
        m.header = Header(stamp=rospy.Time.now(), frame_id=FRAME)
        pub.publish(m)
        r.sleep()
    md = SoundEvent()
    md.header = Header(stamp=rospy.Time.now(), frame_id=FRAME)
    md.sound_flags = SoundEvent.SOUND_DETECTING | SoundEvent.SPEECH_DECODED
    md.text = text
    md.language = lang
    md.intensity = 0.7
    md.speech_probability = 0.95
    md.doa = Vector3(x=1.0, y=0.1, z=0.0)
    md.doa_azimuth = az
    md.doa_values = [1.0]
    pub.publish(md)
    r.sleep()
    m2 = SoundEvent()
    m2.sound_flags = 0
    for _ in range(25):   # ~5s of silence frames so the 2.5s speech_timeout fires
        m2.header = Header(stamp=rospy.Time.now(), frame_id=FRAME)
        pub.publish(m2)
        r.sleep()
    print("said (%s): %r" % (lang, text), flush=True)


def ask(profile, text, lang="en"):
    rospy.init_node("sb_bench_ask", anonymous=True)
    c = actionlib.SimpleActionClient("generate_reply", GenerateReplyAction)
    if not c.wait_for_server(rospy.Duration(10.0)):
        print("ERROR: no action server", flush=True)
        sys.exit(1)
    g = GenerateReplyGoal(request_type="reply", profile=profile, text=text,
                          history_json="[]", context_json="[]",
                          text_language=lang, reply_language=lang, persona="")
    t0 = time.time()
    c.send_goal(g)
    if not c.wait_for_result(rospy.Duration(120.0)):
        print("ERROR: timeout", flush=True)
        sys.exit(1)
    r = c.get_result()
    print(json.dumps({"dt_s": round(time.time() - t0, 1), "err": r.error_code,
                      "emotion": r.emotion, "stype": r.sentence_type,
                      "tools": r.tool_calls_json, "text": r.response_text},
                     ensure_ascii=False, indent=1), flush=True)


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "inject":
        inject(sys.argv[2])
    elif mode == "say":
        say(sys.argv[2], " ".join(sys.argv[3:]))
    elif mode == "ask":
        lang = "en"
        args = sys.argv[2:]
        if args[-1] in ("en", "ru", "zh", "ja") and len(args) > 2:
            lang = args[-1]
            args = args[:-1]
        ask(args[0], " ".join(args[1:]), lang)
    else:
        print(__doc__)
