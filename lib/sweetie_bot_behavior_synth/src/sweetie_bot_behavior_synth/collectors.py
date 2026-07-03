"""Outcome collectors: buffered topic subscribers + log scrapers, all anchored at test start.

Say-assertion surface (explored): SOAR speech = actionlib goal on voice/syn/goal
(TextActionActionGoal, command.type == "voice/say/<lang>", command.command == text).
LLM turn surface: /generate_reply goal (profile, text in) + result (reply text, emotion).
"""
from __future__ import annotations

import os
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional

import rospy
from sweetie_bot_text_msgs.msg import (DetectionArray, TextActionActionGoal, TextCommand,
                                       GenerateReplyActionGoal, GenerateReplyActionResult)
from geometry_msgs.msg import PoseStamped

LOG_DIR = os.path.expanduser("~/.ros/log/latest")


class TopicCollector:
    """Buffering subscriber with predicate waits. Records (walltime, msg)."""

    def __init__(self, topic: str, msg_type):
        self.topic = topic
        self._buf: List[tuple] = []
        self._lock = threading.Lock()
        self._sub = rospy.Subscriber(topic, msg_type, self._cb, queue_size=20)

    def _cb(self, msg):
        with self._lock:
            self._buf.append((time.monotonic(), msg))

    def clear(self):
        with self._lock:
            self._buf = []

    def messages(self, since: float = 0.0):
        with self._lock:
            return [m for t, m in self._buf if t >= since]

    def count(self, since: float = 0.0) -> int:
        return len(self.messages(since))

    def wait_for(self, pred: Callable, timeout: float = 10.0, since: float = 0.0):
        """Return the first message matching pred (arriving or already buffered), else None."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            for m in self.messages(since):
                if pred(m):
                    return m
            rospy.sleep(0.1)
        return None

    def close(self):
        self._sub.unregister()


@dataclass
class SayRecord:
    lang: str
    text: str


class SayCollector(TopicCollector):
    def __init__(self, topic: str = "voice/syn/goal"):
        super().__init__(topic, TextActionActionGoal)

    @staticmethod
    def _rec(msg) -> Optional[SayRecord]:
        t = msg.goal.command.type
        if t.startswith("voice/say/"):
            return SayRecord(lang=t.rsplit("/", 1)[-1], text=msg.goal.command.command)
        return None

    def says(self, since: float = 0.0) -> List[SayRecord]:
        return [r for r in (self._rec(m) for m in self.messages(since)) if r is not None]

    def wait_say(self, timeout: float = 25.0, since: float = 0.0) -> Optional[SayRecord]:
        m = self.wait_for(lambda m: self._rec(m) is not None, timeout, since)
        return self._rec(m) if m is not None else None


@dataclass
class TurnRecord:
    """One LLM turn as seen on the generate_reply action."""
    profile: str = ""
    text_in: str = ""
    text: str = ""            # reply text (response_text)
    emotion: str = ""
    sentence_type: str = ""
    said: Optional[SayRecord] = None   # what actually went to the voice (filled by World)


class ReplyCollector:
    """Pairs /generate_reply goals with results chronologically (tests run turns serially)."""

    def __init__(self, ns: str = "generate_reply"):
        self._goals = TopicCollector(ns + "/goal", GenerateReplyActionGoal)
        self._results = TopicCollector(ns + "/result", GenerateReplyActionResult)

    def clear(self):
        self._goals.clear()
        self._results.clear()

    def turns(self, since: float = 0.0) -> List[TurnRecord]:
        gs = self._goals.messages(since)
        rs = self._results.messages(since)
        out = []
        for g, r in zip(gs, rs):
            out.append(TurnRecord(profile=g.goal.profile, text_in=g.goal.text,
                                  text=r.result.response_text, emotion=r.result.emotion,
                                  sentence_type=r.result.sentence_type))
        return out

    def wait_turn(self, n_before: int, timeout: float = 30.0, since: float = 0.0) -> Optional[TurnRecord]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            ts = self.turns(since)
            if len(ts) > n_before:
                return ts[n_before]
            rospy.sleep(0.15)
        return None

    def close(self):
        self._goals.close()
        self._results.close()


class LogScraper:
    """Tail a node log file from the position it had at anchor time.

    The node may be RESPAWNED between (or during) tests, which rotates to a NEW log file —
    resolve the newest file lazily and, when it changes after anchoring, read the new file
    from its beginning."""

    def __init__(self, glob_prefix: str):
        self._prefix = glob_prefix
        self.path = self._newest()
        self._anchor = 0

    def _newest(self):
        import glob
        cands = sorted(glob.glob(os.path.join(LOG_DIR, self._prefix + "*.log")),
                       key=os.path.getmtime, reverse=True)
        return cands[0] if cands else None

    def anchor(self):
        self.path = self._newest()
        self._anchor = os.path.getsize(self.path) if self.path and os.path.exists(self.path) else 0

    def new_text(self) -> str:
        newest = self._newest()
        out = []
        if self.path and os.path.exists(self.path):
            if os.path.getsize(self.path) < self._anchor:
                self._anchor = 0    # the log rotated by size underneath us
            with open(self.path, "r", errors="replace") as f:
                f.seek(self._anchor)
                out.append(f.read())
        if newest and newest != self.path:
            # node respawned since anchor: include the successor file from its start
            with open(newest, "r", errors="replace") as f:
                out.append(f.read())
        return "\n".join(out)

    def grep(self, pattern: str) -> List[str]:
        rx = re.compile(pattern)
        return [ln for ln in self.new_text().splitlines() if rx.search(ln)]

    def wait_grep(self, pattern: str, timeout: float = 15.0) -> List[str]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            hits = self.grep(pattern)
            if hits:
                return hits
            time.sleep(0.2)
        return []


_REQ_RX = re.compile(r"llm_agent: request profile=(\S+) lang=(\S+) text=((['\"]).*\4)\s*$")
_REP_RX = re.compile(r"llm_agent: reply \[(\w+)\] ((['\"]).*\3)\s*$")


def _unrepr(quoted: str) -> str:
    """The agent log prints %r - recover the raw text (escapes, quotes)."""
    import ast
    try:
        return ast.literal_eval(quoted)
    except (ValueError, SyntaxError):
        return quoted.strip("\"" + chr(39))
_SAY_RX = re.compile(r"use \S+ profile to say: (.*) \((\w+)\)\s*$")


class TurnScraper:
    """Turns parsed from the agent log (reception-independent; the topics proved flaky in the
    test process around soar respawns)."""

    def __init__(self):
        self._log = LogScraper("rosout")

    def anchor(self):
        self._log.anchor()

    def turns(self) -> List[TurnRecord]:
        reqs, reps = [], []
        for ln in self._log.new_text().splitlines():
            m = _REQ_RX.search(ln)
            if m:
                reqs.append((m.group(1), _unrepr(m.group(3))))
                continue
            m = _REP_RX.search(ln)
            if m:
                reps.append((m.group(1), _unrepr(m.group(2))))
        out = []
        for (profile, text_in), (emotion, text) in zip(reqs, reps):
            out.append(TurnRecord(profile=profile, text_in=text_in, text=text, emotion=emotion))
        return out

    def wait_turn(self, n_before: int, timeout: float = 30.0) -> Optional[TurnRecord]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ts = self.turns()
            if len(ts) > n_before:
                return ts[n_before]
            time.sleep(0.2)
        return None

    def wait_turn_for(self, text_in: str, timeout: float = 30.0) -> Optional[TurnRecord]:
        """The turn whose request text matches OUR utterance - pause-openers and follow-up
        turns interleave freely and must not be mistaken for the answer to our ask."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for t in self.turns():
                if t.text_in == text_in:
                    return t
            time.sleep(0.2)
        return None


class VoiceScraper:
    """Say records parsed from the voice node log."""

    def __init__(self):
        self._log = LogScraper("rosout")

    def anchor(self):
        self._log.anchor()

    def says(self) -> List[SayRecord]:
        out = []
        for ln in self._log.new_text().splitlines():
            m = _SAY_RX.search(ln)
            if m and m.group(1).strip():   # empty says = the adapter's quiet silence result
                out.append(SayRecord(lang=m.group(2), text=m.group(1)))
        return out

    def wait_say(self, n_before: int = 0, timeout: float = 15.0) -> Optional[SayRecord]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ss = self.says()
            if len(ss) > n_before:
                return ss[-1]
            time.sleep(0.2)
        return None


def make_collectors():
    """The standard collector set for a World."""
    return {
        "say": VoiceScraper(),
        "turns": TurnScraper(),
        "expr": TopicCollector("control", TextCommand),
        "detections": TopicCollector("detections", DetectionArray),
        "gaze_refs": TopicCollector("/motion/controller/look_at/in_pose_ref", PoseStamped),
        "soar_log": LogScraper("rosout"),
        "agent_log": LogScraper("rosout"),
    }
