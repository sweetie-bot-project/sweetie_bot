"""Message-level synth streams (the transport seams).

SynthDetections publishes a mutable set of SynthEntity as DetectionArray at a fixed rate —
timeline control (spawn/vanish/move) happens by mutating the set. SynthSpeech publishes decoded
TEXT through the microphone's SoundEvent frame protocol (no audio anywhere — user decision):
DETECTING frames while "talking", then ONE DECODED-only frame carrying the text (the sound
module's decode branch is an elif — DETECTING|DECODED together never delivers text), then
silence frames so its speech_timeout can progress.
"""
from __future__ import annotations

import math
import threading
from typing import Dict, Optional

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from sweetie_bot_text_msgs.msg import Detection, DetectionArray, SoundEvent

from .dsl import SynthEntity

FRAME = "base_link"


def _detection(e: SynthEntity, stamp) -> Detection:
    d = Detection()
    d.header = Header(stamp=stamp, frame_id=FRAME)
    d.id = e.id
    d.label = e.label or f"{e.type}_{e.id}"
    d.type = e.type
    d.score = e.score
    x, y, z = e.xyz()
    d.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(w=1.0))
    d.box = Vector3(0.2, 0.2, 0.2)
    for k, v in e.attributes.items():
        d.attribute.append(k)
        d.value.append(v)
    return d


class SynthDetections:
    """Rate-driven DetectionArray publisher over a mutable entity set."""

    def __init__(self, topic: str = "detections", rate_hz: float = 8.0):
        self._pub = rospy.Publisher(topic, DetectionArray, queue_size=2)
        self._entities: Dict[int, SynthEntity] = {}
        self._lock = threading.Lock()
        self._rate = rate_hz
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # -- lifecycle --------------------------------------------------------------------------------
    def start(self, entities=()):
        with self._lock:
            self._entities = {e.id: e for e in entities}
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def stop(self, flush_empty_s: float = 1.0):
        """Stop publishing entities; keep publishing EMPTY frames for flush_empty_s (SWM
        visibility timeout flush), then stop entirely."""
        with self._lock:
            self._entities = {}
        if flush_empty_s > 0:
            rospy.sleep(flush_empty_s)
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    # -- timeline control ---------------------------------------------------------------------------
    def spawn(self, *entities: SynthEntity):
        with self._lock:
            for e in entities:
                self._entities[e.id] = e

    def vanish(self, *ids: int):
        with self._lock:
            for i in ids:
                self._entities.pop(i, None)

    def move(self, id: int, bearing: float = None, dist: float = None, elevation: float = None):
        with self._lock:
            e = self._entities.get(id)
            if e is None:
                return
            if bearing is not None:
                e.bearing = bearing
            if dist is not None:
                e.dist = dist
            if elevation is not None:
                e.elevation = elevation

    def present(self):
        with self._lock:
            return dict(self._entities)

    # -- loop ----------------------------------------------------------------------------------------
    def _loop(self):
        r = rospy.Rate(self._rate)
        while not self._stop.is_set() and not rospy.is_shutdown():
            msg = DetectionArray()
            stamp = rospy.Time.now()
            with self._lock:
                msg.detections = [_detection(e, stamp) for e in self._entities.values()]
            self._pub.publish(msg)
            try:
                r.sleep()
            except rospy.ROSInterruptException:
                break


class SynthSpeech:
    """Decoded-text speech injection through the mic's SoundEvent protocol."""

    def __init__(self, topic: str = "sound_event"):
        self._pub = rospy.Publisher(topic, SoundEvent, queue_size=1)

    def say(self, text: str, lang: str = "en", bearing_deg: float = 0.0,
            talking_s: float = 1.0, silence_s: float = 4.0):
        # NO doa_values/doa_azimuth: sound direction detection is DISABLED in the live deploy
        # (devel f018e571). Populating them here activates the dormant speaker-binding path and
        # diverges from live behavior (found by the harness: off-axis person -> bind fail -> mute).
        r = rospy.Rate(5)
        m = SoundEvent()
        m.sound_flags = SoundEvent.SOUND_DETECTING | SoundEvent.SPEECH_DETECTING
        m.intensity = 0.7
        m.speech_probability = 0.95
        m.doa = Vector3(x=1.0, y=0.0, z=0.0)
        for _ in range(max(2, int(talking_s * 5))):
            m.header = Header(stamp=rospy.Time.now(), frame_id=FRAME)
            self._pub.publish(m)
            r.sleep()
        md = SoundEvent()
        md.header = Header(stamp=rospy.Time.now(), frame_id=FRAME)
        md.sound_flags = SoundEvent.SOUND_DETECTING | SoundEvent.SPEECH_DECODED
        md.text = text
        md.language = lang
        md.intensity = 0.7
        md.speech_probability = 0.95
        md.doa = Vector3(x=1.0, y=0.0, z=0.0)
        self._pub.publish(md)
        r.sleep()
        m2 = SoundEvent()
        m2.sound_flags = 0
        for _ in range(int(silence_s * 5)):
            m2.header = Header(stamp=rospy.Time.now(), frame_id=FRAME)
            self._pub.publish(m2)
            r.sleep()
