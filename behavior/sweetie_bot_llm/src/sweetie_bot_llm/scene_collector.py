"""ROS SceneProvider: turn the vision DetectionArray + directional SoundEvent into a SceneState.

ROS-side glue (the agent core only sees the ROS-free SceneState). Adds a short-term **retention
buffer** so recently-seen entities that leave the frame are remembered for a TTL (poses stored in a
stable frame so their bearing stays correct after she turns). No names, no gallery, no persistence
beyond the TTL — matches the "id drops → forget" rule.
"""
from __future__ import annotations

import math
import time
from typing import Dict, Optional

import rospy
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 - registers do_transform for PointStamped
from geometry_msgs.msg import PointStamped

from sweetie_bot_ai_core.scene import SceneConfig, classify_zone
from sweetie_bot_ai_core.schema import SceneEntity, SceneState, SoundCue, Zone

from sweetie_bot_text_msgs.msg import DetectionArray, SoundEvent

# SoundEvent flags
_SPEECH_DETECTING = 2
_SPEECH_DECODED = 4


class _Remembered:
    __slots__ = ("point_stable", "type", "attributes", "last_t")

    def __init__(self, point_stable, type_, attributes, last_t):
        self.point_stable = point_stable      # (x, y, z) in stable_frame
        self.type = type_
        self.attributes = attributes
        self.last_t = last_t


class SceneCollector:
    def __init__(self, *, detections_topic="detections", sound_topic="sound_event",
                 forward_frame="base_link", stable_frame="odom",
                 front_deg=60.0, side_deg=120.0, retention_ttl_s=20.0,
                 near_m=1.0, mid_m=2.5, bearing_sign=1.0,
                 sound_bearing_sign=1.0, sound_bearing_offset_deg=0.0,
                 min_score=0.3, exclude_types=("sound", "speech", "hand"),
                 clock=time.monotonic):
        self._forward_frame = forward_frame
        self._stable_frame = stable_frame
        self._cfg = SceneConfig(front_deg=front_deg, side_deg=side_deg)
        self._ttl = retention_ttl_s
        self._near_m, self._mid_m = near_m, mid_m
        self._bearing_sign = bearing_sign
        self._sound_sign = sound_bearing_sign
        self._sound_offset = sound_bearing_offset_deg
        self._min_score = min_score
        # detection types that are NOT scene objects for the prompt: 'sound'/'speech' come from
        # the microphone on the same topic and are already covered by SoundCue (double-count),
        # 'hand' is a body part of an already-listed person, not a separate entity.
        self._exclude_types = set(exclude_types or ())
        self._clock = clock

        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf)
        self._latest: Optional[DetectionArray] = None
        self._sound: Optional[SoundEvent] = None
        self._remembered: Dict[int, _Remembered] = {}

        rospy.Subscriber(detections_topic, DetectionArray, self._on_detections, queue_size=1)
        rospy.Subscriber(sound_topic, SoundEvent, self._on_sound, queue_size=1)

    def reset(self) -> None:
        """Drop all perceived/remembered state (latest frame, sound, retention). Test/reset seam."""
        self._latest = None
        self._sound = None
        self._remembered.clear()

    # -- subscribers --------------------------------------------------------------------------

    def _on_detections(self, msg: DetectionArray):
        self._latest = msg
        now = self._clock()
        for det in msg.detections:
            if det.type in self._exclude_types:
                continue
            if det.score and det.score < self._min_score:
                continue
            p_stable = self._to_frame(det, self._stable_frame)
            if p_stable is None:
                continue
            self._remembered[det.id] = _Remembered(
                p_stable, det.type or "person", self._attrs(det), now)
        # prune expired
        cutoff = now - self._ttl
        for eid in [k for k, v in self._remembered.items() if v.last_t < cutoff]:
            del self._remembered[eid]

    def _on_sound(self, msg: SoundEvent):
        self._sound = msg

    # -- geometry helpers ---------------------------------------------------------------------

    def _to_frame(self, det, target_frame):
        """Transform a detection's position into target_frame → (x, y, z) or None."""
        try:
            ps = PointStamped()
            ps.header = det.header
            ps.point = det.pose.position
            out = self._tf.transform(ps, target_frame, rospy.Duration(0.2))
            return (out.point.x, out.point.y, out.point.z)
        except Exception:  # noqa: BLE001 - TF may be unavailable; degrade gracefully
            return None

    def _bearing_dist(self, point):
        """point in forward_frame (REP-103 x-forward, y-left) → (bearing_deg, elevation_deg,
        distance_word)."""
        x, y, z = point
        bearing = self._bearing_sign * math.degrees(math.atan2(-y, x))  # +right by convention
        d = math.hypot(x, y)
        elevation = math.degrees(math.atan2(z, d)) if d > 1e-6 else 0.0  # +up vs her horizon
        dist = "near" if d < self._near_m else ("mid" if d < self._mid_m else "far")
        return bearing, elevation, dist

    @staticmethod
    def _attrs(det) -> Dict[str, str]:
        return {k: v for k, v in zip(det.attribute or [], det.value or [])}

    # -- SceneProvider seam -------------------------------------------------------------------

    def snapshot(self, include_remembered: bool = False) -> SceneState:
        entities = []
        seen_now = set()
        now = self._clock()

        # in-frame entities from the latest detection message
        if self._latest is not None:
            for det in self._latest.detections:
                if det.type in self._exclude_types:
                    continue
                if det.score and det.score < self._min_score:
                    continue
                pf = self._to_frame(det, self._forward_frame)
                if pf is None:
                    continue
                bearing, elevation, dist = self._bearing_dist(pf)
                seen_now.add(det.id)
                entities.append(SceneEntity(
                    id=det.id, type=det.type or "person", bearing_deg=bearing,
                    elevation_deg=elevation,
                    zone=classify_zone(bearing, self._cfg), distance=dist,
                    attributes=self._attrs(det), in_frame=True, last_seen_s=0.0))

        sounds = self._sound_cues()
        self._attribute_speaker(entities, sounds)

        if include_remembered:
            for eid, r in self._remembered.items():
                if eid in seen_now:
                    continue
                pf = self._stable_point_in_forward(r.point_stable)
                if pf is None:
                    continue
                bearing, elevation, dist = self._bearing_dist(pf)
                entities.append(SceneEntity(
                    id=eid, type=r.type, bearing_deg=bearing,
                    elevation_deg=elevation,
                    zone=classify_zone(bearing, self._cfg), distance=dist,
                    attributes=r.attributes, in_frame=False,
                    last_seen_s=max(0.0, now - r.last_t)))

        return SceneState(entities=entities, sounds=sounds)

    def _stable_point_in_forward(self, point_stable):
        try:
            ps = PointStamped()
            ps.header.frame_id = self._stable_frame
            ps.header.stamp = rospy.Time(0)
            ps.point.x, ps.point.y, ps.point.z = point_stable
            out = self._tf.transform(ps, self._forward_frame, rospy.Duration(0.2))
            return (out.point.x, out.point.y, out.point.z)
        except Exception:  # noqa: BLE001
            return None

    def _sound_cues(self):
        cues = []
        s = self._sound
        if s is None or not s.doa_azimuth:
            return cues
        is_speech = bool(s.sound_flags & (_SPEECH_DETECTING | _SPEECH_DECODED))
        for az in s.doa_azimuth[:1]:  # strongest candidate
            bearing = self._sound_sign * math.degrees(az) + self._sound_offset
            intensity = "loud" if (s.intensity or 0) > 0.6 else "normal"
            cues.append(SoundCue(bearing_deg=bearing, zone=classify_zone(bearing, self._cfg),
                                 kind="speech" if is_speech else "sound", intensity=intensity))
        return cues

    def _attribute_speaker(self, entities, sounds):
        """Mark the front entity aligned with a front speech DOA as speaking/interlocutor."""
        speech = [c for c in sounds if c.kind == "speech" and c.zone != Zone.rear]
        if not speech:
            return
        for c in speech:
            best, best_err = None, 35.0   # deg tolerance
            for e in entities:
                if not e.in_frame or e.zone == Zone.rear:
                    continue
                err = abs(e.bearing_deg - c.bearing_deg)
                if err < best_err:
                    best, best_err = e, err
            if best is not None:
                best.is_speaking = True
                best.is_interlocutor = True
