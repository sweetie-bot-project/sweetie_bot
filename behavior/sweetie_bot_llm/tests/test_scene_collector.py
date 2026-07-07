"""Unit tests for SceneCollector: injectable clock + fake TF + duck-typed detection stubs.

Runs under the ROS env (importorskip rospy) but WITHOUT a master: rospy.Subscriber and the TF
listener are stubbed out before construction, so only the pure collection/retention/debounce
logic is exercised. Detection attribute sets mirror real live vision output (gaze_at_robot /
emotion / color / held_by).
"""
import math
import os
import sys
from types import SimpleNamespace

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "src"))
sys.path.insert(0, os.path.abspath(os.path.join(
    HERE, "..", "..", "..", "lib", "sweetie_bot_ai_core", "src")))

rospy = pytest.importorskip("rospy")
pytest.importorskip("sweetie_bot_text_msgs")

import sweetie_bot_llm.scene_collector as sc_mod  # noqa: E402
from sweetie_bot_ai_core.schema import Zone  # noqa: E402


class FakeTF:
    """Identity transform by default; an optional yaw (deg) applies to stable->forward lookups
    so retention bearing-recompute-after-turning is testable."""

    def __init__(self):
        self.yaw_deg = 0.0
        self.fail = False

    def transform(self, ps, target_frame, timeout=None):
        if self.fail:
            raise RuntimeError("TF unavailable")
        x, y, z = ps.point.x, ps.point.y, ps.point.z
        if getattr(ps.header, "frame_id", "") == "odom" and target_frame == "base_link":
            a = math.radians(self.yaw_deg)
            x, y = x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a)
        return SimpleNamespace(point=SimpleNamespace(x=x, y=y, z=z))


def det(id=1, type="person", x=1.5, y=0.0, z=0.0, score=0.9, attrs=None):
    attrs = attrs or {}
    return SimpleNamespace(
        id=id, type=type, score=score,
        header=SimpleNamespace(frame_id="camera", stamp=None),
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z)),
        attribute=list(attrs.keys()), value=list(attrs.values()))


def det_array(*dets):
    return SimpleNamespace(detections=list(dets))


@pytest.fixture
def collector(monkeypatch):
    t = {"now": 100.0}
    monkeypatch.setattr(sc_mod.rospy, "Subscriber", lambda *a, **k: None)
    monkeypatch.setattr(sc_mod.tf2_ros, "Buffer", lambda: FakeTF())
    monkeypatch.setattr(sc_mod.tf2_ros, "TransformListener", lambda buf: None)
    c = sc_mod.SceneCollector(stable_frame="odom", forward_frame="base_link",
                              bearing_sign=1.0, clock=lambda: t["now"])
    c._test_clock = t
    return c


# --- merge window: multi-publisher topic, newest-per-id wins --------------------------------------

def test_merge_window_newest_per_id_wins(collector):
    c, t = collector, collector._test_clock
    c._on_detections(det_array(det(id=1, attrs={"emotion": "neutral"})))
    t["now"] += 0.3
    c._on_detections(det_array(det(id=1, attrs={"emotion": "happy"}),
                               det(id=2, type="pony", y=-1.0)))
    snap = c.snapshot()
    by_id = {e.id: e for e in snap.entities}
    assert set(by_id) == {1, 2}                       # both publishers' frames merged
    assert by_id[1].attributes["emotion"] == "happy"  # newest observation of id 1 wins


def test_merge_window_expires_old_frames(collector):
    c, t = collector, collector._test_clock
    c._on_detections(det_array(det(id=1)))
    t["now"] += 1.0                                   # > 0.6 s merge window
    c._on_detections(det_array(det(id=2, type="pony")))
    snap = c.snapshot()
    assert [e.id for e in snap.entities] == [2]       # id 1's frame aged out of the window


# --- retention: TTL prune + bearing recompute after turning ---------------------------------------

def test_retention_remembers_within_ttl_and_prunes_after(collector):
    c, t = collector, collector._test_clock
    c._on_detections(det_array(det(id=7, type="pony", x=1.0, y=-1.0)))
    t["now"] += 50.0                                   # gone from frame, within the 90 s TTL
    c._on_detections(det_array())
    snap = c.snapshot(include_remembered=True)
    rem = [e for e in snap.entities if not e.in_frame]
    assert [e.id for e in rem] == [7]
    assert rem[0].last_seen_s == pytest.approx(50.0, abs=0.5)
    assert snap.entities == rem or all(e.in_frame is False for e in snap.entities)
    # ...and without include_remembered it never surfaces
    assert c.snapshot().entities == []
    t["now"] += 50.0                                   # 100 s > TTL -> forgotten
    c._on_detections(det_array())
    assert c.snapshot(include_remembered=True).entities == []


def test_retention_bearing_recomputed_after_turn(collector):
    c, t = collector, collector._test_clock
    tf = c._tf
    c._on_detections(det_array(det(id=7, type="pony", x=1.5, y=0.0)))   # dead ahead
    t["now"] += 5.0
    c._on_detections(det_array())
    tf.yaw_deg = 60.0                                  # she turned left by 60 deg
    snap = c.snapshot(include_remembered=True)
    rem = [e for e in snap.entities if not e.in_frame][0]
    # the remembered pony is now off to one side, not "ahead" (stored in the stable frame)
    assert abs(rem.bearing_deg) == pytest.approx(60.0, abs=2.0)


# --- color debounce: mode of the recent window, per track ------------------------------------------

def test_color_debounce_mode_of_window(collector):
    c, t = collector, collector._test_clock
    for color in ["purple", "purple", "magenta", "purple", "magenta", "purple"]:
        c._on_detections(det_array(det(id=3, type="pony", attrs={"color": color})))
        t["now"] += 0.05
    snap = c.snapshot()
    assert snap.entities[0].attributes["color"] == "purple"   # the mode, not the last flip


def test_color_debounce_is_per_track(collector):
    c, t = collector, collector._test_clock
    c._on_detections(det_array(det(id=3, type="pony", attrs={"color": "purple"}),
                               det(id=4, type="pony", y=-1.0, attrs={"color": "orange"})))
    snap = c.snapshot()
    by_id = {e.id: e for e in snap.entities}
    assert by_id[3].attributes["color"] == "purple"
    assert by_id[4].attributes["color"] == "orange"


# --- DOA speaker attribution -----------------------------------------------------------------------

def _sound(az_deg=0.0, speech=True, intensity=0.3):
    return SimpleNamespace(doa_azimuth=[math.radians(az_deg)],
                           sound_flags=(2 if speech else 0), intensity=intensity)


def test_speaker_attribution_marks_aligned_front_entity(collector):
    c = collector
    c._on_detections(det_array(det(id=1, x=1.5, y=0.0),                  # dead ahead
                               det(id=2, x=1.0, y=-1.5)))                # ~56 deg right
    c._on_sound(_sound(az_deg=0.0, speech=True))
    snap = c.snapshot()
    by_id = {e.id: e for e in snap.entities}
    assert by_id[1].is_interlocutor and by_id[1].is_speaking
    assert not by_id[2].is_interlocutor
    assert snap.sounds and snap.sounds[0].kind == "speech"


def test_non_speech_sound_does_not_attribute_speaker(collector):
    c = collector
    c._on_detections(det_array(det(id=1)))
    c._on_sound(_sound(az_deg=0.0, speech=False))
    snap = c.snapshot()
    assert not snap.entities[0].is_interlocutor
    assert snap.sounds[0].kind == "sound"


# --- filtering: min_score + exclude_types -----------------------------------------------------------

def test_min_score_and_exclude_types_filtered(collector):
    c = collector
    c._on_detections(det_array(
        det(id=1, score=0.9),
        det(id=2, score=0.1),                          # below min_score 0.3
        det(id=3, type="sound"),                       # excluded (covered by SoundCue)
        det(id=4, type="speech"),                      # excluded
        det(id=5, type="hand")))                       # excluded (body part)
    snap = c.snapshot(include_remembered=True)
    assert [e.id for e in snap.entities] == [1]        # nothing excluded even reached retention


def test_tf_failure_degrades_gracefully(collector):
    c = collector
    c._tf.fail = True
    c._on_detections(det_array(det(id=1)))
    snap = c.snapshot(include_remembered=True)
    assert snap.entities == []                         # dropped, no crash


# --- zone classification sanity ---------------------------------------------------------------------

def test_zones_and_distance_words(collector):
    c = collector
    c._on_detections(det_array(det(id=1, x=0.5, y=0.0),                 # near, ahead
                               det(id=2, x=0.0, y=-2.0),                # 90 deg right -> side
                               det(id=3, x=3.0, y=0.0)))                # far, ahead
    snap = c.snapshot()
    by_id = {e.id: e for e in snap.entities}
    assert by_id[1].zone == Zone.front and by_id[1].distance == "near"
    assert by_id[2].zone == Zone.side
    assert by_id[3].distance == "far"
