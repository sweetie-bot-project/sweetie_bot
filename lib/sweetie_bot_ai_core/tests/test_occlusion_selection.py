"""camera_occluded is a frame-level flag, not a spatial object: it must survive salience
selection regardless of its zone, and must never speak through the generic arrived/left
event path (the WARNING banner in render_scene is its only voice).

Live repro (2026-07-08): a palm on the lens emitted camera_occluded at pose (0,0,0.05) in
the CAMERA frame; its body-frame bearing swung with every head move, the zone flapped
between front/side/rear, select_salient dropped it whenever it read rear, and
is_occluded() over the SELECTED scene never saw it at a turn -> no banner, no forced
anger. Meanwhile scene_diff voiced nonsense events ("a camera occluded appeared/left").
"""
import pytest

from sweetie_bot_ai_core.scene import (CAMERA_OCCLUDED, SceneConfig, diff as scene_diff,
                                       is_occluded, select_salient)
from sweetie_bot_ai_core.schema import SceneEntity, SceneState, Zone


def _occl(zone):
    return SceneEntity(id=0, type=CAMERA_OCCLUDED, zone=zone)


def _person(eid=1, zone=Zone.front):
    return SceneEntity(id=eid, type="person", zone=zone)


@pytest.mark.parametrize("zone", [Zone.front, Zone.side, Zone.rear])
def test_camera_occluded_survives_selection_any_zone(zone):
    # the flag's zone is numerically unstable (pose ~= camera origin) - selection must
    # keep it no matter what the bearing math said this frame
    state = SceneState(entities=[_occl(zone), _person()])
    sel = select_salient(state, SceneConfig())
    assert is_occluded(sel), f"occlusion flag lost at zone={zone.value}"


def test_camera_occluded_does_not_consume_entity_budget():
    cfg = SceneConfig(max_entities=1)
    state = SceneState(entities=[_occl(Zone.front), _person()])
    sel = select_salient(state, cfg)
    assert is_occluded(sel)
    assert any(e.type == "person" for e in sel.entities), \
        "the occlusion flag must not evict real entities from the ambient block"


def test_no_generic_events_for_camera_occluded():
    cfg = SceneConfig()
    empty = SceneState(entities=[])
    covered = SceneState(entities=[_occl(Zone.front)])
    assert scene_diff(empty, covered, cfg) == [], "no 'a camera occluded appeared' events"
    assert scene_diff(covered, empty, cfg) == [], "no 'a camera occluded left' events"


def test_person_events_unaffected_by_occlusion_filter():
    cfg = SceneConfig()
    prev = SceneState(entities=[_occl(Zone.front)])
    curr = SceneState(entities=[_occl(Zone.front), _person()])
    kinds = [ev.kind for ev in scene_diff(prev, curr, cfg)]
    assert kinds == ["arrived"]
