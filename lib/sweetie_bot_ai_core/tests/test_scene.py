"""Offline unit tests for the scene salience/diff/render core (no ROS, no model)."""
from sweetie_bot_ai_core import (Agent, AgentRequest, SceneConfig, SceneState, Zone,
                                 classify_zone, render_scene, select_salient)
from sweetie_bot_ai_core.scene import bearing_words, diff, render_remembered
from sweetie_bot_ai_core.schema import SceneEntity, SoundCue
from sweetie_bot_ai_core.client import ChatResult

CFG = SceneConfig(front_deg=60, side_deg=120, max_entities=3)


def person(id, bearing, **kw):
    return SceneEntity(id=id, type="person", bearing_deg=bearing,
                       zone=classify_zone(bearing, CFG), **kw)


# --- zones -----------------------------------------------------------------------------------

def test_classify_zone():
    assert classify_zone(0, CFG) == Zone.front
    assert classify_zone(-59, CFG) == Zone.front
    assert classify_zone(90, CFG) == Zone.side
    assert classify_zone(-121, CFG) == Zone.rear
    assert classify_zone(180, CFG) == Zone.rear


# --- salience: front kept, rear dropped, sides fill, cap, interlocutor first ------------------

def test_select_salient_drops_rear_and_caps():
    st = SceneState(entities=[
        person(1, 10), person(2, -30), person(3, 50), person(4, 200),  # 4 is rear (|200-360|? no: 200>120 rear)
        person(5, 90),  # side
    ])
    sel = select_salient(st, CFG)
    ids = [e.id for e in sel.entities]
    assert 4 not in ids                      # rear dropped
    assert len(ids) == 3                      # capped at max_entities
    assert set(ids) <= {1, 2, 3, 5}
    # all three front ones fit before the side one
    assert set(ids) == {1, 2, 3}


def test_side_fills_spare_when_front_sparse():
    st = SceneState(entities=[person(1, 10), person(5, 90), person(6, -100)])
    sel = select_salient(st, CFG)
    ids = {e.id for e in sel.entities}
    assert 1 in ids and 5 in ids and 6 in ids  # 1 front + 2 sides fill spare slots


def test_interlocutor_sorted_first():
    st = SceneState(entities=[person(1, 55), person(2, -5, is_interlocutor=True)])
    sel = select_salient(st, CFG)
    assert sel.entities[0].id == 2


def test_rear_sound_dropped():
    st = SceneState(sounds=[SoundCue(bearing_deg=170, zone=Zone.rear, kind="speech"),
                            SoundCue(bearing_deg=10, zone=Zone.front, kind="speech")])
    sel = select_salient(st, CFG)
    assert len(sel.sounds) == 1 and sel.sounds[0].zone == Zone.front


# --- diff ------------------------------------------------------------------------------------

def test_diff_arrived_left_changed():
    prev = SceneState(entities=[person(1, 10), person(2, -20, attributes={"smiling": "no"})])
    curr = SceneState(entities=[person(2, -20, attributes={"smiling": "yes"}), person(3, 30)])
    evs = {e.kind: e for e in diff(prev, curr, CFG)}
    assert evs["arrived"].entity_id == 3
    assert evs["left"].entity_id == 1
    assert evs["changed"].entity_id == 2 and "smiling" in evs["changed"].detail


# --- render ----------------------------------------------------------------------------------

def test_render_scene_id_only_no_names_and_note():
    st = select_salient(SceneState(entities=[
        person(3, -20, is_interlocutor=True, attributes={"gaze_at_robot": "yes"}),
        person(7, 40, attributes={"holding": "pony"})]), CFG)
    txt = render_scene(st, [], CFG)
    assert "Around you right now:" in txt
    assert "(id 3)" in txt and "(id 7)" in txt          # id used as handle
    assert "looking at you" in txt                       # attr pretty-rendered
    assert "holding pony" in txt
    assert "say an id number" in txt.lower()              # no-speak-id instruction present


def test_render_scene_empty_is_blank():
    assert render_scene(SceneState(), [], CFG) == ""


def test_bearing_words_sides():
    assert "front" in bearing_words(0, CFG)
    assert "right" in bearing_words(90, CFG)
    assert "left" in bearing_words(-90, CFG)


def test_render_remembered():
    ents = [SceneEntity(id=9, type="pony_plushie", bearing_deg=-80, zone=Zone.side,
                        in_frame=False, last_seen_s=14.0)]
    txt = render_remembered(ents, CFG)
    assert "id 9" in txt and "14s ago" in txt and "left" in txt


# --- agent integration: scene block reaches the prompt ---------------------------------------

class SceneStub:
    def __init__(self, state):
        self.state = state

    def snapshot(self, include_remembered=False):
        return self.state


class CapturingRegistry:
    def __init__(self, content):
        self.content = content
        self.system = None

    def chat(self, messages, **kw):
        self.system = messages[0]["content"]
        return ChatResult(content=self.content), "cap"


def test_agent_injects_scene_block_into_prompt():
    scene = SceneState(entities=[person(3, -15, is_interlocutor=True)])
    reg = CapturingRegistry('{"response_text":"Hi!","emotion":"joy","sentence_type":"statement"}')
    agent = Agent(reg, scene_provider=SceneStub(scene), scene_config=CFG)
    agent.handle(AgentRequest(text="hello", profile="simple-en"))
    assert "Around you right now:" in reg.system
    assert "(id 3)" in reg.system


def test_select_salient_passes_remembered_through():
    """Out-of-frame remembered entities must reach the render (live bug: in_frame filter ate them)."""
    from sweetie_bot_ai_core.schema import SceneEntity, SceneState, Zone
    from sweetie_bot_ai_core.scene import SceneConfig, render_scene, select_salient
    cfg = SceneConfig()
    ent = [SceneEntity(id=1, type="human", zone=Zone.front, bearing_deg=0.0, in_frame=True,
                       last_seen_s=0.0),
           SceneEntity(id=2, type="pony", zone=Zone.front, bearing_deg=35.0, in_frame=False,
                       last_seen_s=6.0)]
    sel = select_salient(SceneState(entities=ent), cfg)
    assert any(e.id == 2 for e in sel.entities), "remembered pony dropped by salience filter"
    text = render_scene(sel, [], cfg)
    assert "Recently seen" in text and "pony" in text and "right" in text


def test_elevation_words_render():
    from sweetie_bot_ai_core.schema import SceneEntity, SceneState, Zone
    from sweetie_bot_ai_core.scene import SceneConfig, elevation_words, render_scene, select_salient
    assert elevation_words(30.0) == "above you"
    assert elevation_words(-30.0) == "below you"
    assert elevation_words(5.0) == ""
    cfg = SceneConfig()
    ent = [SceneEntity(id=3, type="pony", zone=Zone.front, bearing_deg=0.0, elevation_deg=-30.0,
                       in_frame=True, last_seen_s=0.0)]
    text = render_scene(select_salient(SceneState(entities=ent), cfg), [], cfg)
    assert "below you" in text


def test_camera_occluded_warning_banner_and_helper():
    from sweetie_bot_ai_core.schema import SceneEntity, SceneState, Zone
    from sweetie_bot_ai_core.scene import SceneConfig, is_occluded, render_scene, select_salient
    cfg = SceneConfig()
    ent = [SceneEntity(id=99, type="camera_occluded", zone=Zone.front, bearing_deg=0.0,
                       distance="near", in_frame=True, last_seen_s=0.0)]
    sel = select_salient(SceneState(entities=ent), cfg)
    assert is_occluded(sel)
    text = render_scene(sel, [], cfg)
    assert "WARNING" in text and "camera" in text
    assert "Around you right now" not in text          # the occluder is not listed as an object
    # not occluded when there's no such entity
    assert not is_occluded(SceneState(entities=[person(1, 0)]))


def test_agent_forces_anger_when_camera_occluded():
    """A blocked camera must deterministically drive the angry animation, overriding the LLM."""
    from sweetie_bot_ai_core.schema import Emotion, SceneEntity, SceneState, Zone
    occ = SceneState(entities=[SceneEntity(id=99, type="camera_occluded", zone=Zone.front,
                                           bearing_deg=0.0, in_frame=True)])
    # the model returns joy; the override must win
    reg = CapturingRegistry('{"response_text":"hey!","emotion":"joy","sentence_type":"statement"}')
    agent = Agent(reg, scene_provider=SceneStub(occ), scene_config=CFG)
    reply = agent.handle(AgentRequest(text="what do you see?", profile="simple-en"))
    assert reply.emotion == Emotion.anger
    assert "WARNING" in reg.system                      # the complaint banner reached the prompt

    # without occlusion the LLM's own emotion is kept
    clear = SceneState(entities=[person(3, -15, is_interlocutor=True)])
    reg2 = CapturingRegistry('{"response_text":"hi!","emotion":"joy","sentence_type":"statement"}')
    agent2 = Agent(reg2, scene_provider=SceneStub(clear), scene_config=CFG)
    reply2 = agent2.handle(AgentRequest(text="hello", profile="simple-en"))
    assert reply2.emotion == Emotion.joy


def test_occluded_by_inference():
    from sweetie_bot_ai_core.schema import SceneEntity, SceneState, Zone
    from sweetie_bot_ai_core.scene import SceneConfig, render_scene, select_salient
    cfg = SceneConfig()
    ent = [SceneEntity(id=1, type="human", zone=Zone.front, bearing_deg=10.0, distance="near",
                       in_frame=True, last_seen_s=0.0),
           SceneEntity(id=2, type="pony", zone=Zone.front, bearing_deg=14.0, distance="mid",
                       in_frame=False, last_seen_s=4.0),
           SceneEntity(id=3, type="pony", zone=Zone.side, bearing_deg=80.0, distance="mid",
                       in_frame=False, last_seen_s=4.0)]
    sel = select_salient(SceneState(entities=ent), cfg)
    text = render_scene(sel, [], cfg)
    assert "hiding behind the person (id 1)" in text   # aligned bearings -> inferred
    p3 = [e for e in sel.entities if e.id == 3][0]
    assert "probably_hidden_behind" not in p3.attributes        # far bearing -> no inference


# --- gaze collapses to a boolean; raw perception telemetry never leaks (user: she should only
#     know WHETHER the human is looking at her, not the raw angles) ---------------------------

def test_render_attr_drops_internal_telemetry():
    from sweetie_bot_ai_core.scene import render_attr
    for k in ("gaze_pitch", "gaze_yaw", "depth_source", "face_source", "attention_state"):
        assert render_attr(k, "3.5", CFG) == "", f"{k} leaked"
    assert render_attr("gaze_at_robot", "yes", CFG) == "looking at you"   # boolean kept


def test_render_attr_is_allowlist_only():
    # unknown attrs must NOT fall back to raw `key: value` — that is how gaze_pitch and
    # held_by: 3 leaked into her speech on the live robot (2026-07-04 session logs)
    from sweetie_bot_ai_core.scene import render_attr
    assert render_attr("some_future_attr", "42", CFG) == ""
    assert render_attr("velocity_x", "0.3", CFG) == ""
    # held_by is phrased, and the tracker-id value never surfaces
    assert render_attr("held_by", "3", CFG) == "being held by someone"
    assert "3" not in render_attr("held_by", "3", CFG)
    # measured color is phrased (anti-confabulation: she speaks the MEASURED color)
    assert render_attr("color", "blue", CFG) == "mostly blue"


def test_scene_block_has_no_raw_gaze_numbers():
    st = select_salient(SceneState(entities=[person(
        3, -10, is_interlocutor=True,
        attributes={"gaze_at_robot": "yes", "gaze_pitch": "3.5", "gaze_yaw": "6.8",
                    "depth_source": "estimated", "attention_state": "DRAWN"})]), CFG)
    txt = render_scene(st, [], CFG)
    assert "looking at you" in txt
    for leak in ("gaze_pitch", "gaze_yaw", "3.5", "6.8", "depth_source", "attention_state", "DRAWN"):
        assert leak not in txt, f"leaked internal telemetry {leak!r}: {txt!r}"


def test_scene_diff_ignores_color_flip():
    # a measured-color flip is noise, not news — no "the pony is now mostly white" events
    prev = SceneState(entities=[person(9, 10, attributes={"color": "blue"})])
    curr = SceneState(entities=[person(9, 10, attributes={"color": "white"})])
    assert diff(prev, curr, CFG) == []


def test_scene_diff_ignores_pure_gaze_number_change():
    # a change in ONLY the raw gaze angles must not produce a 'someone is now ...' event
    prev = SceneState(entities=[person(2, -10, attributes={"gaze_yaw": "1.0"})])
    curr = SceneState(entities=[person(2, -10, attributes={"gaze_yaw": "9.0"})])
    assert diff(prev, curr, CFG) == []


# --- recitation: no parroted position example; talk to the person directly as 'you' ----------

def test_interlocutor_position_is_suppressed():
    """The person she is talking WITH is addressed as 'you' — her position must not be rendered
    (she recited it: 'someone in front of me'). Other entities are still located."""
    from sweetie_bot_ai_core.scene import _describe
    inter = _describe(person(5, 0, is_interlocutor=True), CFG)   # bearing 0 => would be "in front"
    assert "in front" not in inter.lower() and "front of you" not in inter.lower()
    assert "the one talking with you" in inter                    # still identified as the partner
    assert "right" in _describe(person(6, 90), CFG).lower()       # a non-interlocutor is located


def test_solo_person_no_position_but_crowd_located():
    # ONE person -> no position rendered anywhere (she addresses them as "you")
    solo = render_scene(select_salient(SceneState(entities=[person(1, 0)]), CFG), [], CFG).lower()
    assert "in front" not in solo and "to your" not in solo
    # TWO people -> positions returned so she can tell them apart
    pair = render_scene(select_salient(SceneState(entities=[person(1, -40), person(2, 40)]),
                                       CFG), [], CFG).lower()
    assert "left" in pair and "right" in pair


def test_solo_person_arrival_has_no_position():
    solo = diff(SceneState(), SceneState(entities=[person(1, 0)]), CFG)
    assert len(solo) == 1 and "in front" not in solo[0].detail.lower()
    crowd = diff(SceneState(entities=[person(1, -40)]),
                 SceneState(entities=[person(1, -40), person(2, 40)]), CFG)
    arr = [e for e in crowd if e.kind == "arrived"][0]
    assert "right" in arr.detail.lower()


def test_no_mid_distance_leak():
    from sweetie_bot_ai_core.scene import _describe
    assert "mid" not in _describe(person(1, 40, distance="mid"), CFG).lower()
    assert "near" in _describe(person(2, 40, distance="near"), CFG).lower()


def test_id_note_has_no_parroted_position_example():
    from sweetie_bot_ai_core.scene import _ID_NOTE
    low = _ID_NOTE.lower()
    assert "the one in front of" not in low          # the phrase she parroted is gone
    assert "speak to them directly" in low            # replaced by the real rule
    assert "say an id number" in low                  # id/interlocutor prohibition stays


# --- servo faults name the leg location explicitly (front vs hind) ---------------------------

def test_servo_faults_get_leg_location():
    from sweetie_bot_ai_core.schema import RobotState
    summary = RobotState(servo_faults=["leg3_joint2"],
                         overheated_servos=["leg1_hip"]).human_summary()
    assert "hind-left leg" in summary        # leg3
    assert "front-left leg" in summary       # leg1
    assert "leg3_joint2" in summary          # raw name preserved alongside the location
