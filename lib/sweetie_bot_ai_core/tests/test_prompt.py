"""Direct build_system_prompt + render_scene coverage.

Ordering matters for KV-cache reuse (static persona first) and for salience (the occlusion
WARNING banner must lead the scene block). Attribute sets mirror real live detections
(gaze_at_robot / emotion / holding / color plus the internal telemetry keys that leaked once).
"""
from sweetie_bot_ai_core import DEFAULT_PERSONA, SceneConfig, build_system_prompt, render_scene
from sweetie_bot_ai_core.schema import (RobotState, SceneEntity, SceneEvent, SceneState,
                                        SoundCue, Zone)


# --- build_system_prompt ordering ---------------------------------------------------------------

def test_prompt_section_ordering():
    state = RobotState(battery_percent=80, battery_status="discharging", pose="body_nominal")
    p = build_system_prompt(DEFAULT_PERSONA, state, tools_offered=True,
                            scene_block="Around you right now:\n- a pony (id 201)",
                            language_note="Always answer in English regardless.")
    # persona description leads (KV-cacheable static prefix)
    assert p.startswith(DEFAULT_PERSONA.description.strip()[:40])
    i_guidelines = p.index("Guidelines:")
    i_tools = p.index("You have tools available")
    i_scene = p.index("Around you right now:")
    i_state = p.index("Your current live state")
    i_lang = p.index("Always answer in English")
    assert i_guidelines < i_tools < i_scene < i_state < i_lang
    assert "80%" in p and "body_nominal" in p


def test_prompt_omits_optional_sections():
    p = build_system_prompt(DEFAULT_PERSONA, None, tools_offered=False,
                            scene_block=None, language_note=None)
    assert "You have tools available" not in p
    assert "Your current live state" not in p
    assert "Always answer in English" not in p


# --- render_scene ---------------------------------------------------------------------------------

CFG = SceneConfig()


def test_scene_real_attr_set_renders_allowlisted_only():
    # attribute set mirroring a real live detection incl. the internal keys that once leaked
    e = SceneEntity(id=101, type="person", zone=Zone.front, bearing_deg=0.0,
                    is_interlocutor=True,
                    attributes={"gaze_at_robot": "true", "emotion": "happy",
                                "holding": "a cup", "gaze_pitch": "3.5", "gaze_yaw": "-12.0",
                                "attention_state": "engaged"})
    block = render_scene(SceneState(entities=[e]), [], CFG)
    assert "looking at you" in block
    assert "looks happy" in block
    assert "holding a cup" in block
    # internal telemetry must never surface (she parrots raw key/values as speech)
    assert "gaze_pitch" not in block and "3.5" not in block
    assert "gaze_yaw" not in block and "attention_state" not in block
    # unknown attrs are silent by default (allowlist-only)
    e2 = e.model_copy(update={"attributes": {"mystery_key": "42"}})
    block2 = render_scene(SceneState(entities=[e2]), [], CFG)
    assert "mystery_key" not in block2 and "42" not in block2


def test_scene_sound_lines():
    st = SceneState(sounds=[SoundCue(zone=Zone.front, bearing_deg=0.0, kind="speech"),
                            SoundCue(zone=Zone.side, bearing_deg=90.0, kind="sound")])
    block = render_scene(st, [], CFG)
    assert "- You hear a voice directly in front of you." in block
    assert "- You hear a sound to your right." in block


def test_scene_lone_person_is_position_free_but_crowd_is_located():
    lone = SceneEntity(id=101, type="person", zone=Zone.front, bearing_deg=30.0)
    block = render_scene(SceneState(entities=[lone]), [], CFG)
    # a lone person is "you" — never a position to recite back
    assert "in front of you" not in block.split("(These notes")[0]
    two = [SceneEntity(id=101, type="person", zone=Zone.front, bearing_deg=30.0),
           SceneEntity(id=102, type="person", zone=Zone.front, bearing_deg=-30.0)]
    block2 = render_scene(SceneState(entities=two), [], CFG)
    assert "to your right" in block2 and "to your left" in block2


def test_scene_occlusion_banner_leads():
    ents = [SceneEntity(id=1, type="camera_occluded", zone=Zone.front),
            SceneEntity(id=101, type="person", zone=Zone.front)]
    block = render_scene(SceneState(entities=ents), [], CFG)
    assert block.startswith("WARNING: something is pressed right against your camera")


def test_scene_events_and_remembered_sections():
    ev = [SceneEvent(kind="left", entity_id=201, detail="a pony left")]
    gone = SceneEntity(id=201, type="pony", zone=Zone.front, bearing_deg=40.0,
                       in_frame=False, last_seen_s=12.0)
    block = render_scene(SceneState(entities=[gone]), ev, CFG)
    assert "Since you last replied:" in block and "- a pony left." in block
    assert "Recently seen (now OUT of view" in block
    assert "last seen ~12s ago" in block


def test_empty_scene_renders_empty():
    assert render_scene(SceneState(), [], CFG) == ""
