"""Self-talk emotion policy: occluded forces anger; clear clamps to a friendly whitelist.

The reply path already forces Emotion.anger while occluded (agent.py) — the self-talk path
is the only speaker while she is blind (SOAR sends no goals without a visible person), so it
needs the same override: the banner drives the words, the override drives the eyes.

The converse (live T.1#1, 2026-07-09): with a CLEAR lens the 7B occasionally self-tags a
friendly muse [anger] — structurally impossible to be the override (remembered entities are
in_frame=False and is_occluded requires in_frame). Clamp: when not occluded, only the
friendly whitelist survives; anger is reachable ONLY via the occlusion override.
"""
from test_core import _rephrase_agent

from sweetie_bot_ai_core.schema import AgentRequest, Emotion, RequestType, SceneEntity, \
    SceneState, Zone
from sweetie_bot_ai_core.scene import CAMERA_OCCLUDED


def test_self_talk_forces_anger_and_sees_banner_while_occluded():
    # zone=rear on purpose: the flag's zone is numerically unstable and must not matter
    scene = SceneState(entities=[SceneEntity(id=0, type=CAMERA_OCCLUDED, zone=Zone.rear)])
    reg, sp, agent = _rephrase_agent(
        '{"response_text":"Hey! Something is on my camera!",'
        '"emotion":"joy","sentence_type":"statement"}', scene=scene)

    reply = agent.handle(AgentRequest(request_type=RequestType.self_talk,
                                      profile="self-talk-en",
                                      text="Something is blocking your view."))

    assert reply.response_text == "Hey! Something is on my camera!"
    # the model said joy; the blocked camera overrides it deterministically
    assert reply.emotion == Emotion.anger
    # the WARNING banner reached the self-talk prompt (select_salient keeps the flag)
    sys_msg = next(m["content"] for m in reg.last_messages if m["role"] == "system")
    assert "pressed right against your camera" in sys_msg


def _clear_scene_self_talk(structured_json):
    scene = SceneState(entities=[SceneEntity(id=7, type="pony_face", zone=Zone.front)])
    reg, sp, agent = _rephrase_agent(structured_json, scene=scene)
    return agent.handle(AgentRequest(request_type=RequestType.self_talk,
                                     profile="self-talk-en",
                                     text="You notice how calm it is."))


def test_self_talk_emotion_untouched_when_clear():
    # "calm", not "quiet": the silence-inference guard must not blank this fixture's text,
    # and the response_text assert keeps the test premise honest (a voiced friendly muse)
    reply = _clear_scene_self_talk(
        '{"response_text":"What a lovely calm moment.",'
        '"emotion":"joy","sentence_type":"statement"}')
    assert reply.response_text == "What a lovely calm moment."
    assert reply.emotion == Emotion.joy


def test_self_talk_anger_clamped_to_neutral_when_clear():
    # the live T.1#1 shape: friendly muse, clear lens, model self-tags anger
    reply = _clear_scene_self_talk(
        '{"response_text":"That toy on the shelf looks fun.",'
        '"emotion":"anger","sentence_type":"statement"}')
    assert reply.response_text == "That toy on the shelf looks fun."
    assert reply.emotion == Emotion.neutral


def test_self_talk_fear_clamped_to_neutral_when_clear():
    reply = _clear_scene_self_talk(
        '{"response_text":"That toy on the shelf looks fun.",'
        '"emotion":"fear","sentence_type":"statement"}')
    assert reply.emotion == Emotion.neutral


def test_self_talk_sadness_kept_when_clear():
    # sadness is on the friendly whitelist - a wistful muse is legitimate
    reply = _clear_scene_self_talk(
        '{"response_text":"I miss my pony friends a little.",'
        '"emotion":"sadness","sentence_type":"statement"}')
    assert reply.emotion == Emotion.sadness
