"""Self-talk under a covered camera: deterministic anger + the WARNING banner in the prompt.

The reply path already forces Emotion.anger while occluded (agent.py) — the self-talk path
is the only speaker while she is blind (SOAR sends no goals without a visible person), so it
needs the same override: the banner drives the words, the override drives the eyes.
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


def test_self_talk_emotion_untouched_when_clear():
    scene = SceneState(entities=[SceneEntity(id=7, type="pony_face", zone=Zone.front)])
    reg, sp, agent = _rephrase_agent(
        '{"response_text":"What a lovely quiet moment.",'
        '"emotion":"joy","sentence_type":"statement"}', scene=scene)
    reply = agent.handle(AgentRequest(request_type=RequestType.self_talk,
                                      profile="self-talk-en",
                                      text="You notice how calm it is."))
    assert reply.emotion == Emotion.joy
