"""Self-talk silence-inference guard (user rule 2026-07-08, live leak T.1#2 2026-07-09).

Muses must NEVER claim the place is quiet/empty from vision-only inference — out of frame
does not mean absent, and her venues are loud crowded conventions. The model re-infers
"quiet" even with rewritten cues, so the guard is a deterministic post-decode DROP (empty
text = valid self-talk silence). It never re-instructs: injected corrections get parroted
by the 7B (the live prompt-echo defect, S.4#4).

The offender parametrization is the LIVE calibration set — phrasings she was actually
caught voicing. Extend it from the 'self-talk silence guard: dropped' agent-log lines.
"""
import pytest

from test_core import _rephrase_agent

from sweetie_bot_ai_core.schema import AgentRequest, RequestType, SceneEntity, SceneState, Zone


def _self_talk(structured_json):
    scene = SceneState(entities=[SceneEntity(id=7, type="pony_face", zone=Zone.front)])
    reg, sp, agent = _rephrase_agent(structured_json, scene=scene)
    return agent.handle(AgentRequest(request_type=RequestType.self_talk,
                                     profile="self-talk-en",
                                     text="Something caught your attention."))


_LIVE_OFFENDERS = [
    "It's quiet here, I hope someone stops by soon to chat.",   # T.1#2, 2026-07-09
    "It feels nice to be alone.",                               # S.7 calibration list
    "Silence is such a sweet relief.",
    "It feels nice to have some quiet time.",
    "It's just me and the silence.",
    "There is no one here right now.",                          # canonical rule phrasing
]


@pytest.mark.parametrize("offender", _LIVE_OFFENDERS)
def test_live_offender_phrasings_are_dropped(offender):
    reply = _self_talk('{"response_text":"%s",'
                       '"emotion":"neutral","sentence_type":"statement"}' % offender)
    assert reply.response_text == "", "silence-inference leaked: %r" % offender


_BENIGN = [
    "I wonder what my pony friends are doing right now.",
    "Come on, get your hand off my camera!",                   # the occlusion complaint
    "A cheerful tune is humming quietly in my head.",          # adverb 'quietly' must PASS
]


@pytest.mark.parametrize("text", _BENIGN)
def test_benign_muses_pass_through(text):
    reply = _self_talk('{"response_text":"%s",'
                       '"emotion":"joy","sentence_type":"statement"}' % text)
    assert reply.response_text == text


def test_dropped_muse_does_not_poison_the_repeat_window():
    # a never-voiced muse must not enter _recent_replies (it would false-trigger the
    # anti-repeat guard against a later legitimate reply)
    scene = SceneState(entities=[SceneEntity(id=7, type="pony_face", zone=Zone.front)])
    reg, sp, agent = _rephrase_agent(
        '{"response_text":"It feels nice to be alone.",'
        '"emotion":"neutral","sentence_type":"statement"}', scene=scene)
    agent.handle(AgentRequest(request_type=RequestType.self_talk,
                              profile="self-talk-en", text="cue"))
    assert not list(agent._recent_replies)
