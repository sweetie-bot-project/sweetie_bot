"""Greet-the-vanished-human: the greeting candidate must require visibility.

Belief objects persist for memorize_time (600 s) after a person leaves (they are a pure
i-supported mirror of SWM — vision*elaborate*object), so pre-fix
talk-llm*elborate*interlocutor-candidate (no visibility condition) kept offering a
DEPARTED un-greeted human as a greeting candidate. The phantom greeting has TWO
failure modes, both real:
  * usually the greeting's verbolize dies in focusing-on-interlocutor (the look-at at
    an invisible human rides its 5 s deadline -> FINISH PROCESS rule-asking ... WITH
    STATUS missing) and RETRIES every ~5 s — a head+speech resource-churn loop aimed
    at empty space. The SPECIFIC: GREETING marker is SUCCESS-GATED and never prints,
    so the marker alone cannot see this (cost us an invalid RED, timeline 2026-07-12);
  * when the look-at at the remembered position does succeed she voices the greeting,
    its talk-said arms talk-waiting-answer (6 s) for the invisible person, and the
    expiry launches the «Where are you, interlocutor N?» search for someone she never
    actually spoke with (observed in sim 2026-07-12 with a departed bystander).
The primary discriminator is therefore the rule-asking/missing loop (rule-asking is
the greeting proposer's process name, unique to it); the marker and search asserts
guard the voiced-phantom variant.

Scenario shape (each element is load-bearing, learned the hard way):
  * While an interlocutor is visible, the chatter loop (reply -> 6 s waiting-answer ->
    talk-no-answer -> reply) never leaves the >=10 s event gap that the greeting's
    ^pause needs — a newcomer's greeting STARVES (verified in sim: 60 s, zero
    greetings). So the bystander 202 stays only briefly and leaves un-greeted; no
    "greet the newcomer while visible" phase is possible mid-conversation.
  * The goodbye to 101 sets expecting-to-leave(101) (60 s): 101 drops out of BOTH the
    interlocutor set (chatter dies -> pause can mature) and the candidate set (no
    self-re-greeting), while staying VISIBLE — which blocks searching-anyone, whose
    success would finish-talk and kill the substate before the phantom matures.
    A vanished human can never START a talk (talk-start needs a looking-at predicate,
    which needs visible now), so some visible human must anchor the whole window.
  * The goodbye is said in the quiet window right after a chatter turn finishes:
    an in-flight reply landing after the goodbye talk-heard steals most-recent-event
    and the canned reaction (and with it expecting-to-leave) is skipped — hence the
    FINISH-seam timing + retry.
  * Post-fix the window is structurally silent: no candidate (202 invisible, 101
    excluded then still greeted), no chatter (no visible interlocutor), no
    searching-anyone (101 visible), and talk-waiting-answer for 202 can only ever be
    armed by the phantom greeting itself — so the no-search assert is sound.

Default SWM bins on purpose (no @soar_params): the phantom-greeting edge is
preference-resolved, not deadline-overlap-resolved, so the _FAST_BINS rationale from
test_missing_speaker does not apply and the production regime is the honest one.

Mechanism-first asserts: SOAR SPECIFIC markers + the flexbe launch line only (canned
reactions get rephrased + translated -> never assert voiced text; no tracker-id keying —
regexes match marker/behavior names, not human_N).
"""
import rospy

from sweetie_bot_behavior_synth import agent_params, behavior_test, person, scene

_GREET_RX = r"SPECIFIC: GREETING"
# The greeting attempt seam: rule-asking is the greeting proposer's process name (unique
# to it); STATUS missing = its verbolize could not focus the (invisible) target.
_PHANTOM_RX = r"FINISH PROCESS rule-asking .* WITH STATUS missing"
# flexbe.py logs one line per behavior launch; the "Where are you" text distinguishes the
# missing-speaker search from the sibling searching-anyone ("Where did everyone go?").
_SEARCH_RX = r"executing behavior ExecuteJointTrajectoryAndSay.*Where are you"


def _count(world, rx):
    return len(world.col["soar_log"].grep(rx))


def _wait_count(world, rx, n, timeout):
    """Wait until the count from the anchor reaches n (grep()+len, NOT wait_grep: later
    phases must not re-match earlier lines — test_occlusion_proactive idiom)."""
    deadline = rospy.get_time() + timeout
    while rospy.get_time() < deadline:
        if _count(world, rx) >= n:
            return True
        rospy.sleep(0.5)
    return False


@behavior_test
@agent_params(**{"proactive/enabled": False})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_no_greeting_for_vanished_human(world):
    # phase 0: baseline — the present, un-greeted 101 is greeted (pause start)
    assert world.col["soar_log"].wait_grep(_GREET_RX, timeout=30.0), "no baseline greeting"

    # phase 1: bystander 202 appears briefly and leaves UN-GREETED (greeting starves
    # while the 101 chatter runs — see module docstring). The trailing sleep takes 202
    # out of the visible-now bin (2 s) so the goodbye below binds to 101 deterministically.
    world.spawn(person(id=202, bearing=-30.0, dist=1.8))
    rospy.sleep(4.0)
    world.vanish(202)
    rospy.sleep(3.0)

    # phase 2: goodbye to 101 -> canned GOODBYE -> expecting-to-leave(101): chatter dies,
    # 101 stops being a candidate, stays visible (blocks searching-anyone). Timed into
    # the post-turn quiet window, retried if an in-flight reply ate the canned reaction.
    got_goodbye = False
    for _ in range(3):
        world.col["soar_log"].anchor()
        world.col["soar_log"].wait_grep(
            r"FINISH PROCESS (llm-answering-on|rule-asking)", timeout=30.0)
        rospy.sleep(0.5)
        world.speech.say("Okay, goodbye Sweetie!")
        if world.col["soar_log"].wait_grep(r"SPECIFIC: GOODBYE", timeout=15.0):
            got_goodbye = True
            break
    assert got_goodbye, "canned goodbye never fired — cannot set up expecting-to-leave"

    # phase 3: THE DISCRIMINATOR. Pre-fix the departed 202 is now the ONLY greeting
    # candidate: once the pause matures (~15-20 s: goodbye talk-said + 6 s waiting-answer
    # -> talk-no-answer + 10 s just-window) the greeting fires at the empty air and
    # retries on a ~5 s look-at-deadline cadence (timeline 2026-07-12: 4 attempts in the
    # window). Fixed code: the candidate cannot exist -> structural silence.
    world.col["soar_log"].anchor()
    rospy.sleep(40.0)
    assert _count(world, _PHANTOM_RX) == 0, \
        "greeting attempts at a vanished human (rule-asking finished missing)"
    assert _count(world, _GREET_RX) == 0, "greeted a human who is no longer visible"
    assert _count(world, _SEARCH_RX) == 0, \
        "missing-speaker search armed by a phantom greeting to a vanished human"

    # phase 4: 202 RETURNS -> visible candidate again (still un-greeted), pause long
    # mature -> she must greet. Guards the gate against over-blocking and pins the
    # `visible now` strictness decision (a returning human is greeted, not ignored).
    world.spawn(person(id=202, bearing=-30.0, dist=1.8))
    assert _wait_count(world, _GREET_RX, 1, timeout=30.0), \
        "returning (visible) human was not greeted — visibility gate over-blocks"
