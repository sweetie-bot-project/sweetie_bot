"""Missing-speaker episode: «Where are you, interlocutor N?» must not loop.

Live 2026-07 report: with the interlocutor out of frame, the flexbe search
(ExecuteJointTrajectoryAndSay head_look_around + "Where are you, interlocutor human_N?")
re-fired forever — rosout evidence shows bursts of 13 identical launches within 3 s. Every
utterance (including the search's OWN say) re-creates talk-waiting-answer (6 s deadline),
and the is-missing predicate latched after a failed search was never checked by the
proposer (talk_simple.soar).

Contract (user 2026-07-11):
  * the search runs ONCE per disappearance episode. is-missing IS the episode latch:
    removed only when the person is seen again, so the cap re-arms on return;
  * after the failed search she GLANCES ONCE at the person's last remembered position
    (lookat2 uses the previous SWM position for invisible objects), then gives up with
    the sad "interlocutor is missing" reaction — no periodic re-checking of empty space;
  * when the missing person reappears she reacts ("Oh, there you are!" pool) — and never
    on a plain appearance.

Scenario shape mirrors the LIVE trigger: the person ASKS something and walks off before
she answers — her reply's talk-said then arms talk-waiting-answer while the person is
already out of the visibility bins. (A silent vanish lets the talk go cold before the
search's precondition ever matures, and speech from a long-gone person binds to nobody —
both verified in sim.)

Mechanism-first asserts: the flexbe launch line + SOAR SPECIFIC markers only (canned
reactions get rephrased + translated -> never assert voiced text; no tracker-id keying —
regexes match the behavior/marker names, not human_N).
"""
import rospy

from sweetie_bot_behavior_synth import agent_params, behavior_test, person, scene, soar_params

# The search precondition is "interlocutor not seen for the whole `just` bin" (production
# value 10 s) while the talk-waiting-answer predicate that carries it lives only 6 s past
# each of her utterances — at production values the overlap gambles on LLM reply latency.
# Shrinking `just` to 4 s puts the edge inside every reply's predicate window and makes the
# scenario deterministic; the now-bin (binding, is-missing removal) keeps production value.
_FAST_BINS = {"names": ["now", "just", "recently", "long-time"],
              "values": [2.0, 4.0, 120.0]}

# flexbe.py logs one line per behavior launch ("paramters" sic in the source); the text
# argument distinguishes the missing-speaker search from the sibling searching-anyone
# behavior ("Where did everyone go?").
_SEARCH_RX = r"executing behavior ExecuteJointTrajectoryAndSay.*Where are you"
_LATCH_RX = r"SPECIFIC: MISSING-LATCHED"        # is-missing latched (search cap engaged)
_GLANCED_RX = r"SPECIFIC: GLANCED-MISSING"      # one-shot glance at last position done
_GONE_RX = r"SPECIFIC: MISSING\."               # sad give-up reaction (say-missing-answer)
_FOUND_RX = r"SPECIFIC: FOUND-MISSING"          # reappearance reaction
_LOOKAT_HUMAN_RX = r"look-at cmd on human"      # lookat2 cmd log line (label, type)


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


def _say_with_retry(world, text, tries=3):
    """say_and_wait with retries: the ask occasionally never reaches the agent (known
    harness say-pipeline flake — HANDOFF)."""
    for i in range(tries):
        try:
            return world.say_and_wait(text)
        except AssertionError:
            if i == tries - 1:
                raise
            rospy.sleep(2.0)


def _establish_dialogue(world):
    """Wait out the greeting (saying into its generation window is the first-say flake),
    open the dialogue, then DRAIN the turn: the reply text arrives before the voice (and
    any spurious play_animation tool call) finishes, and an ask landing mid-turn can go
    unanswered entirely (verified in sim: TALK-HEARD + SOURCE, yet no GenerateReply goal)."""
    world.col["soar_log"].wait_grep("SPECIFIC: GREETING", timeout=30.0)
    t = _say_with_retry(world, "Hello Sweetie! How are you today?")
    # Drain until the answering PROCESS finishes, not a fixed sleep: say_and_wait returns at
    # the reply TEXT, but the turn keeps running (voicing + spurious dance_stamp tool calls,
    # ~7 s) — a fixed 6 s drain was outrun in sim and the follow-up ask landed mid-turn and
    # went entirely unanswered (no talk-waiting-answer -> the search can never arm).
    world.col["soar_log"].wait_grep(r"FINISH PROCESS llm-answering-on", timeout=30.0)
    rospy.sleep(3.0)
    return t


def _go_missing(world):
    """The live trigger: the person asks something and WALKS OFF before she answers.
    Speech-to-human binding needs the human effectively `visible now` (verified in sim:
    says landing 2.5+ s after the vanish get TALK-HEARD with no TALK-EVENT SOURCE ->
    no reply), so the ask lands while visible and the vanish follows ~1 s later. Her
    reply's talk-said then arms talk-waiting-answer right around the 10 s just-bin edge
    -> the search fires with no timing race.
    Anchors the soar log, so all _count() calls restart at this episode.

    TODO (user 2026-07-11): more realistic scenario — world.move(101, bearing=~65) in a
    couple of steps BEFORE the vanish, so the last remembered position is OFF-CENTER and
    the glance visibly turns her head toward where the person left (a dead-center vanish
    leaves the last position already in her gaze, so the glance produces no head motion).
    Reappearances should then also spawn at that off-center bearing."""
    world.col["soar_log"].anchor()
    world.speech.say("I am still here, can you hear me? Answer me please!")
    rospy.sleep(1.0)    # utterance lands + binds while the human is still visible
    world.vanish(101)


@behavior_test
@agent_params(**{"proactive/enabled": False})
@soar_params(**{"input/swm/time_bins_map": _FAST_BINS})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_missing_speaker_search_once_per_episode(world):
    _establish_dialogue(world)

    # EPISODE 1 runs in the CROWD regime (the live loop context): a bystander keeps some
    # human visible, so searching-anyone can never fire and finish the talk — the hold
    # phase below then guards the livelock (pre-fix, a visible bystander killed the dead-
    # continuer search via the resource arbiter and it relaunched every decide cycle).
    # The bystander is spawned only AFTER the search launches: an un-greeted human spawned
    # during the arming window steals her answer turn (SOAR tie-breaks toward the greeting)
    # and talk-waiting-answer for the vanished speaker never arms (seen in sim run 7).
    # Spawned visible through the search + hold, 202 also gets greeted while VISIBLE, so
    # episode 2 never sees the greet-the-vanished-bystander + search-for-202 hazard.
    _go_missing(world)
    assert _wait_count(world, _SEARCH_RX, 1, timeout=30.0), \
        "missing-speaker search never fired after the interlocutor went off-frame"
    world.spawn(person(id=202, bearing=-40.0, dist=2.0))
    # THE CAP: one search per episode, however long the talk lives.
    rospy.sleep(40.0)
    assert _count(world, _SEARCH_RX) == 1, \
        "search re-fired within one disappearance episode («Где вы» loop)"

    # EPISODE 2: the person returns (visible=now clears the episode latch) and goes
    # missing again -> the search must re-arm, and the cap must hold again. The bystander
    # leaves first so the ask cannot bind to them (empty-room regime works here: with the
    # fast bins the search wins the visibility edge before searching-anyone).
    world.vanish(202)
    rospy.sleep(2.0)
    world.spawn(person(id=101, bearing=0.0, dist=1.5))
    rospy.sleep(4.0)
    _say_with_retry(world, "I am back! Did you miss me?")
    _go_missing(world)
    assert _wait_count(world, _SEARCH_RX, 1, timeout=30.0), \
        "search did not re-arm on a new disappearance episode"
    rospy.sleep(20.0)
    assert _count(world, _SEARCH_RX) == 1, "second-episode search re-fired (cap not re-latched)"


@behavior_test
@agent_params(**{"proactive/enabled": False})
@soar_params(**{"input/swm/time_bins_map": _FAST_BINS})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_missing_speaker_glance_once_then_gives_up(world):
    _establish_dialogue(world)
    _go_missing(world)
    assert _wait_count(world, _SEARCH_RX, 1, timeout=30.0), \
        "missing-speaker search never fired after the interlocutor went off-frame"

    # search failed -> episode latched
    assert world.col["soar_log"].wait_grep(_LATCH_RX, timeout=20.0), \
        "is-missing never latched after the failed search"
    # the give-up phrase must NOT precede the glance (sequenced by the glanced latch)
    assert _count(world, _GONE_RX) == 0, "give-up reaction fired before the glance"

    # the glance: a look-at lands on the (invisible) human, then the glance latch closes
    # NOTE: no look-at-cmd-count assert here — the glance launches ~3 ms after MISSING-LATCHED,
    # faster than any post-search baseline can be taken (anchor race), and lookat2 may also reuse
    # an in-flight cmd without logging a new issue line. The glanced-missing latch itself requires
    # the looking-at-missing-speaker process to have run to termination/deadline, so it IS the
    # reliable evidence that the glance executed.
    assert world.col["soar_log"].wait_grep(_GLANCED_RX, timeout=20.0), \
        "glance at the last known position never ran (no glanced-missing latch)"

    # give-up: the sad missing reaction, once, after the glance
    assert _wait_count(world, _GONE_RX, 1, timeout=30.0), \
        "no 'interlocutor is missing' give-up after the glance"

    # ONE-SHOT: while the episode persists nothing re-runs (user: no periodic re-checking)
    rospy.sleep(20.0)
    assert _count(world, _GLANCED_RX) == 1, "glance re-ran within one episode"
    assert _count(world, _GONE_RX) == 1, "'interlocutor is missing' repeated within one episode"


@behavior_test
@agent_params(**{"proactive/enabled": False})
@soar_params(**{"input/swm/time_bins_map": _FAST_BINS})
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_missing_speaker_found_reaction(world):
    _establish_dialogue(world)
    # negative: a plain appearance + dialogue must never trigger the found reaction
    assert _count(world, _FOUND_RX) == 0, "FOUND-MISSING fired on a plain appearance"

    _go_missing(world)
    assert _wait_count(world, _SEARCH_RX, 1, timeout=30.0), \
        "missing-speaker search never fired after the interlocutor went off-frame"
    # let the search finish and the episode latch (fixed sleep: works pre- and post-fix,
    # the RED run has no MISSING-LATCHED seam to wait on)
    rospy.sleep(10.0)

    # the missing person reappears -> is-missing is removed -> the found reaction fires
    world.spawn(person(id=101, bearing=0.0, dist=1.5))
    assert world.col["soar_log"].wait_grep(_FOUND_RX, timeout=25.0), \
        "no found-you reaction when the missing interlocutor reappeared"
    rospy.sleep(10.0)
    assert _count(world, _FOUND_RX) == 1, "found-you reaction repeated"
