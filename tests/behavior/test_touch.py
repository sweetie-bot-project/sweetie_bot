"""Touch reactions — per-zone canned SOAR behavior (no LLM needed to fire them).

The touch input module is soar.yaml `touch: {type: touch_pulse}`; a synth touch is a KeyPressed
message with the zone name on the touch decoder's output topic (button->name map in
`proto32/touch_config.yaml`: nose, forehead, cheek_left/right, temple_left/right). The exact
topic is resolved at runtime from rosparam (/soar/input/touch/*).

Touch reactions are gated on `-^talking` (they will not interrupt speech), so a momentary tap
right after an utterance is racy - HOLD the touch until she is free. We assert on the command
SOAR emits on voice/syn (the behavioural decision), not on audio-backend success.

Per-zone reactions (talk_simple.soar):
  * nose            -> SOUND ONLY: emits `voice/play_wav squeak_nose`, no spoken phrase (boop);
  * forehead        -> a canned rejection phrase (unwelcome touch);
  * cheek / temple  -> a canned pleasure phrase (the phrase zones).
"""
import threading
import time

import pytest
import rospy


from sweetie_bot_behavior_synth import behavior_test, person, scene


def _touch_topic() -> str:
    try:
        return rospy.get_param("/soar/input/touch/topic")
    except KeyError:
        pytest.skip("no /soar/input/touch/topic param - touch module not configured")


def _voice_collector():
    from sweetie_bot_text_msgs.msg import TextActionActionGoal
    from sweetie_bot_behavior_synth.collectors import TopicCollector
    return TopicCollector("voice/syn/goal", TextActionActionGoal)


def _hold_until(pub, keys, pred, timeout=25.0):
    """Hold a touch (keep publishing the press) until pred() is true or timeout, then release.

    Holding rides out any in-progress speech: the reaction fires the moment she is free."""
    from sweetie_bot_joystick.msg import KeyPressed
    m = KeyPressed()
    m.keys = keys
    deadline = time.monotonic() + timeout
    hit = None
    while time.monotonic() < deadline:
        m.header.stamp = rospy.Time.now()
        pub.publish(m)
        rospy.sleep(0.3)
        hit = pred()
        if hit:
            break
    rel = KeyPressed()
    rel.header.stamp = rospy.Time.now()
    rel.keys = []
    pub.publish(rel)
    return hit


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_boop_nose_plays_sound_not_speech(world):
    """A boop on the nose issues the `squeak_nose` sound command and speaks NO phrase.

    The nose is a sound-only touch zone (user decision); spoken phrases live on the cheek/temple
    zones. Guards against the nose ever regressing to a spoken reaction.
    """
    from sweetie_bot_joystick.msg import KeyPressed
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=1)
    voice = _voice_collector()
    rospy.sleep(0.5)
    world.say_and_wait("Hello Sweetie!")            # establish talk context (most-recent-event)
    since = time.monotonic()

    def _boop_cmd():
        for msg in voice.messages(since):
            c = msg.goal.command
            if c.type == "voice/play_wav" and "squeak_nose" in c.command:
                return c
        return None

    try:
        hit = _hold_until(pub, ["nose"], _boop_cmd, timeout=25.0)
        assert hit is not None, "boop did not issue the squeak_nose sound command"
        # sound-only zone: no spoken phrase must accompany a boop
        says = [m.goal.command.command for m in voice.messages(since)
                if m.goal.command.type.startswith("voice/say/")]
        assert not says, f"boop unexpectedly spoke: {says}"
    finally:
        voice.close()


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_cheek_scratch_speaks_a_phrase(world):
    """A scratch on a cheek voices a canned pleasure phrase (a phrase zone)."""
    from sweetie_bot_joystick.msg import KeyPressed
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=1)
    voice = _voice_collector()
    rospy.sleep(0.5)
    world.say_and_wait("Hello Sweetie!")
    since = time.monotonic()

    def _say_cmd():
        for msg in voice.messages(since):
            if msg.goal.command.type.startswith("voice/say/"):
                return msg.goal.command
        return None

    try:
        hit = _hold_until(pub, ["cheek_left"], _say_cmd, timeout=25.0)
        assert hit is not None, "no spoken reaction to a cheek scratch"
    finally:
        voice.close()


def _tap(pub, zone, hold_s=0.03):
    """One quick tap: press+release, both usually inside a single SOAR input tick."""
    from sweetie_bot_joystick.msg import KeyPressed
    m = KeyPressed()
    m.header.stamp = rospy.Time.now()
    m.keys = [zone]
    pub.publish(m)
    rospy.sleep(hold_s)
    r = KeyPressed()
    r.header.stamp = rospy.Time.now()
    r.keys = []
    pub.publish(r)


def _squeaks(goals, since):
    return [m for m in goals.messages(since)
            if m.goal.command.type == "voice/play_wav"
            and "squeak_nose" in m.goal.command.command]


def _non_squeak_goals(goals, since):
    """Voice goals in the window that are NOT our squeak — the witness that she spoke."""
    return [m for m in goals.messages(since)
            if not (m.goal.command.type == "voice/play_wav"
                    and "squeak_nose" in m.goal.command.command)]


class _SpeechWatch:
    """Speech OCCUPANCY tracker: busy while the voice/syn action server has a live goal.

    A goal-issue-time quiet check loses two races (both bit the 2026-07-08 runs): TTS +
    playback occupy her 5-9 s PAST the goal (a "quiet" tap lands mid-speech and -^talking
    eats it), and say_and_wait can return on the agent's reply log line a beat BEFORE the
    voice goal exists (a burst fired into that gap fell entirely inside the utterance).
    Occupancy comes from the actionlib STATUS stream, not goal/result counting: the result
    topic is LATCHED, so a fresh subscriber replays the previous test's result and the
    count is off by one forever (this exact skew put a tap 1.4 s before speech end). The
    verbolize lead time (^talking latches ~3 s before a reply's goal while the LLM
    generates) is handled by voiding interfered trials instead (see the tap tests)."""

    _LIVE = (0, 1, 6, 7)    # PENDING, ACTIVE, PREEMPTING, RECALLING

    def __init__(self):
        from actionlib_msgs.msg import GoalStatusArray
        from sweetie_bot_text_msgs.msg import TextActionActionGoal
        from sweetie_bot_behavior_synth.collectors import TopicCollector
        self.goals = TopicCollector("voice/syn/goal", TextActionActionGoal)
        self._lock = threading.Lock()
        self._busy = False
        self._last_active = 0.0
        self._status_sub = rospy.Subscriber(
            "voice/syn/status", GoalStatusArray, self._status_cb, queue_size=5)

    def _status_cb(self, msg):
        active = any(s.status in self._LIVE for s in msg.status_list)
        with self._lock:
            self._busy = active
            if active:
                self._last_active = time.monotonic()

    def busy(self):
        with self._lock:
            return self._busy

    def last_activity(self):
        with self._lock:
            last_active = self._last_active
        return max(self.goals.stamps() + [last_active])

    def close(self):
        self.goals.close()
        self._status_sub.unregister()


def _wait_quiet(speech, quiet_s=4.0, timeout=60.0):
    """Wait until she is actually FREE: no say in flight (goal without its result) and no
    voice activity at all for quiet_s. Touch is -^talking gated BY DESIGN (reactions never
    interrupt speech) and SOAR speaks autonomously (lull re-poke every ~25-30 s, proactive
    muses), so the tap tests must strike while she is genuinely quiet — live UX: no boop
    mid-speech. quiet_s also outlasts the touch_pulse latch+refractory (0.6 + 1.5 s)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not speech.busy() and time.monotonic() - speech.last_activity() >= quiet_s:
            return True
        rospy.sleep(0.3)
    return False


def _say_with_retry(world, text, tries=2):
    """say_and_wait with one retry: the ask occasionally never reaches the agent (known
    harness say-pipeline flake, ~1/9 runs — HANDOFF 0.2#5)."""
    for i in range(tries):
        try:
            return world.say_and_wait(text)
        except AssertionError:
            if i == tries - 1:
                raise
            rospy.sleep(2.0)


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_quick_nose_taps_squeak(world):
    """A quick TAP (press+release inside one SOAR input tick) must still boop (user req,
    HANDOFF M.4b#4). The plain joystick module mirrors only the LATEST buffered message per
    tick, so a tap's release overwrites its press in the buffer and the touch vanishes —
    live, quick boops made no squeak. The touch_pulse module latches the rising edge.

    Contract: a tap while she is FREE squeaks. A trial she spoke into (witnessed by a
    non-squeak voice goal in the window — e.g. the lull re-poke, whose ^talking latches
    ~3 s before its goal is visible) is VOID and retried, not a failure: touch reactions
    never interrupt speech by design. A miss with NO witness fails immediately."""
    from sweetie_bot_joystick.msg import KeyPressed
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=4)
    speech = _SpeechWatch()
    rospy.sleep(0.5)
    _say_with_retry(world, "Hello Sweetie!")    # establish talk context (most-recent-event)
    want, hits, voids = 4, 0, 0
    try:
        while hits < want:
            assert _wait_quiet(speech), "she never went quiet between taps"
            since = time.monotonic()
            _tap(pub, "nose")
            deadline = time.monotonic() + 8.0
            hit = False
            while time.monotonic() < deadline and not hit:
                rospy.sleep(0.2)
                hit = bool(_squeaks(speech.goals, since))
            if hit:
                hits += 1
            else:
                assert _non_squeak_goals(speech.goals, since), \
                    "quick tap in a verified-quiet window made no squeak (edge lost)"
                voids += 1
                # generous budget: with nobody answering her, SOAR's talk-waiting-answer
                # deadline re-pokes a reply every ~6 s, so quiet windows are only a few
                # seconds wide and collisions are common
                assert voids <= 6, \
                    f"only {hits}/{want} taps squeaked; {voids} trials voided by speech"
            rospy.sleep(2.5)        # sit out the latch + refractory before the next tap
    finally:
        speech.close()


@behavior_test
@scene(person(id=101, bearing=0.0, dist=1.5))
def test_rapid_tapping_is_rate_limited(world):
    """Rapid tapping must not machine-gun squeaks (the 'cooldown' half of the user request):
    edges inside the latch window merge into one exposure and the refractory absorbs the
    rest — expect 1-2 squeaks from a 6-tap burst, not 6. A burst she spoke into (witnessed
    by a non-squeak voice goal) is void and retried; >2 squeaks is a real failure always."""
    from sweetie_bot_joystick.msg import KeyPressed
    pub = rospy.Publisher(_touch_topic(), KeyPressed, queue_size=8)
    speech = _SpeechWatch()
    rospy.sleep(0.5)
    _say_with_retry(world, "Hello Sweetie!")
    try:
        for _ in range(3):
            assert _wait_quiet(speech), "she never went quiet before the burst"
            since = time.monotonic()
            for _ in range(6):      # a ~0.9 s burst of taps
                _tap(pub, "nose")
                rospy.sleep(0.12)
            rospy.sleep(6.0)        # settle: let reactions and the cooldown play out
            n = len(_squeaks(speech.goals, since))
            if n == 0:
                assert _non_squeak_goals(speech.goals, since), \
                    "burst in a verified-quiet window made no squeak at all (edges lost)"
                continue            # she spoke into the burst — void trial, retry
            assert 1 <= n <= 2, f"rapid tapping produced {n} squeaks (want 1-2)"
            return
        pytest.fail("burst voided by speech interference 3x in a row")
    finally:
        speech.close()
