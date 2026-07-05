"""World — the synth environment injected into each behavior test.

Built by the conftest fixture (or runner.py) from the test's ScenarioSpec: starts the synth
streams, opens the SOAR operational window, exposes timeline control + collectors + turn
helpers, and resets everything afterwards.
"""
from __future__ import annotations

import time
from typing import Optional

import rospy

from .collectors import SayRecord, TurnRecord, make_collectors
from .dsl import ScenarioSpec, SynthEntity
from .resets import agent_reset, set_operational, soar_reconfigure
from .streams import SynthDetections, SynthSpeech


class World:
    def __init__(self, spec: ScenarioSpec):
        self.spec = spec
        self.detections = SynthDetections(rate_hz=spec.detections_rate)
        self.speech = SynthSpeech()
        self.col = make_collectors()
        self._t0 = 0.0
        self._undo_params = lambda: None
        self._truth = {e.id: e for e in spec.entities}

    # -- lifecycle ---------------------------------------------------------------------------------
    def start(self):
        # ALWAYS reconfigure SOAR: kills cross-test belief leakage (waiting-answer predicates,
        # running processes, SWM memory). ~2-4 s per test - the price of determinism.
        ok, self._undo_params = soar_reconfigure(self.spec.soar_params or {})
        assert ok, "soar reconfigure failed"
        agent_reset()
        self.detections.start(self.spec.entities)
        rospy.sleep(1.0)                       # let SWM ingest the initial scene
        self.col["soar_log"].anchor()
        self.col["agent_log"].anchor()
        self.col["turns"].anchor()
        self.col["say"].anchor()
        if self.spec.operational:
            set_operational(True)
        self._t0 = time.monotonic()
        # brief settle so SWM/attention binds the human before the first utterance - but SHORT:
        # idling under operational lets the pause/no-answer machinery spin up and the first
        # say then queues behind a busy talk pipeline (harness-found systematic degradation)
        if any(e.type == "human" for e in self.spec.entities):
            rospy.sleep(2.5)
        return self

    def stop(self):
        try:
            if self.spec.operational:
                set_operational(False)
        finally:
            self.detections.stop(flush_empty_s=1.0)
            agent_reset()
            if self.spec.soar_params:
                # restore overridden params now; the next start() reconfigures anyway
                self._undo_params()

    def override_soar_params(self, **params):
        """Mid-test /soar param override: reconfigure (respawn) + reopen the operational window
        + re-anchor scrapers. Returns an undo callable (params only; window stays open)."""
        from .resets import soar_reconfigure as _rc, set_operational as _so
        ok, undo = _rc(params)
        assert ok, "soar reset with overrides failed"
        if self.spec.operational:
            _so(True)
        self.col["soar_log"].anchor()
        rospy.sleep(2.0)
        return undo

    def override_soar_params(self, **params):
        """Mid-test /soar param override: reconfigure (respawn) + reopen the operational window
        + re-anchor scrapers. Returns an undo callable (params only; window stays open)."""
        from .resets import soar_reconfigure as _rc, set_operational as _so
        ok, undo = _rc(params)
        assert ok, "soar reset with overrides failed"
        if self.spec.operational:
            _so(True)
        self.col["soar_log"].anchor()
        rospy.sleep(2.0)
        return undo

    # -- ground truth --------------------------------------------------------------------------------
    def truth(self, id: int) -> SynthEntity:
        return self._truth[id]

    # -- timeline ------------------------------------------------------------------------------------
    def spawn(self, *entities: SynthEntity):
        for e in entities:
            self._truth[e.id] = e
        self.detections.spawn(*entities)

    def vanish(self, *ids: int):
        self.detections.vanish(*ids)

    def move(self, id: int, **kw):
        self.detections.move(id, **kw)
        e = self._truth.get(id)
        if e is not None:
            for k, v in kw.items():
                setattr(e, k, v)

    # -- waits / turns ---------------------------------------------------------------------------------
    def wait_seen(self, type_: str, timeout: float = 8.0) -> bool:
        """Wait until the soar log registers an SWM object of this type (REGISTER ... <type>)
        or, cheaper, until detections stream contains it (it always does) + a grace period."""
        hits = self.col["soar_log"].wait_grep(rf"(?:REGISTER|look-at cmd on).*{type_}", timeout)
        return bool(hits)

    def say_and_get_turn(self, text: str, lang: Optional[str] = None,
                         timeout: float = 35.0) -> TurnRecord:
        """Input speech seam -> wait for the LLM turn only (text + emotion from the agent log),
        with NO voice/TTS dependency. Use this when the assertion is on the model's reply itself
        (the agreed seam after the LLM), not the voiced output — it skips the TTS pipeline
        entirely (and does not depend on SOAR's emotion->animation mapping)."""
        lang = lang or self.spec.lang
        self.speech.say(text, lang=lang)
        turn = self.col["turns"].wait_turn_for(text, timeout=timeout)
        assert turn is not None, f"no LLM turn within {timeout}s after saying {text!r}"
        return turn

    def say_and_wait(self, text: str, lang: Optional[str] = None,
                     timeout: float = 35.0) -> TurnRecord:
        """Speak (text seam) and wait for the full turn (agent-log scrape) + the voiced say
        (voice-log scrape). Log scraping is reception-independent (topics proved flaky in the
        test process around soar respawns)."""
        lang = lang or self.spec.lang
        n_says = len(self.col["say"].says())
        self.speech.say(text, lang=lang)
        turn = self.col["turns"].wait_turn_for(text, timeout=timeout)
        assert turn is not None, f"no LLM turn within {timeout}s after saying {text!r}"
        # pair the voiced say by ORDER (first new say after the turn started): the voice
        # pipeline TRANSFORMS text (P25 translation round-trip), so content matching is
        # impossible - language/presence stay assertable and P25 surfaces honestly
        import time as _time
        deadline = _time.monotonic() + 12.0
        turn.said = None
        while _time.monotonic() < deadline and turn.said is None:
            new_says = self.col["say"].says()[n_says:]
            if new_says:
                turn.said = new_says[0]
            else:
                _time.sleep(0.2)
        return turn

    def expect_quiet(self, seconds: float = 8.0, since: Optional[float] = None) -> None:
        """Assert the voice receives nothing to say for `seconds` (silence behaviors)."""
        n_before = len(self.col["say"].says())
        rospy.sleep(seconds)
        says = self.col["say"].says()[n_before:]
        assert not says, f"expected silence but she said: {[s.text for s in says]}"

    def gaze_ref_count(self, since: Optional[float] = None) -> int:
        return self.col["gaze_refs"].count(self._t0 if since is None else since)

    def scene_block_seen(self, pattern: str, timeout: float = 5.0) -> bool:
        """Check the agent log for scene-block content markers (needs agent-side debug logging;
        falls back to reply-based checks when absent)."""
        return bool(self.col["agent_log"].wait_grep(pattern, timeout))
