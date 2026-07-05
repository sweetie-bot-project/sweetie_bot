"""ROS1 node: the Sweetie Bot LLM agent, exposed as a GenerateReply action server.

Thin ROS glue around sweetie_bot_ai_core.Agent. Keeps the old CompleteRaw service (completion.py)
untouched and running in parallel - this node is additive and selected via the SOAR backend flag.

Concurrency: SimpleActionServer serialises goals (single-flight, matching one shared GPU). Preempt
is honoured so a new goal / cancel can abort an in-flight reply (barge-in groundwork). Generation
itself is blocking (requests-based client); a future async upgrade can add token-streaming feedback.

Proactive self-talk (LLM-side, bypasses SOAR): a background timer emits spontaneous asides in two
situations - nobody visible (she muses to herself), or a human present but silent for ~>=1 turn
(sometimes, probabilistically). Guarded by the same lock as real replies (one GPU) and voiced here
(SOAR is not in the loop) on the robot default language. See proactive.py for the pure decision.
"""
from __future__ import annotations

import os
import random
import threading

import rospy
import actionlib
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, TriggerResponse

from sweetie_bot_ai_core import (Agent, LanguagePolicy, PersonaRegistry, SceneConfig, ToolRegistry,
                                 build_llm_registry)
from sweetie_bot_ai_core.translation import LibreTranslateProvider
from sweetie_bot_ai_core.schema import AgentRequest, RequestType

from sweetie_bot_text_msgs.msg import (GenerateReplyAction, GenerateReplyFeedback,
                                       GenerateReplyResult, TextActionAction, TextActionGoal)

from .agent_bridge import goal_to_request, fill_result
from .state_collector import StateCollector
from .scene_collector import SceneCollector
from .tool_adapters import ToolAdapters
from .proactive import ProactiveConfig, choose_proactive_cue

# entity types that count as a present human (ponies are pony_*, other objects are their label)
_HUMAN_TYPES = {"person", "human", "face", "body"}


class LLMAgentNode:
    def __init__(self):
        rospy.init_node("llm_agent")

        # --- config (rosparam with safe defaults) -------------------------------------------
        providers_cfg = rospy.get_param("~llm", {
            "providers": {"local": {"url": "http://localhost:11434/v1",
                                    "model": "qwen2.5:14b", "priority": 10}}})
        default_options = rospy.get_param("~default_options", {"temperature": 0.8})
        tools_cfg = rospy.get_param("~tools", None)
        persona_dir = rospy.get_param("~persona_dir", "")
        default_persona = rospy.get_param("~default_persona", "sweetie")
        lp_cfg = rospy.get_param("~language_policy", {})
        scene_cfg = rospy.get_param("~scene", {}) or {}

        # --- build core ----------------------------------------------------------------------
        registry = build_llm_registry(providers_cfg, logger=rospy.logwarn,
                                      default_options=default_options)
        personas = (PersonaRegistry.from_dir(persona_dir, default_name=default_persona)
                    if persona_dir and os.path.isdir(persona_dir)
                    else PersonaRegistry(default_name=default_persona))
        tools = ToolRegistry.from_config(tools_cfg) if tools_cfg else ToolRegistry()

        translate = None
        if lp_cfg.get("translate_url"):
            translate = LibreTranslateProvider(lp_cfg["translate_url"])
        policy = LanguagePolicy(native_languages=lp_cfg.get("native_languages", ["en"]),
                                pivot=lp_cfg.get("pivot", "en"), provider=translate)

        self._state = StateCollector(
            ignored_servos=set(rospy.get_param("/disabled_servos", {}).keys()))
        # scene provider (environmental awareness); tolerate a missing vision stack gracefully
        scene_kwargs = {k: scene_cfg[k] for k in (
            "detections_topic", "sound_topic", "forward_frame", "stable_frame", "front_deg",
            "side_deg", "retention_ttl_s", "near_m", "mid_m", "bearing_sign",
            "sound_bearing_sign", "sound_bearing_offset_deg", "min_score",
                        "exclude_types") if k in scene_cfg}
        try:
            self._scene = SceneCollector(**scene_kwargs)
        except Exception as e:  # noqa: BLE001 - never block the node on scene setup
            rospy.logwarn("llm_agent: scene provider unavailable (%r); running without it", e)
            self._scene = None
        self._effector = ToolAdapters(self._state, scene_collector=self._scene)
        self._agent = Agent(registry, personas=personas, tools=tools,
                            state_provider=self._state, scene_provider=self._scene,
                            effector=self._effector, language_policy=policy,
                            scene_config=SceneConfig(front_deg=scene_cfg.get("front_deg", 60.0),
                                                     side_deg=scene_cfg.get("side_deg", 120.0)),
                            logger=rospy.loginfo)

        # --- persona switching (clean runtime switch) ---------------------------------------
        rospy.Subscriber("~set_persona", String, self._on_set_persona, queue_size=1)
        # gate proactive self-talk on SOAR operational state (latched Bool from soar_node)
        rospy.Subscriber("/soar/operational", Bool, self._on_operational, queue_size=1)
        # reset seam for the behavior-synth harness: clears everything that persists between goals
        rospy.Service("~reset", Trigger, self._on_reset)

        # --- action server -------------------------------------------------------------------
        self._server = actionlib.SimpleActionServer(
            "generate_reply", GenerateReplyAction, execute_cb=self._execute, auto_start=False)
        self._server.start()

        # --- proactive self-talk driver (autonomous; bypasses SOAR) -------------------------
        self._agent_lock = threading.Lock()     # serialise agent.handle across action + timer
        self._operational = True   # SOAR operational; proactive self-talk is gated OFF when False
        self._last_activity = rospy.get_time()   # time of the last real conversational turn
        self._last_selftalk = 0.0                # time of the last proactive aside
        self._pro = self._load_proactive_cfg()
        self._voice = actionlib.SimpleActionClient(
            rospy.get_param("~proactive/voice_ns", "voice/syn"), TextActionAction)
        self._pro_seq = 0
        if self._pro.enabled:
            rospy.Timer(rospy.Duration(self._pro.period), self._proactive_tick)
            rospy.loginfo("llm_agent: proactive self-talk driver ON "
                          "(period=%.1fs alone_after=%.0fs lull_after=%.0fs lull_prob=%.2f)",
                          self._pro.period, self._pro.alone_after, self._pro.lull_after,
                          self._pro.lull_prob)

        rospy.loginfo("llm_agent: ready (personas=%s, providers=%s)",
                      personas.names(), list(registry.health().keys()))

    # -- proactive helpers ------------------------------------------------------------------------

    def _load_proactive_cfg(self) -> ProactiveConfig:
        """Build ProactiveConfig from ~proactive rosparams, falling back to dataclass defaults."""
        d = ProactiveConfig()
        p = rospy.get_param("~proactive", {}) or {}
        for f in ("enabled", "period", "min_gap", "alone_after", "alone_gap", "lull_after",
                  "lull_prob", "profile", "persona", "cue_alone", "cue_lull"):
            if f in p:
                setattr(d, f, p[f])
        return d

    def _robot_lang(self) -> str:
        """Robot default spoken language = first prefix of /voice/lang (fallback /hmi/lang, en)."""
        for key in ("/voice/lang", "/hmi/lang"):
            v = rospy.get_param(key, "")
            if v:
                return str(v).split(",")[0].strip()
        return "en"

    def _humans_present(self) -> bool:
        if self._scene is None:
            return False
        try:
            snap = self._scene.snapshot(include_remembered=False)
        except TypeError:
            snap = self._scene.snapshot()
        except Exception:  # noqa: BLE001 - a scene hiccup must not crash the driver
            return False
        return any((e.type or "") in _HUMAN_TYPES for e in snap.entities)

    def _proactive_tick(self, _evt):
        # live-re-read the tunables so they can be adjusted with rosparam set without a restart
        self._pro = self._load_proactive_cfg()
        if not self._pro.enabled or not self._operational or self._server.is_active():
            return
        # a real turn holds the lock; if we cannot take it, stay out of the way this tick
        if not self._agent_lock.acquire(blocking=False):
            return
        try:
            now = rospy.get_time()
            cue = choose_proactive_cue(self._humans_present(), now - self._last_activity,
                                       now - self._last_selftalk, self._pro, random.random())
            if cue is not None:
                self._fire_self_talk(cue)
        finally:
            self._agent_lock.release()

    def _fire_self_talk(self, cue: str):
        """Generate one spontaneous aside and voice it. Caller must hold self._agent_lock."""
        lang = self._robot_lang()
        self._pro_seq += 1
        rospy.loginfo("llm_agent: [proactive #%d] self_talk lang=%s cue=%r",
                      self._pro_seq, lang, cue)
        req = AgentRequest(request_type=RequestType.self_talk, profile=self._pro.profile,
                           text=cue, persona=(self._pro.persona or None),
                           text_language=lang, reply_language=lang)
        try:
            reply = self._agent.handle(req)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn("llm_agent: proactive self_talk failed: %r", e)
            return
        self._last_selftalk = rospy.get_time()
        txt = (reply.response_text or "").strip()
        if reply.error_code.value != 0 or not txt:
            rospy.loginfo("llm_agent: [proactive] silence (err=%s empty=%s)",
                          reply.error_code.name, not txt)
            return
        # staleness guard: a real turn may have arrived (and be blocked on the lock) while
        # we were generating - drop this now-stale aside instead of voicing it over the
        # conversation the human just resumed.
        if self._server.is_active():
            rospy.loginfo("llm_agent: [proactive] dropped (real turn arrived) %r", txt)
            return
        emo = getattr(reply.emotion, "value", reply.emotion)
        rospy.loginfo("llm_agent: [proactive] say [%s] %r", emo, txt)
        self._voice_say(txt, lang)

    def _voice_say(self, text: str, lang: str):
        if not self._voice.wait_for_server(rospy.Duration(2.0)):
            rospy.logwarn_throttle(30.0, "llm_agent: voice/syn unavailable for proactive say")
            return
        g = TextActionGoal()
        g.command.type = "voice/say/%s" % lang
        g.command.command = text
        self._voice.send_goal(g)   # fire-and-forget; do not block the driver on TTS

    # -- callbacks --------------------------------------------------------------------------------

    def _on_reset(self, _req):
        try:
            self._agent.reset_ambient()
            if self._scene is not None:
                self._scene.reset()
            self._agent.registry.reset_breakers()
            self._agent.personas.reset_active()
            rospy.loginfo("llm_agent: state reset (ambient scene, retention, breakers, persona)")
            return TriggerResponse(success=True, message="reset")
        except Exception as e:  # noqa: BLE001
            rospy.logerr("llm_agent: reset failed: %r", e)
            return TriggerResponse(success=False, message=repr(e))

    def _on_operational(self, msg: Bool):
        self._operational = bool(msg.data)
        rospy.loginfo("llm_agent: operational=%s (proactive self-talk %s)",
                      self._operational, "enabled" if self._operational else "gated OFF")

    def _on_set_persona(self, msg: String):
        if self._agent.personas.set_active(msg.data.strip()):
            rospy.loginfo("llm_agent: active persona -> %s", msg.data.strip())
        else:
            rospy.logwarn("llm_agent: unknown persona %s", msg.data)

    def _execute(self, goal):
        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return
        request = goal_to_request(goal)
        rospy.loginfo("llm_agent: request profile=%s lang=%s text=%r",
                      request.profile, request.text_language, request.text[:80])
        with self._agent_lock:
            reply = self._agent.handle(request)
        if request.request_type != RequestType.self_talk:
            self._last_activity = rospy.get_time()   # a real turn happened; reset the lull clock

        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

        # deliberate conversational pause: give the human a beat to breathe and think before
        # she answers (tunable live via ~reply_delay; only for real spoken replies).
        if request.request_type == RequestType.reply and reply.error_code.value == 0:
            delay = float(rospy.get_param("~reply_delay", 1.5))
            if delay > 0:
                rospy.sleep(delay)

        result = GenerateReplyResult()
        fill_result(result, reply)
        if reply.error_code.value == 0:
            rospy.loginfo("llm_agent: reply [%s] %r", result.emotion, result.response_text[:600])
            self._server.set_succeeded(result)
        else:
            rospy.logwarn("llm_agent: error %s: %s", reply.error_code.name, reply.error_desc)
            self._server.set_succeeded(result)   # report error via result.error_code, not abort


def main():
    LLMAgentNode()
    rospy.spin()


if __name__ == "__main__":
    main()
