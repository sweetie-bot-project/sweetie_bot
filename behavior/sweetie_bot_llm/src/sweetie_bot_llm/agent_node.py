"""ROS1 node: the Sweetie Bot LLM agent, exposed as a GenerateReply action server.

Thin ROS glue around sweetie_bot_ai_core.Agent. Keeps the old CompleteRaw service (completion.py)
untouched and running in parallel — this node is additive and selected via the SOAR backend flag.

Concurrency: SimpleActionServer serialises goals (single-flight, matching one shared GPU). Preempt
is honoured so a new goal / cancel can abort an in-flight reply (barge-in groundwork). Generation
itself is blocking (requests-based client); a future async upgrade can add token-streaming feedback.
"""
from __future__ import annotations

import os

import rospy
import actionlib
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from sweetie_bot_ai_core import (Agent, LanguagePolicy, PersonaRegistry, SceneConfig, ToolRegistry,
                                 build_llm_registry)
from sweetie_bot_ai_core.translation import LibreTranslateProvider

from sweetie_bot_text_msgs.msg import (GenerateReplyAction, GenerateReplyFeedback,
                                       GenerateReplyResult)

from .agent_bridge import goal_to_request, fill_result
from .state_collector import StateCollector
from .scene_collector import SceneCollector
from .tool_adapters import ToolAdapters


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

        self._state = StateCollector()
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
        # reset seam for the behavior-synth harness: clears everything that persists between goals
        rospy.Service("~reset", Trigger, self._on_reset)

        # --- action server -------------------------------------------------------------------
        self._server = actionlib.SimpleActionServer(
            "generate_reply", GenerateReplyAction, execute_cb=self._execute, auto_start=False)
        self._server.start()
        rospy.loginfo("llm_agent: ready (personas=%s, providers=%s)",
                      personas.names(), list(registry.health().keys()))

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

    def _on_set_persona(self, msg: String):
        if self._agent.personas.set_active(msg.data.strip()):
            rospy.loginfo("llm_agent: active persona -> %s", msg.data.strip())
        else:
            rospy.logwarn("llm_agent: unknown persona '%s'", msg.data)

    def _execute(self, goal):
        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return
        request = goal_to_request(goal)
        rospy.loginfo("llm_agent: request profile=%s lang=%s text=%r",
                      request.profile, request.text_language, request.text[:80])
        reply = self._agent.handle(request)

        if self._server.is_preempt_requested():
            self._server.set_preempted()
            return

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
