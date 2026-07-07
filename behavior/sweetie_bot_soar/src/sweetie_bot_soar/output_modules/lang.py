"""SOAR 'lang-model' output module.

Thin adapter between SOAR and the LLM agent. The heavy LLM logic that used to live here (prompt
building, regex/keyword/BERT parsing, multi-call attribute extraction) now lives in the
``sweetie_bot_ai_core`` agent behind the GenerateReply action; this module only translates the
SOAR command into a goal and writes the contract WMEs (status/result/emotion/sentence-type) back.

The SOAR production rules are UNCHANGED: they emit ``(^request <name> ^event ... ^predicate ...
^text ...)`` and wait for ``^status succeed ^result ^emotion ^sentence-type`` — exactly what this
module produces, for request names simple-en/failsafe-en/complex-en.

Two backends, selected by the ``backend`` config flag (parallel-run / rollback):
  * 'agent'  -> AgentLangModel: non-blocking GenerateReply action client (new structured path).
  * 'legacy' -> LegacyLangModel: the original CompleteRaw + regex/BERT path (preserved verbatim
                in lang_legacy.py). Default, so an unconfigured deploy keeps today's behaviour.
"""
import json

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from .output_module import OutputModule, OutputModulesLoader
from .lang_legacy import LegacyLangModel
from .wme_parsing import TalkEvent, Predicate, WMEParseError
from .history_contract import build_history, drop_duplicate_tail

from sweetie_bot_text_msgs.msg import GenerateReplyAction, GenerateReplyGoal


class AgentLangModel(OutputModule):
    """New LLM backend: non-blocking GenerateReply action client."""

    def _init(self, name, config):
        action_ns = self.getConfigParameter(config, "action_ns",
                                             default_value="generate_reply", allowed_types=str)
        self._requests = self.getConfigParameter(config, "requests",
                                                 default_value={}, allowed_types=dict)
        self._default_persona = config.get("persona") or ""
        self._default_language = config.get("text_language", "en")
        self._timeout = float(config.get("timeout", 30.0))
        self._client = actionlib.SimpleActionClient(action_ns, GenerateReplyAction)
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            rospy.logwarn("lang-model(agent): action server '%s' not available yet", action_ns)
        self._deadline = None

    # -- helpers ------------------------------------------------------------------------------

    @staticmethod
    def _build_history(events, max_events=None):
        # thin call: the serialization contract lives in history_contract.py (stdlib-only,
        # pinned from BOTH pythons — see that module's docstring)
        return build_history(events, max_events=max_events)

    # -- OutputModule hooks -------------------------------------------------------------------

    def startHook(self, cmd_id):
        request_name = None
        events, predicates = [], []
        text = None
        for idx in range(cmd_id.GetNumberChildren()):
            item_id = cmd_id.GetChild(idx)
            attr = item_id.GetAttribute()
            if item_id.IsIdentifier():
                try:
                    if attr == 'event':
                        events.append(TalkEvent(item_id.ConvertToIdentifier()))
                    elif attr == 'predicate':
                        predicates.append(Predicate(item_id.ConvertToIdentifier()))
                except WMEParseError as e:
                    # bookkeeping events (talk-waiting-answer etc.) legitimately have no text
                    rospy.logdebug("lang-model(agent): ^%s parse skip: %s", attr, e)
            if attr == 'request':
                request_name = item_id.GetValueAsString()
            elif attr == 'text':
                text = item_id.GetValueAsString()

        if request_name is None:
            rospy.logerr("lang-model(agent): no ^request supplied")
            return "error"

        req_cfg = self._requests.get(request_name, {}) if isinstance(self._requests, dict) else {}
        if not isinstance(req_cfg, dict):
            req_cfg = {}
        profile = req_cfg.get("profile", request_name)        # default profile == request name
        persona = req_cfg.get("persona", self._default_persona)
        language = req_cfg.get("text_language", self._default_language)

        history = self._build_history(events)
        if text is None:
            for ev in sorted(events, key=lambda e: e.stamp, reverse=True):
                if ev.type == 'talk-heard':
                    text = ev.text
                    break
        # avoid duplicating the current utterance both as history tail and as `text`
        history = drop_duplicate_tail(history, text)

        # Silence turns (talk-no-answer / ignored / illegible) are handled by SOAR's OWN talk
        # rules (pause/ignored follow-ups are deliberate behaviors) - the agent answers whatever
        # SOAR asks it to verbalize. Monologue cannot chain: add-talk-ignored re-arms only on a
        # QUESTION reply, and the persona ends non-questions with statements. (An adapter-level
        # silence guard was tried and disproven by the behavior-synth harness: error-returns
        # churned at ms rate and starved real turns; empty results wedged the say pipeline.)
        goal = GenerateReplyGoal()
        goal.request_type = "reply"
        goal.profile = profile
        goal.text = text or ""
        goal.history_json = json.dumps(history)
        context = [p.text for p in predicates]
        if any(ev.type in ('talk-ignored', 'talk-no-answer') for ev in events):
            # silence framed as the interlocutor's current state (an attribute of the human),
            # not as something the human said - the weaving directive keeps her from reciting it
            context.append("The person you are talking with has gone quiet and is not answering right now.")
        goal.context_json = json.dumps(context)
        goal.text_language = language
        goal.reply_language = language
        goal.persona = persona

        self._client.send_goal(goal)
        self._deadline = rospy.Time.now() + rospy.Duration(self._timeout)
        rospy.logdebug("lang-model(agent): sent goal profile=%s text=%r", profile, text)
        return None   # keep running; poll in updateHook (non-blocking)

    def updateHook(self, cmd_id, request_abort=False):
        # new module framework: cmd_id is None if the command WME vanished from the output link
        if cmd_id is None or request_abort:
            self._client.cancel_goal()
            rospy.loginfo("lang-model(agent): aborted (%s)",
                          "command removed" if cmd_id is None else "by SOAR")
            return "error"

        state = self._client.get_state()
        if state in (GoalStatus.PENDING, GoalStatus.ACTIVE):
            if self._deadline is not None and rospy.Time.now() > self._deadline:
                self._client.cancel_goal()
                rospy.logerr("lang-model(agent): timed out")
                return "error"
            return None

        if state != GoalStatus.SUCCEEDED:
            rospy.logerr("lang-model(agent): action terminal state %s", state)
            return "error"

        result = self._client.get_result()
        if result is None or result.error_code != 0:
            desc = getattr(result, 'error_desc', 'unknown') if result is not None else 'no result'
            rospy.logerr("lang-model(agent): agent error: %s", desc)
            return "error"

        # contract WMEs consumed by the (unchanged) SOAR rules
        cmd_id.CreateStringWME('result', result.response_text)
        cmd_id.CreateStringWME('emotion', result.emotion)
        cmd_id.CreateStringWME('sentence-type', result.sentence_type)
        rospy.loginfo("lang-model(agent): succeed [%s] %r",
                      result.emotion, result.response_text[:80])
        return "succeed"


class LangModel(object):
    """Dispatcher registered as 'lang-model'; picks a backend by config (default 'legacy').

    The new ModulesLoader instantiates ``module_cls(module_name, module_config)``; ``__new__``
    returns a fully-constructed backend instance (not a LangModel), so no further init happens.
    """

    def __new__(cls, name, config):
        backend = (config or {}).get("backend", "legacy")
        if backend == "agent":
            rospy.loginfo("lang-model: using AGENT backend")
            return AgentLangModel(name, config)
        rospy.loginfo("lang-model: using LEGACY backend")
        return LegacyLangModel(name, config)


OutputModulesLoader.register("lang-model", LangModel)
