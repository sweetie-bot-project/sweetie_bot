import random
from collections import deque

from .output_module import OutputModulesLoader, OutputModule

import rospy
import actionlib
from  actionlib import GoalStatus

from sweetie_bot_text_msgs.msg import (TextActionAction, TextActionGoal, TextActionResult,
                                       GenerateReplyAction, GenerateReplyGoal,
                                       GenerateReplyActionResult)

class TextAction(OutputModule):

    def _init(self, name, config):
        # get configuration
        action_ns = self.getConfigParameter(config, 'action_ns', allowed_types=str)
        # create actionlib client
        self._action_client = actionlib.SimpleActionClient(action_ns, TextActionAction)
        # LLM rephrase interceptor: generic dilution of canned speech (scripted rule phrases are
        # occasionally rephrased in character by the agent) - NO rule changes, covers all say
        # paths. LLM-authored texts (recent generate_reply results) always pass verbatim.
        reph = self.getConfigParameter(config, 'llm_rephrase', default_value={}, allowed_types=dict)
        self._reph_p = float(reph.get('probability', 0.0))
        self._reph_timeout = float(reph.get('timeout', 3.0))
        self._reph_profile = str(reph.get('profile', 'rephrase-en'))
        self._reph_client = None
        self._recent_replies = deque(maxlen=8)
        if self._reph_p > 0.0:
            reph_ns = str(reph.get('action_ns', 'generate_reply'))
            self._reph_client = actionlib.SimpleActionClient(reph_ns, GenerateReplyAction)
            self.createSubscriber(reph_ns + '/result', GenerateReplyActionResult,
                                  self._on_agent_reply, queue_size=5)
        self._phase = 'say'
        self._pending_goal = None
        self._reph_deadline = None

    def _on_agent_reply(self, msg):
        text = (msg.result.response_text or '').strip()
        if text:
            self._recent_replies.append(text)

    def _should_rephrase(self, goal):
        if self._reph_client is None or self._reph_p <= 0.0:
            return False
        if not goal.command.type.startswith('voice/say'):
            return False
        text = (goal.command.command or '').strip()
        if not text or text in self._recent_replies:
            return False
        if random.random() >= self._reph_p:
            return False
        # a not-yet-connected agent action server must not delay speech: skip, speak canned
        if not self._reph_client.wait_for_server(rospy.Duration(0.2)):
            rospy.logwarn_throttle(30.0, "TextAction: rephrase agent server not connected - canned")
            return False
        return True

    def _send_voice(self, goal):
        rospy.loginfo("TextAction output module: executing (%s, %s).", goal.command.type, goal.command.command)
        self._action_client.send_goal(goal)
        self._phase = 'say'

    def startHook(self, cmd_id):
        goal = TextActionGoal()
        # process WME
        # extract command 
        name_id = cmd_id.FindByAttribute("command", 0);
        type_id = cmd_id.FindByAttribute("type", 0);
        if name_id is None or type_id is None:
            rospy.logerr("TextAction output module: type and command attributes must present.")
            return "error"
        goal.command.type = type_id.GetValueAsString()
        goal.command.command = name_id.GetValueAsString()
        if self._should_rephrase(goal):
            # async LLM hop; the voice goal is sent from updateHook once the rephrase lands
            # (verbatim fallback on timeout/error) - never blocks the SOAR decision cycle
            self._pending_goal = goal
            self._phase = 'rephrase'
            self._reph_deadline = rospy.Time.now() + rospy.Duration(self._reph_timeout)
            # dedicated 'rephrase' request_type: the agent runs a constrained rewording path
            # (no scene/ambient/conversation latitude) so the line is restated, not answered.
            self._reph_client.send_goal(GenerateReplyGoal(
                request_type='rephrase', profile=self._reph_profile, text=goal.command.command,
                history_json='[]', context_json='[]', text_language='en', reply_language='en'))
            rospy.loginfo("TextAction output module: rephrasing canned line %r", goal.command.command)
            return None
        # send goal to server
        self._send_voice(goal)
        return None

    def updateHook(self, cmd_id, external_abort_request):
        # rephrase hop in flight: resolve it, then hand the (possibly rewritten) goal to voice
        if self._phase == 'rephrase':
            aborted = cmd_id is None or external_abort_request is not None or \
                (cmd_id is not None and cmd_id.FindByAttribute("abort", 0) is not None)
            if aborted:
                self._reph_client.cancel_goal()
                self._phase = 'say'
                return "aborted"
            st = self._reph_client.get_state()
            if st == GoalStatus.SUCCEEDED:
                res = self._reph_client.get_result()
                text = (res.response_text or '').strip() if res is not None and res.error_code == 0 else ''
                if text:
                    self._pending_goal.command.command = text
                self._send_voice(self._pending_goal)
                return None
            if st in (GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.LOST) \
                    or rospy.Time.now() > self._reph_deadline:
                self._reph_client.cancel_goal()
                rospy.logwarn("TextAction output module: rephrase timed out/failed - canned text used")
                self._send_voice(self._pending_goal)
                return None
            return None

        # get goal state
        status = self._action_client.get_state()
        
        # Goal is active. 
        if status in (GoalStatus.ACTIVE, GoalStatus.PENDING):
            # Check for agent abort request.
            agent_abort_request = False
            if cmd_id is None:
                agent_abort_request = True
            else:
                abort_id = cmd_id.FindByAttribute("abort", 0)
                if abort_id is not None:
                    agent_abort_request = True
            if agent_abort_request:
                self._action_client.cancel_goal()
                return "aborted"

            # Check for external abort request
            if external_abort_request is not None:
                self._action_client.cancel_goal()
                return "aborted"

            # continue execution (ACTIVE, PENDING)
            return None

        # Goal is completed.
        if status == GoalStatus.SUCCEEDED:
            return 'succeed'
        if status in (GoalStatus.RECALLED, GoalStatus.PREEMPTED):
            rospy.loginfo("TextAction output module:  behavior execution was preempted.")
            return "aborted"
        if status in (GoalStatus.REJECTED, GoalStatus.ABORTED, GoalStatus.LOST):
            rospy.loginfo("TextAction output module:  behavior execution has failed with error.")
            return "failed"

        # continue execution (RECALLING, PREEMPTING)
        return None

OutputModulesLoader.register("text-action", TextAction)


