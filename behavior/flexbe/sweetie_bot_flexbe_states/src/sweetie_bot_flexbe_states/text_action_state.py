#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from sweetie_bot_text_msgs.msg import TextCommand
from sweetie_bot_text_msgs.msg import TextActionAction, TextActionGoal

class TextActionState(EventState):
    '''
    Call action server for TextAction

    -- type         string          Type field of TextCommand.
    -- command      string          Command string.
    -- topic        string          Topic where to publish message.

    <= continue                     Action is running.
    <= done                         Action is finished. 
    <= command_error                Cannot send the action goal.
    '''

    def __init__(self, type, command, topic = 'voice/syn'):
        super(TextActionState, self).__init__(outcomes = ['done', 'command_error'])

        self._topic = topic

        # Check and form message
        if not isinstance(type, str):
            raise TypeError('TextActionState: "type" parameter must be string')
        if not isinstance(command, str):
            raise TypeError('TextActionState: "command" parameter must be string')
        self._message = TextCommand(type = type, command = command)

        # Form action goal
        self._action_goal = TextActionGoal(command = self._message)
        
        self._client = ProxyActionClient({ topic: TextActionAction })

        # Error in enter hook
        self._error = False


    def execute(self, userdata):

        if self._error:
            return 'command_error'

        # Check if actiong has been finished
        if self._client.has_result(self._topic):
            result = self._client.get_result(self._topic)

            return 'done'

    def on_enter(self, userdata):

        self._error = False
        try:
            self._client.send_goal(self._topic, self._action_goal)
        except Exception as e:
            Logger.logwarn('Failed to send TextAction goal:\n{0}'.format(str(e)))
            self._error = True
            return

        Logger.loginfo('Send TextActionGoal `{0}`: type = "{1}" cmd = "{2}".'.format(self._topic, self._message.type, self._message.command))

    def on_exit(self, userdata):
        # Check if action is still going, when on_exit() triggered manually by user

        if not self._client.has_result(self._topic):
            self._client.cancel(self._topic)
            Logger.loginfo('Canceled active TextAction goal.')
