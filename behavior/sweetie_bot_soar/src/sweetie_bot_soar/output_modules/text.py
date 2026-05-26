from .output_module import OutputModule, OutputModulesLoader

import rospy
from sweetie_bot_text_msgs.msg import TextCommand

class TextCmd(OutputModule):

    def _init(self, name, config):
        # get configuration
        cmd_topic = self.getConfigParameter(config, 'topic', allowed_types=str)
        # create publisher
        self._cmd_pub = rospy.Publisher(cmd_topic, TextCommand, queue_size=1)

    def startHook(self, cmd_id):
        # extract text command parameters
        type_id = cmd_id.FindByAttribute("type", 0)
        if not type_id: # or type_id.GetValueType() != "string":
            rospy.logerr("textcmd output module: type attribute is empty")
            return "failed"
        command_id = cmd_id.FindByAttribute("command", 0)
        if not command_id: # or command_id.GetValueType() != "string": 
            rospy.logerr("textcmd output module: command attribute is empty")
            return "failed"
        # publish message
        msg = TextCommand()
        msg.type = type_id.GetValueAsString()
        msg.command = command_id.GetValueAsString()
        self._cmd_pub.publish(msg)
        return "succeed"

OutputModulesLoader.register("textcmd", TextCmd)
