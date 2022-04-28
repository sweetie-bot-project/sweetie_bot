from . import output_module

import rospy
from sweetie_bot_text_msgs.msg import SpeechVisualization as SpeechVisualizationMsg

class SpeechVisualization(output_module.OutputModule):

    def __init__(self, config):
        super(SpeechVisualization, self).__init__("speech-visualization")
        # module initialization
        cmd_topic = config.get("topic")
        if not cmd_topic:
            raise RuntimeError("SpeechVisualization output module: 'topic' parameter is not defined.")
        # create publisher
        self._cmd_pub = rospy.Publisher(cmd_topic, SpeechVisualizationMsg, queue_size=1)

    def startHook(self, cmd_id):
        msg = SpeechVisualizationMsg()
        # extract attributes
        for attr in ('text', 'character', 'scene'):
            attr_id = cmd_id.FindByAttribute(attr, 0)
            if attr_id:
                setattr(msg, attr, attr_id.GetValueAsString())
        # copy tags
        while True:
            attr_id = cmd_id.FindByAttribute('tag', len(msg.tags))
            if not attr_id:
                break
            msg.tags.append(attr_id.GetValueAsString())
        # publish message
        self._cmd_pub.publish(msg)
        return "succeed"

output_module.register("speech-visualization", SpeechVisualization)
