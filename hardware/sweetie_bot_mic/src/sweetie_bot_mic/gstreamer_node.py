#!/usr/bin/env python3

import sys
import string
import rospy
from .gstreamer import GstreamerPipeline

class GstreamerNode(GstreamerPipeline):

    def __init__(self):
        # Node
        rospy.init_node('gstreamer')
        # get parameters
        if rospy.get_param('~warinings_as_errors', False):
            on_warning = self.on_error
        else:
            on_warning = None
        pipeline_template = rospy.get_param('~gstreamer_pipeline')
        if not isinstance(pipeline_template, str):
            raise TypeError("'gstreamer_pipeline' parameter must present and contain string.")
        # gstreamer template substitution
        pipeline_string = ''
        formatter = string.Formatter()
        for fmt in formatter.parse(pipeline_template):
            literal_text, field_name, _, _ = fmt
            pipeline_string += literal_text
            if field_name is None:
                break
            if field_name == '':
                raise ValueError("substitute groups ({}) in 'gstreamer_pipeline' parameter must be named.")
            value = rospy.get_param('~' + field_name)
            if value is None:
                raise ValueError(f"unable to substitute group '{field_name}' in 'gstreamer_pipeline': ROS parameter '~{field_name}' is not declared.")
            pipeline_string += value
        # init gstreamer pipeline
        super(GstreamerNode, self).__init__(pipeline_string, on_error = self.on_error, on_warning = on_warning)
        # start streaming
        self.start()

    def on_error(self, msg):
        rospy.signal_shutdown('gstreamer error') 

    def finalize(self):
        # stop gstreamer
        self.close()

def main():
    try:
        node = GstreamerNode()
    except Exception as e:
        rospy.logerr(str(e))
        raise
    rospy.spin()
    node.finalize()
         
if __name__ == '__main__':
    main()



