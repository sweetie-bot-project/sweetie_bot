#!/usr/bin/env python3

import sys
import weakref
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
        pipeline_string = rospy.get_param('~gstreamer_pipeline')
        if not isinstance(pipeline_string, str):
            raise TypeError("'gstreamer_pipeline' parameter must present and contain string.")
        # init gstreamer pipeline
        super(GstreamerNode, self).__init__(pipeline_string, on_error = self.on_error, on_warning = on_warning)
        # start streaming
        self.start()

    def on_error(self, msg):
        rospy.signal_shutdown('gstreamer error') 

    def finalize(self):
        # stop gstreamer
        self.gstreamer_audio = None

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



