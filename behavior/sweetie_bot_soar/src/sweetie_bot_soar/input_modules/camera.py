from copy import copy
import input_module
import rospy
import tf

from cob_object_detection_msgs.msg import DetectionArray as DetectionArrayMsg, Detection as DetectionMsg

class Camera:
    def __init__(self, agent, config):
        input_link_id = agent.GetInputLink()
        # add sensor element  
        self._sensor_id = input_link_id.CreateIdWME("camera")
        # configuration
        detection_topic = config.get("topic")
        if not detection_topic:
            raise RuntimeError("Camera input module: 'topic' parameter is not defined.")
        # add topic subscriber and tf buffer
        self._detections_sub = rospy.Subscriber(detection_topic, DetectionArrayMsg, self.detectionCallback)
        # message buffers
        self._detections_msg = []
        self._detections_wme_map = {}
        self._detections_new_value = False

    def detectionCallback(self, msg):
        # buffer msg
        self._detections_msg = msg
        self._detections_new_value = True

    def update(self):
        # check if input was updated
        if not self._detections_new_value:
            return
        self._detections_new_value = False
        # form list of WMEs for addition and deletion
        del_wme_map = self._detections_wme_map.copy()
        # iterate detected objects
        for detection_msg in self._detections_msg.detections:
            key_tuple = (detection_msg.id, detection_msg.label, detection_msg.detector)
            # check if corrsponding wme exists
            wme_id = self._detections_wme_map.get(key_tuple)
            if wme_id:
                # WME exists, remove it from deletion list
                del del_wme_map[key_tuple]
                # TODO coordinate update?
                pass
            else:
                # add new WME
                wme_id = self._sensor_id.CreateIdWME("visible")
                # add attributes
                wme_id.CreateIntWME("id", detection_msg.id)
                wme_id.CreateStringWME("label", detection_msg.label)
                wme_id.CreateStringWME("type", detection_msg.detector)
                # register it in WMEs map
                self._detections_wme_map[key_tuple] = wme_id
        # delete no longer visible objects
        for key_tuple, wme_id in del_wme_map.iteritems():
            del self._detections_wme_map[key_tuple]
            wme_id.DestroyWME()

    def __del__(self):
        # remove sensor wme and ROS subscriber
        self._sensor_id.DestroyWME()
        self._detections_sub.unregister()

input_module.register("camera", Camera)
