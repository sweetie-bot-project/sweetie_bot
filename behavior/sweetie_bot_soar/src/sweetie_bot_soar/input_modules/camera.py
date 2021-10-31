from . import input_module

from copy import copy
import rospy
import tf

from sweetie_bot_text_msgs.msg import DetectionArray as DetectionArrayMsg, Detection as DetectionMsg

class Camera:
    def __init__(self, name, config, agent):
        self._detections_sub = None

        # get input link WME ids
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)
        # configuration
        detection_topic = config.get("topic")
        if not detection_topic:
            raise RuntimeError("Camera input module: 'topic' parameter is not defined.")
        self._timeout = config.get("timeout")
        if not self._timeout or not isinstance(self._timeout, (int,float)) or self._timeout < 0.0:
            raise RuntimeError("Camera input module: 'timeout' parameter is not defined or incorrect.")
        # add topic subscriber and tf buffer
        self._detections_sub = rospy.Subscriber(detection_topic, DetectionArrayMsg, self.detectionCallback)
        # message buffers
        self._detections_msg = []
        self._detections_wme_map = {}

    def detectionCallback(self, msg):
        # buffer msg
        self._detections_msg = msg # should be atomic

    def update(self):
        # get current time
        time_now = rospy.Time.now().to_sec();
        # iterate detected objects
        for detection_msg in self._detections_msg.detections:
            key_tuple = (detection_msg.id, detection_msg.label, detection_msg.type)
            # check if corrsponding wme exists
            value_tuple = self._detections_wme_map.get(key_tuple)
            if value_tuple:
                (wme_id, seen_id, _) = value_tuple
                # renew timestamp
                timestamp = detection_msg.header.stamp.to_sec()
                self._detections_wme_map[key_tuple] = (wme_id, seen_id, timestamp)
            else:
                # add new WME
                wme_id = self._sensor_id.CreateIdWME("detection")
                # add attributes
                wme_id.CreateIntWME("id", detection_msg.id)
                wme_id.CreateStringWME("label", detection_msg.label)
                wme_id.CreateStringWME("type", detection_msg.type)
                # add timestamp
                timestamp = detection_msg.header.stamp.to_sec()
                seen_id = wme_id.CreateFloatWME("seen", time_now - timestamp)
                # register it in WMEs map
                self._detections_wme_map[key_tuple] = (wme_id, seen_id, timestamp)

        # update WME elements and form remove list
        remove_list = []
        for key_tuple, value_tuple in self._detections_wme_map.iteritems():
            (wme_id, seen_id, timestamp) = value_tuple
            if timestamp + self._timeout < time_now:
                #element expired
                remove_list.append( (key_tuple, wme_id) )
            else:
                #element not expired yet, renew seen element in SOAR memory
                seen_id.Update(time_now - timestamp)
                
        # delete expired WMEs 
        for (key_tuple, wme_id) in remove_list:
            del self._detections_wme_map[key_tuple]
            wme_id.DestroyWME()

    def __del__(self):
        # remove sensor wme and ROS subscriber
        self._sensor_id.DestroyWME()
        if self._detections_sub:
            self._detections_sub.unregister()

input_module.register("camera", Camera)
