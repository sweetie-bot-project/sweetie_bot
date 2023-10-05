from . import input_module

from threading import Lock
import rospy

from sweetie_bot_herkulex_msgs.msg import HerkulexState

class HerkulexServos:
    def __init__(self, name, config, agent):
        self._servo_state_sub = None
        # get input link WME ids
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)

        # get configuration from parameters
        servo_state_topic = config.get("topic")
        if not servo_state_topic or not isinstance(servo_state_topic, str):
            raise RuntimeError("HerkulexServos input module: 'topic' parameter is not defined or is not string.")
        self._overheat_temperature = config.get("overheat_temperature")
        if not self._overheat_temperature or not isinstance(self._overheat_temperature, (float, int)):
            raise RuntimeError("HerkulexServos input module: 'overheat_temperature' parameter is not defined or is not number.")
        self._ignore_joints = config.get("ignore_joints", [])
        if not isinstance(self._ignore_joints, list):
            raise RuntimeError("HerkulexServos input module: 'ignore_joints' parameter must be list of strings.")

        # subscriber    
        self._servo_state_sub = rospy.Subscriber(servo_state_topic, HerkulexState, self.newServoStateCallback)

        # message buffers
        self._lock = Lock()
        self._overheat_servos = set()
        self._failed_servos = set()
        self._off_servos = set()
        # WME ids
        self._status_wmes_ids = {}

    def newServoStateCallback(self, msg):
        # check ignore list
        if msg.name in self._ignore_joints:
            return
        # update servo state
        with self._lock:
            # check if servo has FAILED
            if not msg.respond_sucess or msg.status_error:
                self._failed_servos.add(msg.name)
            else:
                self._failed_servos.discard(msg.name)
            # check if servo is OFF
            if not (msg.status_detail & HerkulexState.STATUS_DETAIL_MOTOR_ON):
                self._off_servos.add(msg.name)
            else:
                self._off_servos.discard(msg.name)
            # check if servo is overheated
            if msg.temperature > self._overheat_temperature:
                self._overheat_servos.add(msg.name)
            else:
                self._overheat_servos.discard(msg.name)

    def update_status_wme(self, status, value):
        if value:
            if status not in self._status_wmes_ids:
                self._status_wmes_ids[status] = self._sensor_id.CreateStringWME('status', status)
        else:
            status_id = self._status_wmes_ids.get(status)
            if status_id != None:
                status_id.DestroyWME()
                del self._status_wmes_ids[status]

    def update(self):
        with self._lock:
            # set status flags
            self.update_status_wme('normal', len(self._off_servos) == 0 and len(self._failed_servos) == 0 and len(self._overheat_servos) == 0) 
            self.update_status_wme('failed', len(self._failed_servos) != 0)
            self.update_status_wme('overheat', len(self._overheat_servos) != 0)
            self.update_status_wme('off', len(self._off_servos) != 0)

    def __del__(self):
        # remove sensor wme and ROS subscriber
        self._sensor_id.DestroyWME()
        if self._servo_state_sub:
            self._servo_state_sub.unregister()

input_module.register("herkulex_servos", HerkulexServos)
