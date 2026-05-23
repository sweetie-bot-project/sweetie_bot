from . import input_module
from .input_module import InputModule

from threading import Lock
import rospy

from sweetie_bot_herkulex_msgs.msg import HerkulexState

class HerkulexServos(InputModule):
    def __init__(self, name, config, agent):
        super(HerkulexServos, self).__init__(name)

        # preinit 
        self._servo_state_sub = None

        # get input link WME ids
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)

        # get configuration from parameters
        servo_state_topic = self.getConfigParameter(config, 'topic', allowed_types = str)
        self._overheat_temperature = self.getConfigParameter(config, "overheat_temperature", allowed_types = (float, int))
        self._ignore_joints = self.getConfigParameter(config, "ignore_joints", default_value = [], check_func = lambda jnts: all(isinstance(v, str) for v in jnts))
        joint_groups = self.getConfigParameter(config, "groups", default_value = {}, 
                                                         check_func = lambda grps: all(isinstance(k, str) and isinstance(v, list) for k, v in grps.items()),
                                                         error_desc= f"input module {name}: groups dict must contain (str, list) pairs where key represents group name and list elements are servo names in group.")
        self._groups = {}
        try:
            for k, v in joint_groups.items():
                self._groups[k] = set(v)
        except TypeError:
            raise TypeError(f"input module {name}: group {k}: incorect group declaration {v}")

        # subscriber    
        self._servo_state_sub = rospy.Subscriber(servo_state_topic, HerkulexState, self.newServoStateCallback)

        # message buffers
        self._lock = Lock()
        self._overheat_servos = set()
        self._failed_servos = set()
        self._off_servos = set()
        # WME ids
        self._status_wmes_ids = {}
        self._set_wmes = {'failed': {}, 'off': {}, 'overheat': {}}
        for k in self._groups.keys():
            self._set_wmes[k] = {}

    def newServoStateCallback(self, msg):
        # check ignore list
        if msg.name in self._ignore_joints:
            return
        # update servo state
        with self._lock:
            # check if servo has FAILED
            critical = HerkulexState.STATUS_ERROR_OVER_VOLTAGE \
                     | HerkulexState.STATUS_ERROR_TEMPERATURE \
                     | HerkulexState.STATUS_ERROR_OVERLOAD \
                     | HerkulexState.STATUS_ERROR_DRIVER_FAULT \
                     | HerkulexState.STATUS_ERROR_EEP_REGS
            if not msg.respond_sucess or msg.status_error & critical:
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

    def update_set_wmes(self, name, values):
		# get current set values: bind new name to dict element
        set_values_ids = self._set_wmes[name]
        # compare with new values
        addlist = values - set_values_ids.keys()
        dellist = set_values_ids.keys() - values
        # add or remove new wmes
        for value in addlist:
            set_values_ids[value] = self._sensor_id.CreateStringWME(name, value)
        for value in dellist:
            wme_id = set_values_ids[value]
            wme_id.DestroyWME()
            del set_values_ids[value]

    def update(self):
        with self._lock:
            # set status flags
            self.update_status_wme('normal', len(self._off_servos) == 0 and len(self._failed_servos) == 0 and len(self._overheat_servos) == 0) 
            self.update_status_wme('failed', len(self._failed_servos) != 0)
            self.update_status_wme('overheat', len(self._overheat_servos) != 0)
            self.update_status_wme('off', len(self._off_servos) != 0)
            # update detailed servo info
            self.update_set_wmes('failed', self._failed_servos)
            self.update_set_wmes('off', self._off_servos)
            self.update_set_wmes('overheat', self._overheat_servos)
            # groups
            for group, servos in self._groups.items():
                status = set()
                if not servos.isdisjoint(self._failed_servos):
                    status.add('failed')
                if not servos.isdisjoint(self._overheat_servos):
                    status.add('overheat')
                if not servos.isdisjoint(self._off_servos):
                    status.add('off')
                if len(status) == 0:
                    status.add('normal')
                self.update_set_wmes(group, status)

    def __del__(self):
        # remove sensor wme and ROS subscriber
        self._sensor_id.DestroyWME()
        if self._servo_state_sub:
            self._servo_state_sub.unregister()
        # superclass destructor
        super(HerkulexState, self).__del__()

input_module.register("herkulex_servos", HerkulexServos)
