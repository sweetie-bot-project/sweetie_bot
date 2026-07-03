from . import input_module
from .input_module import InputModule, InputModulesLoader
from .wme_proxy import SetWMEProxy

from threading import Lock
import rospy

from sweetie_bot_herkulex_msgs.msg import HerkulexState

class HerkulexServos(InputModule):
    def _init(self, name, config, agent):
        
        # get configuration from parameters
        servo_state_topic = self.getConfigParameter(config, 'topic', allowed_types = str)
        self._overheat_temperature = self.getConfigParameter(config, "overheat_temperature", allowed_types = (float, int))
        self._ignore_joints = self.getConfigParameter(config, "ignore_joints", default_value = [], check_func = lambda jnts: all(isinstance(v, str) for v in jnts))
        # hardware-disabled servos (single source of truth: /disabled_servos, hardware.yaml)
        # are unioned in: a known-dead servo must never surface as failed/off/overheat to SOAR
        disabled = list(rospy.get_param("/disabled_servos", {}).keys())
        if disabled:
            self._ignore_joints = list(set(self._ignore_joints) | set(disabled))
            rospy.loginfo(f"input module {name}: ignoring hardware-disabled servos {sorted(disabled)}")
        joint_groups = self.getConfigParameter(config, "groups", default_value = {}, 
                                                         check_func = lambda grps: all(isinstance(k, str) and isinstance(v, list) for k, v in grps.items()),
                                                         error_desc= f"input module {name}: groups dict must contain (str, list) pairs where key represents group name and list elements are servo names in group.")
        self._groups = {}
        try:
            for k, v in joint_groups.items():
                self._groups[k] = set(v)
        except TypeError:
            raise TypeError(f"input module {name}: group {k}: incorect group declaration {v}")

        # message buffers
        self._lock = Lock()
        self._overheat_servos = set()
        self._failed_servos = set()
        self._off_servos = set()
        # WME ids
        self._status_wmes = SetWMEProxy(self._sensor_id, 'status')
        self._failed_servos_wmes = SetWMEProxy(self._sensor_id, 'falied')
        self._off_servos_wmes = SetWMEProxy(self._sensor_id, 'off')
        self._overheat_servos_wmes = SetWMEProxy(self._sensor_id, 'overheat')
        self._groups_status_wmes = { group: SetWMEProxy(self._sensor_id, group) for group in self._groups.keys() }

        # subscriber    
        self._servo_state_sub = self.createSubscriber(servo_state_topic, HerkulexState, self.newServoStateCallback)

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

    def update(self):
        with self._lock:
            # set status flags
            self._status_wmes.set_present('normal', len(self._off_servos) == 0 and len(self._failed_servos) == 0 and len(self._overheat_servos) == 0) 
            self._status_wmes.set_present('failed', len(self._failed_servos) != 0)
            self._status_wmes.set_present('overheat', len(self._overheat_servos) != 0)
            self._status_wmes.set_present('off', len(self._off_servos) != 0)
            # update detailed servo info
            self._failed_servos_wmes.assign(self._failed_servos)
            self._off_servos_wmes.assign(self._off_servos)
            self._overheat_servos_wmes.assign(self._overheat_servos)
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
                self._groups_status_wmes[group].assign(status)


InputModulesLoader.register("herkulex_servos", HerkulexServos)
