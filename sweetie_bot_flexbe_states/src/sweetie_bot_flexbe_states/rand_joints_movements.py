#!/usr/bin/env python
import random

import rospy
from rospy.rostime import Time, Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxyServiceCaller

from std_msgs.msg import Header
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState


class RandJointsMovements(EventState):
    '''
    Periodically reposition joints to random point using given FollowJointState controller.

    -- controller       string         FollowJointState controller name without `controller` prefix.
    -- duration         float          How long this state will be executed (seconds).
    -- interval         float[2]       Maximal and minimal interval between movements in form [min, max].
    -- joints           string[]       Joint names.
    -- minimal          float[]        Minimal values for joint positions.
    -- maximal          float[]        Maximal values for joint positions.

    ># config       dict            Dictionary with keys 'duration', 'interval', 'joints', 'min' and 'max' to override default configuration. dict or key values can be set to None to use default value from parameters.

    <= done 	                    Finished.
    <= failed 	                    Failed to activate FollowJointState controller.

    '''

    def __init__(self, controller = 'joint_state_head', duration = 120, interval = [2.0,4.0], joints = [], minimal = [], maximal = []):
        super(RandJointsMovements, self).__init__(outcomes = ['done', 'failed'], input_keys = ['config'])

        # Store topic parameter for later use.
        self._controller = 'motion/controller/' + controller 

        # create proxies
        self._set_operational_caller = ProxyServiceCaller({ self._controller + '/set_operational': SetBool })
        self._publisher = ProxyPublisher({ self._controller + '/in_joints_ref': JointState })

        # state
        self._start_timestamp = None
        self._movement_timestamp = None
        self._movement_duration = Duration()

        # user input
        self.set_movement_parameters(duration, interval, joints, minimal, maximal)

        # error in enter hook
        self._error = False

    def set_movement_parameters(self, duration, interval, joints, minimal, maximal):
        if not isinstance(duration, (int, float)) or duration < 0:
            raise ValueError("Duration must be nonegative number.")
        if len(interval) != 2 or any([ not isinstance(t, (int, float)) for t in interval])or any([ t < 0 for t in interval]) or interval[0] > interval[1]:
            raise ValueError("Interval must represent interval of time.")
        if len(joints) != len(minimal) or len(joints) != len(maximal):
            raise ValueError("joints, minimal and maximal arrays must have the same size.")
        if any([ not isinstance(v, str) for v in joints]):
            raise ValueError("joints: list of string is expected.")
        if any([ not isinstance(v, (int, float)) for v in minimal]):
            raise ValueError("minimal: list of numbers is expected.")
        if any([ not isinstance(v, (int, float)) for v in maximal]):
            raise ValueError("maximal: list of numbers is expected.")
        self._duration = Duration.from_sec(duration)
        self._interval = interval
        self._joints = joints
        self._minimal = minimal
        self._maximal = maximal
    
    def on_enter(self, userdata):
        self._error = False

        # override configuration if necessary
        try:
            if userdata.config != None:
                # process dict
                duration = userdata.config.get("duration", self._duration.to_sec())
                interval = userdata.config.get("interval", self._interval)
                joints = userdata.config.get("joints", self._joints)
                minimal = userdata.config.get("min2356", self._minimal)
                maximal = userdata.config.get("max2356", self._maximal)
                # check parameters
                self.set_movement_parameters(duration, interval, joints, minimal, maximal)
        except Exception as e:
            Logger.logwarn('Failed to process `config` input key:\n%s' % str(e))
            self._error = True
            return
            
        # activate controller
        # TODO use actionlib
        try: 
            res = self._set_operational_caller.call(self._controller + '/set_operational', True)
        except Exception as e:
            Logger.logwarn('Failed to activate `' + self._controller + '` controller:\n%s' % str(e))
            self._error = True
            return

        if not res.success:
            Logger.logwarn('Failed to activate `' + self._controller + '` controller (SetBoolResponse: success = false).')
            self._error = True

        # set start timestamp
        self._start_timestamp = Time.now()
        self._movement_timestamp = Time.now();

        Logger.loginfo('Start random pose generation, controller `%s`.' % self._controller)

    def execute(self, userdata):
        if self._error:
            return 'failed'
        
        # check if time elasped
        if Time.now() - self._start_timestamp > self._duration:
            return 'done'
        # check if we have tosend new point
        if Time.now() - self._movement_timestamp > self._movement_duration:
            # determine new interval 
            self._movement_timestamp = Time.now()
            self._movement_duration = Duration.from_sec(random.uniform(*self._interval))
            # form message
            msg = JointState()
            msg.header = Header(stamp = Time.now())
            msg.name = self._joints;
            for i in range(0, len(self._joints)):
                x = random.uniform(0, 1)
                msg.position.append(self._minimal[i]*x + self._maximal[i]*(1-x))
            # send message
            try: 
                self._publisher.publish(self._controller + '/in_joints_ref', msg)
            except Exception as e:
                Logger.logwarn('Failed to send JointState message `' + self._topic + '`:\n%s' % str(e))

    def on_exit(self, userdata):
        try: 
            res = self._set_operational_caller.call(self._controller + '/set_operational', False)
        except Exception as e:
            Logger.logwarn('Failed to deactivate `' + self._controller + '` controller:\n%s' % str(e))
            return
        Logger.loginfo('Done random pose generation.')
 

