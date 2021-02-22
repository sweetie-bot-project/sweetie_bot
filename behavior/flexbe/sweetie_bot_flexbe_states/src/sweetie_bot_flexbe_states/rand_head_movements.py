#!/usr/bin/env python
import random

import rospy
from rospy.rostime import Time, Duration

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher, ProxyServiceCaller

from std_msgs.msg import Header
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState


class SweetieBotRandHeadMovements(EventState):
    '''
    Periodically reposition eyes and head of SweetieBot to random point using given FollowJointState controller.

    -- controller   string          FollowJointState controller name without `controller` prefix.
    -- duration     float           How long this state will be executed (seconds).
    -- interval     float[2]        Array of floats, maximal and minimal interval between movements.
    -- max2356      float[4]        Max values for joint52, joint53, eyes_pitch, eyes_yaw.
    -- min2356      float[4]        Min values for joint52, joint53, eyes_pitch, eyes_yaw.

    ># config       dict            Dictionary with keys 'duration', 'interval', 'max2356', 'min2356' to override default configuration. dict or key values can be set to None to use default value from parameters.

    <= done                         Finished.
    <= failed                       Failed to activate FollowJointState controller.

    '''

    def __init__(self, controller = 'joint_state_head', duration = 120, interval = [3, 5], max2356 = [0.3, 0.3, 1.5, 1.5], min2356 = [-0.3, -0.3, -1.5, -1.5]):
        super(SweetieBotRandHeadMovements, self).__init__(outcomes = ['done', 'failed'], input_keys = ['config'])

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
        self.set_movement_parameters(duration, interval, min2356, max2356)

        # error in enter hook
        self._error = False

    def set_movement_parameters(self, duration, interval, min2356, max2356):
        if not isinstance(duration, (int, float)) or duration < 0:
            raise ValueError("Duration must be nonegative number.")
        if len(interval) != 2 or any([ not isinstance(t, (int, float)) for t in interval])or any([ t < 0 for t in interval]) or interval[0] > interval[1]:
            raise ValueError("Interval must represent interval of time.")
        if len(min2356) != 4 or any([ not isinstance(v, (int, float)) for v in min2356]):
            raise ValueError("min2356: list of four numbers was expected.")
        if len(max2356) != 4 or any([ not isinstance(v, (int, float)) for v in max2356]):
            raise ValueError("max2356: list of four numbers was expected.")
        self._duration = Duration.from_sec(duration)
        self._interval = interval
        self._min2356 = min2356
        self._max2356 = max2356
    
    def on_enter(self, userdata):
        self._error = False

        # override configuration if necessary
        try:
            if userdata.config != None:
                # process dict
                duration = userdata.config.get("duration", self._duration.to_sec())
                interval = userdata.config.get("interval", self._interval)
                min2356 = userdata.config.get("min2356", self._min2356)
                max2356 = userdata.config.get("max2356", self._max2356)
                # check parameters
                self.set_movement_parameters(duration, interval, min2356, max2356)
        except Exception as e:
            Logger.logwarn('Failed to process `config` input key:\n%s' % str(e))
            self._error = True
            return
            
        # activate head controller
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

        Logger.loginfo('Start random pose generation (eyes, head), controller `%s`.' % self._controller)

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
            msg.name = [ 'joint52', 'joint53', 'eyes_pitch', 'eyes_yaw' ]
            # random durection
            x = random.uniform(0, 1)
            y = random.uniform(0, 1)
            # compute head position
            msg.position = [ 0, 0, 0, 0 ]
            msg.position[0] = self._min2356[0]*x + self._max2356[0]*(1-x)
            msg.position[1] = self._min2356[1]*y + self._max2356[1]*(1-y)
            # compute eyes position
            msg.position[2] = self._min2356[2]*y + self._max2356[2]*(1-y)
            msg.position[3] = self._min2356[3]*x + self._max2356[3]*(1-x)
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
        Logger.loginfo('Done random pose generation for eyes and head.')
 

