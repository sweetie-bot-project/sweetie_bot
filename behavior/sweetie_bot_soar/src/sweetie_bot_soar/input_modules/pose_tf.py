from threading import Lock
from math import acos, atan2, degrees

import rospy, tf
from numpy.linalg import norm
from tf.transformations import quaternion_matrix

from .bins import BinsMap
from .wme_proxy import SingleValueWMEProxy
from . import input_module
from .input_module import InputModule, InputModulesLoader

from flexbe_core.proxy import ProxyTransformListener


class PoseTF(InputModule):

    def _init(self, name, config, agent):

        # get configuration from psearameters
        self._target_frame = self.getConfigParameter(config, "target_frame", allowed_types = str)
        self._base_frame = self.getConfigParameter(config, "base_frame", allowed_types = str)
        try:
            self._incline_bins_map = BinsMap( config['inclination_bins_map'] )
            self._heading_bins_map = BinsMap( config['heading_bins_map'] )
            self._time_bins_map = BinsMap( config['time_bins_map'] )
        except KeyError as e:
            raise ValueError('PoseTF input module: "inclination_bins_map" and "heading_bins_map" parameters must present.') from e

        # transform listener
        self._listener = tf.TransformListener();
        self._listener.waitForTransform(self._base_frame, self._target_frame, rospy.Time(), rospy.Duration(5.0)) # raise exception if tf is not available

        # state
        time_now = rospy.Time.now()
        self._inclination_change_time = time_now
        self._heading_change_time = time_now

        # WME cache
        self._inclination_wme = SingleValueWMEProxy(self._sensor_id, "inclination", self._incline_bins_map(0.0))
        self._inclination_time_wme = SingleValueWMEProxy(self._sensor_id, "inclination-change-time", self._time_bins_map(0.0)) 
        self._heading_wme = SingleValueWMEProxy(self._sensor_id, "heading", self._heading_bins_map(0.0))
        self._heading_time_wme = SingleValueWMEProxy(self._sensor_id, "heading-change-time", self._time_bins_map(0.0)) 
        self._x_wme = SingleValueWMEProxy(self._sensor_id, "x", 0.0)
        self._y_wme = SingleValueWMEProxy(self._sensor_id, "y", 0.0)
        self._status_wme = SingleValueWMEProxy(self._sensor_id, "status", "ok")

    def update(self):
        # time
        time_now = rospy.Time.now()
        # get base pose from tf
        try:
            [p, quat] = self._listener.lookupTransform(self._base_frame, self._target_frame, rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, 'PoseTF input module: unable to get transform from %s to %s: %s', self._base_frame, self._target_frame, e)
            self._status_wme.update("no-tf")
            return

        # caluclate properties
        T = quaternion_matrix(quat)
        inclination = degrees(acos(T[2,2])) # z component of unit vector of base_frame Oz axis
        # y and x components of unit vector of base_frame Ox axis
        xy = T[0,0:2]
        xy /= norm(xy)
        heading = degrees(atan2(xy[1],xy[0])) 

        #
        # update WMEs
        #
        self._status_wme.update("ok")
        # update heading, inclination and corresponding timestamps
        if self._inclination_wme.update(self._incline_bins_map(inclination)):
            self._inclination_change_time = time_now
        if self._heading_wme.update(self._heading_bins_map(heading)):
            self._heading_change_time = time_now
        # update time-related WMEs
        self._heading_time_wme.update(self._time_bins_map( (time_now - self._heading_change_time).to_sec() ))
        self._inclination_time_wme.update(self._time_bins_map( (time_now - self._inclination_change_time).to_sec() ))
        # numerical values
        self._x_wme.update(p[0])
        self._y_wme.update(p[1])

InputModulesLoader.register("pose_tf", PoseTF)
