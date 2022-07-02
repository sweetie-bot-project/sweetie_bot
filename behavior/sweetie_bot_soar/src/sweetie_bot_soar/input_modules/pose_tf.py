from . import input_module

from threading import Lock
from math import acos, atan2, degrees

import rospy, tf
from numpy.linalg import norm
from tf.transformations import quaternion_matrix

from .bins import BinsMap
from .wme import WME

class PoseTF:
    def __init__(self, name, config, agent):
        self._tf_listener = None

        # get input link WME ids
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)

        # get configuration from psearameters
        self._base_frame = config.get("base_frame")
        if not self._base_frame or not isinstance(self._base_frame, str):
            raise RuntimeError("PoseTF input module: 'base_frame' parameter is not defined or is not string.")
        self._world_frame = config.get("world_frame")
        if not self._world_frame or not isinstance(self._world_frame, str):
            raise RuntimeError("PoseTF input module: 'world_frame' parameter is not defined or is not string.")
        try:
            self._incline_bins_map = BinsMap( config['incline_bins_map'] )
        except KeyError:
            raise RuntimeError('PoseTF input module: "incline_bins_map" parameter must present.')

        # transform listener
        self._listener = tf.TransformListener();
        self._listener.waitForTransform(self._world_frame, self._base_frame, rospy.Time(), rospy.Duration(5.0)) # raise exception if tf is not available

        # WME cache
        self._inclination_wme = WME(self._sensor_id, "inclination", self._incline_bins_map(0.0))
        self._heading_wme = WME(self._sensor_id, "heading", 0.0)
        self._x_wme = WME(self._sensor_id, "x", 0.0)
        self._y_wme = WME(self._sensor_id, "y", 0.0)
        self._status_wme = WME(self._sensor_id, "status", "ok")

    def update(self):
        # get base pose from tf
        try:
            [p, quat] = self._listener.lookupTransform(self._world_frame, self._base_frame, rospy.Time())
        except tf.Exception:
            rospy.logerr("PoseTF input module: unable to transform.")
            self._status_wme.update("error")

        # caluclate properties
        T = quaternion_matrix(quat)
        inclination = degrees(acos(T[2,2])) # z component of unit vector of base_frame Oz axis
        # y and x components of unit vector of base_frame Ox axis
        xy = T[0,0:2]
        xy /= norm(xy)
        heading = degrees(atan2(xy[1],xy[0])) 

        # update WMEs
        self._status_wme.update("ok")
        self._inclination_wme.update(self._incline_bins_map(inclination))
        self._heading_wme.update(heading)
        self._x_wme.update(p[0])
        self._y_wme.update(p[1])

    def __del__(self):
        # remove sensor wme
        self._sensor_id.DestroyWME()

input_module.register("pose_tf", PoseTF)
