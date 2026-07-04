from .output_module import OutputModulesLoader, OutputModule
from ..input_modules.swm import ObjectKeyTuple, SpatialWorldModel

import math

import rospy
import actionlib
import tf
from  actionlib import GoalStatus

from sweetie_bot_control_msgs.msg import SetOperationalAction, SetOperationalGoal, SetOperationalResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class LookAt(OutputModule):

    def _init(self, name, config):
        # preinit
        self._timer = None

        # get configuration
        action_ns = self.getConfigParameter(config, "controller", allowed_types = str)
        self._period = rospy.Duration( self.getConfigParameter(config, "period", allowed_types = float, default_value = 0.1) )
        # create actionlib client
        self._set_operational_aclient = actionlib.SimpleActionClient(action_ns, SetOperationalAction)
        # create publisher
        self._pose_pub = rospy.Publisher(action_ns + "/in_pose_ref", PoseStamped, queue_size = 1)
        # safety gate: frontal safe cone + anti-churn rate limit (see soar.yaml look-at block)
        cone = self.getConfigParameter(config, "safe_cone", default_value={}, allowed_types=dict)
        self._cone_frame = cone.get("reference_frame", "base_link")
        self._cone_max_yaw = float(cone.get("max_yaw", 1.0))
        self._cone_min_dist = float(cone.get("min_distance", 0.35))
        self._cone_low_z = float(cone.get("low_z", -0.05))
        self._cone_low_z_dist = float(cone.get("low_z_distance", 0.7))
        # how long a goal may sit with an out-of-cone target before failing (releases the
        # head/eyes resources; holding them silently starves focusing-on-interlocutor - P19)
        self._unsafe_fail_s = float(cone.get("unsafe_fail_s", 1.5))
        self._tf_listener = tf.TransformListener()
        # object to monitor
        self._object_key = None
        self._object_not_found = False

    def _target_in_safe_cone(self, obj, swm):
        """True if the object pose is a safe gaze target (frontal, not too close, not low+close).

        TF hiccups fail OPEN (allow) - the gate protects against bad geometry, not bad TF.
        """
        try:
            pose_stamped = PoseStamped(pose=obj.pose,
                                       header=Header(frame_id=swm.world_frame, stamp=rospy.Time()))
            p = self._tf_listener.transformPose(self._cone_frame, pose_stamped).pose.position
        except Exception as e:  # noqa: BLE001
            rospy.logwarn_throttle(5.0, "lookat output module: safe-cone TF failed (%s); allowing", e)
            return True
        dist_h = math.hypot(p.x, p.y)
        yaw = math.atan2(p.y, p.x)
        if dist_h < self._cone_min_dist:
            rospy.logwarn_throttle(2.0, "lookat: target too close (%.2fm < %.2fm) - rejected",
                                   dist_h, self._cone_min_dist)
            return False
        if abs(yaw) > self._cone_max_yaw:
            rospy.logwarn_throttle(2.0, "lookat: target outside frontal cone (|%.2f| > %.2f rad) - rejected",
                                   yaw, self._cone_max_yaw)
            return False
        if p.z < self._cone_low_z and dist_h < self._cone_low_z_dist:
            rospy.logwarn_throttle(2.0, "lookat: target low and close (z=%.2f, d=%.2f) - rejected",
                                   p.z, dist_h)
            return False
        return True

    def startHook(self, cmd_id):
        # extract object key
        label_id = cmd_id.FindByAttribute("label", 0)
        type_id = cmd_id.FindByAttribute("type", 0)
        if label_id is None or type_id is None:
            rospy.logerr("lookat output module: label and type attributes must present.")
            return "error"
        object_key = ObjectKeyTuple( 0, label_id.GetValueAsString(), type_id.GetValueAsString() )
        # get resources
        resources = []
        while True:
            resource_id = cmd_id.FindByAttribute("resource", len(resources))
            if resource_id is not None:
                resources.append(resource_id.GetValueAsString())
            else:
                break
        # DO NOT activate the controller yet. LookAtJoints solves IK toward its stored
        # target_point EVERY tick while operational; before the first in_pose_ref arrives
        # that member is (0,0,0) = the world origin at the robot's feet. Activating while
        # the safe-cone gate blocks publication therefore ground the head toward its own
        # feet / wrapped it up-backwards (the head-dive AND head-up live runaways).
        # Activate only when the first safe pose is ready: publish pose FIRST (the RTT
        # port keeps the last sample), SetOperational SECOND (_publishCallback).
        self._object_key = object_key
        self._resources = resources
        self._goal_sent = False
        self._object_not_found = False
        self._unsafe_since = None
        self._timer = self.createTimer(self._period, self._publishCallback)
        rospy.loginfo('lookat output module: waiting for a safe target to activate (object: %s, resources: %s)', object_key, resources)
        return None

    def _publishCallback(self, event):
        # get SWM
        swm = SpatialWorldModel.get_swm()
        if swm is  None:
            return
        # get object
        obj = swm.get_object(self._object_key)
        if obj is None:
            self._object_not_found = True
            return
        # safe-cone gate every tick: never stream references toward an unsafe target (outside
        # the frontal cone / too close / low-and-close). Track the streak: updateHook fails the
        # goal after a short grace so the head/eyes resources are RELEASED (a silently-held goal
        # starves focusing-on-interlocutor and mutes the talk pipeline - P19).
        if not self._target_in_safe_cone(obj, swm):
            if self._unsafe_since is None:
                self._unsafe_since = rospy.Time.now()
            return
        self._unsafe_since = None
        # publish pose
        pose_stamped = PoseStamped(pose = obj.pose, header = Header(frame_id = swm.world_frame, stamp = rospy.Time.now()))
        self._pose_pub.publish(pose_stamped)
        # first safe pose is on the wire: NOW it is safe to activate the controller
        if not self._goal_sent:
            try:
                goal = SetOperationalGoal(operational = True, resources = self._resources)
                self._set_operational_aclient.send_goal(goal)
                self._goal_sent = True
                rospy.loginfo('lookat output module: set operational (object: %s, resources: %s)', self._object_key, self._resources)
            except Exception as e:
                rospy.logwarn('lookat output module: Failed to send the SetOperational command:\n%s' % str(e))
                self._object_not_found = True  # let updateHook fail the command

    def _cancel_goal(self, reason):
        if self._goal_sent:
            self._set_operational_aclient.cancel_goal()
        self.removeTimer(self._timer)
        rospy.loginfo(f"output module '%s': %s", self._name, reason)

    def updateHook(self, cmd_id, external_abort_request):
        # PRE-ACTIVATION state: controller not started yet (no safe target seen so far).
        # Honor aborts, fail on missing object or a persistent out-of-cone target.
        if not self._goal_sent:
            agent_abort = external_abort_request
            if cmd_id is None or (cmd_id is not None and cmd_id.FindByAttribute("abort", 0) is not None):
                agent_abort = True
            if agent_abort:
                self._cancel_goal(reason="behavior aborted before activation")
                return "succeed"
            if self._object_not_found:
                self._cancel_goal(reason="SWM object is missing (not activated)")
                return "failed"
            if self._unsafe_since is not None and \
                    (rospy.Time.now() - self._unsafe_since).to_sec() > self._unsafe_fail_s:
                self._cancel_goal(reason="target outside the safe gaze cone (never activated)")
                return "failed"
            return None
        # get goal state
        status = self._set_operational_aclient.get_state()

        # Controller is active.
        if status in [ GoalStatus.ACTIVE, GoalStatus.PENDING ]:
            # check for abort request from agent
            # look at can always be interrupted so abort request type is unimportant

            # check for agent abort request
            agent_abort_request = False
            if cmd_id is not None:
                # via ^abort WME
                abort_id = cmd_id.FindByAttribute("abort", 0)
                if abort_id is not None:
                    agent_abort_request = True
            else:
                # by deleting command
                agent_abort_request = True
            if agent_abort_request:
                self._cancel_goal(reason="behavior aborted by agent")
                return "succeed"

            # check for exteral abort request
            if external_abort_request:
                self._cancel_goal(reason="behavior aborted by external request")
                return "succeed"

            # check if object is missing
            if self._object_not_found:
                self._cancel_goal(reason="SWM object is missing")
                return "failed"

            # target persistently outside the safe cone: fail (releases head/eyes for others)
            if self._unsafe_since is not None and \
                    (rospy.Time.now() - self._unsafe_since).to_sec() > self._unsafe_fail_s:
                self._cancel_goal(reason="target outside the safe gaze cone")
                return "failed"

            # continue exection (ACTIVE, PENDING)
            return None

        # Controller is stopped.
        if status in (GoalStatus.REJECTED, GoalStatus.ABORTED):
            self.removeTimer(self._timer)
            rospy.logerr("lookat output module: activation LookAt controller failed.")
            return "failed"
        elif status in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            self.removeTimer(self._timer)
            rospy.loginfo("flexbe output module: stopped by external reason: %s.", status)
            return "succeed"

        # continue exection (RECALLING, PREEMPTING)
        return None


OutputModulesLoader.register("look-at", LookAt)



        



