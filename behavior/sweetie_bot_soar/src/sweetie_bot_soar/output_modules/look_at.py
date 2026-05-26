from .output_module import OutputModulesLoader, OutputModule
from ..input_modules.swm import ObjectKeyTuple, SpatialWorldModel

import rospy
import actionlib
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
        # object to monitor
        self._object_key = None
        self._object_not_found = False

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
        # start controller
        try:
            goal = SetOperationalGoal(operational = True, resources = resources)
            self._set_operational_aclient.send_goal(goal)
        except Exception as e:
            rospy.logwarn('lookat output module: Failed to send the SetOperational command:\n%s' % str(e))
            return 'error'
        # start publication
        self._object_key = object_key
        self._object_not_found = False
        self._timer = rospy.Timer(self._period, self._publishCallback)
        # log
        rospy.loginfo('lookat output module: set opertional')
        # start 
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
        # publish pose
        pose_stamped = PoseStamped(pose = obj.pose, header = Header(frame_id = swm.world_frame, stamp = rospy.Time.now()))
        self._pose_pub.publish(pose_stamped)

    def _cancel_goal(self, reason):
        self._set_operational_aclient.cancel_goal()
        self._timer.shutdown()
        self._timer = None
        rospy.loginfo(f"output module '%s': %s", self._name, reason)

    def updateHook(self, cmd_id, external_abort_request):
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

            # continue exection (ACTIVE, PENDING)
            return None

        # Controller is stopped.
        if status in (GoalStatus.REJECTED, GoalStatus.ABORTED):
            self._timer.shutdown()
            rospy.logerr("lookat output module: activation LookAt controller failed.")
            return "failed"
        elif status in (GoalStatus.SUCCEEDED, GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            self._timer.shutdown()
            rospy.loginfo("flexbe output module: stopped by external reason: %s.", status)
            return "succeed"

        # continue exection (RECALLING, PREEMPTING)
        return None

    def __deinit(self):
        # stop timer
        if self._timer is not None:
            self._timer.shutdown()
        # deactivate controller
        if self._set_operational_aclient.get_state() in (GoalStatus.ACTIVE, GoalStatus.PENDING):
            self._set_operational_aclient.cancel_goal()
        
OutputModulesLoader.register("look-at", LookAt)



        



