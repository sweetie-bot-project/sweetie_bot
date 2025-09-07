from . import output_module
from ..input_modules.swm import ObjectKeyTuple, SpatialWorldModel

import rospy
import actionlib
from  actionlib import GoalStatus

from sweetie_bot_control_msgs.msg import SetOperationalAction, SetOperationalGoal, SetOperationalResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class LookAt(output_module.OutputModule):

    def __init__(self, config):
        super(LookAt, self).__init__("look-at")
        # module initialization
        action_ns = self.getConfigParameter(config, "controller", allowed_types = str)
        self._period = rospy.Duration( self.getConfigParameter(config, "period", allowed_types = float, default_value = 0.1) )
        # create actionlib client
        self._set_operational_aclient = actionlib.SimpleActionClient(action_ns, SetOperationalAction)
        # create publisher
        self._pose_pub = rospy.Publisher(action_ns + "/in_pose_ref", PoseStamped, queue_size = 1)
        # timer
        self._timer = None
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
        # start controller
        try:
            goal = SetOperationalGoal(operational = True, resources = ['head'])
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

    def updateHook(self, cmd_id):
        # check goal state
        status = self._set_operational_aclient.get_state()
        # Controller is active.
        if status in [ GoalStatus.ACTIVE, GoalStatus.PENDING ]:
            # Check for abort request
            abort_id = cmd_id.FindByAttribute("abort", 0)
            if abort_id is not None:
                # cancel goal and stop timer
                self._set_operational_aclient.cancel_goal()
                self._timer = None
                rospy.loginfo("lookat output module: behavior aborted by request")
                return "succeed"
            # check if object is missing
            if self._object_not_found:
                # cancel goal and stop timer
                self._set_operational_aclient.cancel_goal()
                self._timer = None
                rospy.loginfo("lookat output module: SWM object is not found")
                return "failed"
            # continue exection
            return None
        # Controller is stopped.
        self._timer = None
        if status == GoalStatus.REJECTED:
            rospy.logerr("lookat output module: activation LookAt controller failed.")
            return "failed"
        else:
            rospy.loginfo("flexbe output module: stopped by external reason: %s.", status)
            return "succeed"

    def __del__(self):
        # stop timer
        if self._timer is not None:
            self._timer.shutdown()
            self._timer.join()
        # deactivate controller
        if self._set_operational_aclient.get_state() in (GoalStatus.ACTIVE, GoalStatus.PENDING):
            self._set_operational_aclient.cancel_goal()
        
output_module.register("look-at", LookAt)



        



