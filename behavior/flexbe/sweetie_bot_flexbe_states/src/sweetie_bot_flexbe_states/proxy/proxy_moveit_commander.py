import rospy
import moveit_commander

class ProxyMoveitCommander(object):
    """
    Proxy object for moveit_commander classes. 
    It provides access to RobotCommander, PlanningScene and MoveGroupCommander.
    """
    _is_initialized = False
    _robot = None
    _scene = None
    _move_groups = {}

    def __init__(self):
        if not self._is_initialized:
            moveit_commander.roscpp_initialize({})
            self._is_initialized = True

    #def __del__(self):
        #if self._is_initialized:
            #moveit_commander.roscpp_shutdown()

    def getRobotCommander(self):
        """
        Get RobotCommander object.
        :return: RobotCommander object.
        """
        if not self._robot:
            self._robot = moveit_commander.RobotCommander()
        return self._robot

    def getPlanningScene(self):
        """
        Get PlanningSceneInterface object.
        """
        if not self._scene:
            self._scene = moveit_commander.PlanningSceneInterface()
        return self._robot

    def getMoveGroupCommander(self, group_name):
        """
        Get MoveGroupCommander object.
        :param group_name: move group name.
        """
        if group_name not in self._move_groups:
            group = moveit_commander.MoveGroupCommander(group_name)
            self._move_groups[group_name] = group
        return self._move_groups[group_name]

