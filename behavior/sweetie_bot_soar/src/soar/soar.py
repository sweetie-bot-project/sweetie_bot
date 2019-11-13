import Python_sml_ClientInterface as sml

import input_modules, output_modules

import os, sys
import rospy, rospkg

def soarPrintCallback(mid, user_data, agent, message):
    rospy.loginfo("SOAR log: " + message.strip())

class SoarNode:
    def __init__(self):
        # create SOAR kernel and agent
        self.kernel = sml.Kernel.CreateKernelInNewThread()
        self.agent = self.kernel.CreateAgent("agent")
        if self.agent.HadError():
            raise RuntimeError("SOAR kernel: " + self._kernel.GetLastErrorDescription())
        # register print callback
	self.agent.RegisterForPrintEvent(sml.smlEVENT_PRINT, soarPrintCallback, None)
        # perform configuration
        self.configured = False
        self.input_modules = list()
        self.output_modules_map = dict()
        self.active_output_modules = set()

    def reconfigure(self):
        """
            Reset SOAR and reload input and putput modules. 
        """
        # peform deconfiguration
        if self.configured:
            # destroy input and output modules
            self.input_modules.clear()
            self.output_modules_map.clear()
            self.active_output_modules.clear()
            # reset soar 
            self.agent.InitSoar()
            # clear production memory
            # TODO here?
            self.agent.ExecutionCommandLine("production excise")
            # finished
            self.configured = False

        # input and output link initialization
        try:
            # load input modules (they are creating WME)
            input_link_config = rospy.get_param("~input")
            self.input_modules = input_modules.load_modules(self.agent, input_link_config)
            # load output modules
            output_link_config = rospy.get_param("~output")
            self.output_modules_map = { m.getCommandName(): m for m in  output_modules.load_modules(output_link_config) }
            print self.output_modules_map
        except RuntimeError as e:
            rospy.logerr("SOAR configuration: input/output link initialization failed: " + str(e))
            return False
        except KeyError as e:
            rospy.logerr("SOAR configuration: input/output link initialization failed: " + str(e))
            return False
    
        # load reasoninig rules
        agent_pkg = rospy.get_param("~agent_package", None)
        agent_path = None
        if agent_pkg:
            try:
                agent_path = rospkg.RosPack().get_path(agent_pkg)
            except rospkg.ResourceNotFound:
                rospy.logerr("SOAR configuration: incorrect agent package name: " + agent_pkg)
                return False
        # get agent file name
        agent_file = rospy.get_param("~agent_file", None)
        if not agent_file:
            rospy.logerr("SOAR configuration: agent_file parameters are not supplied.")
            return False
        if agent_path:
            agent_file = os.path.join(agent_path, agent_file)
        # load productions
        self.agent.LoadProductions(agent_file)

        # check if everything ok
        if self.agent.HadError():
            rospy.logerr("SOAR configuration failed: " + self.agent.GetLastErrorDescription())
            return False

        self.configured = True
        return True

    def step(self):
        """ Perform SOAR reasoning cycle. """
        #check if soar is configured
        if not self.configured:
            rospy.logerr("SOAR step: attempt to execute unconfigured enviroment")
            return

        # check for error
        if self.agent.HadError():
            rospy.logerr("SOAR skip step: " + agent.GetLastErrorDescription())
            return

        rospy.loginfo("SOAR major step.")

        #start new major step: wait until output
        while True:
            # update input link
            for m in self.input_modules:
                m.update()
            self.agent.Commit()

            # update output link
            output_link_id = self.agent.GetOutputLink()
            remove_list = []
            for m in self.active_output_modules:
                m.update(output_link_id)
                if not m.isRunning():
                    remove_list.append(m)
            self.active_output_modules.difference_update( remove_list )

            # invoke SOAR for one step
            self.agent.RunSelf(1)
            if self.agent.HadError():
                rospy.logerr("SOAR step: " + self.agent.GetLastErrorDescription())
                return

            # debug output
            print(self.agent.ExecuteCommandLine("print S1 --depth 4").strip())

            # check for output
            if self.agent.GetNumberCommands() == 0:
                # new reason cycle
                rospy.loginfo("SOAR minor step.")
                continue
            else:
                # major step finished
                break

        #debug output
        #print(self.agent.ExecuteCommandLine("print S1 --depth 4").strip())

        # process output link
        nop_del_list = []
        for cmd_idx in range(self.agent.GetNumberCommands()):
            cmd_id = self.agent.GetCommand(cmd_idx)
            cmd_name = cmd_id.GetCommandName()
            # select corresponding module
            module = self.output_modules_map.get(cmd_name)
            if not module:
                # unknown command
                cmd_id.CreateStringWME("status", "error") 
                rospy.logerr("SOAR step: unknown command '%s' is skipped." % cmd_name)
                continue
            if module.isRunning():
                # unknown outout module is busy
                cmd_id.CreateStringWME("status", "busy") 
                rospy.logerr("SOAR step: attemting to execute command '%s' before previous command instance is finished." % cmd_name)
                continue
            # start module 
            module.start(cmd_id)
            if module.isRunning():
                self.active_output_modules.add(module)

def main():
    rospy.init_node("soar")

    # SOAR initialization
    soar = SoarNode()
    if not soar.reconfigure():
        rospy.logerr("SOAR configuration failed.")
        sys.exit()
    # start timer
    timer = rospy.Timer(rospy.Duration(1), lambda event: soar.step())

    rospy.spin()
