import Python_sml_ClientInterface as sml

from . import input_modules
from . import output_modules

import os, threading
import rospy, rospkg, tf
from enum import Enum

class SoarState(Enum):
    UNCONFIGURED = 0
    STOPPED = 1
    RUNNING = 2
    ERROR = 3

class Soar:
    def __init__(self):
        # create SOAR kernel and agent
        self._kernel = sml.Kernel.CreateKernelInNewThread()
        self._agent = self._kernel.CreateAgent("agent")
        if self._agent.HadError():
            raise RuntimeError("SOAR kernel: " + self._agent.GetLastErrorDescription())
        # register  callback
        self._agent.RegisterForPrintEvent(sml.smlEVENT_PRINT, Soar.printCallback, self)
        self._kernel.RegisterForSystemEvent(sml.smlEVENT_SYSTEM_START, Soar.startCallback, self) 
        self._kernel.RegisterForSystemEvent(sml.smlEVENT_SYSTEM_STOP, Soar.stopCallback, self) 
        self._kernel.RegisterForUpdateEvent(sml.smlEVENT_AFTER_ALL_OUTPUT_PHASES, Soar.updateCallback, self) 
        # perform configuration
        self._lock_cond = threading.Condition() # condition variable to protect 
        self._state = SoarState.UNCONFIGURED
        self._stop_request = False
        self._update_timestamp = rospy.Time()
        self._update_period = 0.1
        self._input_modules = list()
        self._output_modules_map = dict()
        self._active_output_modules = set()

    def __del__(self):
        # remove all output modules
        self._input_modules.clear()
        self._output_modules_map.clear()
        self._active_output_modules.clear()

    def checkNoErrors(self):
        # check soar kernel
        if self._kernel.HadError():
            rospy.logerr("SOAR kernel error: " + agent.GetLastErrorDescription())
            return False

        return True

    def printCallback(event_id, self, agent, message):
        rospy.loginfo("SOAR log: " + message.strip())

    def startCallback(event_id, self, kernel):
        # called in SOAR stream when kernel is started
        with self._lock_cond:
            self._state = SoarState.RUNNING
            self._lock_cond.notify()
        # log status
        rospy.loginfo("SOAR kernel is started.")

    def stopCallback(event_id, self, kernel):
        # called in SOAR stream when kernel is started
        with self._lock_cond:
            self._state = SoarState.STOPPED
            self._lock_cond.notify()
        # log status
        if self.checkNoErrors():
            rospy.loginfo("SOAR kernel is stopped.")

    def updateCallback(event_id, self, kernel, run_flags):
        with self._lock_cond:
            # check if SOAR kernel is running
            if self._state not in [ SoarState.RUNNING, SoarState.STOPPED ]:
                return

            # check for stop request
            if self._stop_request:
                self._kernel.StopAllAgents()
                self._stop_request = False

            # check if update should be performed
            update_disabled = (run_flags & sml.sml_DONT_UPDATE_WORLD)
            update_forced = (run_flags & sml.sml_UPDATE_WORLD)
            update_delay_expired  = (rospy.Time.now() - self._update_timestamp).to_sec() > self._update_period
            new_output_commands = self._agent.GetNumberCommands() != 0

            if not update_disabled:
                if update_forced or update_delay_expired:
                    # update input link
                    self._update_io_link()
                    # renew update timestamp
                    self._update_timestamp = rospy.Time.now()

                if new_output_commands:
                    # process output commands
                    self._process_output_commands()

    def start(self):
        with self._lock_cond:
            # check if kernel is configured
            if self._state != SoarState.STOPPED:
                return False
            # start all agents: RunAllAgentsForever() blocks until kernel is stopped so run in new thread
            run_thread = threading.Thread(target = self._kernel.RunAllAgentsForever)
            run_thread.start()
            # wait until state has changed (hardcoded timeout 10 seconds)
            self._lock_cond.wait_for(lambda: self._state != SoarState.STOPPED, 10)
            return self._state == SoarState.RUNNING

    def stop(self):
        with self._lock_cond:
            # check if kernel is configured
            if self._state != SoarState.RUNNING:
                return False
            # request stop
            self._stop_request = True
            # wait until state has changed (hardcoded timeout 10 seconds)
            self._lock_cond.wait_for(lambda: self._state != SoarState.RUNNING, 10)
            return self._state == SoarState.RUNNING

    def cleanup(self):
        # peform deconfiguration
        with self._lock_cond:
            if self._state != SoarState.STOPPED:
                return False

            # destroy input and output modules
            self._input_modules.clear()
            self._output_modules_map.clear()
            self._active_output_modules.clear()
            # reset soar 
            self._agent.InitSoar()
            # clear production memory
            self._agent.ExecuteCommandLine("production excise")

            # finished
            self._state = SoarState.UNCONFIGURED
            return True

    def configure(self):
        with self._lock_cond:
            if self._state != SoarState.UNCONFIGURED:
                return False

            # get configuration parameters
            self._update_period = rospy.get_param("~update_period", 0.1)
            if not isinstance(self._update_period, (float, int)) or self._update_period < 0:
                rospy.logerr("update_period parameter must be positive number.")
                return False

            # input and output link initialization
            try:
                # load input modules (they are creating WME)
                input_link_config = rospy.get_param("~input")
                self._input_modules = input_modules.load_modules(self._agent, input_link_config)
                rospy.loginfo("Loaded %d input modules" % len(self._input_modules))
                # load output modules
                output_link_config = rospy.get_param("~output")
                self._output_modules_map = { m.getCommandName(): m for m in  output_modules.load_modules(output_link_config) }
                rospy.loginfo("Loaded %d output modules" % len(self._output_modules_map))
            except RuntimeError as e:
                rospy.logerr("SOAR configuration: input/output link initialization failed: " + str(e))
                return False
            except KeyError as e:
                rospy.logerr("SOAR configuration: input/output link initialization failed: " + str(e))
                return False
            except tf.Exception as e:
                rospy.logerr("SOAR configuration: tf exception: " + str(e))
                return False
            except rospy.exceptions.ROSException as e:
                rospy.logerr("SOAR configuration: ROS exception: " + str(e))
                return False
        
            # load reasoning rules
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
                rospy.logerr("SOAR configuration: agent_file parameter are not supplied.")
                return False
            if agent_path:
                agent_file = os.path.join(agent_path, agent_file)
            # load productions
            self._agent.LoadProductions(agent_file)

            # check if everything ok
            if self._agent.HadError():
                rospy.logerr("SOAR configuration failed: " + self._agent.GetLastErrorDescription())
                return False

            self._state = SoarState.STOPPED
            return True

    def getState(self):
        return self._state

    def _update_io_link(self):
        # update input link
        for m in self._input_modules:
            m.update()

        # update output link
        output_link_id = self._agent.GetOutputLink()
        remove_list = []
        for m in self._active_output_modules:
            m.update(output_link_id)
            if not m.isRunning():
                remove_list.append(m)
        self._active_output_modules.difference_update( remove_list )

        # commit changes
        self._agent.Commit()

    def _process_output_commands(self):
        # process output link
        cmd_list = []
        for cmd_idx in range(self._agent.GetNumberCommands()):
            cmd_id = self._agent.GetCommand(cmd_idx)
            cmd_name = cmd_id.GetCommandName()
            cmd_list.append(cmd_name)
            # select corresponding module
            module = self._output_modules_map.get(cmd_name)
            if not module:
                # unknown command
                cmd_id.CreateStringWME("status", "error") 
                rospy.logerr("SOAR step: unknown command '%s' is skipped." % cmd_name)
                continue
            if module.isRunning() and cmd_id.GetTimeTag() != module.getTimeTag():
                # output module is busy
                cmd_id.CreateStringWME("status", "busy") 
                rospy.logerr("SOAR step: attemting to execute command '%s' before previous command instance is finished." % cmd_name)
                continue
            # start module 
            module.start(cmd_id)
            if module.isRunning():
                self._active_output_modules.add(module)
        # print executed commands list
        if any([cmd != 'nop' for cmd in cmd_list]):
            rospy.loginfo("SOAR output commands: %s " % str(cmd_list))

    def step(self, minor_step = False):
        """ Perform SOAR reasoning cycle. """
        if not self.checkNoErrors():
            return False

        # check agent state
        if self._agent.GetRunState() in [sml.sml_RUNSTATE_HALTED, sml.sml_RUNSTATE_INTERRUPTED]:
            rospy.logwarn("SOAR agent halted or interrupted!")
            return False

        with self._lock_cond:
            #check if soar is configured
            if self._state != SoarState.STOPPED:
                rospy.logerr("SOAR step: attempt to execute unconfigured enviroment")
                return False

            # update input link
            self._update_io_link()

        # invoke SOAR for one step
        self._agent.RunSelfTilOutput()

        # step finished succesfully
        return True

