import Python_sml_ClientInterface as sml

from . import input_modules
from . import output_modules

import os, sys
import rospy, rospkg
from std_srvs.srv import Trigger, TriggerResponse
from std_srvs.srv import SetBool, SetBoolResponse

def soarPrintCallback(mid, user_data, agent, message):
	rospy.loginfo("SOAR log: " + message.strip())

class Soar:
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

	def __del__(self):
		# remove all output modules
		self.input_modules.clear()
		self.output_modules_map.clear()
		self.active_output_modules.clear()

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
			self.agent.ExecuteCommandLine("production excise")
			# finished
			self.configured = False

		# input and output link initialization
		try:
			# load input modules (they are creating WME)
			input_link_config = rospy.get_param("~input")
			self.input_modules = input_modules.load_modules(self.agent, input_link_config)
			rospy.loginfo("Loaded %d input modules" % len(self.input_modules))
			# load output modules
			output_link_config = rospy.get_param("~output")
			self.output_modules_map = { m.getCommandName(): m for m in  output_modules.load_modules(output_link_config) }
			rospy.loginfo("Loaded %d output modules" % len(self.output_modules_map))
		except RuntimeError as e:
			rospy.logerr("SOAR configuration: input/output link initialization failed: " + str(e))
			return False
		except KeyError as e:
			rospy.logerr("SOAR configuration: input/output link initialization failed: " + str(e))
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

	def _update_io_link(self):
		# update input link
		for m in self.input_modules:
			m.update()

		# update output link
		output_link_id = self.agent.GetOutputLink()
		remove_list = []
		for m in self.active_output_modules:
			m.update(output_link_id)
			if not m.isRunning():
				remove_list.append(m)
		self.active_output_modules.difference_update( remove_list )

		# commit changes
		self.agent.Commit()

	def _process_output_commands(self):
		# process output link
		cmd_list = []
		for cmd_idx in range(self.agent.GetNumberCommands()):
			cmd_id = self.agent.GetCommand(cmd_idx)
			cmd_name = cmd_id.GetCommandName()
			cmd_list.append(cmd_name)
			# select corresponding module
			module = self.output_modules_map.get(cmd_name)
			if not module:
				# unknown command
				cmd_id.CreateStringWME("status", "error") 
				rospy.logerr("SOAR step: unknown command '%s' is skipped." % cmd_name)
				continue
			if module.isRunning() and cmd_id.GetTimeTag() != module.getTimeTag():
				# unknown outout module is busy
				cmd_id.CreateStringWME("status", "busy") 
				rospy.logerr("SOAR step: attemting to execute command '%s' before previous command instance is finished." % cmd_name)
				continue
			# start module 
			module.start(cmd_id)
			if module.isRunning():
				self.active_output_modules.add(module)
		# print executed commands list
		rospy.loginfo("SOAR commands: %s " % str(cmd_list))


	def step_io_link_update(self):
		""" Update input/output link. """
		#check if soar is configured
		if not self.configured:
			rospy.logerr("SOAR step: attempt to execute unconfigured enviroment")
			return False
		# check if soar kernel is ok
		if self.agent.HadError():
			rospy.logerr("SOAR skip step: " + agent.GetLastErrorDescription())
			return False
		# update io-link
		self._update_io_link()
		# process command 
		self._process_output_commands()


	def step(self, minor_step = False):
		""" Perform SOAR reasoning cycle. """
		#check if soar is configured
		if not self.configured:
			rospy.logerr("SOAR step: attempt to execute unconfigured enviroment")
			return False

		# update input link
		self._update_io_link()

		#start new major step: wait until output
		while True:
			# check soar kernel
			if self.agent.HadError():
				rospy.logerr("SOAR skip step: " + agent.GetLastErrorDescription())
				return False

			# check agent state
			if self.agent.GetRunState() in [sml.sml_RUNSTATE_HALTED, sml.sml_RUNSTATE_INTERRUPTED]:
				rospy.logerr("SOAR agent halted or interrupted!")
				return False

			# invoke SOAR for one step
			self.agent.RunSelf(1)

			# check for output
			if self.agent.GetNumberCommands() == 0:
				# check if we should exit
				if minor_step:
					return True
				# new reason cycle
				continue
			else:
				# minor step finished
				break

		# process output commands
		self._process_output_commands()

		# step finished succesfully
		return True


class SoarNode:
	def __init__(self, node_name):
		rospy.init_node(node_name)
		self.configured = False
		# create node interface 
		self.reconfigure_srv = rospy.Service('~reconfigure', Trigger, self.reconfigureCallback)
		self.reload_prod_srv = rospy.Service('~reload_prod', Trigger, self.reloadProdCallback)
		self.set_operational_srv = rospy.Service('~set_operational', SetBool, self.setOperationalCallback)
		self.trigger_operational_srv = rospy.Service('~toggle_operational', Trigger, self.triggerOperationalCallback)
		self.step_srv = rospy.Service('~step', Trigger, self.stepCallback)
		self.step_srv = rospy.Service('~io_update', Trigger, self.ioUpdateCallback)
		# create SOAR envelopment
		self.soar = Soar()
		self.timer = None
		self.period = 1.0
		# configure node
		self.reconfigure()

	def reconfigureCallback(self, req):
		success = self.reconfigure()
		return TriggerResponse(success = success)

	def reloadProdCallback(self, req):
		rospy.logerr('reload_prod service not implemented yet.')
		return TriggerResponse(success = False, message = 'Service is not implemented.')

	def setOperationalCallback(self, req):
		if not self.configured:
			return SetBoolResponse(success = False, message = 'Node is not configured.')
		# start/stop timer
		if req.data:
			# timer must be running
			if not self.timer:
				self.timer = rospy.Timer(rospy.Duration(self.period), self.timerCallback)
		else:
			# kill timer
			if self.timer:
				self.timer.shutdown()
				self.timer = None
		# success
		return SetBoolResponse(success = True)

	def triggerOperationalCallback(self, req):
		if not self.configured:
			return TriggerResponse(success = False, message = 'Node is not configured.')
                # toggle timer
		if self.timer:
			self.timer.shutdown()
			self.timer = None
		else:
                        self.timer = rospy.Timer(rospy.Duration(self.period), self.timerCallback)
		return TriggerResponse(success = True)

	def timerCallback(self, event):
		success = self.soar.step()
		if not success:
			rospy.logerr('Soar cycle timer stopped due to error.')
			self.timer.shutdown()
			self.timer = None

	def stepCallback(self, req):
		if not self.configured:
			return TriggerResponse(success = False, message = 'Node is not configured.')
		if self.timer:
			return TriggerResponse(success = False, message = 'Node is running.')
		# invoke step
		success = self.soar.step()
		if success:
			return TriggerResponse(success = True)
		else:
			return TriggerResponse(success = False, message = 'Errors during execution.')

	def ioUpdateCallback(self, req):
		if not self.configured:
			return TriggerResponse(success = False, message = 'Node is not configured.')
		if self.timer:
			return TriggerResponse(success = False, message = 'Node is running.')
		# update io-link
		self.soar.step_io_link_update()
		return TriggerResponse(success = True)

	def reconfigure(self):
		# reset configuration
		self.configured = False
		if self.timer:
			self.timer.shutdown()
			self.timer = None
		# read timer parameters
		self.period = rospy.get_param("~soar_period", 1.0)
		if not isinstance(self.period, (int,float)) or self.period < 0:
			rospy.logerr("`soar_period` parameter must be positive float number.")
			return False
		autostart = rospy.get_param("~autostart", True)
		if not isinstance(autostart, bool):
			rospy.logerr("`autostart` parameter must be boolean.")
			return False
		# reconfigure SOAR
		if not self.soar.reconfigure():
			rospy.logerr("SOAR configuration failed.")
			return False
		# if everything OK start timer
		if autostart:
			self.timer = rospy.Timer(rospy.Duration(self.period), lambda event: self.soar.step())
		# configuration is finished
		self.configured = True
		return True

def main():
	# SOAR initialization
	soar_node = SoarNode("soar")
	# ROS main loop
	rospy.spin()
