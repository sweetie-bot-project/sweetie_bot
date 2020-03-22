SOAR integration package
-----------------------

This packed integrate [SOAR](https://soar.eecs.umich.edu/) cognitive architecture into SweetieBot project
using python version of Soar Markup Language (SML). It contains node which executes SOAR agent and input and output
modules which implement SOAR agent input and output link.

### Installation

Package depends on SOAR libraries and python SML package which should be installed in `/opt/soar` directory.

### Package content

#### SOAR node

SOAR node loads SOAR agent and input/output modules and then starts reasoning cycle.

1. Invoke input modules which update SOAR agent input-link WMEs.
2. Invoke active output modules which may update output-link WMEs. Active output modules represents 
	prolonged actions performed by robot. After action is finished coresponding output module adds
	result WME to output-link and change its state to inactive.
3. Perform SOAR reasoning cycle. 
4. If no new command is added to output-link then go to 1 and start new minor cycle. 
5. Activate output module responding to output-link command.
6. Wait for timer and go to 1 to start new major cycle.

Configuration parameters:

* `~agent_package` (`string`), `~agent_file` (`string`) --- SOAR productions files to be loaded.
* `~soar_period` (`string`) --- major reasoning cycle period.
* `~autostart` (`bool`) --- start reason cycle after component is loaded.
* `~input` --- input modules configurations in format `{ <module_name>: { <parameter1>: <value1>, <parameter2>: <value2> ... }, ... }.
	Note that only modules mentioned in map are loaded.
* `~output` --- output modules configurations in format `{ <module_name>: { <parameter1>: <value1>, <parameter2>: <value2> ... }, ... }.
	Note that only modules mentioned in map are loaded.

Services:

* `~reconfigure` (`std_srvs/Trigger`) --- perform full reinitialization: SOAR agent, production rules, input and output modules configuration.
* `~reload_prod` (`std_srvs/Trigger`) --- reload production rules.
* `~set_operational` (`std_srvs/SetBool`) --- start/stop reason cycle.

#### Input modules

    * `rand`. Provides random number.
        Input-link WME:

              <input-link>
                ^rand <rnd> --- random value updated each cycle.
       
    * `clock`. Maintain realtime clock and cycle counter: 
         Input-link WME:
		
              <input-link>
                ^clock
                  ^cycle <cycle> --- current minor cycle number.
                   ^time <time> --- UNIX time in seconds.

    * `joystick`. Detects pressed joystick keys. Listen to `sweetie_bot_joystick/KeyPressed` messages.
         Input-link WME:
		 
			  <input-link>
                ^joystick
                  ^pressed <key1> --- key name as present in `KeyPressed` message.
                  ^pressed <key2> --- key name as present in `KeyPressed` message.
			      ...

         Configuation parameteres:
         * `topic`(`string`) --- sweetie_bot_joystick/KeyPressed topic name.

### Output modules

    * `nop`. Do nothing.
         Output-link command:
		 
			  <output-link>
                ^nop *

         Configuation parameteres:
         * `topic`(`string`) --- sweetie_bot_text_msgs/TextCommand topic name.

    * `textcmd`. Send text command to specified topic.
         Output-link command:
		 
			  <output-link>
                ^textcmd
                  ^type <type> --- type field of TextCommand
                  ^command <cmd> --- command field of TextCommand
                  ^status <s> --- result: completed|error.

         Configuation parameteres:
         * `topic`(`string`) --- sweetie_bot_text_msgs/TextCommand topic name.
       
    * `flexbe`. Execute flexbe behavior.
         Output-link command:
		 
			  <output-link>
                ^flexbe
                  ^param --- behavior parameters
                    ^<parameter1> <value1> --- parameter value
                    ...
                  ^input --- input keys of behavior
                    ^<key1> <value1> --- key values
                    ...
                  ^status <s> --- command result: completed|error.
                  ^outcome <cmd> --- behavior outcome (if reached)

         Configuation parameteres:
         * `action_ns`(`string`) --- flexbe_msgs/BehaviorExecution action.

