_registered_output_modules_types = {}

def register(name, class_type):
    if name in _registered_output_modules_types:
        raise RuntimeError("Dublicate input module name: " + name)
    _registered_output_modules_types[name] = class_type

def load_modules(output_link_config):
    output_modules = []
    # get output modules configuration from Parameter Server
    if not isinstance(output_link_config, dict):
        raise RuntimeError("Output link configuration parameters tree is not supplied.")
    # process configuration
    for module_name, module_config in output_link_config.items():
        module_type = _registered_output_modules_types.get(module_name)
        if module_type:
            output_modules.append( module_type(module_config) )
        else: 
            raise RuntimeError("Output module %s type is unknown." % module_name)
    return output_modules

class OutputModule(object):
    def __init__(self, name):
        self._is_running = False
        self._name = name
        self._cmd_timetag = None

    def start(self, cmd_wme_id):
        # execute start hook
        state = self.startHook(cmd_wme_id)
        if state:
            self._finish(cmd_wme_id, state)
            return False
        # preserve command time tag for future access
        self._cmd_timetag = cmd_wme_id.GetTimeTag()
        # set running flag
        self._is_running = True
        return True

    def update(self, output_link_id):
        # check if mdule is active
        if not self._is_running:
            return False
        # find command WME because it can be changed or deleted
        cmd_index = 0
        while (True):
            cmd_id = output_link_id.FindByAttribute(self._name, cmd_index)
            if not cmd_id:
                # nothing found: abort execution
                state = self.abortHook()
                if state:
                    self._is_running = False
                return self._is_running
            # check if timetag is the same
            if cmd_id.GetTimeTag() == self._cmd_timetag and cmd_id.IsIdentifier():
                # command found
                cmd_id = cmd_id.ConvertToIdentifier()
                break
            # continue search
            cmd_index += 1

        # update command state
        state = self.updateHook(cmd_id)
        if state:
            self._finish(cmd_id, state)
            return False
        # continue execution
        return True

    def getCommandName(self):
        return self._name

    def getTimeTag(self):
        return self._cmd_timetag

    def isReady(self):
        return not self._is_running

    def isRunning(self):
        return self._is_running

    def _finish(self, cmd_id, state):
        cmd_id.CreateStringWME("status", state)
        self._is_running = False

    def startHook(self, cmd_id):
        return None

    def updateHook(self, cmd_id):
        return "succeed"

    def abortHook(self):
        pass

