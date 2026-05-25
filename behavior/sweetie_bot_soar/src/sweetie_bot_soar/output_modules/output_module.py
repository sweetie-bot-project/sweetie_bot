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

    def getConfigParameter(self, config, name, default_value = None, allowed_types = None, check_func = lambda v: True, error_desc = None):
        # check input config
        if allowed_types is None:
            if default_value is not None:
                allowed_types = type(default_value)
            else:
                raise ValueError('`%s` output module:  default_value or allowed_types must be supplied.' % self._name)
        # get parameter
        value = config.get(name)
        # check if paramter is not specified and default_value exists
        if value == None and default_value is not None:
            value = default_value
        # check if parameter value correct
        if not isinstance(value, allowed_types) or not check_func(value):
            if error_desc is None:
                error_desc = '`%s` output module: parameter %s is not present or invalid.' % (self._name, name)
            raise ValueError(error_desc)
        # return parater value
        return value

    def start(self, cmd_wme_id):
        if self._is_running:
            raise ValueError(f"output module '{self._name}': attempting to start running module")
        # execute start hook
        result = self.startHook(cmd_wme_id)
        if result is not None:
            cmd_wme_id.CreateStringWME('status', result)
            return False
        # preserve command time tag for future access
        self._cmd_timetag = cmd_wme_id.GetTimeTag()
        # set running flag
        self._is_running = True
        return True

    def update(self, output_link_id, abort_request = None):
        # check if module is active
        if not self._is_running:
            return False

        # find command WME because it can be changed or deleted
        cmd_index = 0
        while (True):
            cmd_id = output_link_id.FindByAttribute(self._name, cmd_index)
            if not cmd_id:
                # nothing found
                break
            # check if timetag is the same
            if cmd_id.GetTimeTag() == self._cmd_timetag and cmd_id.IsIdentifier():
                # command found
                cmd_id = cmd_id.ConvertToIdentifier()
                break
            # continue search
            cmd_index += 1

        # update output module state
        result = self.updateHook(cmd_id, abort_request)
        if result is not None:
            # execution is finished
            self._is_running = False
            if cmd_id is not None:
                cmd_id.CreateStringWME('status', result)
            return False
        else:
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

    # module interface

    def startHook(self, cmd_id):
        return None

    def updateHook(self, cmd_id, request_abort = False):
        raise NotImplementedError


