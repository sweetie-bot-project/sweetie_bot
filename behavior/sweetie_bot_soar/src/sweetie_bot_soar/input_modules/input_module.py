_registered_input_modules_types = {}

def register(module_type, class_type):
    if module_type in _registered_input_modules_types:
        raise RuntimeError("Dublicate input module name: " + module_type)
    _registered_input_modules_types[module_type] = class_type

def load_modules(agent, input_link_config):
    input_modules = []
    # get input modules configuration from Parameter Server
    if not isinstance(input_link_config, dict):
        raise RuntimeError("Input link configuration is not valid.")
    # process configuration
    for module_name, module_config in input_link_config.items():
        module_type = module_config.get("type")
        if module_type == None or not isinstance(module_type, str):
            raise RuntimeError("Input module %s does not have valid 'type' parameter." % module_name)
        module = _registered_input_modules_types.get(module_type)
        if module_type:
            input_modules.append( module(module_name, module_config, agent) )
        else: 
            raise RuntimeError("Input module %s type is unknown." % module_name)
    return input_modules

class InputModule:
    def __init__(self, name):
        self._name = name

    def getConfigParameter(self, config, name, default_value = None, allowed_types = None, check_func = lambda v: True, error_desc = None):
        # check input config
        if allowed_types is None:
            if default_value is not None:
                allowed_types = type(default_value)
            else:
                raise ValueError('getConfigParameter: default_value or allowed_types must be supplied.')
        # get parameter
        value = config.get(name)
        # check if paramter is not specified and default_value exists
        if value == None and default_value is not None:
            value = default_value
        # check if parameter value correct
        if not isinstance(value, allowed_types) or not check_func(value):
            if error_desc is None:
                error_desc = '`%s` input module: parameter %s is not present or invalid.' % (self._name, name)
            raise RuntimeError(error_desc)
        # return parater value
        return value

class InputModuleFlatSoarView(InputModule):
    def __init__(self, name, agent):
        super(InputModuleFlatSoarView, self).__init__(name)
        # create sensor WME
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)
        # create child id map
        self._child_ids = {}
    
    def updateChildWME(self, attrib, value):
        # bool to int
        if isinstance(value, bool):
            value = 1 if value else 0
        # check if corresponding child WME exists
        child_id = self._child_ids.get(attrib)
        if child_id == None:
            # create WME and add it to map
            if isinstance(value, int):
                self._child_ids[attrib] = self._sensor_id.CreateIntWME(attrib, value)
            elif isinstance(value, float):
                self._child_ids[attrib] = self._sensor_id.CreateFloatWME(attrib, value)
            elif isinstance(value, str):
                self._child_ids[attrib] = self._sensor_id.CreateStringWME(attrib, value)
            else:
                raise TypeError('SOAR attribute value must be int, float or string.')
        else:
            # update existing WME
            if child_id.GetValue() != value:
                child_id.Update(value)

    def removeChildWME(self, attrib):
        child_id = self._child_ids.get(attrib)
        if child_id != None:
            del self._child_ids[attrib]
            child_id.DestroyWME()

    def __del__(self):
        # remove WME
        self._sensor_id.DestroyWME()
