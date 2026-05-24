_registered_input_modules_types = {}

def register(module_type, class_type):
    if module_type in _registered_input_modules_types:
        raise ValueError("Dublicate input module name: " + module_type)
    _registered_input_modules_types[module_type] = class_type

def load_modules(agent, input_link_config):
    input_modules = []
    # get input modules configuration from Parameter Server
    if not isinstance(input_link_config, dict):
        raise TypeError("input link configuration must be dictionary.")
    # process configuration
    for module_name, module_config in input_link_config.items():
        module_type = module_config.get("type")
        if module_type == None or not isinstance(module_type, str):
            raise TypeError(f"input module '{module_name}' does not have valid 'type' parameter.")
        module = _registered_input_modules_types.get(module_type)
        if module_type:
            input_modules.append( module(module_name, module_config, agent) )
        else: 
            raise ValueError(f"input module '{module_name}' type '{module_type}' is unknown.")
    return input_modules

def get_config_parameter(module, config, name, default_value = None, allowed_types = None, optional = False, check_func = lambda v: True, error_desc = ''):
    # check input config
    if allowed_types is None:
        if default_value is not None:
            allowed_types = type(default_value)
        else:
            raise ValueError(f"input module {module} get_config_parameter() for '{name}': 'default_value' or 'allowed_types' arguments must be supplied.")
    # get parameter
    value = config.get(name)
    # check if paramter is not specified and default_value exists
    if value is None:
        if default_value is not None:
            value = default_value
        else:
            if optional:
                return None
            else:
                raise KeyError(f"input module '{module}': parameter '{name}' must be supplied.")
    # check parameter type
    if not isinstance(value, allowed_types):
        raise TypeError(f"input module '{module}': parameter '{name}' must be one of following types: {allowed_types}")
    # and value
    if not check_func(value):
        raise ValueError(f"input module '{module}': parameter '{name}' value is invalid: {error_desc}")
    # return parameter value
    return value

class InputModule:
    def __init__(self, name, agent):
        self._name = name
        # create sensor WME
        input_link_id = agent.GetInputLink()
        self._sensor_id = input_link_id.CreateIdWME(name)

    @property
    def sensor_id(self):
        return self._sensor_id

    def getConfigParameter(self, *args, **kwargs):
        return get_config_parameter(self._name, *args, **kwargs)

    def __del__(self):
        # remove WME
        self._sensor_id.DestroyWME()


