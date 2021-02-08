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

    def getConfigParameter(self, config, name, default_value = None, allowed_types = (str,), check_func = lambda v: True, error_desc = None):
        # get parameter
        value = config.get(name)
        # check if paramter is not specified and default_value exists
        if value == None and default_value != None:
            value = default_value
        # check if parameter value correct
        if value == None or not isinstance(value, allowed_types) or not check_func(value):
            if error_desc == None:
                error_desc = '`%s` input module: parameter %s is not present or invalid.' % (self._name, name)
            raise RuntimeError(error_desc)
        # return parater value
        return value
