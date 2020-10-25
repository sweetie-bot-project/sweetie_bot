_registered_input_modules_types = {}

def register(name, class_type):
    if name in _registered_input_modules_types:
        raise RuntimeError("Dublicate input module name: " + name)
    _registered_input_modules_types[name] = class_type

def load_modules(agent, input_link_config):
    input_modules = []
    # get input modules configuration from Parameter Server
    if not isinstance(input_link_config, dict):
        raise RuntimeError("Input link configuration is not valid.")
    # process configuration
    for module_name, module_config in input_link_config.items():
        module_type = _registered_input_modules_types.get(module_name)
        if module_type:
            input_modules.append( module_type(agent, module_config) )
        else: 
            raise RuntimeError("Input module {} type is unknown." % module_name)
    return input_modules
