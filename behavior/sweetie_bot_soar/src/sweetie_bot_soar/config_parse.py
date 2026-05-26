def get_config_parameter(config, name, default_value = None, allowed_types = None, optional = False, check_func = lambda v: True, error_desc = ''):
    # check input config
    if allowed_types is None:
        if default_value is not None:
            allowed_types = type(default_value)
        else:
            raise ValueError(f"get_config_parameter() for '{name}': 'default_value' or 'allowed_types' arguments must be supplied.")
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
                raise KeyError(f"parameter '{name}' must be supplied.")
    # check parameter type
    if not isinstance(value, allowed_types):
        raise TypeError(f"parameter '{name}' must be one of following types: {allowed_types}")
    # and value
    if not check_func(value):
        raise ValueError(f"parameter '{name}' value is invalid: {error_desc}")
    # return parameter value
    return value
