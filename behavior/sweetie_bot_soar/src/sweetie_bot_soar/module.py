
class ModulesLoader:
    _loader_name = 'module loader'
    _modules_registry = {}

    @classmethod 
    def register(cls, module_type, module_class):
        if module_type in cls._modules_registry:
            raise ValueError(f"{cls._loader_name}: dublicate module type name: {module_type}.")
        cls._modules_registry[module_type] = module_class

    @classmethod 
    def load(cls, config, *args, **kwargs):
        modules = []
        # check configuration paramrter
        if not isinstance(config, dict):
            raise TypeError("{cls._loader_name}: modules configuration must be dictionary.")
        try:
            # process configuration
            for module_name, module_config in config.items():
                module_type = module_config.get("type")
                if module_type == None or not isinstance(module_type, str):
                    raise TypeError(f"{cls._loader_name}: module '{module_name}' does not have valid 'type' parameter.")
                module_cls = cls._modules_registry.get(module_type)
                if module_cls:
                    modules.append( module_cls(module_name, module_config, *args, **kwargs) )
                else: 
                    raise ValueError(f"{cls._loader_name}: module '{module_name}' type '{module_type}' is unknown.")
        except Exception as e:
            # deinit configured modules
            for module in modules:
                module.deinit()
            # rethrow exception
            raise
        return modules

class UnlodableModule:
    # constructor
    def __init__(self, name, config, *args, **kwargs):
        self._name = name
        self._destroyed = False
        # call internal init
        try: 
            self._init(name, config, *args, **kwargs)
        except Exception as e:
            self.deinit()
            raise

    # explicit destructor
    def deinit(self):
        if self._destroyed:
            return
        # call parent internal deinit method
        try:
            self._deinit()
        except Exception as e:
            rospy.logerr(f"module '%s': deinitialization falied: %s", self._name, ': '.join(e.args))

    # module interface
    def _init(self, name, config, *args, **kwargs):
        raise NotImplementedError

    def _deinit(self):
        pass

    # methods
    @property
    def name(self):
        return self._name

