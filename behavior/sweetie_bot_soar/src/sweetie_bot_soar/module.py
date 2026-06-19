import rospy

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
        self._subscribers = []
        self._timers = []
        # call internal init
        try: 
            self._init(name, config, *args, **kwargs)
        except Exception as e:
            self.deinit()
            raise

    # properties

    @property
    def name(self):
        return self._name

    @property
    def is_destroyed(self):
        return self._destroyed

    # resource managment methods

    def createSubscriber(self, *args, **kwargs):
        sub = rospy.Subscriber(*args, **kwargs)
        self._subscribers.append(sub)
        return sub

    def removeSubscriber(self, sub):
        self._subscribers.remove(sub)
        sub.unregister()

    def createTimer(self, *args, **kwargs):
        timer = rospy.Timer(*args, **kwargs)
        self._timers.append(timer)
        return timer

    def removeTimer(self, timer):
        self._timers.remove(timer)
        timer.shutdown()

    # explicit destructor

    def deinit(self):
        for sub in self._subscribers:
            sub.unregister()
        for timer in self._timers:
            timer.shutdown()
        self._destroyed = True

    # module interface

    def _init(self, name, config, *args, **kwargs):
        raise NotImplementedError

