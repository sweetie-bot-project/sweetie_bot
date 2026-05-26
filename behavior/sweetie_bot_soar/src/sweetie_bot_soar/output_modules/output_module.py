from ..config_parse import get_config_parameter
from ..module import ModulesLoader, UnlodableModule

class OutputModulesLoader(ModulesLoader):
    _loader_name = 'oputput module loader'
    _modules_registry = {}

class OutputModule(UnlodableModule):

    def __init__(self, name, config, *args, **kwargs):
        self._is_running = False
        self._cmd_timetag = None
        # unlodabale module constructor
        super(OutputModule, self).__init__(name, config, *args, **kwargs)

    # module interface

    def startHook(self, cmd_id):
        return None

    def updateHook(self, cmd_id, request_abort = False):
        raise NotImplementedError

    # module commom methods

    def getConfigParameter(self, config, name, *args, **kwargs):
        try:
            return get_config_parameter(config, name, *args, **kwargs)
        except (KeyError, ValueError, TypeError) as e:
            e.args = (f"input module '{self._name}'",) + e.args
            raise

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

