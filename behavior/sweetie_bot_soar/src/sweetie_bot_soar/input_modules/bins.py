class BinsMap:
    def __init__(self, param):
        error = RuntimeError("BinsMap parameter must contains numerical list `values` and string list `names`.")
        # check provided configuration parameter
        if not isinstance(param, dict):
            raise error
        # extract names and values lists
        names = param.get('names')
        values = param.get('values')
        if not isinstance(names, list) or not isinstance(values, list):
            raise error
        if len(values) + 1 != len(names):
            raise RuntimeError("BinMap has inconsistent names and values lists lengths." % name)
        # check if parameters are correct
        v_prev = None
        for v in values:
            if not isinstance(v, (int, float)):
                raise error
            if v_prev != None and v_prev >= v:
                raise RuntimeError("BinMap values list must be ordered." % name)
        for n in names:
            if not isinstance(n, str):
                raise error
        # create bin map
        self._values = values
        self._names = names

    def __call__(self, value):
        k = 0
        for v in self._values:
            if v > value:
                break
            k = k + 1
        return self._names[k]
