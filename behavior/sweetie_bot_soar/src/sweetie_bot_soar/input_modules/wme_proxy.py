class SingleValueWMEProxy:
    def __init__(self, parent_id, attr, value):
        # convert bool to int
        if isinstance(value, bool):
            value = 1 if value else 0
        # create WME of the specific type
        if isinstance(value, str):
            self._wme_id = parent_id.CreateStringWME(attr, value)
        elif isinstance(value, float):
            self._wme_id = parent_id.CreateFloatWME(attr, value)
        elif isinstance(value, int):
            self._wme_id = parent_id.CreateIntWME(attr, value)
        else:
            raise TypeError("WME value must be str, float or int")

    def update(self, value):
        # convert bool to int
        if isinstance(value, bool):
            value = 1 if value else 0
        # check value and update WME
        old_value = self._wme_id.GetValue()
        if type(value) != type(old_value):
            raise TypeError("WME value type mismatch")
        if old_value != value:
            self._wme_id.Update(value)
            return True
        else:
            return False
    
    def destroy(self):
        self._wme_id.DestroyWME()
        self._wme_id = None

    def getId(self):
        return self._wme_id

class SetWMEProxy:
    def __init__(self, parent_id, attr, values = []):
        # create empty set of WMEs
        self._parent_id = parent_id
        self._wmes = {}
        self._attr = attr
        # update value
        self.update(values)

    def _createWME(self, value):
        if isinstance(value, str):
            return self._parent_id.CreateStringWME(self._attr, value)
        elif isinstance(value, float):
            return self_parent_id.CreateFloatWME(self._attr, value)
        elif isinstance(value, int):
            return self_parent_id.CreateIntWME(self._attr, value)
        else:
            raise TypeError("WME value must be str, float or int")

    def values(self):
        return self._wmes.keys()

    def add(self, value):
        if value not in self._wmes:
            self._wmes[value] = self._createWME(value)
            return True
        else:
            return False

    def discard(self, value):
        wme_id = self._wmes.get(value)
        if wme_id is not None:
            wme_id.DestroyWME()
            del self._wmes[value]
            return True
        else:
            return False

    def remove(self, value):
        wme_id = self._wmes.get(value)
        if wme_id is not None:
            wme_id.DestroyWME()
            del self._wmes[value]
        else:
            raise KeyError(f"WME set {self._attr} does not contains {self._value}")

    def set_present(self, value, present):
        if present:
            self.add(value)
        else:
            self.discard(value)

    def update(self, values):
        changed = False
        for value in values: 
            if self.add(value):
                changed = True
        return True

    def difference_update(self, values):
        changed = False
        for value in values: 
            if self.discard(value):
                changed = True
        return changed

    def assign(self, values):
        # compare with new values
        addlist = values - self._wmes.keys()
        dellist = self._wmes.keys() - values
        # add or remove new wmes
        for value in addlist:
            self._wmes[value] = self._createWME(value)
        for value in dellist:
            wme_id = self._wmes[value]
            wme_id.DestroyWME()
            del self._wmes[value]
        # return True if anything has changed
        return len(addlist) != 0 or len(dellist) != 0

    def clear(self):
        for wme_id in self._wmes.values():
            wme_id.DestroyWME()
        self._wmes.clear()








