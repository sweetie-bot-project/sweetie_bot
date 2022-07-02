class WME:
    def __init__(self, parent_id, attr, value):
        if isinstance(value, str):
            self._wme_id = parent_id.CreateStringWME(attr, value)
        elif isinstance(value, float):
            self._wme_id = parent_id.CreateFloatWME(attr, value)
        elif isinstance(value, int):
            self._wme_id = parent_id.CreateIntWME(attr, value)
        else:
            raise TypeError("WME value must be str, float or int")

    def update(self, value):
        old_value = self._wme_id.GetValue()
        if type(value) != type(old_value):
            raise TypeError("WME value mismatch")
        if old_value != value:
            self._wme_id.Update(value)
    
    def destroy(self):
        self._wme_id.DestroyWME()
        self._wme_id = None

    def get_id(self):
        return self._wme_id
