import evdev
import pytest

class TestInput:
    @pytest.mark.parametrize(
        ("dev"),
        [
            ({"path":"/dev/input/event0", "name":"CAP11XX capacitive touch sensor"})
        ],
        ids=[
             'cap11xx',
            ]
    )
    def test_input(self, dev):
        device = evdev.InputDevice(dev['path'])
        assert device.name == dev['name']
        return True
