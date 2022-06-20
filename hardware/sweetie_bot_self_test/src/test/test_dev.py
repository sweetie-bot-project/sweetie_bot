import stat
import os
import pytest

class TestDev:
    @pytest.mark.parametrize(
        ("path"),
        [
            ("/dev/ttyAMA2"),
            ("/dev/ttyAMA0"),
            ("/dev/ttyAMA4"),
            ("/dev/i2c-1"),
            ("/dev/dri/card0"),
            ("/dev/dri/card1"),
            ("/dev/video0"),
            ("/dev/rtc0")
        ],
        ids=['serh',
             'serf',
             'serb',
             'i2c1',
             'drm0',
             'drm1',
             'cam0',
             'rtc0']
    )
    def test_dev(self, path):
        if not os.path.exists(path):
            print("check config", end = '')
        mode = os.lstat(path).st_mode
        return stat.S_ISCHR(mode)
        

