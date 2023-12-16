import stat
import os
import pytest

class TestDev:
    @pytest.mark.parametrize(
        ("path"),
        [
            ("/dev/ttyAMA0"),
            ("/dev/ttyAMA1"),
            ("/dev/ttyAMA2"),
            ("/dev/ttyAMA3"),
            ("/dev/ttyAMA4"),
            ("/dev/i2c-1"),
            ("/dev/dri/card0"),
            ("/dev/dri/card1"),
            ("/dev/video0"),
            ("/dev/rtc0")
        ],
        ids=['AMA0',
             'AMA1',
             'AMA2',
             'AMA3',
             'AMA4',
             'i2c1',
             'drm0',
             'drm1',
             'video0',
             'rtc0']
    )
    def test_dev(self, path):
        if not os.path.exists(path):
            print("check config", end = '')
        mode = os.lstat(path).st_mode
        return stat.S_ISCHR(mode)
        

