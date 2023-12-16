import stat
import os
import pytest
import subprocess as sp

class TestCam:
    @pytest.mark.parametrize(
        ("timeout"),
        [
            ("1000")
        ],
        ids=['cam']
    )
    def test_cam(self, timeout):
        cmd = f'/usr/bin/raspistill --timeout {timeout} --output /tmp/camera.jpg --preview 0,0,1600,800 --rotation 180'
        child = sp.Popen(cmd, shell=True, stdout=sp.PIPE)
        streamdata = child.communicate()[0]
        assert 0 == child.returncode
