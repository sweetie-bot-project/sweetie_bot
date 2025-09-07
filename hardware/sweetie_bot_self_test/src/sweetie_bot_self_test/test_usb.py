import re
import pytest
import subprocess

device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
df = subprocess.check_output("lsusb")
df = df.decode("utf-8")
devices = []
for i in df.split('\n'):
    if i:
        info = device_re.match(i)
        if info:
            dinfo = info.groupdict()
            #dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
            devices.append(dinfo['id'])

class TestUSB:
    @pytest.mark.parametrize(
        ("dev_id"),
        [
            pytest.param(("2886:0018"),marks=pytest.mark.xfail(reason='no respeaker'))
        ],
        ids=[
             'respeaker'
            ]
    )
    def test_usb(self, dev_id):
        assert dev_id in devices
