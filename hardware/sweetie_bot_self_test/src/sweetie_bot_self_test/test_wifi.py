import stat
import os
import pytest
import subprocess as sp
import socket, struct

def get_default_gateway_linux():
    """Read the default gateway directly from /proc."""
    with open("/proc/net/route") as fh:
        for line in fh:
            fields = line.strip().split()
            if fields[1] != '00000000' or not int(fields[3], 16) & 2:
                # If not default route or not RTF_GATEWAY, skip it
                continue
            return socket.inet_ntoa(struct.pack("<L", int(fields[2], 16)))

gw = get_default_gateway_linux()
cmd = 'iwgetid -r'
child = sp.Popen(cmd, shell=True, stdout=sp.PIPE)
streamdata = child.communicate()[0]
ssid = streamdata.decode().strip()

cmd = 'iwconfig wlan0 | grep "Link Quality="'
child = sp.Popen(cmd, shell=True, stdout=sp.PIPE)
streamdata = child.communicate()[0]
quality = streamdata.decode().strip()
if quality != "":
    quality_dict = dict(x.split("=") for x in quality.strip().split("  "))
    values = [(k,v) for k,v in quality_dict.items()]
else:
    quality_dict = {}
    values = []

cmd = "ip route get 1 | awk '{print $(NF-2);exit}'"
child = sp.Popen(cmd, shell=True, stdout=sp.PIPE)
streamdata = child.communicate()[0]
ip = streamdata.decode().strip()

hostname = socket.gethostname()

class TestNet:
    @pytest.mark.parametrize(
        ("value"),
        [
            (ssid)
        ],
        ids=['wlan0:'+ssid]
    )
    def test_wifi(self, value):
        assert '' != value
        cmd = f'ping -c 1 {gw}'
        child = sp.Popen(cmd, shell=True, stdout=sp.PIPE)
        streamdata = child.communicate()[0]
        assert 0 == child.returncode

    @pytest.mark.parametrize(
        ("key", "value"), values,
        ids=list(quality_dict.values())
    )
    def test_wifi_quality(self, key, value):
        if 'Link Quality' == key:
            (q, q_max) = value.split('/')
            assert int(q) / int(q_max) > 0.5
        if 'Signal level' == key:
            assert int(value.split(' ')[0]) > -67
        assert '' != value

    @pytest.mark.parametrize(
        ("value"),
        [
            (ip)
        ],
        ids=[ip]
    )
    def test_ip(self, value):
        assert '' != value

    @pytest.mark.parametrize(
        ("value"),
        [
            (hostname)
        ],
        ids=[hostname]
    )
    def test_hostname(self, value):
        assert '' != value

