import stat
import os
import pytest
import subprocess as sp
import socket, struct

cmd = f'timedatectl show'
child = sp.Popen(cmd, shell=True, stdout=sp.PIPE)
streamdata = child.communicate()[0]
ntp_data = streamdata.decode().strip()
ntp = dict(x.split("=") for x in ntp_data.split("\n"))
ntp = {k[:8]: v for k, v in ntp.items() if v == 'yes' or v == 'no'}
values = [tuple(i[0]) for i in ntp.values()]

class TestNTP:
    @pytest.mark.parametrize(
        ("value"), values,
        ids=list(ntp.keys())
    )
    def test_ntp(self, value):
        assert 'y' == value[0]
