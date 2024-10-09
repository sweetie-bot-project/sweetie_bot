import stat
import os
import pytest
import subprocess as sp
import time

timeout=60

while timeout>0:
  cmd = f'timedatectl show'
  child = sp.Popen(cmd, shell=True, stdout=sp.PIPE)
  streamdata = child.communicate()[0]
  ntp_data = streamdata.decode().strip()
  ntp=dict()
  for x in ntp_data.split("\n"):
      k, v = x.split("=")
      ntp[k]=v
  #ntp = dict(x.split("=") for x in ntp_data.split("\n"))
  ntp = {k[:7]: v for k, v in ntp.items() if v == 'yes' or v == 'no' and k in ['CanNTP', 'NTP', 'NTPSynchronized']}
  values = [tuple(i[0]) for i in ntp.values()]
  if ntp['NTPSync'] == 'yes':
      break
  timeout-=1
  time.sleep(1)

class TestNTP:
    @pytest.mark.parametrize(
        ("value"), values,
        ids=list(ntp.keys())
    )
    def test_ntp(self, value):
        assert 'y' == value[0]
