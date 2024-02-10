import smbus
import errno
import pytest

class TestI2CDev:
    bus = smbus.SMBus(1)
    @pytest.mark.parametrize(
        ("dev_id"),
        [
            (0x68)
            #(0x2b)
        ],
        ids=['rtc'
             #'touch'
            ]
    )
    def test_i2c(self, dev_id):
        try:
            ret = self.bus.read_byte(dev_id)
        except OSError as e:
                if e.errno == errno.EBUSY:
                    # taken by the kernel or something else
                    return True
                else:
                    print('i2c device:', hex(dev_id), e, end="")
                    raise
        return True
