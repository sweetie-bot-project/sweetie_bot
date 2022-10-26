# -*- coding: utf-8 -*-
import pytest
import warnings

import serial
from array import array
from .herkulex_io import HerkulexIO
from .herkulex_reg import HerkulexRegisters
import time

class TestHekulexServo:
    @pytest.mark.parametrize(
        ("limb", "port", "baud", "servo_ids"),
        [
            ("head", "/dev/ttyAMA2", 115200, [51,52,53,54]),
            ("ears", "/dev/ttyAMA2", 115200, [100,101,102,103]),
            ("leg1", "/dev/ttyAMA0", 115200, [11,12,13,14,15,16]),
            ("leg2", "/dev/ttyAMA0", 115200, [21,22,23,24,25,26]),
            ("leg3", "/dev/ttyAMA4", 115200, [31,32,33,34,35,36]),
            ("leg4", "/dev/ttyAMA4", 115200, [41,42,43,44,45,46])
        ],
        ids=['head',
             'ears',
             'leg1',
             'leg2',
             'leg3',
             'leg4']
    )
    def test_servo(self, limb, port, baud, servo_ids):
        herk = HerkulexIO(port, baud)
        cmd = 4 # stat packet
        addr = 54
        dlen = 2
        ret = True
        servos = ()
        #print(f"{limb} test started")
        for servo_id in servo_ids:
            inp = herk.write_packet(servo_id, cmd, (addr, dlen) )   # RAM_READ
            #print(herk.hexdump(inp, "<<") )

            # let's wait point one second before reading output (let's give device time to answer)
            time.sleep(0.1)
            out = herk.read_packet(servo_id, cmd, addr, dlen)
            #print( herk.hexdump(out, ">>") )

            data = out[7:] # cut header

            if data != []:
                regs =  HerkulexRegisters(cmd, data)
                regs.values[48] = regs.convert_voltage_raw_to_volts(regs.values[48])
                if regs.values[48] > 13:
                    warnings.warn(UserWarning(f"Overvoltage servo {servo_id} in {limb} {regs.values[48]}"))

                regs.values[49] = regs.convert_temperature_raw_to_celsius(regs.values[49])
                if regs.values[49] > 80:
                    warnings.warn(UserWarning(f"Overheat servo {servo_id} in {limb} {regs.values[49]}"))
                
                if regs.status_error_code > 0:
                    warnings.warn(UserWarning(f"Error on servo {servo_id} in {limb} {regs.status_error_text}"))
                
                #print( regs )
            else:
                seros.append(servo_id)
                ret = False

        if not ret:
             print(f"Servos {servos} is not responding!") #, end="")

        return ret

