#!/usr/bin/env python
# -*- coding: utf-8 -*-
from array import array
from prettytable import PrettyTable

class HerkulexRegisters(object):

    reg_number = range(1,61)
    values = [''] * 60
    reg_name = ["Model No1","Model No2","Version1","Version2","Baud Rate","Reserved","ID","ACK Policy","Alarm LED Policy","Torque Policy","Reserved","Max. Temperature","Min. Voltage","Max. Voltage","Acceleration Ratio","Max. Acceleration Time","Dead Zone","Saturatior Offset","Saturator Slope","PWM Offset","Min. PWM","Max. PWM","Overload PWM Threshold","Min. Position","Max. Position","Position Kp","Position Kd","Position Ki","Position FF 1st Gain","Position FF 2nd Gain","Reserved","Reserved","LED Blink Period","ADC Fault Check Period","Garbage Check Period","Stop Detection Period","Overload Detection Period","Stop Threshold","Inposition Margin","Reserved","Reserved","Calibration Difference","Status Error","Status Detail","Reserved","Reserved","Torque Control","LED Control","Voltage","Temperature","Current Control Mode","Tick","Calibrated Position","Absolute Position","Differential Position","PWM","Reserved","Absolute Goal Position","Absolute Desired Trajectory","Desired Velocity"]
    reg_size = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2]
    eep_addr = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,20,21,22,24,26,28,30,32,34,36,38,40,42,44,45,46,47,48,49,50,51,52,53,'','','','','','','','','','','','','','','','','','']
    ram_addr = ['','','','','','',0,1,2,3,4,5,6,7,8,9,10,11,12,14,15,16,18,20,22,24,26,28,30,32,34,36,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,60,62,64,66,68,70,72]
    status_error = {1:"Exceed Input Voltage limit", 2:"Exceed allowed POT limit", 4:"Exceed Temperature limit", 8:"Invalid Packet", 16:"Overload detected", 32:"Driver fault detected", 64:"EEP REG distorted", 128:"Reserved"}
    status_detail = {1:"Moving flag", 2:"Inposition flag", 4:"Checksum Error", 8:"Unknown Command", 16:"Exceed REG range", 32:"Garbage detected", 64:"MOTOR_ON flag", 128:"Reserved"}
    temperature_conversion_table = ( -7947,-7178,-6320,-5781,-5380,-5058,-4786,-4549,-4340,-4151,-3979,-3820,-3673,-3535,
		-3406,-3283,-3167,-3057,-2951,-2850,-2753,-2659,-2569,-2482,-2397,-2315,-2236,-2159,-2083,-2010,-1938,-1868,-1800,-1733,-1667,-1603,
		-1539,-1477,-1417,-1357,-1298,-1240,-1183,-1126,-1071,-1016,-962,-909,-856,-804,-753,-702,-652,-602,-553,-504,-456,-408,-361,-314,
		-267,-221,-175,-129,-84,-39,5,49,93,137,181,224,267,310,352,394,437,478,520,562,603,644,686,727,767,808,849,889,929,970,1010,1050,
		1090,1130,1170,1209,1249,1289,1328,1368,1407,1447,1486,1526,1565,1605,1644,1684,1723,1762,1802,1841,1881,1920,1960,1999,2039,2079,
		2119,2158,2198,2238,2278,2318,2359,2399,2439,2480,2520,2561,2602,2643,2684,2725,2766,2808,2850,2891,2933,2976,3018,3060,3103,3146,
		3189,3232,3276,3320,3364,3408,3453,3497,3542,3588,3633,3679,3725,3772,3818,3866,3913,3961,4009,4057,4106,4156,4205,4256,4306,4357,
		4409,4461,4513,4566,4619,4673,4728,4783,4839,4895,4952,5009,5068,5127,5186,5247,5308,5370,5433,5496,5561,5626,5693,5760,5828,5898,
		5968,6040,6113,6187,6263,6339,6417,6497,6578,6861,6746,6832,6920,7010,7102,7196,7292,7391,7492,7596,7703,7812,7925,8041,8160,8284,
		8411,8542,8679,8820,8966,9118,9276,9441,9613,9793,9982,10181,10390,10611,10845,11093,11359,11643,11949,12280,12641,13036,13472,
		13959,14509,15139,15873,16750,17829,19218,21132,24101,30091)

    def convert_voltage_raw_to_volts(self, raw):
        return raw * 0.074075

    def convert_temperature_raw_to_celsius(self, raw):
        if raw >= len(self.temperature_conversion_table):
            return 0.0;
        return self.temperature_conversion_table[raw] / 100.0

    def bitflag(self, byte, flags):
      rez = ''
      for flag in sorted(flags):
        if (flag & byte) == flag:
          rez += flags[flag]+','
      return rez[:-1]


    table = ''
    status_error_code = 0
    status_error_text = ''
    status_detail_code = 0
    status_detail_text = ''

    def __init__(self, cmd, raw_data):
        self.cmd = cmd
        self.packet = raw_data
        self.packet_length = len(raw_data)

        if self.packet_length in (0,1,3):
          raise Exception('Wrong packet data lenght %d' % self.packet_length)
        if self.packet_length == 2: # stat packet
          self.status_error_code = self.packet[0]
          self.status_error_text = self.bitflag(self.status_error_code, self.status_error)
          self.status_detail_code = self.packet[1]
          self.status_detail_text = self.bitflag(self.status_detail_code, self.status_detail)
        else:  # length >=4 (ack packet)
          self.status_error_code = self.packet[-2]
          self.status_error_text = self.bitflag(self.status_error_code, self.status_error)
          self.status_detail_code = self.packet[-1]
          self.status_detail_text = self.bitflag(self.status_detail_code, self.status_detail)

          addr = self.data_addr = self.packet[0]
          if self.cmd == 2:
            regs = self.eep_addr
          else: # ram
            regs = self.ram_addr

          self.data_length = self.packet[1]
          self.data = data = self.packet[2:-2]
         
          if len(self.data) != self.data_length:
            raise Exception('Wrong data lenght %d' % self.data_length)

          self.table = PrettyTable()
          self.table.start=81
          idx = -1
 
          while len(data) > 0:
            try:
              idx = regs.index(addr)
            except Exception:
              data.pop(0)
              addr+=1
              continue

            if self.table.start == 81:
              self.table.start=idx
            if self.reg_size[idx] == 1:
              self.values[idx] = data.pop(0)
              addr+=1
            if (self.reg_size[idx] == 2):
              if (len(data) > 1):
                self.values[idx] = data.pop(1)*255 + data.pop(0)
                addr+=2
              else:
                data.pop(0)
                addr+=1
                idx-=1

          
          #print(idx)
          self.table.end=idx+1

          #print('%d %d' % (self.table.start, self.table.end))

    def __repr__(self):
        if self.table != '':
          self.table.add_column("#",self.reg_number)
          self.table.add_column("Register name",self.reg_name)
          self.table.align["Register name"]="l"
          self.table.add_column("SIZE", self.reg_size)
          self.table.add_column("EEP", self.eep_addr)
          self.table.add_column("RAM", self.ram_addr)
          if self.cmd == 2:
            self.table.add_column("EEP Val", self.values)
          else:
            self.table.add_column("RAM Val", self.values)

          return '%s\n%s=%d (%s)\n%s=%d (%s)' % (self.table.get_string(), self.reg_name[42], self.status_error_code, self.status_error_text, self.reg_name[43], self.status_detail_code, self.status_detail_text)
        else:
          return '%s=%d (%s)\n%s=%d (%s)' % (self.reg_name[42], self.status_error_code, self.status_error_text, self.reg_name[43], self.status_detail_code, self.status_detail_text)


