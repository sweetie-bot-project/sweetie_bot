#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
from array import array

class HerkulexIO:
    ser = None;
    def __init__(self, port, baud):
        # configure the serial connections (the parameters differs on the device you are connecting to)
        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )


    '''
    @brief Return {src} in hex dump.
    @param[in] length        {Int} Nb Bytes by row.
    @param[in] sep                {Char} For the text part, {sep} will be used for non ASCII char.
    @return {Str} The hexdump
    '''
    def hexdump(self, src, prefix='', length=16, sep='.' ):
        result = [];

        # Python3 support
        try:
                xrange(0,1);
        except NameError:
                xrange = range;

        for i in xrange(0, len(src), length):
                subSrc = src[i:i+length];
                hexa = '';
                isMiddle = False;
                for h in xrange(0,len(subSrc)):
                        if h == length/2:
                                hexa += ' ';
                        h = subSrc[h];
                        if not isinstance(h, int):
                                h = ord(h);
                        h = hex(h).replace('0x','');
                        if len(h) == 1:
                                h = '0'+h;
                        hexa += h+' ';
                hexa = hexa.strip(' ');
                text = '';
                for c in subSrc:
                        if not isinstance(c, int):
                                c = ord(c);
                        if 0x20 <= c < 0x7F:
                                text += chr(c);
                        else:
                                text += sep;
                result.append(('%s %08X:  %-'+str(length*(2+1)+1)+'s  |%s|') % (prefix, i, hexa, text));

        return '\n'.join(result);


    def XOR(self, input_list):
        """ XOR all elements in a list """
        if len(input_list) == 0: return 0

        def recXOR(index):
            if index == 0:
                return input_list[0]
            else:
                return input_list[index] ^ recXOR(index - 1)
        return recXOR(len(input_list)- 1)


    def write_packet(self, servo_id, cmd, data = ()):

        if (cmd == 2) and (data[0] + data[1] > 54):
          raise Exception('Wrong address or length')

        if (cmd == 4) and (data[0] + data[1] > 74):
          raise Exception('Wrong address or length')

        # Number of bytes following standard header: 0xff, 0xff, len, id, cmd, checksum1, checksum2
        length = 7 + len(data)  # 7 + data_len

        # directly from Herkulex manual:
        # Check Sum1 = (PacketSize ^ pID ^ CMD ^ Data[0] ^ Data[1] ^ ... ^ Data[n]) & 0xFE
        checksum1 = ((length ^ servo_id ^ cmd ^ self.XOR(data)) & 0xFE)

        # Check Sum2 = ( ~CheckSum1) & 0xFE
        checksum2 = (( ~checksum1 ) & 0xFE)


        # packet: FF FF LENGTH ID CMD CHECKSUM1 CHECKSUM2 DATA1 DATA2 ... DATAn
        packet = [0xFF, 0xFF, length, servo_id, cmd, checksum1, checksum2]
        packet.extend(data)

        packetStr = array('B', packet)
        self.ser.write(packetStr)
        return packetStr

    def read_packet(self, servo_id, cmd, data_addr=0, data_len=0):
        buf = []
        data = []

        while self.ser.inWaiting() > 0:
          buf.extend(self.ser.read(1))

        while len(buf) >= 9:
          try:
            if buf[0] == buf[1] == 0xFF: 
              if buf[6] != (( ~buf[5] ) & 0xFE): # checksum2
                raise Exception('Wrong checksum2')
              length = buf[2]
              if length > len(buf): # TODO check max packet length
                raise Exception('Wrong length')
              pid = buf[3]
              cmd_ack = buf[4]
              data = buf[7:length]
              if buf[5] != ((length ^ pid ^ cmd_ack ^ self.XOR(data)) & 0xFE): # checksum1
                raise Exception('Wrong checksum1')
              if pid != servo_id:
                print('Wrong servo_id')
                buf = buf[length:]
                continue
              if cmd_ack != (cmd | 0x40):
                print('Wrong cmd (skip packet)')
                buf = buf[length:]
                continue
              if cmd == 7 and len(data) != 2:
                print('Wrong ack stat packet (skip packet)')
                buf = buf[length:]
                continue
              if cmd in [2,4]:
                if data[0] != data_addr: # TODO max_data_addr
                  print('Wrong data addr (skip packet) %d %d' % data[0], data_addr)
                  buf = buf[length:]
                  continue
                if (data[1] != data_len):
                  print('Wrong data len (skip packet)')
                  buf = buf[length:]
                  continue
                if (len(data) != data_len+4):
                  print('Wrong ack read packet')
                  buf = buf[length:]
                  continue
              break  

            else: 
              raise Exception('Garbage')

          except Exception:
            pass

          # delete first char
          buf.pop(0)

        return buf # all ok! return packet data

