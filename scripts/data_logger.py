import serial
import os
import sys
import subprocess
import serial.tools.list_ports
import pdb
import struct

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

ports = serial.tools.list_ports.comports()

i = 1
ports = sorted(ports)
for port, desc, hwid in ports:
  print("Serial device ({}) : {}".format(i, port))
  i += 1

port = input("Select a device by its number:")

try:
  port = int(port)
except ValueError:
  print("Invalid port number")

port -= 1 # offset by 1

if (port >= len(ports)) or (port < 0):
  print("Invalid port number")
  exit

portName = ports[port].device
ser = serial.Serial(portName)
rl = ReadLine(ser)

while True:
  data = rl.readline()

  for i in range (0, len(data)-1, 2):
    print(struct.unpack('>h', data[i:(i+2)])[0])
