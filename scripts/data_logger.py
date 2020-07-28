import serial
import os
import sys
import subprocess
import serial.tools.list_ports
import pdb
import struct
import numpy as np
from scipy.io.wavfile import write
import pcm_to_wav

if __name__ == '__main__':
  try:
    # Grab user input for serial port
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

    # Create output file
    f = open("output.txt","w+")
    trailingSample = None

    # Start streaming
    while True:
      data = ser.read(ser.in_waiting+2)

      # If there's a trailing sample prepend it to the data
      if (trailingSample):
        data.insert(0, trailingSample)

      for i in range (0, len(data), 2):
        if ((i+1) == len(data)):
          trailingSample = data[i]
          break

        trailingSample = None
        bits = str(data[i+1]) + str(data[i])
        sample = struct.unpack('>h', bits)[0]
        f.write(str(sample)+"\n")
        print(sample)

  # Cleanup
  except KeyboardInterrupt:
    ser.close()
    print("Writing to output.wav and exiting...")
    os.system("python pcm_to_wav.py output.txt")
    f.close()

