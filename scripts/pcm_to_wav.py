import scipy
import matplotlib.pyplot as plt
import sys
import numpy as np
import pdb
from scipy.io.wavfile import write

if __name__ == '__main__':
  filename = sys.argv[1]

  data = []

  with open(filename) as f:
    lines = f.readlines()

    for line in lines:
      try:
        data.append((int(line.split()[0])))
      except:
        break

  # scaled = np.int16(data/np.max(np.abs(data)) * 32767)
  scaled = np.int16(data)
  write('output.wav', 50000, scaled)