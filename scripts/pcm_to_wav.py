import scipy
import os
import matplotlib.pyplot as plt
import sys
import numpy as np
import pdb
from scipy.io.wavfile import write

# param filename: txt file containing raw pcm data
# param fs:       samping rate for conversion
# i.e: python pcm_to_wav.py <filename> <fs>
if __name__ == '__main__':
  filename = sys.argv[1]
  fs = int(sys.argv[2])
  output = os.path.splitext(filename)[0]+'.wav'

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
  write(output, fs, scaled)
