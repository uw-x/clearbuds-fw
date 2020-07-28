import scipy
import matplotlib.pyplot as plt
import sys
import numpy as np
import pdb

filename = sys.argv[1]
print("plotting " + filename + "...")

x = []

with open(filename) as f:
  lines = f.readlines()

  for line in lines:
    try:
      x.append((int(line.split()[0])))
    except:
      break

plt.plot(x)
plt.show()


# N = len(x)
# T = 1.0 / 50000
# y = x
# x = np.linspace(0.0, N*T, N)
# yf = np.fft.fft(y)
# xf = np.linspace(0.0, 1.0/(2.0*T), N/2)

# plt.plot(xf, 2.0/N * np.abs(yf[0:N/2]))
# plt.grid()
# plt.show()


# freqResponse = np.fft.fft(x)
# freqResponse = abs(freqResponse)
# plt.plot(freqResponse)
# plt.show()