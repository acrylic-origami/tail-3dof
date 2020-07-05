import matplotlib.pyplot as plt
import numpy as np

plt.ion()
plt.close('all')
plt.show()

A = np.random.rand(1024) - 0.5
B = [0]
for a in A:
	B.append((a * 256 + B[-1] * 250) / 1270)
	
plt.plot(B)
# ax = plt.gca()
# ax_ = ax.twinx()
# ax_.plot(A)
input()
# plt.plot(np.convolve(A, np.hanning(64), 'valid'))
# ax = plt.gca()
# ax_ = ax.twinx()
# ax_.plot(np.convolve(A, 0.98 ** np.arange(A.shape[0]), 'full'))
# input()