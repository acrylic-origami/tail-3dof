import math
import numpy as np
import scipy.misc
import PIL.Image
import sys
import csv
import matplotlib as mpl
mpl.use('WebAgg')
import matplotlib.pyplot as plt
import matplotlib.axes as pltax
# from functools import reduce

# plt.ion()
# plt.close('all')
# plt.show()

N = int(sys.argv[1])
amin = (float('inf'), -1)
amax = (0, -1)
ds = []
with open(sys.argv[2], 'r') as f:
	D = {}
	for l in csv.reader(f): # , delimiter='\t'):
		l = np.array([float(l_) for l_ in l])
		if l[3] not in D:
			D[l[3]] = np.zeros((N,N,3), dtype=np.uint8)
		
		xyz = l[3:]
		d = math.sqrt(sum([l_ ** 2 for l_ in xyz]))
		if d < amin[0]:
			amin = (d, xyz)
		if d > amax[0]:
			amax = (d, xyz)
		ds.append(d)
			
		D[int(l[3])][int(l[4]),int(l[5]),:] = ((l[:3] / math.pi) * 256).astype(np.uint8)

print(amin, amax)

for fn, d in D.items():
	PIL.Image.fromarray(d, mode='RGB').save('%s/%d.png' % (sys.argv[3], fn))

# E_sa = [4 * math.pi * d ** 2 / 2 for d in ds]
# cum_ds = reduce(lambda x, a: a + [x + a[-1]], ds, [0.0])[1:]
# print(E_sa)
# print([(i, E_sa[i], ds[i]) for i in range(len(ds)) if E_sa[i] < i and E_sa[i - 1] > i - 1])

CONE_ANGLE = 75 * math.pi / 180
E_d = np.mean(ds)
M_d = np.max(ds)
es = []
fs = []
for x in range(-int(M_d), int(M_d)):
	for y in range(0, int(M_d)):
		for z in range(-int(M_d), int(M_d)):
			xyz = (x, y, z)
			d = np.linalg.norm(xyz)
			y_ = y + int(N/2)
			z_ = z + int(N/2)
			if y / d > math.cos(CONE_ANGLE):
				if x in D and y_ < D[x].shape[0] and z_ < D[x].shape[1]: # and D[x][y_, z_] == 0:
					# if d < E_d:
					es.append(d)
					# else:
					# 	fs.append((xyz, d))

es.sort() # key=lambda x: x[1])
# fs.sort(key=lambda x: x[1])
ax = plt.gca()
NFILT = 32
ax.plot(np.array(es) / 16 * 0.625)
# plt.plot([f[1] for f in fs])
ax.plot(np.cbrt(3/4/math.pi/(CONE_ANGLE / math.pi)*np.arange(len(es))) / 16 * 0.625)
ax.axhline(y=E_d / 16 * 0.625)
ax2 = ax.twinx()
ax2.plot(np.convolve(np.ones(NFILT), np.array(es[1:]) - es[:-1], 'same'), alpha=0.3)
ax2.plot(NFILT * (3/4/math.pi/3/(CONE_ANGLE / math.pi)*np.arange(len(es))**(-2/3)), alpha=0.3)
ax2.set_zorder(-1)
plt.sca(ax)

ax.set_xlabel('Volume (cm^3 · 5.96) AKA number of points')
ax.set_ylabel('Radius (meter)')
ax2.set_ylabel('dr/dV (/m^2 · 655.36)')
plt.show()