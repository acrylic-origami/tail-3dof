import numpy as np
import matplotlib.pyplot as plt

plt.ion()
plt.show()

def spline(a, av, b, bv, t):
	t = float(t)
	D = 6
	M = np.array([[ (t ** (i - 1)) * i for i in range(D, 1, -1) ], [ t ** i for i in range(D, 1, -1) ]])
	# M = np.array([ [ t ** 3 * 4, t ** 2 * 3, t * 2 ], [ t ** 4, t ** 3, t ** 2 ] ])
	B = np.array([ av - bv, a - b - bv * t ])
	MMT = np.dot(M, M.T)
	x = np.dot(M.T, (np.array([ MMT[1][1] * B[0] - MMT[0][1] * B[1], MMT[0][0] * B[1] - MMT[1][0] * B[0]]) / (MMT[0][0] * MMT[1][1] - MMT[0][1] * MMT[1][0]))[...,np.newaxis])
	return np.hstack((np.squeeze(x), [bv, b]))
	# return [(2*b + bv * t - 2*a + av * t)/t**3, (-3*b - 2*bv * t + 3*a - av * t)/t**2, bv, b]

CONTROL_TIME = 0.5
def spline_(x0, v0, xf, vf, duration):
	vc = (xf - x0) * 2 / duration - v0 * CONTROL_TIME - vf * (1 - CONTROL_TIME);
	xc = x0 + (v0 + vc) * CONTROL_TIME * duration / 2;
	return (lambda t:\
		# [
			(x0 + v0 * t + (vc - v0) * t ** 2 / 2 / CONTROL_TIME * duration) if t < CONTROL_TIME else \
				(xc + (vc * (t - CONTROL_TIME) + (vf - vc) * (t - CONTROL_TIME) ** 2 / 2 / (1 - CONTROL_TIME)) * duration))
			# v0 + (vc - v0) * t / CONTROL_TIME if t < CONTROL_TIME else \
			# 	vc + (vf - vc) * (t - CONTROL_TIME) / (1 - CONTROL_TIME)
		# ]

def smoothstep(t):
	return 3 * t ** 2 - 2 * t ** 3
	
def spline__(a, av, b, bv, t):
	l = (av * t + a)
	m = (b - a) * t + a
	r = (bv * (t - 1) + b)
	av_ = max(abs(av), 1)
	bv_ = max(abs(bv), 1)
	
	if t < 1 / av_ and t >= (1 - 1 / bv_):
		s0 = smoothstep(t * av_)
		s1 = smoothstep((1 - t) * bv_)
		s2 = smoothstep((t - (1 - 1 / bv_)) / (1 / av_ - (1 - 1 / bv_)))
		return (l * (1 - s0) + m * s0) * (1 - s2) + (r * (1 - s1) + m * s1) * s2
	elif t < 1 / av_:
		s = smoothstep(t * av_)
		return l * (1 - s) + m * s
	elif t >= 1 / av_ and t < (1 - 1 / bv_):
		return m
	else:
		s = smoothstep((1 - t) * bv_)
		return r * (1 - s) + m * s
	# return (lambda t: (av * t + a) * smoothstep())

def poly(k, t):
	return np.sum([k_ * t ** (len(k) - i - 1) for i, k_ in enumerate(k)], axis=0)

cmap = plt.get_cmap('viridis').colors
for i in range(1,16, 4):
	# k = 
	plt.plot(np.linspace(0, 1), [spline__(1, -0.3 * i, 0.5, 1 * i, t) for t in np.linspace(0, 1)], color=cmap[int(i * len(cmap) / 64)])
	input()
	plt.clf()