import numpy as np
import matplotlib.pyplot as plt
import math

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
	
def cubic(a, av, b, bv, t):
	a_ = bv - 2 * (b - av - a)
	b_ = -bv + 3 * (b - av - a)
	return a_ * (t ** 3) + b_ * (t ** 2) + av * t + a

def cubic_v_sol(a, av, b, bv):
	a_ = bv - 2 * (b - av - a)
	b_ = -bv + 3 * (b - av - a)
	cv = b - a
	det = math.sqrt(b_ ** 2 - 4 * a_ * (av - cv))
	sols = [(b_ + det) / 2 / a_, (b_ - det) / 2 / a_]
	if sols[0] > 0 and sols[0] < 1:
		return sols[0]
	else:
		return sols[1]

def sgn(x):
	return 1 if x > 0 else -1
EXC = 0.3
MAXM = 2.0
def spline___(a, av, b, bv, t):
	dp = b - a
	
	# ob intersections (if any)
	at = EXC / abs(av - dp)
	ap = a + at * av
	dbt = EXC / abs(bv - dp)
	bt = 1 - dbt
	bp = b - dbt * bv
	ob = [at > 0 and at < 1, bt > 0 and bt < 1 and bt > at]
	
	if not any(ob):
		return cubic(a, av, b, bv, t)
	elif all(ob):
		ct = (at + bt) / 2
		c = (ap + bp) / 2
		cv = (bp - ap) / (bt - at)
		cv = cv if abs(cv - dp) < MAXM else dp + MAXM * sgn(cv)
	
		if t == 0:
			print(ap, bp, at, bt, ct, c, cv)
		
		if t < ct:
			return cubic(a, av * ct, c, cv * ct, t / ct)
		else:
			return cubic(c, cv * (1 - ct), b, bv * (1 - ct), (t - ct) / (1 - ct))
		
	else:
		if ob[1]:
			a, b, av, bv, t = (b, a, bv, av, (1 - t))
			ap, at, bp, bt = (bp, bt, ap, at)
			
		cv = (b - ap) / (1 - at)
		ct = cubic_v_sol(a, av, b, cv)
		c = cubic(a, av, b, cv, ct)
		if t < ct:
			return cubic(a, av, b, cv, t / ct)
		else:
			return cubic(c, cv, b, bv, (t - ct) / (1 - ct))

ALPHA_F = EXC
def qsp(ts, qs, t):
	assert len(ts) == 4 and len(qs) == 5 and ts[0] == 0.0 and ts[-1] == 1.0
	T = [[1.0 / (b - a) for a in ts[:i]] for i, b in enumerate(ts)]
	Tt = [t - a for a in ts]
	
	if t > ts[2]:
		sp = [0, 0, 
			(1 - Tt[1] * T[3][1]) * (1 - Tt[2] * T[3][2]),
			(1 - Tt[2] * T[3][2]) * (Tt[1] * T[3][1] + Tt[2] * T[3][2]),
			(Tt[2] * T[3][2]) ** 2
		]
		sp_ = [0,0,
			-T[3][2] * (1 - Tt[1] * T[3][1]) - T[3][1] * (1 - Tt[2] * T[3][2]),
			-Tt[1] * T[3][1] * T[3][2] + T[3][1] * (1 - Tt[2] * T[3][2]) + T[3][2] * (1 - 2 * Tt[2] * T[3][2]) ,
			2 * Tt[2] * (T[3][2] ** 2)
		]
		sp__ = [0,0,
			2 * T[3][2] * T[3][1],
			-2 * T[3][2] * (T[3][1] + T[3][2]),
			2 * T[3][2] ** 2
		]
	elif t > ts[1]:
		sp = [0,
			(1 - Tt[0] * T[2][0]) * (1 - Tt[1] * T[2][1]),
			Tt[0] * T[2][0] * (1 - Tt[1] * T[2][1]) + (1 - Tt[1] * T[3][1]) * (Tt[1] * T[2][1]),
			(Tt[1] ** 2) * T[3][1] * T[2][1],
		0]
		sp_ = [0,
			-T[2][1] * (1 - Tt[0] * T[2][0]) - T[2][0] * (1 - Tt[1] * T[2][1]) ,
			-Tt[0] * T[2][0] * T[2][1] + T[2][0] * (1 - Tt[1] * T[2][1]) - T[3][1] * Tt[1] * T[2][1] + T[2][1] * (1 - Tt[1] * T[3][1]),
			2 * Tt[1] * T[2][1] * T[3][1],
		0]
		sp__ = [0,
			2 * T[2][1] * T[2][0],
			-2 * T[2][1] * (T[2][0] + T[3][1]),
			2 * T[2][1] * T[3][1],
		0]
	else:
		sp = [
			(1 - Tt[0] * T[1][0]) ** 2,
			Tt[0] * T[1][0] * (2 - Tt[0] * (T[2][0] + T[1][0])),
			(Tt[0] ** 2) * T[2][0] * T[1][0],
		0, 0]
		sp_ = [
			-2 * T[1][0] * (1 - Tt[0] * T[1][0]),
			2 * (1 - Tt[0] * (T[1][0] + T[2][0])) * T[1][0],
			2 * Tt[0] * T[2][0] * T[1][0],
		0, 0]
		sp__ = [
			2 * T[1][0] ** 2,
			-2 * T[1][0] * (T[1][0] + T[2][0]),
			2 * T[2][0] * T[1][0],
		0, 0]
	
	return [sum([a * b for a, b in zip(qs, sp)]), sum([a * b for a, b in zip(qs, sp_)]), sum([a * b for a, b in zip(qs, sp__)])]

def qsp_params(x, v0, v1, dd, k, q):
	k[0] = 0.0;
	k[1] = 1.0 / 3.0 if v0 == 0 else min(2.0 * ALPHA_F * x / abs(v0), 1.0 / 3.0)
	k[2] = 2.0 / 3.0 if v1 == 0 else (1.0 - min(2.0 * ALPHA_F * x / abs(v1), 1.0 / 3.0))
	k[3] = 1.0

	q[0] = 0.0
	q[1] = min(abs(v0 / 6), ALPHA_F * x) * sgn(v0)
	q[3] = dd - min(abs(v1 / 6), ALPHA_F * x) * sgn(v1)
	q[2] = (q[3] - q[1]) / (1 - (k[1] + k[2]) / 2) * k[2] / 2 + q[1]
	q[4] = dd
	
def lerp_qsp(a, b, v0, v1, tf, t):
	k = list(range(4))
	q = list(range(5))
	qsp_params(1.0 / tf, v0, v1, (b - a) / tf, k, q)
	tr = qsp(k, q, t / tf)
	tr[0] = (tr[0] * tf) + a
	return tr

def spline____(a, av, b, bv, t):
	return lerp_qsp(a, b, av, bv, 1.0, t)[0]

def poly(k, t):
	return np.sum([k_ * t ** (len(k) - i - 1) for i, k_ in enumerate(k)], axis=0)

cmap = plt.get_cmap('viridis').colors
for i in range(1,16,2):
	# k = 
	it = spline_(1, -0.3 * i, 0.5, 1 * i, 1.0)
	plt.plot(np.linspace(0, 1), [spline____(1, -0.3 * i, 0.5, 1 * i, t) for t in np.linspace(0, 1)], color=cmap[int(i * len(cmap) / 16)]) # spline__(1, -0.3 * i, 0.5, 1 * i, t)
	# input()
	# plt.clf()
plt.xlabel('Normalized time')
plt.ylabel('Position')
plt.legend(["%dx" % a for a in range(1,16,2)])
plt.title('Quadratic spline vs. endpoint velocities')
input()