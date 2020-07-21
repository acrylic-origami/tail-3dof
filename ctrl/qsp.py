import sys
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np

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

D0 = 0.6 # radians of excursion
def qsp_params(x, v0, v1, dd, k, q):
	k[0] = 0.0;
	k[1] = 1.0 / 3.0 if v0 == 0 else min(2.0 * D0 * x / abs(v0), 1.0 / 3.0)
	k[2] = 2.0 / 3.0 if v1 == 0 else (1.0 - min(2.0 * D0 * x / abs(v1), 1.0 / 3.0))
	k[3] = 1.0

	q[0] = 0.0
	q[1] = min(abs(v0 / 6), D0 * x) * sgn(v0)
	q[3] = dd - min(abs(v1 / 6), D0 * x) * sgn(v1)
	q[2] = (q[3] - q[1]) / (1 - (k[1] + k[2]) / 2) * k[2] / 2 + q[1]
	q[4] = dd

def s_tau(x, v0, v1, dd, p = False):
	k = list(range(4))
	q = list(range(5))
	qsp_params(x, v0, v1, dd * x, k, q)
	# print(q)
	tau_ = [qsp(k, q, k[1] / 2)[2], qsp(k, q, 0.5)[2], qsp(k, q, (1 + k[2]) / 2)[2]]
	
	if(p):
		f = plt.gcf()
		plt.figure(1)
		A = np.array([qsp(k, q, float(t) / 127) for t in range(128)]) * [1, 1, x]
		plt.clf()
		plt.plot(A[:,1:])
		ax2 = plt.gca().twinx()
		ax2.plot(A[:,0])
		
		input()
		plt.figure(f.number)
		
	return np.max(np.abs([
		(v0, tau_[0]),
		(qsp(k, q, k[1])[1], max(tau_[0], tau_[1])),
		(qsp(k, q, k[2])[1], max(tau_[1], tau_[2])),
		(v1, tau_[2])
	]), axis=0) * [1, x]


def sgn(x):
	return 1 if x > 0 else -1

DERIV = [-1/60, 3/20, -3/4, 0, 3/4, -3/20, 1/60][::-1]

if __name__ == '__main__':
	
	plt.ion()
	plt.clf()
	plt.show()
	# #define SMAX_HS 1577 // UNFLOAT 6.16
	# #define SMAX_DS 1920 // UNFLOAT 7.5
	# #define SMAX_HW 350 // UNFLOAT 7.5
	# #define T0_HW 36310 // UNFLOAT 20.0
	# #define T0_DS 5120 // UNFLOAT 20.0
	# #define T0_HS 4096 // UNFLOAT 16.0
	
	SMAX_HS = 1577 / 256
	T0_HS = 4096 / 256
	I_HS = 9
	SMAX_DS = 1920 / 256
	T0_DS = 5120 / 256
	SMAX_HW = 350 / 256
	T0_HW = 36310 / 256
	
	def deriv_sim(smax):
		v = np.linspace(-smax, smax, 4)
		dds = np.linspace(-math.pi / 2, math.pi / 2, 4)
		
		for v0 in v:
			for v1 in v:
				for dd in dds:
					tmin = D0 / max(abs(v0), abs(v1)) * 6 # first break point
					s_tau_ = []
					for x in np.linspace(1 / tmin / 30, 1 / tmin, 8):
						k = list(range(4))
						q = list(range(5))
						qsp_params(1.0 / tf, v0, v1, (b - a) / tf, k, q)
						# print(q)
						tau_ = [qsp(k, q, k[1] / 2)[2], qsp(k, q, 0.5)[2], qsp(k, q, (1 + k[2]) / 2)[2]]
						s_tau_.append(np.array([
							(v0, tau_[0]),
							(qsp(k, q, k[1])[1], max(tau_[0], tau_[1])),
							(qsp(k, q, k[2])[1], max(tau_[1], tau_[2])),
							(v1, tau_[2])
						]) * [1, x])
						
						A = np.array([qsp(k, q, float(t) / 127) for t in range(128)]) * [1, 1, x]
						plt.clf()
						plt.plot(A[:,1:])
						ax2 = plt.gca().twinx()
						ax2.plot(A[:,0])
						# input()
					
					s_tau_ = np.abs(np.array(s_tau_))
					s_tau_dx = np.apply_along_axis(lambda x: np.convolve(x, DERIV, 'valid'), 0, s_tau_)
					
					plt.clf()
					plt.plot(np.reshape(s_tau_dx, (s_tau_dx.shape[0], -1)))
					plt.legend(np.arange(8))
					input()
				
	def newton():
		v0 = 0
		v1 = 0
		dd = 1.064
		
		xy = lambda x: s_tau(x, v0, v1, dd) / [SMAX_HS, T0_HS / I_HS] # I_HS needed to account for the fact that s_tau spits out angular acceleration, not torque
		rsc = lambda x: (np.linalg.norm(xy(x)) - 1)
		
		xys = []
		x = [None, 1.0]
		R = [None, rsc(x[1])]
		print(xy(x[1]), R[1])
		THRESH = 0.05
		while abs(R[1]) > THRESH and x[1] > 0:
			drdx = (R[1] - R[0]) / (x[1] - x[0]) if R[0] != None and x[0] != None else 100
			xys.append(xy(x[1]))
			
			xy_unnorm = xy(x[1]) * [SMAX_HS, T0_HS]
			plt.annotate('t=%.2f' % (1 / x[1]), xy_unnorm, textcoords='offset points', xytext=(0, 5), ha='center')
			
			x[0] = x[1]
			x[1] -= R[1] / drdx
			
			R[0] = R[1]
			R[1] = rsc(x[1])
		
		xys = np.array(xys) * [SMAX_HS, T0_HS]
		print(xys)
		plt.scatter(*(np.array(xys).T))
		plt.gca().add_patch(Ellipse((0, 0), SMAX_HS * 2, T0_HS * 2, fill=False))
		plt.xlim([min(-SMAX_HS, np.min(xys[:,0])), max(SMAX_HS, np.max(xys[:,0]))])
		plt.ylim([min(-T0_HS, np.min(xys[:,1])), max(T0_HS, np.max(xys[:,1]))])
		plt.xlabel('Speed (rad/s)')
		plt.ylabel('Torque (kg·cm)')
		plt.title('Newton\'s method iteration for joint 1 path\n(61º, 0-speed endpoints)')
		yl = plt.ylim()
		plt.ylim((yl[0], yl[1] * 1.05))
		input()
	
	newton()
	
	# A = np.array([qsp([0, 0.499, 0.501, 1], [-2, -4, 0, 3, -1], float(t) / 99) for t in range(100)])
	
	# plt.grid(True)
	# plt.plot(A[:,0])
	# plt.plot(A[:,1])
	# ax2 = plt.gca().twinx()
	# ax2.plot(A[:,2])
	# input()