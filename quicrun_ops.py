import math
import matplotlib.pyplot as plt
import numpy as np

plt.ion()
plt.show()

def run():
	V = 7.4
	kV = np.array([[3600, 2850, 2170, 1800]])
	Pmax = np.array([[240, 180, 125, 95]])
	Ipmax = np.array([[62, 50, 35, 28]])
	R0 = np.array([[0.0183, 0.0289, 0.0488, 0.0747]])
	I0 = np.array([[2.8, 2.4, 1.6, 1.3]])
	
	vmax = kV * V / 60 # * 2 * math.pi # rad/s
	v = np.linspace(0, vmax[0,0])[:,np.newaxis]
	
	Pin = I0 * V + (V / R0 - I0 * V) * (1 - v / vmax)
	T = Pmax / (kV * (V / 2) / 60 * math.pi * 2) * (1 - v / vmax)
	Pout = T * v
	eta = Pout / Pin
	
	max_T = np.amax(T)
	max_p = np.amax(Pout)
	
	clr = ['b', 'g', 'r', 'c']
	for i in range(kV.shape[1]):
		msk = np.squeeze(v < vmax[0,i])
		plt.plot(v[msk], Pout[msk,i] / max_p, clr[i] + '--')
		plt.plot(v[msk], T[msk,i] / max_T, clr[i] + ':')
		plt.plot(v[msk], eta[msk,i], clr[i] + '-')
	
	plt.ylim([0, 1])
	plt.ylabel('Rel.: \\eta (-), Torque (:), Pout (--)')
	plt.xlabel('Speed (rps)')
	plt.title('Performance curves for Hobbywing Quicrun motors')
		
if __name__ == '__main__':
	run()
	input()