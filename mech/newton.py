import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import random
import numpy as np

plt.ion()
plt.show()

def origin_sgn(a, b, c, x):
	return 2*a*a * x*x*x + 3*a*b * x*x + (2*a*c+b*b+1) * x + b*c
	
def origin_sgn_p(a, b, c, x):
	return 6*a*a * x*x + 6*a*b * x + (2*a*c+b*b+1)

def quad(a, b, c, x):
	return a*x*x + b*x + c

def quad_p(a, b, c, x):
	return 2*a*x + b
	
def origin_dist_sq(a, b, c, x):
	y = quad(a, b, c, x)
	return y*y + x*x

def origin_dist_sq_p(a, b, c, x):
	return 2*quad_p(a, b, c, x)*quad(a, b, c, x) + 2*x

def run():
	while True:
		T0 = 16
		SMAX = 6.16
		a = 0 # 76/256 # random.random() * 2 - 1
		b = -18075/256 # random.random() * 2 - 1
		c = 0 # random.random() * 2 - 1
		d = -407/256 # random.random() * 2 - 1
		
		mid_ = -c / d
		print(mid_)
		a_ = b * (SMAX/T0) ** 2
		b_ = b * 2 * SMAX/T0 * mid_
		c_ = mid_ ** 2 * b + a
		print(a_ * 256, b_ * 256, c_ * 256)
		R2 = T0 * T0

		x = T0
		# x = x + 0.1 * abs(x)
		# _last_inside = origin_dist_sq(a_, b_, c_, mid_) < R2
		# easiest way... is sadly numerical root finding.
		# pros: easy to make into integer algorithm
		plt.clf()
		xs = [x]
		last_ann = float('infinity')
		for i in range(16):
			dist = origin_dist_sq(a_, b_, c_, x)
			delta = dist - R2
			print(dist - R2)
			if delta < 0:
				break
				
			# if _last_inside:
			# 	x += abs(dist / origin_dist_sq_p(a_, b_, c_, x)) # head away if inside
			# else:
			if last_ann > 1:
				xp = x * SMAX / T0 + mid_
				plt.annotate('t=%.2f' % abs(1/((xp - c) / d)), (xp, quad(b, 0, a, xp)), textcoords='offset points', xytext=(0, 5), ha='center')
				last_ann = 0
				
			dx = delta / origin_dist_sq_p(a_, b_, c_, x)
			x -= dx
			last_ann += abs(dx)
			print('** ', dx)
				
			xs.append(x)
			# _last_inside = _inside
			
		if i == 8:
			print('N')
		
		# x_ = x * SMAX / T0
		
		plt.gca().add_patch(Ellipse((mid_, 0), SMAX * 2, T0 * 2, fill=False))
		# xs_ = np.linspace(-1, 1) * abs(mid_)
		xs_ = np.array(xs) * SMAX / T0 + mid_
		plt.scatter(xs_, quad(b, 0, a, np.array(xs_)))
		plt.xlabel('Speed (rad/s)')
		plt.ylabel('Torque (kg·cm)')
		plt.title('Newton\'s method iteration for joint 1 path\n(61º, 0-speed endpoints)')
		print(xs_)
		# plt.plot(xs, quad(b, 0, a, np.array(xs) * SMAX / T0))
		input()
run()