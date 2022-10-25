
from zlac8015d import ZLAC8015D
import time
import numpy as np
import threading

global motors

motors = ZLAC8015D.Controller(port="/dev/ttyUSB0")

motors.disable_motor()

motors.set_accel_time(50,50)
motors.set_decel_time(50,50)

motors.set_mode(3)
motors.enable_motor()

# cmds = [140, 170]
#cmds = [100, 50]
#cmds = [150, -100]
cmds = [-50, 30]

# motors.set_rpm(cmds[0],cmds[1])


last_time = time.time()
i = 0
period = 0.0
wave_period = 5.0
t = 0.0
last_stamp = time.time()
rpmL_cmd = 0.0
rpmR_cmd = 0.0
inc_t = 0.05
while True:
	try:
		startTime = time.time()

		## we change speed at every inc_t
		if (time.time() - last_stamp) > inc_t:
			rpmL_cmd = 50.0*np.sin((2*np.pi/wave_period)*t)
			rpmR_cmd = -50.0*np.sin((2*np.pi/wave_period)*t)

			t += inc_t
			if t > wave_period:
				t = 0.0
			
			motors.set_rpm(int(rpmL_cmd), int(rpmR_cmd))

			last_stamp = time.time()

		rpmL_fb, rpmR_fb = motors.get_rpm()
		print("period: {:.5f} t: {:.3f} rpmL_cmd: {:.2f} rpmR_cmd: {:.2f} rpmL_fb: {:.2f} rpmR_fb: {:.2f}".format(\
				period, t, rpmL_cmd, rpmR_cmd, rpmL_fb, rpmR_fb))

		# print(period)
		period = time.time() - startTime
		## this loop takes 0.015 ~ 0.03 seconds for set and get RPM

	except KeyboardInterrupt:
		motors.disable_motor()
		break

	last_time = time.time()

