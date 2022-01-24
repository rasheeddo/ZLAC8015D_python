
from ZLAC8015D import *
import time

motors = ZLAC8015D()

motors.disable_motor()

motors.set_accel_time(1000,1000)
motors.set_decel_time(1000,1000)

motors.set_mode(3)
motors.enable_motor()

# cmds = [140, 170]
#cmds = [100, 50]
#cmds = [150, -100]
cmds = [-50, 30]

motors.set_rpm(cmds[0],cmds[1])

start_time = time.time()
i = 0
while True:
	try:
		period = time.time() - start_time
		rpmL, rpmR = motors.get_rpm()

		print("rpmL: {:.1f} | rpmR: {:.1f}".format(rpmL,rpmR))
		time.sleep(1)

		# if (i % 10) == 0:
			

	except KeyboardInterrupt:
		motors.disable_motor()
		break