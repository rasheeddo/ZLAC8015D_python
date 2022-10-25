
from zlac8015d import ZLAC8015D
import time

motors = ZLAC8015D.Controller(port="/dev/ttyUSB0")

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

last_time = time.time()
i = 0
period = 0.0
while True:
	try:
		# motors.set_rpm(cmds[0],cmds[1])
		rpmL, rpmR = motors.get_rpm()

		print("period: {:.4f} rpmL: {:.1f} | rpmR: {:.1f}".format(period,rpmL,rpmR))
		period = time.time() - last_time
		time.sleep(0.01)
			

	except KeyboardInterrupt:
		motors.disable_motor()
		break

	last_time = time.time()