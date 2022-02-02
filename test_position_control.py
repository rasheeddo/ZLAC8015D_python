
from ZLAC8015D import *
import time

motors = ZLAC8015D()

motors.disable_motor()

motors.set_accel_time(200,200)
motors.set_decel_time(200,200)
motors.set_maxRPM_pos(100,100)

motors.set_mode(1)
motors.set_position_async_control()
motors.enable_motor()

for i in range(10):

	motors.set_relative_angle(90,90)
	motors.move_left_wheel()
	motors.move_right_wheel()
	print(i)
	# time.sleep(0.01)
	l_meter, r_meter = motors.get_wheels_travelled()
	print(l_meter, r_meter)
	# reg = motors.get_wheels_travelled()
	# print(reg)
	time.sleep(2.0)
