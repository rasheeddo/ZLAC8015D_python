
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
import numpy as np

class ZLAC8015D:

	def __init__(self):

		self.client = ModbusClient(method='rtu', port='/dev/ttyUSB0', baudrate=115200, timeout=1)

		self.client.connect()

		self.ID = 1

		######################
		## Register Address ##
		######################
		## Common
		self.CONTROL_REG = 0x200E
		self.OPR_MODE = 0x200D
		self.L_ACL_TIME = 0x2080
		self.R_ACL_TIME = 0x2081
		self.L_DCL_TIME = 0x2082
		self.R_DCL_TIME = 0x2083

		## Velocity control
		self.L_CMD_RPM = 0x2088
		self.R_CMD_RPM = 0x2089
		self.L_FB_RPM = 0x20AB
		self.R_FB_RPM = 0x20AC

		## Position control
		self.POS_CONTROL_TYPE = 0x200F

		self.L_MAX_RPM_POS = 0x208E
		self.R_MAX_RPM_POS = 0x208F

		self.L_CMD_REL_POS_HI = 0x208A
		self.L_CMD_REL_POS_LO = 0x208B
		self.R_CMD_REL_POS_HI = 0x208C
		self.R_CMD_REL_POS_LO = 0x208D

		self.L_FB_POS_HI = 0x20A7
		self.L_FB_POS_LO = 0x20A8
		self.R_FB_POS_HI = 0x20A9
		self.R_FB_POS_LO = 0x20AA

		## Troubleshooting
		self.L_FAULT = 0x20A5
		self.R_FAULT = 0x20A6

		########################
		## Control CMDs (REG) ##
		########################
		self.EMER_STOP = 0x05
		self.ALRM_CLR = 0x06
		self.DOWN_TIME = 0x07
		self.ENABLE = 0x08
		self.POS_SYNC = 0x10
		self.POS_L_START = 0x11
		self.POS_R_START = 0x12

		####################
		## Operation Mode ##
		####################
		self.POS_REL_CONTROL = 1
		self.POS_ABS_CONTROL = 2
		self.VEL_CONTROL = 3

		self.ASYNC = 0
		self.SYNC = 1

		#################
		## Fault codes ##
		#################
		self.NO_FAULT = 0x0000
		self.OVER_VOLT = 0x0001
		self.UNDER_VOLT = 0x0002
		self.OVER_CURR = 0x0004
		self.OVER_LOAD = 0x0008
		self.CURR_OUT_TOL = 0x0010
		self.ENCOD_OUT_TOL = 0x0020
		self.MOTOR_BAD = 0x0040
		self.REF_VOLT_ERROR = 0x0080
		self.EEPROM_ERROR = 0x0100
		self.WALL_ERROR = 0x0200
		self.HIGH_TEMP = 0x0400
		self.FAULT_LIST = [self.OVER_VOLT, self.UNDER_VOLT, self.OVER_CURR, self.OVER_LOAD, self.CURR_OUT_TOL, self.ENCOD_OUT_TOL, \
					self.MOTOR_BAD, self.REF_VOLT_ERROR, self.EEPROM_ERROR, self.WALL_ERROR, self.HIGH_TEMP]

		##############
		## Odometry ##
		##############
		## 8 inches wheel
		self.travel_in_one_rev = 0.655
		self.cpr = 16385
		self.R_Wheel = 0.105 #meter

	## Some time if read immediatly after write, it would show ModbusIOException when get data from registers
	def modbus_fail_read_handler(self, ADDR, WORD):

		read_success = False
		reg = [None]*WORD
		while not read_success:
			result = self.client.read_holding_registers(ADDR, WORD, unit=self.ID)
			try:
				for i in range(WORD):
					reg[i] = result.registers[i]
				read_success = True
			except AttributeError:
				pass

		return reg

	def rpm_to_radPerSec(self, rpm):
		return rpm*2*np.pi/60.0

	def rpm_to_linear(self, rpm):

		W_Wheel = self.rpm_to_radPerSec(rpm)
		V = W_Wheel*self.R_Wheel

		return V

	def set_mode(self, MODE):
		if MODE == 1:
			print("Set relative position control")
		elif MODE == 2:
			print("Set absolute position control")
		elif MODE == 3:
			print("Set speed rpm control")
		else:
			print("set_mode ERROR: set only 1, 2, or 3")
			return 0

		result = self.client.write_register(self.OPR_MODE, MODE, unit=self.ID)
		return result

	def get_mode(self):

		# result = self.client.read_holding_registers(self.OPR_MODE, 1, unit=self.ID)
		registers = self.modbus_fail_read_handler(self.OPR_MODE, 1)

		mode = registers[0]

		return mode

	def enable_motor(self):
		result = self.client.write_register(self.CONTROL_REG, self.ENABLE, unit=self.ID)

	def disable_motor(self):
		result = self.client.write_register(self.CONTROL_REG, self.DOWN_TIME, unit=self.ID)

	def get_fault_code(self):

		fault_codes = self.client.read_holding_registers(self.L_FAULT, 2, unit=self.ID)

		L_fault_code = fault_codes.registers[0]
		R_fault_code = fault_codes.registers[1]

		L_fault_flag = L_fault_code in self.FAULT_LIST
		R_fault_flag = R_fault_code in self.FAULT_LIST

		return (L_fault_flag, L_fault_code), (R_fault_flag, R_fault_code)

	def clear_alarm(self):
		result = self.client.write_register(self.CONTROL_REG, self.ALRM_CLR, unit=self.ID)

	def set_accel_time(self, L_ms, R_ms):

		if L_ms > 32767:
			L_ms = 32767
		elif L_ms < 0:
			L_ms = 0

		if R_ms > 32767:
			R_ms = 32767
		elif R_ms < 0:
			R_ms = 0

		result = self.client.write_registers(self.L_ACL_TIME, [int(L_ms),int(R_ms)], unit=self.ID)


	def set_decel_time(self, L_ms, R_ms):

		if L_ms > 32767:
			L_ms = 32767
		elif L_ms < 0:
			L_ms = 0

		if R_ms > 32767:
			R_ms = 32767
		elif R_ms < 0:
			R_ms = 0

		result = self.client.write_registers(self.L_DCL_TIME, [int(L_ms), int(R_ms)], unit=self.ID)

	def int16Dec_to_int16Hex(self,int16):

		lo_byte = (int16 & 0x00FF)
		hi_byte = (int16 & 0xFF00) >> 8

		all_bytes = (hi_byte << 8) | lo_byte

		return all_bytes


	def set_rpm(self, L_rpm, R_rpm):

		if L_rpm > 3000:
			L_rpm = 3000
		elif L_rpm < -3000:
			L_rpm = -3000

		if R_rpm > 3000:
			R_rpm = 3000
		elif R_rpm < -3000:
			R_rpm = -3000

		left_bytes = self.int16Dec_to_int16Hex(L_rpm)
		right_bytes = self.int16Dec_to_int16Hex(R_rpm)

		result = self.client.write_registers(self.L_CMD_RPM, [left_bytes, right_bytes], unit=self.ID)

	def get_rpm(self):


		# rpms = self.client.read_holding_registers(self.L_FB_RPM, 2, unit=self.ID)
		# fb_L_rpm = np.int16(rpms.registers[0])/10.0
		# fb_R_rpm = np.int16(rpms.registers[1])/10.0

		registers = self.modbus_fail_read_handler(self.L_FB_RPM, 2)
		fb_L_rpm = np.int16(registers[0])/10.0
		fb_R_rpm = np.int16(registers[1])/10.0

		return fb_L_rpm, fb_R_rpm

	def get_linear_velocities(self):

		rpmL, rpmR = self.get_rpm()

		VL = self.rpm_to_linear(rpmL)
		VR = self.rpm_to_linear(-rpmR)

		return VL, VR

	def map(self, val, in_min, in_max, out_min, out_max):

			return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def set_maxRPM_pos(self, max_L_rpm, max_R_rpm):

		if max_L_rpm > 1000:
			max_L_rpm = 1000
		elif max_L_rpm < 1:
			max_L_rpm = 1

		if max_R_rpm > 1000:
			max_R_rpm = 1000
		elif max_R_rpm < 1:
			max_R_rpm = 1

		result = self.client.write_registers(self.L_MAX_RPM_POS, [int(max_L_rpm), int(max_R_rpm)], unit=self.ID)

	def set_position_async_control(self):

		result = self.client.write_register(self.POS_CONTROL_TYPE, self.ASYNC, unit=self.ID)

	def move_left_wheel(self):

		result = self.client.write_register(self.CONTROL_REG, self.POS_L_START, unit=self.ID)

	def move_right_wheel(self):

		result = self.client.write_register(self.CONTROL_REG, self.POS_R_START, unit=self.ID)

	def deg_to_32bitArray(self, deg):

		dec = int(self.map(deg, -1440, 1440, -65536, 65536))
		HI_WORD = (dec & 0xFFFF0000) >> 16
		LO_WORD = dec & 0x0000FFFF

		return [HI_WORD, LO_WORD]

	def set_relative_angle(self, ang_L, ang_R):

		L_array = self.deg_to_32bitArray(ang_L)
		R_array = self.deg_to_32bitArray(ang_R)
		all_cmds_array = L_array + R_array

		result = self.client.write_registers(self.L_CMD_REL_POS_HI, all_cmds_array, unit=self.ID)

	def get_wheels_travelled(self):

		# read_success = False
		# while not read_success:

		# 	result = self.client.read_holding_registers(self.L_FB_POS_HI, 4, unit=self.ID)
		# 	try:
		# 		l_pul_hi = result.registers[0]
		# 		l_pul_lo = result.registers[1]
		# 		r_pul_hi = result.registers[2]
		# 		r_pul_lo = result.registers[3]

		# 		l_pulse = ((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF)
		# 		r_pulse = ((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF)
		# 		l_travelled = (float(l_pulse)/self.cpr)*self.travel_in_one_rev  # unit in meter
		# 		r_travelled = (float(r_pulse)/self.cpr)*self.travel_in_one_rev  # unit in meter

		# 		read_success = True

		# 	except AttributeError:
		# 		# print("error")
		# 		pass

		registers = self.modbus_fail_read_handler(self.L_FB_POS_HI, 4)
		l_pul_hi = registers[0]
		l_pul_lo = registers[1]
		r_pul_hi = registers[2]
		r_pul_lo = registers[3]

		l_pulse = np.int32(((l_pul_hi & 0xFFFF) << 16) | (l_pul_lo & 0xFFFF))
		r_pulse = np.int32(((r_pul_hi & 0xFFFF) << 16) | (r_pul_lo & 0xFFFF))
		l_travelled = (float(l_pulse)/self.cpr)*self.travel_in_one_rev  # unit in meter
		r_travelled = (float(r_pulse)/self.cpr)*self.travel_in_one_rev  # unit in meter

		return l_travelled, r_travelled