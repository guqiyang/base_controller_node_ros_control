from sensor_msgs.msg import Joy

#Mapping Joystick
class RemapXbox360():

	joy_msg = Joy()

	buff = []
	buff_time = 3 #5
	buff_rate = 30
	buff_size = buff_rate*buff_time

	JOY_IS_ON = False

	# Left Stick
	LS_H, LS_V = -0,-0
	# Right Stick
	RS_H, RS_V = -0,-0
	# Direction Panel
	DP_H, DP_V = -0,-0
	# Buttons
	X,A,B,Y = -0,-0,-0,-0
	LB,RB,LT,RT,L,R = -0,-0,-0,-0,-0,-0
	BACK,START = -0,-0
	# TODO: Buttons_Toggle_Buff
	X_,A_,B_,Y_ = -0,-0,-0,-0
	LB_,RB_,LT_,RT_,L_,R_ = -0,-0,-0,-0,-0,-0
	BACK_,START_ = -0,-0

	def __init__(self):

		pass

	def update(self, msg):

		# Left Stick
		self.LS_H,self.LS_V = msg.axes[0],msg.axes[1]
		# Right Stick
		self.RS_H,self.RS_V = msg.axes[3],msg.axes[4]
		# Direction Panel
		self.DP_H,self.DP_V = msg.axes[6],msg.axes[7]
		# Buttons
		self.X,self.A,self.B,self.Y = msg.buttons[2],msg.buttons[0],msg.buttons[1],msg.buttons[3]
		self.LB,self.RB = msg.buttons[4],msg.buttons[5]
		self.LT,self.RT = (msg.axes[2]-1)/2,(msg.axes[5]-1)/2
		self.L,self.R = msg.buttons[9],msg.buttons[10]
		self.BACK,self.START = msg.buttons[6],msg.buttons[7]

	def switch(self, n):

		if n == 0:
			return 1
		else:
			return 0

	def deadzone(self, indata):

		if abs(indata) < 0.01:

			outdata = 0

		else:

			outdata = indata

		return outdata

	def buff(self, msg):

		if len(self.buff) < buff_size:

			self.buff.append(msg)

		else:

			self.buff.pop(0)
			self.buff.append(joy_msg)

		pass

	def buff_clear(self, msg):

		self.buff = []
