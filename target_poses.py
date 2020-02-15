from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
# ##DELETE THIS
# class Position():
# 	def __init__(self):
# 		self.x = 0
# 		self.y = 0
# 		self.z = 0

# class Orientation():
# 	def __init__(self):
# 		self.x = 0
# 		self.y = 0
# 		self.z = 0
# 		self.w = 0

# class EAngle():
# 	def __init__(self):
# 		self.roll = 0
# 		self.pitch = 0
# 		self.yaw = 0

# class Pose():
# 	def __init__(self):
# 		self.position = Position()
# 		self.orientation = Orientation()
# ##DELETE THIS

startv = Pose()
startv.position.x = 0.474
startv.position.y = 0.759
startv.position.z = 0.224
startv.orientation.x = -0.7071067811865476
startv.orientation.y = -0.7071067811865475
startv.orientation.z = 4.329780281177466e-17
startv.orientation.w = 4.329780281177467e-17

starth = Pose()
starth.position.x = 0.464
starth.position.y = 0.804
starth.position.z = 0.116
starth.orientation.x = 0.7071067811865476
starth.orientation.y = 0.7071067811865475
starth.orientation.z = 4.329780281177466e-17
starth.orientation.w = 4.329780281177467e-17

brick1 = Pose()
brick1.position.x = 0.615
brick1.position.y = 0.194
brick1.position.z = -0.131
brick1.orientation.x = -0.7071067811865476
brick1.orientation.y = -0.7071067811865475
brick1.orientation.z = 4.329780281177466e-17
brick1.orientation.w = 4.329780281177467e-17

brick2 = Pose()
brick2.position.x = 0.615
brick2.position.y = 0.346
brick2.position.z = -0.125
brick2.orientation.x = -0.7071067811865476
brick2.orientation.y = -0.7071067811865475
brick2.orientation.z = 4.329780281177466e-17
brick2.orientation.w = 4.329780281177467e-17

brick3 = Pose()
brick3.position.x = 0.615
brick3.position.y = 0.532
brick3.position.z = -0.122
brick3.orientation.x = 0.7071067811865476
brick3.orientation.y = 0.7071067811865475
brick3.orientation.z = 4.329780281177466e-17
brick3.orientation.w = 4.329780281177467e-17

brick4 = Pose()
brick4.position.x = 0.615
brick4.position.y = 0.240
brick4.position.z = -0.0499
brick4.orientation.x = -0.7071067811865476
brick4.orientation.y = -0.7071067811865475
brick4.orientation.z = 4.329780281177466e-17
brick4.orientation.w = 4.329780281177467e-17

brick5 = Pose()
brick5.position.x = 0.615
brick5.position.y = 0.449
brick5.position.z = -0.058
brick5.orientation.x = -0.7071067811865476
brick5.orientation.y = -0.7071067811865475
brick5.orientation.z = 4.329780281177466e-17
brick5.orientation.w = 4.329780281177467e-17

brick6 = Pose()
brick6.position.x = 0.615
brick6.position.y = 0.247
brick6.position.z = 0.123
brick6.orientation.x = -0.7071067811865476
brick6.orientation.y = -0.7071067811865475
brick6.orientation.z = 4.329780281177466e-17
brick6.orientation.w = 4.329780281177467e-17

brick7 = Pose()
brick7.position.x = 0.615
brick7.position.y = 0.441
brick7.position.z = 0.138
brick7.orientation.x = -0.7071067811865476
brick7.orientation.y = -0.7071067811865475
brick7.orientation.z = 4.329780281177466e-17
brick7.orientation.w = 4.329780281177467e-17

brick8 = Pose()
brick8.position.x = 0.635
brick8.position.y = 0.334
brick8.position.z = 0.209
brick8.orientation.x = -0.7071067811865476
brick8.orientation.y = -0.7071067811865475
brick8.orientation.z = 4.329780281177466e-17
brick8.orientation.w = 4.329780281177467e-17

brick9 = Pose()
brick9.position.x = 0.635
brick9.position.y = 0.349
brick9.position.z = 0.406
brick9.orientation.x = 0.7071067811865476
brick9.orientation.y = 0.7071067811865475
brick9.orientation.z = 4.329780281177466e-17
brick9.orientation.w = 4.329780281177467e-17


brick_directions_notf = [
	{
	'id':'tv',
	'pose':startv,
	'isVertical':True
	},
	{'id':'th',
	'pose':starth,
	'isVertical':False
	},
	{'id':'b1',
	'pose':brick1,
	'isVertical':True
	},
	{'id':'b2',
	'pose':brick2,
	'isVertical':True
	},
	{'id':'b3',
	'pose':brick3,
	'isVertical':True
	},
	{'id':'b4',
	'pose':brick4,
	'isVertical':False
	},
	{'id':'b5',
	'pose':brick5,
	'isVertical':False
	},
	{'id':'b6',
	'pose':brick6,
	'isVertical':True
	},
	{'id':'b7',
	'pose':brick7,
	'isVertical':True
	},
	{'id':'b8',
	'pose':brick8,
	'isVertical':False
	},
	{'id':'b9',
	'pose':brick9,
	'isVertical':True
	}]

bdf = brick_directions_notf

tower_instructions = [	bdf[0],
						bdf[2],
						bdf[0],
						bdf[3],
						bdf[0],
						bdf[4],
						bdf[1],
						bdf[5],
						bdf[1],
						bdf[6],
						bdf[0],
						bdf[7],
						bdf[0],
						bdf[8],
						bdf[1],
						bdf[9],
						bdf[0],
						bdf[10],
						]















