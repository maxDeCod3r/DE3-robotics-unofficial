import math
import numpy as np

def qte(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

def etq(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def q_extrapolator(obj):
	return obj.orientation.x, obj.orientation.y, obj.orientation.z, obj.orientation.w

print(etq(1.57, 1.57, 0))

exit(0)

##DELETE THIS
class Position():
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0

class Orientation():
	def __init__(self):
		self.x = 0
		self.y = 0
		self.z = 0
		self.w = 0

class EAngle():
	def __init__(self):
		self.roll = 0
		self.pitch = 0
		self.yaw = 0

class Pose():
	def __init__(self):
		self.position = Position()
		self.orientation = Orientation()
##DELETE THIS

# pick up position
startv = Pose()
startv.position.x = 0.474
startv.position.y = 0.708
startv.position.z = 0.254
startv.orientation.x = 0.646
startv.orientation.y = 0.762
startv.orientation.z = -0.003
startv.orientation.w = 0.003

starth = Pose()
starth.position.x = 0.464
starth.position.y = 0.804
starth.position.z = 0.116
starth.orientation.x = 0.661
starth.orientation.y = 0.748
starth.orientation.z = 0.043
starth.orientation.w = -0.006

brick1 = Pose()
brick1.position.x = 0.615
brick1.position.y = 0.194
brick1.position.z = -0.131
brick1.orientation.x = 0.656
brick1.orientation.y = 0.753
brick1.orientation.z = -0.013
brick1.orientation.w = -0.015

brick2 = Pose()
brick2.position.x = 0.615
brick2.position.y = 0.346
brick2.position.z = -0.125
brick2.orientation.x = 0.667
brick2.orientation.y = 0.742
brick2.orientation.z = -0.045
brick2.orientation.w = 0.033

brick3 = Pose()
brick3.position.x = 0.615
brick3.position.y = 0.532
brick3.position.z = -0.122
brick3.orientation.x = 0.689
brick3.orientation.y = 0.724
brick3.orientation.z = 0.008
brick3.orientation.w = 0.014

brick4 = Pose()
brick4.position.x = 0.615
brick4.position.y = 0.240
brick4.position.z = -0.0499
brick4.orientation.x = 0.692
brick4.orientation.y = 0.720
brick4.orientation.z = -0.030
brick4.orientation.w = -0.021

brick5 = Pose()
brick5.position.x = 0.615
brick5.position.y = 0.449
brick5.position.z = -0.058
brick5.orientation.x = 0.677
brick5.orientation.y = 0.734
brick5.orientation.z = -0.032
brick5.orientation.w = -0.016

brick6 = Pose()
brick6.position.x = 0.615
brick6.position.y = 0.247
brick6.position.z = 0.123
brick6.orientation.x = 0.656
brick6.orientation.y = 0.753
brick6.orientation.z = -0.040
brick6.orientation.w = 0.000

brick7 = Pose()
brick7.position.x = 0.615
brick7.position.y = 0.441
brick7.position.z = 0.138
brick7.orientation.x = 0.671
brick7.orientation.y = 0.735
brick7.orientation.z = -0.060
brick7.orientation.w = 0.005

brick8 = Pose()
brick8.position.x = 0.635
brick8.position.y = 0.334
brick8.position.z = 0.209
brick8.orientation.x = 0.680
brick8.orientation.y = 0.729
brick8.orientation.z = -0.048
brick8.orientation.w = -0.042

brick9 = Pose()
brick9.position.x = 0.635
brick9.position.y = 0.349
brick9.position.z = 0.406
brick9.orientation.x = 0.674
brick9.orientation.y = 0.737
brick9.orientation.z = -0.008
brick9.orientation.w = 0.016




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


for item in brick_directions_notf:
	angles = qte(*q_extrapolator(item['pose']))
	clean_angles = []
	for one in angles:
		if one > -185 and one < -172:
			clean_angles.append(-180)
		elif one > 175 and one < 185:
			clean_angles.append(180)
		elif one < 7 and one > -5:
			clean_angles.append(0)
		elif one > 85 and one < 100:
			clean_angles.append(90)
		else:
			clean_angles.append(one)
	# print('Brick: ', item['id'], ' (Vertical:', item['isVertical'], ')')
	# print('Initial: ', angles)
	# print('Clean: ', clean_angles)
	for ix in range (0, 3):
		clean_angles[ix] = math.radians(clean_angles[ix])
	quats = etq(clean_angles[0], clean_angles[1], clean_angles[2])
	# print(q_extrapolator(item['pose']))
	# print(quats)
	# print()
	# print('Begin data')
	print('---')
	print(item['id'])
	print('x_:_ ', quats[0])
	print('y_:_ ', quats[1])
	print('z_:_ ', quats[2])
	print('w_:_ ', quats[3])
	print('---')
	print()


# CORRECTED FOR  ERRORS
# ---
# tv
# x_:_  -0.7071067811865476
# y_:_  -0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# th
# x_:_  0.7071067811865476
# y_:_  0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# b1
# x_:_  -0.7071067811865476
# y_:_  -0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# b2
# x_:_  -0.7071067811865476
# y_:_  -0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# b3
# x_:_  0.7071067811865476
# y_:_  0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# b4
# x_:_  -0.7071067811865476
# y_:_  -0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# b5
# x_:_  -0.7071067811865476
# y_:_  -0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# b6
# x_:_  -0.7071067811865476
# y_:_  -0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# b7
# x_:_  -0.7071067811865476
# y_:_  -0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# b8
# x_:_  -0.7071067811865476
# y_:_  -0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---

# ---
# b9
# x_:_  0.7071067811865476
# y_:_  0.7071067811865475
# z_:_  4.329780281177466e-17
# w_:_  4.329780281177467e-17
# ---





# Brick:  tv  (Vertical: True )
# Initial:  [-179.959958775756, 0.48404050300017853, 99.30511432490599]
# Clean:  [-180, 0, 90]
# ---

# Brick:  th  (Vertical: False )
# Initial:  [176.7489708415661, -3.774051353934406, 97.07726408520018]
# Clean:  [-180, 0, 90]
# ---

# Brick:  b1  (Vertical: True )
# Initial:  [-177.73984770155016, -0.31707646225879454, 97.74150389024307]
# Clean:  [-180, 0, 90]
# ---

# Brick:  b2  (Vertical: True )
# Initial:  [-178.68432711359827, 6.2577884502559975, 96.08354223678543]
# Clean:  [-180, 0, 90]
# ---

# Brick:  b3  (Vertical: True )
# Initial:  [178.22758990527294, 0.5298789221469992, 92.78136914285481]
# Clean:  [-180, 0, 90]
# ---

# Brick:  b4  (Vertical: False )
# Initial:  [-175.84410045162116, 0.6463100993122008, 92.21552176954216]
# Clean:  [-180, 0, 90]
# ---

# Brick:  b5  (Vertical: False )
# Initial:  [-176.05042286692418, 1.1368228542886731, 94.57227316004405]
# Clean:  [-180, 0, 90]
# ---

# Brick:  b6  (Vertical: True )
# Initial:  [-176.53430985789132, 3.0082644567182153, 97.90742822617871]
# Clean:  [-180, 0, 90]
# ---

# Brick:  b7  (Vertical: True )
# Initial:  [-175.25111166494776, 5.0410815408391505, 95.08110720439343]
# Clean:  [-180, 0, 90]
# ---

# Brick:  b8  (Vertical: False )
# Initial:  [-172.6669557315482, 0.23170476390131117, 93.87854586635889]
# Clean:  [-180, 0, 90]
# ---

# Brick:  b9  (Vertical: True )
# Initial:  [179.43701764607906, 1.9695292014695087, 94.97541131205563]
# Clean:  [-180, 0, 90]
# ---

# startv = Pose()
# startv.position.x = 0.474118380271
# startv.position.y = 0.708592081278
# startv.position.z = 0.254463995665
# startv.orientation.x = 0.646837059517
# startv.orientation.y = 0.762613332443
# startv.orientation.z = -0.00349921264719
# startv.orientation.w = 0.00323714782189

# ## layer 1
# # brick 1
# brick1 = Pose()
# brick1.position.x = 0.614432692238
# brick1.position.y = 0.194920200078
# brick1.position.z = -0.131955624179
# brick1.orientation.x = 0.656912926072
# brick1.orientation.y = 0.753670267934
# brick1.orientation.z = -0.0139518496021
# brick1.orientation.w = -0.0158707493391

# # brick 2
# brick2 = Pose()
# brick2.position.x = 0.618107604964
# brick2.position.y = 0.346427058986
# brick2.position.z = -0.12642347212
# brick2.orientation.x = 0.667702789864
# brick2.orientation.y = 0.742295147084
# brick2.orientation.z = -0.0450738163396
# brick2.orientation.w = 0.0337527792232

# # brick 3
# brick3 = Pose()
# brick3.position.x = 0.615060266097
# brick3.position.y = 0.532057264536
# brick3.position.z = -0.122815510307
# brick3.orientation.x = 0.689053473867
# brick3.orientation.y = 0.724504622211
# brick3.orientation.z = 0.00896708942344
# brick3.orientation.w = 0.0147632602552

# ##horizontal 1
# #pick up position
# starth = Pose()
# starth.position.x = 0.464710515625
# starth.position.y = 0.804733475267
# starth.position.z = 0.116929175434
# starth.orientation.x = 0.661218573375
# starth.orientation.y = 0.748928202611
# starth.orientation.z = 0.0430656607147
# starth.orientation.w = -0.00647259034557

# #brick 4
# brick4 = Pose()
# brick4.position.x = 0.622954841097
# brick4.position.y = 0.240541825386
# brick4.position.z = -0.0499969605648
# brick4.orientation.x = 0.692591952198
# brick4.orientation.y = 0.720363275918
# brick4.orientation.z = -0.0303026653131
# brick4.orientation.w = -0.0217919006618


# #brick 5
# brick5 = Pose()
# brick5.position.x = 0.60878876974
# brick5.position.y = 0.449420093325
# brick5.position.z = -0.0582220466391
# brick5.orientation.x = 0.677785576319
# brick5.orientation.y = 0.734386540736
# brick5.orientation.z = -0.0320204279271
# brick5.orientation.w = -0.0160565723407

# ##layer 2
# #brick 6
# brick6 = Pose()
# brick6.position.x = 0.654521558648
# brick6.position.y = 0.247991453631
# brick6.position.z = 0.123275208559
# brick6.orientation.x = 0.656490490121
# brick6.orientation.y = 0.753242934575
# brick6.orientation.z = -0.0405574649753
# brick6.orientation.w = 0.00064025573316

# #brick 7
# brick7 = Pose()
# brick7.position.x = 0.621248794776
# brick7.position.y = 0.441610520059
# brick7.position.z = 0.138989144559
# brick7.orientation.x = 0.671416445783
# brick7.orientation.y = 0.73860918343
# brick7.orientation.z = -0.0602439761379
# .orientation.w = 0.00520517280538

# ##horizontal 2
# #brick 8
# brick8 = Pose()
# brick8.position.x = 0.637564580857
# brick8.position.y = 0.334499476763
# brick8.position.z = 0.209965609695
# brick8.orientation.x = 0.680984543038
# brick8.orientation.y = 0.729464268389
# brick8.orientation.z = -0.0481331171548
# brick8.orientation.w = -0.0427216142147

# ##layer 3
# #brick 9
# brick9 = Pose()
# brick9.position.x = 0.635704106485
# brick9.position.y = 0.349136096976
# brick9.position.z = 0.406618422176
# brick9.orientation.x = 0.674781286383
# brick9.orientation.y = 0.737779159737
# brick9.orientation.z = -0.00827498306978
# brick9.orientation.w = 0.016841961284













