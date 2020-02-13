from math import radians

table = {
    'id':'t1',
    'rframe':'world',
    'x':0.85,
    'y':0.20,
    'z':0,
    'roll':0,
    'pitch':0,
    'yaw':radians(90)
    }

bricks_end =[{
    'id':'f1',
    'rframe':'t1',
    'x':0.26,
    'y':-0.19,
    'z':0.822,
    'roll':0,
    'pitch':radians(90),
    'yaw':radians(90)
    },
    {
    'id':'f2',
    'rframe':'f1',
    'x':0,
    'y':0,
    'z':0.186,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'id':'f3',
    'rframe':'f1',
    'x':0,
    'y':0,
    'z':0.372,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'id':'g1',
    'rframe':'f1',
    'x':-0.127,
    'y':0,
    'z':0.085,
    'roll':0,
    'pitch':1.57,
    'yaw':0
    },
    {
    'id':'g2',
    'rframe':'f1',
    'x':-0.127,
    'y':0,
    'z':0.287,
    'roll':0,
    'pitch':1.57,
    'yaw':0
    },
    {
    'id':'h1',
    'rframe':'f1',
    'x':-0.254,
    'y':0,
    'z':0.093,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'id':'h2',
    'rframe':'f1',
    'x':-0.254,
    'y':0,
    'z':0.287,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'id':'i1',
    'rframe':'f1',
    'x':-0.381,
    'y':0,
    'z':0.186,
    'roll':0,
    'pitch':1.57,
    'yaw':0
    },
    {
    'id':'j1',
    'rframe':'f1',
    'x':-0.508,
    'y':0,
    'z':0.186,
    'roll':0,
    'pitch':0,
    'yaw':0
    }]

target_positions = [{
    'num':1,
    'rframe':'t1',
    # 'x':0.46,
    'x':0.2,
    # 'y':-0.19,
    'y':0.2,
    'z':0.9,
    'roll':radians(90),
    # 'pitch':-1.570796,
    'pitch':0,
    # 'yaw':radians(90)
    'yaw':radians(90)
    },
    {
    'num':2,
    'rframe':'a1',
    'x':0,
    'y':0,
    'z':-0.186,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'num':3,
    'rframe':'a1',
    'x':0,
    'y':0,
    'z':-0.372,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'num':4,
    'rframe':'a1',
    'x':0.127,
    'y':0,
    'z':-0.085,
    'roll':0,
    'pitch':1.57,
    'yaw':0
    },
    {
    'num':5,
    'rframe':'a1',
    'x':0.127,
    'y':0,
    'z':-0.287,
    'roll':0,
    'pitch':1.57,
    'yaw':0
    },
    {
    'num':6,
    'rframe':'a1',
    'x':0.254,
    'y':0,
    'z':-0.093,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'num':7,
    'rframe':'a1',
    'x':0.254,
    'y':0,
    'z':-0.287,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'num':8,
    'rframe':'a1',
    'x':0.381,
    'y':0,
    'z':-0.186,
    'roll':0,
    'pitch':1.57,
    'yaw':0
    },
    {
    'num':9,
    'rframe':'a1',
    'x':0.508,
    'y':0,
    'z':-0.186,
    'roll':0,
    'pitch':0,
    'yaw':0
    }]

bricks_start_v1 =[{
    'id':'a1',
    'rframe':'t1',
    'x':-0.419,
    'y':0.134,
    'z':0.770,
    'roll':radians(90),
    'pitch':0,
    'yaw':radians(90)
    },
    {
    'id':'a2',
    'rframe':'a1',
    'x':0,
    'y':0,
    'z':0.192,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'id':'a3',
    'rframe':'a1',
    'x':0,
    'y':0,
    'z':0.384,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'id':'b1',
    'rframe':'a1',
    'x':0,
    'y':0.087,
    'z':0,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'id':'b2',
    'rframe':'a1',
    'x':0,
    'y':0.087,
    'z':0.192,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    # {
    # 'id':'c1',
    # 'rframe':'a1',
    # 'x':0,
    # 'y':0.087,
    # 'z':0.384,
    # 'roll':0,
    # 'pitch':0,
    # 'yaw':0
    # },
    {
    'id':'c2',
    'rframe':'a1',
    'x':0,
    'y':0.175,
    'z':0,
    'roll':0,
    'pitch':0,
    'yaw':0
    },
    {
    'id':'d1',
    'rframe':'a1',
    'x':0,
    'y':0.175,
    'z':0.192,
    'roll':0,
    'pitch':0,
    'yaw':0
    }]#,
    # {
    # 'id':'e1',
    # 'rframe':'a1',
    # 'x':0,
    # 'y':0.175,
    # 'z':0.384,
    # 'roll':0,
    # 'pitch':0,
    # 'yaw':0
    # }]

bricks_start_v2 =[{
    'id':'a1',
    'rframe':'t1',
    'x':-0.119,
    'y':0.134,
    'z':0.830,
    'roll':radians(90),
    'pitch':radians(90),
    'yaw':radians(180)
    }]

def getBuildable():
    return bricks_start_v2

def getAll():
    return bricks_start_v2+bricks_start_v1+bricks_end+[table]

def getTable():
    return table

def getTargets():
    return bricks_end

def getNodes():
    nodes = ['baxter::base', 't1::Table']
    for obj in bricks_start_v2 + bricks_end:
        name = obj['id']
        node_name = name+'::Brick'
        nodes.append(node_name)
    return nodes

def getDirections():
    return target_positions

