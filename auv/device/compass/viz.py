import numpy as np
from transforms3d.euler import euler2mat

def fullTransform(euler: np.array, bodyFrame: np.array):
    yaw, pitch, roll = euler  # scalars
    rotMatrix = euler2mat(ai=yaw, aj=pitch, ak=roll, axes='szyx')
    worldFrame = rotMatrix @ bodyFrame
    print(f"YPR full transform is: {worldFrame}")
    return worldFrame

def yawTransform(euler: np.array, bodyFrame: np.array):
    yaw = float(euler[0])
    R_yaw = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])
    worldFrame = R_yaw @ bodyFrame
    print(f"Yaw only transform is: {worldFrame}")
    return worldFrame

def compare(YPR,BodyFrame):
    print("############################")

    yawin = YPR[0]
    pitchin = YPR[1]
    rollin = YPR[2]

    velocity_body = np.array([BodyFrame[0],BodyFrame[1],BodyFrame[2]])  # shape (3,)
    yaw = np.deg2rad(yawin)
    pitch = np.deg2rad(pitchin)
    roll = np.deg2rad(rollin)
    eulerAngle = np.array([yaw, pitch, roll])  # shape (3,)

    # Run transforms
    print(f"Inputs: yaw = {yawin}, pitch = {pitchin}, roll = {rollin}")
    print(f"Velocity before transform is: {velocity_body}")
    res1 = fullTransform(eulerAngle, velocity_body)
    res2 = yawTransform(eulerAngle, velocity_body)
    print("############################")
    print("\n")

# test 1
compare(YPR=[180,1,3],BodyFrame=[1,0,0])
compare(YPR=[180,359, 5],BodyFrame=[1,0,0])
compare(YPR=[180,3,1],BodyFrame=[1,0,0])
compare(YPR=[180,5,359],BodyFrame=[1,0,0])

# compare(YPR=[90,5,-179],BodyFrame=[1,0,0])
# compare(YPR=[90,3, 179],BodyFrame=[1,0,0])
# compare(YPR=[0,5,-179],BodyFrame=[1,0,0])
# compare(YPR=[0,3, 179],BodyFrame=[1,0,0])