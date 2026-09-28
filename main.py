import sim
import sys
import numpy as np
import keyboard
import time
import m_p3dx as p3dx


state_control = 0


def connect():
    sim.simxFinish(-1)  # just in case, close all opened connections

    clientID = sim.simxStart('127.0.0.1', 1998, True, True, 5000, 5)  # Connect to CoppeliaSim

    if clientID != -1:
        print('Connected to remote API server')
    else:
        print('Failed connecting to remote API server')
        sys.exit('Could not connect')

    return clientID


def object_distNor_estimate(sensor):
    #? in model use sensor 0 to 15
    d = min(sensor[3], sensor[4])
    q = sensor[5] - sensor[2]

    return d, q


def kontroler(d, q):
    """
    Reverse object-following controller.

    Instead of driving the robot toward the object to hold it at
    dRef (normal following), this pushes the robot AWAY from the
    object: it backs up faster the closer the object gets, and it
    turns away from the object's bearing instead of toward it.
    """
    v_max = 0.3
    w_max = 0.2

    #? Parameters (signs flipped vs. normal following -> reverse/evasive behavior)
    K1 = -0.5     # negative: object closer than dRef -> v < 0 (reverse)
    K2 = 0.01     # was -0.01 (flipped: turn away instead of toward)

    dRef = 0.5
    QRef = 0.0

    #? P1: linear velocity
    # Object closer than dRef -> back away (v < 0).
    # Object farther than dRef -> stop (never drive forward toward it).
    if d < dRef:
        v = K1 * (dRef - d)
    else:
        v = 0.0

    #? P2: angular velocity (now turns AWAY from the object's bearing)
    w = K2 * (QRef - q)

    #? Saturation
    if v > v_max:
        v = v_max
    elif v < -v_max:
        v = -v_max

    if w > w_max:
        w = w_max
    elif w < -w_max:
        w = -w_max

    return v, w


def keyboard_control():
    ret_velocity = [0.0, 0.0]
    if keyboard.is_pressed('a'):
        ret_velocity[1] = 0.02
        ret_velocity[0] = 0.0
    elif keyboard.is_pressed('d'):
        ret_velocity[1] = -0.02
        ret_velocity[0] = 0.0
    elif keyboard.is_pressed('w'):
        ret_velocity[0] = 0.1
        ret_velocity[1] = 0.00
    elif keyboard.is_pressed('s'):
        ret_velocity[0] = -0.1
        ret_velocity[1] = 0.0
    else:
        ret_velocity[0] = 0.0
        ret_velocity[1] = 0.0
    return ret_velocity


## ------------ Main Program ----------------
print('Main Program Started')

#? Connect to the simulator
clientID = connect()

#? Get the sensors handles
sensorsHandle = p3dx.getSensorsHandle(clientID)
#? Get the motors handles
motorsHandle = p3dx.getMotorHandle(clientID)

velRobot = [0.0, 0.0]   # [m/s, rad/s]
_velRobot = [0.0, 0.0]  # [m/s, rad/s]

#? Set print float precision to 2 digits after decimal point
np.set_printoptions(precision=2)


## ---------- Routine 100 Hz ----------------
while True:

    sensorValue = p3dx.getDistance(clientID, sensorsHandle)
    #? Print sensor 2 to 5
    # print(sensorValue[2:6])

    #? Estimate distance and orientation
    d, q = object_distNor_estimate(sensorValue)

    if state_control == 1:
        #? Reverse Object Following Control
        velRobot[0], velRobot[1] = kontroler(d, q)
    else:
        #? Keyboard Control
        velRobot = keyboard_control()

    if keyboard.is_pressed('c'):
        state_control = 1 - state_control
        time.sleep(0.3)
        if state_control == 1:
            print('Switched to Reverse Object Following Control')
        else:
            print('Switched to Keyboard Control')

    print('d = %.2f, q = %.2f, v = %.2f, w = %.2f' % (d, q, velRobot[0], velRobot[1]))

    #? Acceleration and deceleration
    _velRobot[0] = 0.1 * velRobot[0] + 0.9 * _velRobot[0]
    _velRobot[1] = 0.1 * velRobot[1] + 0.9 * _velRobot[1]

    #? Robot kinematics
    p3dx.robotWheelKine(clientID, motorsHandle, _velRobot)

    #? Press q to quit
    if keyboard.is_pressed('q'):
        _velRobot[0] = 0.0
        _velRobot[1] = 0.0
        p3dx.robotWheelKine(clientID, motorsHandle, _velRobot)
        break

    time.sleep(0.01)


#? Stop the simulation
sim.simxFinish(clientID)
print('Main Program Ended')