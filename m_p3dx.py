import sim
import numpy as np
from numpy.linalg import inv



# # # # # # # # # # # # # # # # #
#  Ultrasound Array PioneerP3DX #
#      Object Distance[x]       #
# # # # # # # # # # # # # # # # #
#    ╔══════════╦══════════╗    #
#    ║       11 ║ 12       ║    #
#    ║     10   ║   13     ║    #
#    ║   09     ║     14   ║    #
#  ╔═╣ 08       ║       15 ╠═╗  #
#  ║ ╠══════════╬══════════╣ ║  #
#  ║ ║ 07       ║       00 ║ ║  #
#  ╚═╣   06     ║     01   ╠═╝  #
#    ║     05   ║   02     ║    #
#    ║       04 ║ 03       ║    #
#    ╚══════════╩══════════╝    #
# # # # # # # # # # # # # # # # #
# ======= Main  Program ======= #
# ----------------------------- #



def getSensorsHandle(clientID):
    isNewCoppeliaSim = True
    sensorsHandle = np.array([])
    for i in range (16):
        if(isNewCoppeliaSim):
            sensorHandle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor['+str(i)+']', sim.simx_opmode_blocking)[1]
        else:
            sensorHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i+1), sim.simx_opmode_blocking)[1]
        # First call proximity sensor must use opmode_streaming
        _, _, _, _, _ = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
        sensorsHandle = np.append(sensorsHandle, sensorHandle)
        sensorsHandle = np.int32(sensorsHandle)
    return sensorsHandle

def getDistance(clientID, sensorsHandle):
    distances = np.array([])
    for i in range (16):
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, sensorsHandle[i], sim.simx_opmode_buffer)
        distance = detectedPoint[2]
        if detectionState == False:
            distance = 2.0
        distances = np.append(distances, distance)
    return distances
    
def getMotorHandle(clientID):
    isNewCoppeliaSim = True
    if (isNewCoppeliaSim):
        motorLeftHandle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking)[1]
        motorRightHandle = sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking)[1]
    else:
        motorLeftHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)[1]
        motorRightHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)[1]
    motorsHandle = (motorLeftHandle, motorRightHandle)
    return motorsHandle

def setWheelVel(clientID, motorsHandle, veloCmd):
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[0], veloCmd[0], sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[1], veloCmd[1], sim.simx_opmode_oneshot)
    return 0

def robotWheelKine(clientID,motorHandle,Vel):
    # Robot parameters
    r = 0.0975 # Wheel radius [m]
    L = 0.28 # widthbody
    
    # Robot kinematics
    r_2 = r/2
    r_2L = r_2*L

    matrixKine = np.array([[    r_2,    r_2 ],
                           [    -r_2L, r_2L ]])
    
    matrixKineInv = inv(matrixKine)
    velWheel = np.dot(matrixKineInv, Vel)
    # Set wheel velocity
    setWheelVel(clientID, motorHandle, velWheel)

    return 0



    
    