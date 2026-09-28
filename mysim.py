import sim
import sys
import numpy as np
import keyboard
import time

def connect():
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',1998,True,True,5000,5) # Connect to CoppeliaSim

    if clientID!=-1:
        print ('Connected to remote API server')
    else:
        print ('Failed connecting to remote API server')
        sys.exit('Could not connect')

    return clientID

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

def setVelocityUsingKeyboardvelocity(veloRef, keyA, keyB, keyC, keyD, veloID, veloStep, VeloMax, GainAcc, GainDec):
    if keyboard.is_pressed(keyA):
        veloRef[veloID] -= (veloStep*GainAcc)
        veloRef[veloID+1] += (veloStep*GainAcc)
        if veloRef[veloID]<-VeloMax:
            veloRef[veloID]=-VeloMax
        if veloRef[veloID+1]>VeloMax:
            veloRef[veloID+1]=VeloMax 
    elif keyboard.is_pressed(keyB):
        veloRef[veloID] += (veloStep*GainAcc)
        veloRef[veloID+1] -= (veloStep*GainAcc)
        if veloRef[veloID]>VeloMax:
            veloRef[veloID]=VeloMax
        if veloRef[veloID+1]<-VeloMax:
            veloRef[veloID+1]=-VeloMax
    elif keyboard.is_pressed(keyC):
        veloRef[veloID] += (veloStep*GainAcc)
        veloRef[veloID+1] += (veloStep*GainAcc)
        if veloRef[veloID]>VeloMax:
            veloRef[veloID]=VeloMax
        if veloRef[veloID+1]>VeloMax:
            veloRef[veloID+1]=VeloMax    
    elif keyboard.is_pressed(keyD):
        veloRef[veloID] -= (veloStep*GainAcc)
        veloRef[veloID+1] -= (veloStep*GainAcc)
        if veloRef[veloID]<-VeloMax:
            veloRef[veloID]=-VeloMax
        if veloRef[veloID+1]<-VeloMax:
            veloRef[veloID+1]=-VeloMax  
    else:
        if veloRef[veloID]>0:
            veloRef[veloID] -= (veloStep*GainDec)
            if veloRef[veloID]<0.0:
                veloRef[veloID]=0.0
        else:
            veloRef[veloID] += (veloStep*GainDec)
            if veloRef[veloID]>0.0:
                veloRef[veloID]=0.0
        if veloRef[veloID+1]>0:
            veloRef[veloID+1] -= (veloStep*GainDec)
            if veloRef[veloID+1]<0.0:
                veloRef[veloID+1]=0.0
        else:
            veloRef[veloID+1] += (veloStep*GainDec)
            if veloRef[veloID+1]>0.0:
                veloRef[veloID+1]=0.0
    return veloRef

def setRobotMotionUsingKeyboardvelocity(veloRef):
    setVelocityUsingKeyboardvelocity(veloRef, 'left', 'right', 'up', 'down', 0, 0.05, 1.5, 1, 2)
    return veloRef

def setVelocityUsingKeyboard(veloRef, keyFwd, keyRev, veloID, veloStep, VeloMax, GainAcc, GainDec):
    if keyboard.is_pressed(keyFwd):
        veloRef[veloID] += (veloStep*GainAcc)
        if veloRef[veloID]>VeloMax:
            veloRef[veloID]=VeloMax
    elif keyboard.is_pressed(keyRev):
        veloRef[veloID] -= (veloStep*GainAcc)
        if veloRef[veloID]<-VeloMax:
            veloRef[veloID]=-VeloMax
    else:
        if veloRef[veloID]>0:
            veloRef[veloID] -= (veloStep*GainDec)
            if veloRef[veloID]<0.0:
                veloRef[veloID]=0.0
        else:
            veloRef[veloID] += (veloStep*GainDec)
            if veloRef[veloID]>0.0:
                veloRef[veloID]=0.0
    return veloRef

def setRobotMotionUsingKeyboard(veloRef):
    setVelocityUsingKeyboard(veloRef, 'up', 'down', 0, 0.05, 1.0, 1, 2)
    setVelocityUsingKeyboard(veloRef, 'right', 'left', 1, 0.05, 1.0, 1, 1.5)
    return veloRef

def initInvKinematics():
    # wheelradius diameter roda 0.195
    wheelradius = 0.195/2.0
    # widthbody = as antar roda Pioneer P3DX
    widthbody = 0.28
    R2 = wheelradius/2.0
    RB = wheelradius/widthbody
    #forward kinematik
    kinemmat = np.matrix([[R2,R2],[RB,-RB]])
    #invers kinematik = forward . matriks Indentitas
    invkinemmat = kinemmat.I
    return invkinemmat

def solveKinematics(invkinemmat, velo):
    velomat = np.matrix([[velo[0]],[velo[1]]])
    veloangmat = invkinemmat * velomat
    return [veloangmat[0,0],veloangmat[1,0]]

def setRobotMotion(clientID, motorsHandle, veloCmd):
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[0], veloCmd[0], sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motorsHandle[1], veloCmd[1], sim.simx_opmode_oneshot)
    return 0

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
print('Program Started')
# --- Connection to Coppelia Simulator
client_id = connect()
# --- Object Handle
sensors_handle = getSensorsHandle(client_id)
motors_handle = getMotorHandle(client_id)
# --- Velocity Initialization
velo_init = [0.0, 0.0]
velo_cmd = [0.0, 0.0]
# --- Simulation Process
samp_time, n = 0.1, 1.0
# Do input Invers Kinematics model
inv_kinem_const = initInvKinematics()
time_start = time.time()
while(True):
    t_now = time.time() - time_start
    if t_now >= samp_time*n:
        # Get data from sensors
        object_distances = getDistance(client_id, sensors_handle)
        front_left_distance = object_distances[2]
        front_right_distance = object_distances[5]
        # Do Motion command using keyboard output velocity
        # setRobotMotionUsingKeyboardvelocity(velo_cmd)
        # Do Motion command using keyboard v and w
        setRobotMotionUsingKeyboard(velo_cmd)
        # Calculate inverse kinematics
        velo_ang_cmd = solveKinematics(inv_kinem_const, velo_cmd)
        # Set velocity command for robot motion
        setRobotMotion(client_id, motors_handle, velo_ang_cmd)
        # Update sample
        n += 1.0
        # print the information (if necessary)
        print('t = ', round(t_now, 2), 'front side of object distance = ', front_left_distance, '(L),', front_right_distance, '(R)',velo_cmd)
    if keyboard.is_pressed('esc'): 
        setRobotMotion(client_id, motors_handle, velo_init)
        break
# --- Simulation Finished
sim.simxFinish(client_id)
print('Program ended \n')
