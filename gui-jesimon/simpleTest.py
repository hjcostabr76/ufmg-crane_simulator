# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
import sys
import numpy as np
import matplotlib.pyplot as mpl
try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time

def AtuadorBraco(clientID,braco,val):
    newVal = val * 0.1
    sim.simxSetJointTargetVelocity(clientID,braco,newVal,sim.simx_opmode_oneshot)
    time.sleep(0.2)
    sim.simxSetJointTargetVelocity(clientID,braco,0,sim.simx_opmode_oneshot)

def AtuadorGuindaste(clientID,guindaste,val):
    newVal = val * 0.1
    sim.simxSetJointTargetVelocity(clientID,guindaste,newVal,sim.simx_opmode_oneshot)
    time.sleep(0.2)
    sim.simxSetJointTargetVelocity(clientID,guindaste,0,sim.simx_opmode_oneshot)

def AtuadorGarra(clientID,garra,val):
    newVal = val * 0.1
    sim.simxSetJointTargetVelocity(clientID,garra,newVal,sim.simx_opmode_oneshot)
    time.sleep(0.2)
    sim.simxSetJointTargetVelocity(clientID,garra,0,sim.simx_opmode_oneshot)

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections

# clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim
clientID=sim.simxStart('127.0.0.1', 19999,True,True,5000,5) # Connect to CoppeliaSim
print('clientID: ', clientID)

if clientID!=-1:
    print ('Connected to remote API server')

    # Get Coppelia Objects ID
    braco =  sim.simxGetObjectHandle(clientID,'Atuador_braco',sim.simx_opmode_blocking)[-1]
    garra = sim.simxGetObjectHandle(clientID,'Atuador_garra',sim.simx_opmode_blocking)[-1] 
    guindaste = sim.simxGetObjectHandle(clientID,'Atuador_guindaste',sim.simx_opmode_blocking)[-1]
    ima = sim.simxGetObjectHandle(clientID,'suctionPad',sim.simx_opmode_blocking)[-1]
    camera = sim.simxGetObjectHandle(clientID,'Vision_sensor',sim.simx_opmode_blocking)[-1]
    proximity_sensor = sim.simxGetObjectHandle(clientID,'Proximity_sensor',sim.simx_opmode_blocking)[-1]
    err_code,resolution,image = sim.simxGetVisionSensorImage(clientID,camera,0,sim.simx_opmode_streaming)
    status = sim.simxReadProximitySensor(clientID,proximity_sensor,sim.simx_opmode_streaming)[1]

     # Enables interactive mode for imshow
    mpl.ion()
     
    try:
        while(1):
            command = input('Command: ')

            # Move hoist up
            if command == 'w':
                AtuadorGuindaste(clientID,guindaste,1)
            # Move hoist down
            if command == 's':
                AtuadorGuindaste(clientID,guindaste,-1)
            # Move arm right
            if command == 'a': 
                AtuadorBraco(clientID,braco,1)
            # Move arm left
            if command == 'd':
                AtuadorBraco(clientID,braco,-1)
            # Activate/Deactivate magnet 
            if command == 'e':
                sim.simxCallScriptFunction(clientID,'Base',sim.sim_scripttype_childscript,'AtuadorIma',[],[],[],bytearray(),sim.simx_opmode_blocking)
            # Move grap up
            if command == 'q':
                AtuadorGarra(clientID,garra,1)
            # Move grap down
            if command == 'r':
                AtuadorGarra(clientID,garra,-1)

            # Save or print Sensor Vision Image
            err_code,resolution,image = sim.simxGetVisionSensorImage(clientID,camera,0,sim.simx_opmode_buffer)
            img = np.array(image, dtype = np.uint8)
            img.resize([resolution[0],resolution[1],3])
            mpl.imshow(img,origin='lower')
            plot = True
            if plot:
                mpl.show()
            else:
                mpl.savefig("teste.jpg")


            # Get proximity sensor status
            status = sim.simxReadProximitySensor(clientID,proximity_sensor,sim.simx_opmode_buffer)
            print("\nStatus: ",status,'\n')
    
    except KeyboardInterrupt:
        sim.simxFinish(clientID)
        sys.exit()

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
