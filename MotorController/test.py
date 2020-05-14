from myserial import *
from MainController import *
import math
import time


com = MotorControler('/dev/ttyACM0')

def singleMotorController(Vz,Vp, k):
    err = Vz-Vp
    Vout = Vz + (k * err)
    return Vout

def velocity(vel, k, it=1):
    Vz = vel
    VoutL = vel
    VoutR = vel
    for i in range(it):
        com.SetPWMControl( VoutL,  VoutR)
        raw = com.WaitOnMeasurement(1)
        vel = {'L': -raw[2], 'R': -raw[1]}
        VoutL = singleMotorController(Vz, vel['L'], k)
        VoutR = singleMotorController(Vz, vel['R'], k)
	print('currL: '+str(vel['L'])+' currR: '+str(vel['R'])+' VL: '+str(VoutL)+' VR: '+str(VoutR) )
	time.sleep(0.04)
