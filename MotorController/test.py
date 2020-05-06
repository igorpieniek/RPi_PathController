from myserial import *
from MainController import *
import math
import numpy as np
import time


com = MotorControler('COM6')
main = MainController(com)

def addAngle(array):
    outarray = []
    for index, point in enumerate(array):
        if index == len(array)-1: break
        if array[index+1][0] - point[0] == 0 and array[index+1][1] - point[1] > 0 : angle = math.pi/2
        elif array[index+1][0] - point[0] == 0 and array[index+1][1] - point[1] < 0 : angle = -math.pi/2
        else: 
            angle = math.atan( (array[index+1][1] - point[1]) / ( array[index+1][0] - point[0] ) )
            if ( array[index+1][0] - point[0] ) < 0 : angle += math.pi 
       

        outarray.append([point[0], point[1], angle])
    return outarray

path = np.array([[0,0,],
                 [-1,0,],
                 [-2,0,],
                 [-3,0,],
                 [-4,0,],
                 [-4,1,],
                 [-4,2,],
                 [-4,3,],
                 [-4,4,],
                 [-3,4,],
                 [-2,3.5,],
                 [-1,3,],
                 [0,3,],
                 [0,4,],
                 [0,5,],
                 [-0.5,5,],
                 [-1.5,5.5,],
                 [-2,6,],
                 [-3,6,],
                 [-4,6,],
                 ])

path = [[a[0]*0.3, a[1]*0.3] for a in path] # ustalona wielkośc 'kratek' na 5 cm
path = addAngle(path)


firstIT = True
for pa in path:
    if firstIT:
        main.addPathPoint(Position(pa[0], pa[1], math.radians(90)),'POSITION')
        firstIT = False
    else: main.addPathPoint(Position(pa[0], pa[1], pa[2] ),'PATH') 

status = main.addPathPoint(Position(path[-1][0],path[-1][1],path[-1][2]),'PATH_END')

while status:
    output = main.mainProcess()
    time.sleep(0.04)
    if not output['status']: 
        print('KONIEC SCIEZKI KONIEC TESTU')
        break

del com