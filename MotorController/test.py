from myserial import *
from MainController import *
import math
import numpy as np

com = MotorControler('COM3')
main = MainController(com)

def addAngle(array):
    outarray = []
    for index, point in enumerate(array):
        if index == len(array)-1: break
        if array[index+1][0] - point[0] == 0 and array[index+1][1] - point[1] > 0 : angle = math.pi/2
        elif array[index+1][0] - point[0] == 0 and array[index+1][1] - point[1] < 0 : angle = -math.pi/2
        else: angle = math.atan( (array[index+1][1] - point[1]) / ( array[index+1][0] - point[0] ) )
        outarray.append([point[0], point[1], angle])
    return outarray

path = np.array([[0,0,],
                 [0,1],
                 [1,3],
                 [2,4],
                 [3,5],
                 [4,6],
                 [5,7],
                 [6,7],
                 [7,7],
                 [8,7],
                 [9,7],
                 [10,7],
                 [11,6],
                 [11,5],
                 [12,4],
                 ])

path = [[a[0]*0.05, a[1]*0.05] for a in path] # ustalona wielkośc 'kratek' na 5 cm
path = addAngle(path)


firstIT = True
for pa in path:
    if firstIT:
        main.addPathPoint(Position(pa[0], pa[1], pa[2]),'POSITION')
        firstIT = False
    else: main.addPathPoint(Position(pa[0], pa[1], pa[2] ),'PATH') 

status = main.addPathPoint(Position(path[-1][0],path[-1][1],path[-1][2]),'PATH_END')

while status:
    output = main.mainProcess()
    if not output['status']: break