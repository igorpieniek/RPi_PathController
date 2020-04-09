from myserial import *
from MainController import *
import math

com = MotorControler('COM3')

main = MainController(com)

main.addPathPoint(Position(0, 0, 0),'POSITION')

for i in range(10):
    main.addPathPoint(Position(0, -i, math.radians(80) ),'PATH') 

main.addPathPoint(Position(10, 10, 0.3),'PATH_END')

main.mainProcess()