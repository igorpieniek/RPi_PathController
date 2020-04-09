from myserial import *
from MainController import *

com = MotorControler('COM3')

main = MainController(com)

main.addPathPoint(Position(1, 1, 0),'POSITION')

for i in range(10):
    main.addPathPoint(Position(-i, 2+1.3+i/10, 0.6+i*0.3),'PATH') 

main.addPathPoint(Position(10, 10, 0.3),'PATH_END')

main.mainProcess()