from PositionController import *

import math

class MainController(object):
    """description of class"""
    def __init__(self, motorControlerObj):     
        self.__motorControler = motorControlerObj
        self.__positionControl = PositionController(motorControlerObj)

        self.__path = []
        self.__rawPath = []
        self.__setMargin()

        self.__k_st = 9 # parametr w poprawce stanleya <1,10>
        self.__k_con = 0.7 # parametr przy poprawce sterowania skretu <0.5, 1>
        self.__Vx  = 0.05 # wymagana predkosci pojazdu - przydaje sie przy obliczeniach kata ale nie przy jego zadawniu
        self.__MAXpercentage = 100 # maksymalne wypelnienie
    #---------------MAIN PROCESS------------------------------------------
    def mainProcess(self):
        pos = self.__getPosition()
        pathPoint, dist = self.__getClosestPathPoint(pos)
        if pathPoint == None:
            self._stopMotors()
            return {'status': False }
        vel = self.__mainConversion(pos, pathPoint, dist)
        self.__setMotorsPWM(vel)
        return {'status':True, 'VR': vel['R'], 'VL':vel['L']}
        

    #---------------CONVERTION FUNCTIONS------------------------------------------
    # Konwertuje sciezke na sciezke w lokalnym ukladzie wspolrzednych pierwszego punktu sciezki 
    def addPathPoint(self, point , id):
        if id == 'POSITION':
            self.__stopMotors()
            self.__resetPath()
            self.__rawPath.append([point.x, point.y, point.angle])
        elif id == 'PATH':
            self.__rawPath.append([point.x, point.y, point.angle])
        elif id == 'PATH_END':
            print('PATH_END')
            for line in self.__rawPath: print(str(line))
            self.__convertPath()
            return True
        else: raise NameError('NO SUCH ID NAME AS: '+ id)

        return False


    def __resetPath(self):
        self.__path = []
        self.__rawPath = []

    def __convertPath(self):
        pLoc = Position(x = self.__rawPath[0][0], y = self.__rawPath[0][1], angle =  self.__rawPath[0][2] )
        print('PATH CONVERSION START')
        tempPath = self.__rawPath[1:]
        path = []
        for line in tempPath:
            conv_coord = self.__globalToLocalCoordinates(pLoc, Position(x = line[0], y = line[1], angle = line[2] ) )
            print('X: '+str(conv_coord.x)+' Y: '+str(conv_coord.y)+ ' angle: ' + str(conv_coord.angle) )
            path.append(conv_coord)
        self.__path = path

    def __setMargin(self):
        #TODO: add method of margin calculation
        self.__margin = 0.05
                                    
    def __mainConversion(self, curentPos, pathPoint, dist):

        # Stanley
        fi_e = curentPos.angle - pathPoint.angle
        delta = fi_e + math.atan( self.__k_st * dist / self.__Vx )
        if delta >= math.radians(90) or delta <= math.radians(-90):
            VR =  (math.cos(delta) + self.__k_con * math.sin(delta)) #test!!
            VL =  (-math.cos(delta) - self.__k_con * math.sin(delta))
        else:
            VR =  (math.cos(delta) + self.__k_con * math.sin(delta))
            VL =  (math.cos(delta) - self.__k_con * math.sin(delta))

        # Konwersja na procenty
        VR = self.__percentageConversion(VR)
        VL = self.__percentageConversion(VL)
        print('VR = '+ str(VR)+ ' VL = '+ str(VL))
        return {'R': VR, 'L': VL}

    def __percentageConversion(self, V):
        max_value = (math.cos(math.radians(45)) + self.__k_con * math.sin(math.radians(45)))
        if abs(V) <= max_value: return int( (V * self.__MAXpercentage) / max_value )
        elif V < 0 :  return -self.__MAXpercentage
        else:        return self.__MAXpercentage

    #---------------TOOLS FUNCTIONS------------------------------------------
    def __stopMotors(self):
        self.__setMotorsPWM({'L': 0, 'R': 0})
    
    def __setMotorsPWM(self, vel):
        self.__motorControler.SetPWMControl(vel['L'], vel['R'])

    def __getClosestPathPoint(self,current_pos):
        min_dist = None
        closestPathPoint = None
        for pathPoint in self.__path: #poszukiwanie punktu na sciezce ktory jest najblizej
            dist = self.__get2PointsDist( current_pos.x, current_pos.y, pathPoint.x,  pathPoint.y)
            if min_dist == None or min_dist > dist : min_dist, closestPathPoint = dist, pathPoint


        if not min_dist == None and min_dist < self.__margin: # zapobieganie blokowaniu sie na jednym punkcie sciezki
            self.__path.remove(closestPathPoint) # usuniecie punktu sciezki ktory jest za blisko
            closestPathPoint, min_dist = self.__getClosestPathPoint(current_pos) # powtorzenie dzialania funkcji
        
        return [closestPathPoint, min_dist]

    def __getPosition(self):
        return self.__positionControl.getCoordinates()

    def __get2PointsDist(self,x1,y1,x2,y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    
    def __globalToLocalCoordinates(self, pointRef, point  ):
        x_out =  (point.x - pointRef.x ) * math.cos( pointRef.angle ) + (point.y - pointRef.y) * math.sin( pointRef.angle ) 
        y_out = -(point.x - pointRef.x ) * math.sin( pointRef.angle ) + (point.y - pointRef.y) * math.cos( pointRef.angle ) 
        angle_out = point.angle - pointRef.angle
        return Position(x = x_out, y = y_out, angle = angle_out )

   


