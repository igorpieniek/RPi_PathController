from PositionController import *

import math

class MainController(object):
    """description of class"""
    def __init__(self, path, motorControlerObj):
        self._rawPath = path
        self._motorControler = motorConrolerObj
        self._positionControl = PositionController(motorConrolerObj)

        self._path = self._convertPath()
        self._setMargin()

        self._k_st = 9 # parametr w poprawce stanleya <1,10>
        self._k_con = 0.7 # parametr przy poprawce sterowania skretu <0.5, 1>
        self._Vx  = 0.05 # wymagana predkosci pojazdu - przydaje sie przy obliczeniach keta ale nie przy jego zadawniu
        self._MAXpercentage = 50 # maksymalne wypelnienie
    #---------------MAIN PROCESS------------------------------------------
    def mainProcess(self):
        pos = self._getPosition()
        pathPoint, dist = self._getClosestPathPoint(pos)
        if pathPoint == None:
            self._stopMotors()
            return {'status': False }
        vel = self._mainConversion(pos, pathPoint, dist)
        self._setMotorsPWM(vel)
        return {'status':True, 'VR': vel['R'], 'VL':vel['L']}
        

    #---------------CONVERTION FUNCTIONS------------------------------------------
    # Konwertuje sciezke na sciezke w lokalnym ukladzie wspolrzednych pierwszego punktu sciezki    
    def _convertPath(self):
        pLoc = Position(x = self._rawPath[0][0], y = self._rawPath[0][1], angle = math.radians( self._rawPath[0][2] ) )

        tempPath = self._rawPath[1:]
        path = []
        for line in tempPath:
            conv_coord = self._globalToLocalCoordinates(pLoc, Position(x = line[0], y = line[1], angle = line[2] ) )
            path.append(conv_coord)
        return path

    def _setMargin(self):
        #TODO: add method of margin calculation
        self._margin = 0.05
                                    
    def _mainConversion(self, curentPos, pathPoint, dist):

        # Stanley
        fi_e = curentPos.angle - pathPoint.angle
        delta = fi_e + math.atan( self._k_st * dist / self._Vx )
        VR =  (math.cos(delta) + self._k_con * math.sin(delta))
        VL =  (math.cos(delta) - self._k_con * math.sin(delta))
        # Konwersja na procenty
        VR = self._percentageConversion(VR)
        VL = self._percentageConversion(VL)
        return {'R': VR, 'L': VL}

    def _percentageConversion(self, V):
        max_value = (math.cos(math.radians(45)) + self._k_con * math.sin(math.radians(45)))
        if abs(V) <= max_value:
            percent = (V * self._MAXpercentage) / max_value
            return percent
        elif V < 0 : return -max_value
        else:        return max_value

    #---------------TOOLS FUNCTIONS------------------------------------------
    def _stopMotors(self):
        self._setMotorsPWM({'L': 0, 'R': 0})
    
    def _setMotorsPWM(self, vel):
        self._motorControler.SetPWMControl(vel['L'], vel['R'])

    def _getClosestPathPoint(self,current_pos):
        min_dist = None
        closestPathPoint = None
        for pathPoint in self._path: #poszukiwanie punktu na sciezce ktory jest najblizej
            dist = self._get2PointsDist( current_pos.x, current_pos.y, pathPoint.x,  pathPoint.y)
            if min_dist == None or min_dist > dist : min_dist, closestPathPoint = dist, pathPoint


        if not min_dist == None and min_dist < self._margin: # zapobieganie blokowaniu sie na jednym punkcie sciezki
            self._path.remove(closestPathPoint) # usuniecie punktu sciezki ktory jest za blisko
            closestPathPoint = self._getClosestPathPoint(current_pos) # powtorzenie dzialania funkcji
        
        return [closestPathPoint, min_dist]

    def _getPosition(self):
        return self._positionControl.getCoordinates()

    def _get2PointsDist(self,x1,y1,x2,y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    
    def _globalToLocalCoordinates(self, pointRef, point  ):
        x_out =  (point.x - pointRef.x ) * math.cos( pointRef.angle ) + (point.y - pointRef.y) * math.sin( pointRef.angle ) 
        y_out = -(point.x - pointRef.x ) * math.sin( pointRef.angle ) + (point.y - pointRef.y) * math.cos( pointRef.angle ) 
        angle_out = point.angle - pointRef.angle
        return Position(x = x_out, y = y_out, angle = angle_out )

   


