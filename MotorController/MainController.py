from PositionController import *
#from myserial import MotorControler
import math

class MainController(object):
    """description of class"""
    def __init__(self, path, motorControlerObj):
        self._rawPath = path
        self._positionControl = PositionController()
        self._path = self._convertPath()
        self._setMargin()
        self._motorControler = motorConrolerObj

        self._k_st = 9 # parametr w poprawce stanleya <1,10>
        self._k_con = 0.7 # parametr przypoprawce sterowania skrętu <0.5, 1>
        self._Vx  = 0.05 # wymagana prędkosci pojazdu - przydaje sie przy obliczeniach kąta ale nie przy jego zadawniu
        self._MAXpercentage = 50 # maksymalne wypełnienie
    #---------------MAIN PROCESS------------------------------------------
    def mainProcess(self):
        pos = self._getPosition()
        pathPoint = self._getClosestPathPoint(pos)
        if pathPoint == None:
            self._stopMotors()
            return
        vel = self._mainConversion(pos, pathPoint)
        self._setMotorsPWM(vel)

    #---------------CONVERTION FUNCTIONS------------------------------------------
    # Konwertuje sciezke na sciezke w lokalnym ukladzie wspolrzednych pierwszego punktu sciezki    
    def _convertPath(self):
        x_loc = self._rawPath[0][0]
        y_loc = self._rawPath[0][1]
        angle = math.radians( self._rawPath[0][2] ) 
        tempPath = self._rawPath[1:]
        path = []
        for line in tempPath:
            conv_coord = self._globalToLocalCoordinates(x,loc, y_loc, angle, line[0], line[1], line[2])
            path.append(conv_coord)
        return path

    def _setMargin(self):
        #TODO: add method of margin calculation
        self._margin = 0.05
                                    
    def _mainConversion(self, curentPos, pathPoint):

        temp = pathPoint.copy()
        temp.pop('dist')
        if temp == self._path[-1]:
            pass

        # Stanley
        fi_e = curentPos['angle'] - pathPoint['angle']
        delta = fi_e + math.atan( self._k_st * pathPoint['dist'] / self._Vx )
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
            dist = self._get2PointsDist( current_pos['x'], current_pos['y'], current_pos['angle'], 
                                         pathPoint['x'],   pathPoint['y'],   pathPoint['angle'])
            if min_dist == None or min_dist > dist : min_dist, closestPathPoint = dist, pathPoint


        if not min_dist == None and min_dist < self._margin: # zapobieganie blokowaniu się na jednym punkcie sciezki
            self._path.remove(closestPathPoint) # usuniecie punktu sciezki ktory jets za blisko
            closestPathPoint = self._getClosestPathPoint(current_pos) # powtorzenie dzialania funkcji
        else: closestPathPoint.update({'dist': min_dist})

        return closestPathPoint

    def _getPosition(self):
        return self._positionControl.getCoordinates()

    def _get2PointsDist(self,x1,y1,x2,y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    
    def _globalToLocalCoordinates(self, x_ref, y_ref, angle_ref, x,y, angle  ):
        x_out =  (x - x_ref) * math.cos(angle_ref) + (y - y_ref) * math.sin(angle_ref) 
        y_out = -(x - x_ref) * math.sin(angle_ref) + (y - y_ref) * math.cos(angle_ref) 
        angle_out = angle - angle_ref
        return {'x':x_out, 'y': y_out, 'angle': angle_out}

   


