from PositionController import *
import matplotlib.pyplot as plt
import numpy as np 
import math

class MainController(object):
    """description of class"""
    def __init__(self, motorControlerObj):     
        self.__motorControler = motorControlerObj
        self.__positionControl = PositionController(motorControlerObj)

        self.__path = []
        self.__rawPath = []
        self.__setMargin()

        self.__k_st = 8 # parametr w poprawce stanleya <1,10>
        self.__k_con = 0.7 # parametr przy poprawce sterowania skretu <0.5, 1>
        self.__Vx  = 0.05 # wymagana predkosci pojazdu - przydaje sie przy obliczeniach kata ale nie przy jego zadawniu
        self.__MAXpercentage = 85 # maksymalne wypelnienie
        self.__MINpercentage = 45
        self.__motorOffset = {'L':-5, 'R':5}
        self.plotInit()

    def plotInit(self):
        plt.axis([-2, 2, -2, 2])

    def plotUpdate(self, x,y, color):
        plt.scatter(x,y, c= color)
        plt.pause(0.01)


    #---------------MAIN PROCESS------------------------------------------
    def mainProcess(self):
        pos = self.__getPosition()
        print(pos)
        self.plotUpdate( pos.x,  pos.y, 'red') # update plot
        pathPoint, dist = self.__getClosestPathPoint(pos)
        if not pathPoint :
            self.__stopMotors()
            return {'status': False }
        self.plotUpdate( pathPoint.x,  pathPoint.y, 'blue')
        print('Current aim:', str(pathPoint), 'current dist: ', str(dist))
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
            for line in self.__rawPath: print(str(line[0]), str(line[1]), str(math.degrees(line[2] ) ) )
            self.__convertPath()
            return True
        else: raise NameError('NO SUCH ID NAME AS: '+ id)

        return False


    def __resetPath(self):
        self.__path = []
        self.__rawPath = []

    def __convertPath(self):
        if self.__rawPath:
            pLoc = Position(x = self.__rawPath[0][0], y = self.__rawPath[0][1], angle =  self.__rawPath[0][2] )
            print('PATH CONVERSION START')
            tempPath = self.__rawPath[1:]
            path = []
            for line in tempPath:
                conv_coord = self.__globalToLocalCoordinates(pLoc, Position(x = line[0], y = line[1], angle = line[2] ) )
                self.plotUpdate( conv_coord.x,  conv_coord.y, 'black')
                print(conv_coord)
                path.append(conv_coord)
            self.__path = path

    def __setMargin(self):
        #TODO: add method of margin calculation
        self.__margin = 0.02
                                    
    def __mainConversion(self, curentPos, pathPoint, dist):

        # Stanley
        fi_e = -(curentPos.angle - pathPoint.angle)
        #delta = fi_e + math.atan( (self.__k_st * dist) / self.__Vx ) #na ten moment wylaczona ze wzgledu na dziwne wyniki
        VR =  (math.cos(fi_e) + self.__k_con * math.sin(fi_e))
        VL =  (math.cos(fi_e) - self.__k_con * math.sin(fi_e))

        # Konwersja na procenty
        VR = -self.__percentageConversion(VR, 'R')
        VL = -self.__percentageConversion(VL, 'L')
        print('Kontroler out: ',' VR = '+ str(-VR)+ ' VL = '+ str(-VL),' fie= '+ str(math.degrees(fi_e)),)# ' delta= '+ str(math.degrees(delta)) )
        return {'R': VR, 'L': VL}

    def __percentageConversion(self, V, motor):
        max_value = (math.cos(math.radians(45)) + self.__k_con * math.sin(math.radians(45)))
        if V <= max_value and V >=0: return int( (((V * self.__MAXpercentage) / max_value)/2) +  self.__MINpercentage ) + self.__motorOffset[motor]
        elif  V >= -max_value and V <=0: return int( (((V * self.__MAXpercentage) / max_value)/2) -  self.__MINpercentage ) - self.__motorOffset[motor]
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
        if not self.__path: return [[], None]
        for pathPoint in self.__path: #poszukiwanie punktu na sciezce ktory jest najblizej
            dist = self.__get2PointsDist( current_pos.x, current_pos.y, pathPoint.x,  pathPoint.y)
            if min_dist == None or min_dist > dist : min_dist, closestPathPoint = dist, pathPoint

        if closestPathPoint == self.__path[-1]: return [[], None]
        if not min_dist == None and min_dist <= self.__margin: # zapobieganie blokowaniu sie na jednym punkcie sciezki
            self.__path.remove(closestPathPoint) # usuniecie punktu sciezki ktory jest za blisko
            closestPathPoint, min_dist = self.__getClosestPathPoint(current_pos) # powtorzenie dzialania funkcji
        
        return [closestPathPoint, min_dist]

    def __getPosition(self):
        return self.__positionControl.getCoordinates()

    def __get2PointsDist(self,x1,y1,x2,y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    
    def __globalToLocalCoordinates(self, pointRef, point  ):
        x_out =  (point.x - pointRef.x ) * math.cos( pointRef.angle - math.radians(90) ) + (point.y - pointRef.y) * math.sin( pointRef.angle - math.radians(90)) 
        y_out = -(point.x - pointRef.x ) * math.sin( pointRef.angle - math.radians(90) ) + (point.y - pointRef.y) * math.cos( pointRef.angle - math.radians(90) ) 
        angle_out = point.angle - pointRef.angle  + math.radians(90)
        return Position(x = x_out, y = y_out, angle = angle_out )

   


