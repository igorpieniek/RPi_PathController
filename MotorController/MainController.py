from PositionController import *
import numpy as np 
import math
import sys

class MainController(object):
    """description of class"""
    def __init__(self, motorControlerObj):     
        self.__motorControler = motorControlerObj
        self.__positionControl = PositionController(motorControlerObj)

        self.__path = []
        self.__rawPath = []
        self.__setMargin()

        self.__k_st = 1.5 # parametr w poprawce stanleya <1,10>
        self.__kpR = 0.1 # parametr P w regulatorze proporcjonalnym
        self.__kpL = 0.2 # parametr P w regulatorze proporcjonalnym

        self.__k_con = 1# parametr przy poprawce sterowania skretu <0.5, 1>
        self.__Vx  = 14 # wymagana predkosci pojazdu - przydaje sie przy obliczeniach kata ale nie przy jego zadawniu
        self.__MAXpercentage = 80 # maksymalne wypelnienie
        self.__MINpercentage = 42
        self.__motorOffset = {'L':0, 'R':0}
        
	self.__prevDist = None
	self.__prevPathPoint = None

        self.__file = open('/home/ubuntu/catkin_ws/src/path_controller/src/measurmentLog.txt', 'w')
    def __del__(self):
        self.__file.close()
    #---------------LOG DATA ---------------------------------------------
    def __logWrite(self, pos, pathPoint):
        toSave = [str(self.__refPos.x), str(self.__refPos.y), str(self.__refPos.angle),
                  str(pos.x), str(pos.y), str(pos.angle),
                  str(pathPoint.x), str(pathPoint.y), str(pathPoint.angle),
                  str(self.__fie)]
        toSave = ','.join(toSave)
        self.__file.write( toSave+'\n'  )
    #---------------MAIN PROCESS------------------------------------------
    def mainProcess(self):
        if not self.__path: return {'status': False}
        pos = self.__getPosition()
        print(pos)
        pathPoint, dist = self.__getClosestPathPoint(pos)
	if not self.__prevPathPoint== pathPoint: self.__prevDist = None
	self.__prevPathPoint = pathPoint 
	if self.__prevDist == None: self.__prevDist= dist
        if not pathPoint:
	    # print('STOPED - prev difference!', dist-self.__prevDist)
            self.__stopMotors()
            self.__resetPath()
            return {'status': False }

	if (float(dist) - float(self.__prevDist))>0.015 and dist>0.15:
	    print('MOTOR STOPPED - DIST TO GOAL >0.2 AND IT GROW')
            self.__stopMotors()
            self.__resetPath()
            return {'status': False }


        try: self.__logWrite(pos, pathPoint)
        except: print('Writting file error ', sys.exc_info()[0])
       

	print('Current aim:', str(pathPoint), 'current dist: ', str(round(dist,3)), ' prev ', str(self.__prevDist) )
        self.__prevDist = dist
        vel = self.__mainConversion(pos, pathPoint, dist)
        vel = self.__velocityController(vel)
	self.__setMotorsPWM(vel)

        return {'status':True, 'VR': vel['R'], 'VL':vel['L']}
        
    #---------------VELOCITY REAGULATOR------------------------------------------ 
    def __singleMotorController(self,Vz,Vp,k):
        if abs(Vp)==0: return int(Vz)
	err = Vz-Vp
        Vout = Vz + (k * err)
        return int(Vout)

    def __velocityController(self,Vz):
        Vp = self.__getVelocity()
        VoutL = self.__singleMotorController(Vz['L'],-Vp['L'], self.__kpL)
        VoutR = self.__singleMotorController(Vz['R'],-Vp['R'], self.__kpR)
    	print('CurrL: ',round(-Vp['L'],2), 'CurrR: ',round(-Vp['R'],2), 'VL', VoutL, 'VR', VoutR )
        return {'L': VoutL, 'R': VoutR}
    #---------------CONVERTION FUNCTIONS------------------------------------------
    # Konwertuje sciezke na sciezke w lokalnym ukladzie wspolrzednych pierwszego punktu sciezki 
    def addPathPoint(self, point , id):
        if id == 'STOP':
	    self.__stopMotors()
            self.__resetPath()
	    del self.__positionControl
            self.__positionControl = PositionController(self.__motorControler)
	    print('PROCESS STOPPED BY STOP MSG!')
	elif id == 'POSITION':
            self.__stopMotors()
            self.__resetPath()
            del self.__positionControl
            self.__positionControl = PositionController(self.__motorControler)
            self.__rawPath.append([point.x, point.y, point.z])
        elif id == 'PATH':
            self.__rawPath.append([point.x, point.y, point.z])
        elif id == 'PATH_END':
            print('PATH_END')
            for line in self.__rawPath: print(str(round(line[0],2)), str(line[1]), str(math.degrees(line[2] ) ) )
            self.__prevDist=None
	    self.__convertPath()
            return True
        else: raise NameError('NO SUCH ID NAME AS: '+ id)

        return False


    def __resetPath(self):
        print('PATH RESET\n')
        self.__path = []
        self.__rawPath = []

    def __convertPath(self):
        if self.__rawPath:
            self.__refPos = Position(x = self.__rawPath[0][0], y = self.__rawPath[0][1], angle =  self.__rawPath[0][2] )
            print('PATH CONVERSION START')
            tempPath = self.__rawPath[1:]
            path = []
            for line in tempPath:
                conv_coord = self.__globalToLocalCoordinates(self.__refPos, Position(x = line[0], y = line[1], angle = line[2] ) )
                print(conv_coord)
                path.append(conv_coord)
            self.__path = path

    def __setMargin(self):
        #TODO: add method of margin calculation
        self.__margin = 0.1
                                    
    def __mainConversion(self, curentPos, pathPoint, dist):
	
	V = self.__getVelocity()
	Vx = ((-V['R'])+ (-V['L']))*0.5*0.1 
	if Vx == 0: Vx=0.1
        # Stanley
        fi_e = -(curentPos.angle - pathPoint.angle)
        delta = fi_e + math.atan( (self.__k_st * dist) / float(Vx) ) #na ten moment wylaczona ze wzgledu na dziwne wyniki

        if math.fmod( int(abs(fi_e)/math.pi),2)==1:
            fi_e= math.fmod(fi_e,2*math.pi)-(2*math.pi)
        else: fi_e =  math.fmod(fi_e,math.pi)
	
	if fi_e > (0.5*math.pi): fi_e = 0.5*math.pi
	elif fi_e < -(0.5*math.pi): fi_e = -0.5*math.pi

        VR =  (math.cos( fi_e) + self.__k_con * math.sin(fi_e))
        VL =  (math.cos(fi_e) - self.__k_con * math.sin(fi_e))
	
        self.__fie = fi_e
        # Konwersja na procenty
        VR = -1*self.__percentageConversion(VR, 'R')
        VL = -1*self.__percentageConversion(VL, 'L')
        print('Kontroler out: ',' VL = '+ str(-VL)+ ' VR = '+ str(-VR),' fie= '+ str(math.degrees(fi_e))+ ' delta= '+ str(math.degrees(delta)) )
        return {'R': VR, 'L': VL}

    def __percentageConversion(self, V, motor):
        max_value = (math.cos(math.radians(45)) + self.__k_con * math.sin(math.radians(45)))
        diff  = self.__MAXpercentage - self.__MINpercentage
        if V <= max_value and V >0: return int( (V / max_value)*diff + self.__MINpercentage + self.__motorOffset[motor] )
        elif  V >= -max_value and V <0:return -1 * int( (-V / max_value)*diff + self.__MINpercentage + self.__motorOffset[motor] )
        elif V < 0 :  return -self.__MAXpercentage - self.__motorOffset[motor]
        elif V > 0 :  return self.__MAXpercentage + self.__motorOffset[motor]
	else: return 0
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

	print('MIN DIST = ', min_dist,'punkt:',str(closestPathPoint))
	if len(self.__path)>0:
            if closestPathPoint == self.__path[-1] and min_dist<=self.__margin:
	    	print('najblizszy puynkt to punkt ostatni z listy', str(closestPathPoint))
	    	return [[], None]
        if not min_dist == None and min_dist <= self.__margin: # zapobieganie blokowaniu sie na jednym punkcie sciezki      
            self.__path.remove(closestPathPoint)
            print('USUWAM PUNKT NA SCIEZCE:', str(closestPathPoint), 'zostalo punktow:', str( len(self.__path)))
	    closestPathPoint, min_dist = self.__getClosestPathPoint(current_pos) # powtorzenie dzialania funkcji
        
        return [closestPathPoint, min_dist]

    def __getPosition(self):
        return self.__positionControl.getCoordinates()

    def __getVelocity(self):
	try:return self.__positionControl.getVelocity()
        except:
		print('ERROR: POSITION CONTROLLER OBJECT DOESNT EXIST!!')
		self.__positionControl = PositionController(self.__motorControler)
		return self.__positionControl.getVelocity()


    def __get2PointsDist(self,x1,y1,x2,y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)  
    
    def __globalToLocalCoordinates(self, pointRef, point  ):
        x_out =  (point.x - pointRef.x ) * math.cos( pointRef.angle - math.radians(90) ) + (point.y - pointRef.y) * math.sin( pointRef.angle - math.radians(90)) 
        y_out = -(point.x - pointRef.x ) * math.sin( pointRef.angle - math.radians(90) ) + (point.y - pointRef.y) * math.cos( pointRef.angle - math.radians(90) ) 
        angle_out = point.angle - pointRef.angle  + math.radians(90)
      	
        #if math.fmod( int(abs(angle_out)/math.pi),2)==1:
	#	 angle_out= math.fmod(angle_out,2*math.pi)-(2*math.pi)
	#else: angle_out =  math.fmod(angle_out,math.pi)

        
	angle_out= math.fmod(angle_out,2*math.pi)
	

	return Position(x = x_out, y = y_out, angle = angle_out )

