import math
import numpy as np


class PositionController(object):
    def __init__(self, motorControler ):
        self.__motorControler = motorControler
        self.__lastLocation = Position(x = 0, y = 0, angle = math.pi/2)

        self.__timeout = 1 # timeout do wiadomosci
        self.__wheelbase = 0.2127 #rozstaw kol w [m]
        self.__wheelDiameter = 0.0898 # srednica kola [m]
        self.__impulsesPerRevolution= 1800 # ilosc impulsow enkodera na obrot(przelozenie * ilosc impulsow na obrot walu silnika)
        self.__maxValType = np.iinfo(np.uint16()).max
        self.__halfUint16 =  self.__maxValType//2 #polowa zakresu liczby 2bajtowej uint16

        self.__updateMeasurments()
        self.__lastMeasure = {'L' : self.__impulsesL, 'R' : self.__impulsesR}


    #-------------MAIN PROCESS FUNCTIONS-----------------------------------------------------------------------
    def getCoordinates(self):
        self.__updateMeasurments()
        SR = self.__encoderConversion('R') 
        SL = self.__encoderConversion('L')
        self.__lastLocation = self.__globalCoordinate(self.__lastLocation, SL, SR)
 
        return self.__lastLocation
    #-------------LOW LEVEL CONVERSION FUNCTIONS---------------------------------------------------------------
    def __getSign(self, diff):
        if diff > self.__halfUint16: return -1 # jazda do tylu
        else:                       return  1 # jazda do przodu   

    def __getDiff(self, current, last):
        diff = current - last
        if self.__getSign(diff) +1 : #przyrost dodatni
            numOfImpulses = self.__getSign(diff) * int(diff)  # realna wartosc zwrocona wraz ze znakiem w zaleznosci od kierunku jazdy
        else: #przyrost ujemny
            numOfImpulses = self.__getSign(diff) * int(1 + self.__maxValType - diff)
        return numOfImpulses

    def __getCurrentImpulses(self,motor):
        if motor=='L': return self.__impulsesL
        else:          return self.__impulsesR       

    def __encoderConversion(self, motor):
        currentImp = self.__getCurrentImpulses(motor)
        S = ( self.__getDiff(currentImp, self.__lastMeasure[motor]) / self.__impulsesPerRevolution) * math.pi* self.__wheelDiameter # imp /imp na obrot *pi* D
        self.__lastMeasure[motor] = currentImp
        return S
    #-------------HIGH LEVEL CONVERSION FUNCTIONS---------------------------------------------------------------
    # na podstawie przesuniecia drogi obu kol okresla przemieszczenie wzgledem ostatniego pomiaru
    def __localCoordinate(self,SL, SR):
        if SL == SR and SL == 0: return Position(x = 0, y = 0, angle = 0)
        elif SL == SR:
            return Position(x = 0, y = SL, angle = 0)
        else:
            fi = ( SL - SR ) / self.__wheelbase #w radianach 
            R = ( (SL * self.__wheelbase) /  (SL - SR) ) - (0.5* self.__wheelbase)
            y = R * math.sin( fi )
            x =  R * (1 - math.cos( fi ))
            fi *= -1 
            return Position(x = x, y = y, angle = fi )

    # okresla aktualna pozycje na podstawie poprzedniej pozycji oraz drogi jaka przebyly kola robota
    def __globalCoordinate(self, lastPos, SL, SR):
        loc = self.__localCoordinate(SL,SR)
        x_out = lastPos.x + ( loc.x * math.cos( lastPos.angle - math.radians(90)) ) - (loc.y * math.sin( lastPos.angle - math.radians(90)) )
        y_out = lastPos.y + ( loc.x * math.sin( lastPos.angle - math.radians(90)) ) + (loc.y * math.cos( lastPos.angle - math.radians(90)) )

        fi_out = loc.angle + lastPos.angle 
        return Position(x = x_out, y = y_out, angle = fi_out )

    def __updateMeasurments(self):
        self.__rawData = self.__motorControler.WaitOnMeasurement(5)
        if not self.__rawData[0]: raise NameError('ODBIOR DANYCH NIE DZIALA')
        self.__impulsesR = np.uint16(self.__rawData[3])
        self.__impulsesL = np.uint16(self.__rawData[4])




class Position(object):
    def __init__(self, x= None, y=None, angle = None):
        self.x = x
        self.y = y
        self.angle = angle

    def __str__(self):
       return 'Pos( '+ str(round(self.x, 4))+', ' + str(round(self.y,4))+ ', ' + str(round(math.degrees(self.angle),2))+ ' )'