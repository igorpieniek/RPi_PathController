import math
import numpy as np


class PositionController(object):
    def __init__(self, motorControler ):
        self._motorControler = motorControler
        self._lastLocation = Position(x = 0, y = 0, angle = 0)

        self._timeout = 1 # timeout do wiadomosci
        self._wheelbase = 0.2127 #rozstaw kol w [m]
        self._wheelDiameter = 0.0898 # srednica kola [m]
        self._impulsesPerRevolution= 3600 # ilosc impulsow enkodera na obrot(przelozenie * ilosc impulsow na obrot walu silnika)
        self._maxValType = np.iinfo(np.uint16()).max
        self._halfUint16 =  self._maxValType//2 #polowa zakresu liczby 2bajtowej uint16

        self._lastMeasure = {'L' : np.uint16(0), 'R' : np.uint16(0)}


    #-------------MAIN PROCESS FUNCTIONS-----------------------------------------------------------------------
    def getCoordinates(self):
        self._updateMeasurments()
        SR = self._encoderConversion('R') 
        SL = self._encoderConversion('L')
        self._lastLocation = self._globalCoordinate(self._lastLocation, SL, SR)
 
        return self._lastLocation
    #-------------LOW LEVEL CONVERSION FUNCTIONS---------------------------------------------------------------
    def _getSign(self, diff):
        if diff > self._halfUint16: return -1 # jazda do tylu
        else:                       return  1 # jazda do przodu   

    def _getDiff(self, current, last):
        diff = current - last
        if self._getSign(diff) +1 : #przyrost dodatni
            numOfImpulses = self._getSign(diff) * int(diff)  # realna wartość zwrocona wraz ze znakiem w zaleznosci od kierunku jazdy
        else: #przyrost ujemny
            numOfImpulses = self._getSign(diff) * int(1 + self._maxValType - diff)
        print(numOfImpulses)
        return numOfImpulses

    def _getCurrentImpulses(self,motor):
        if motor=='L': return self._impulsesL
        else:          return self._impulsesR       

    def _encoderConversion(self, motor):
        currentImp = self._getCurrentImpulses(motor)
        S = ( self._getDiff(currentImp, self._lastMeasure[motor]) / self._impulsesPerRevolution) * math.pi* self._wheelDiameter
        self._lastMeasure[motor] = currentImp
        return S
    #-------------HIGH LEVEL CONVERSION FUNCTIONS---------------------------------------------------------------
    # na podstawie przesuniecia drogi obu kol okresla przemieszczenie względem ostatniego pomiaru
    def _localCoordinate(self,SL, SR):
        if SL == SR and SL == 0: return Position(x = 0, y = 0, angle = 0)
        elif SL == SR:
            return Position(x = 0, y = SL, angle = 0)
        else:
            fi = ( SL - SR ) / self._wheelbase
            R = ( (SL * self._wheelbase) /  (SL - SR) ) - (0.5* self._wheelbase)
            y = R * math.sin( fi )
            x =  y/ math.tan( 0.5*( math.pi-fi) )
            fi *= -1
            return Position(x = x, y = y, angle = fi )

    # okresla aktualna pozycje na podstawie poprzedniej pozycji oraz drogi jaka przebyly kola robota
    def _globalCoordinate(self, lastPos, SL, SR):
        loc = self._localCoordinate(SL,SR)
        x_out = lastPos.x + ( loc.x * math.cos( lastPos.angle ) ) - (loc.y * math.sin( lastPos.angle ) )
        y_out = lastPos.y + ( loc.x * math.sin( lastPos.angle ) ) + (loc.y * math.cos( lastPos.angle ) )

        fi_out = loc.angle + lastPos.angle 
        return Position(x = x_out, y = y_out, angle = fi_out )

    def _updateMeasurments(self):
        self._rawData = self._motor_motorControler.WaitOnMeasurement(1)
        self._impulsesL = np.uint16(self._rawData[3])
        self._impulsesR = np.uint16(self._rawData[4])




class Position(object):
    def __init__(self, x= None, y=None, angle = None):
        self.x = x
        self.y = y
        self.angle = angle