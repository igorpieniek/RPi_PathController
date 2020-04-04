import math
import numpy as np

class PositionController(object):
    def __init__(self ):
        self._wheelbase = 0.2127 #rozstaw kol w [m]
        self._wheelDiameter = 0.0898 # srednica kola [m]
        self._impulsesPerRevolution= 3600 # ilosc impulsow enkodera na obrot(przelozenie * ilosc impulsow na obrot walu silnika)
        self._maxValType = np.iinfo(np.uint16()).max
        self._halfUint16 =  self._maxValType//2 #polowa zakresu liczby 2bajtowej uint16

        self._lastMeasure = {'L' : np.uint16(0), 'R' : np.uint16(0)}

        self._lastLocation = {'x': 0, 'y' : 0, 'angle' : 0}
    #-------------MAIN PROCESS FUNCTIONS-----------------------------------------------------------------------
    def getCoordinates(self):
        SR = self._encoderConversion('R') 
        SL = self._encoderConversion('L')
        coor = self._globalCoordinate(self._lastLocation['x'],
                                      self._lastLocation['y'],
                                      self._lastLocation['angle'],
                                      SL,
                                      SR)
        self._lastLocation = {'x': coor['x'], 'y' : coor['y'], 'angle' : coor['angle']}
        return {'x': coor['x'], 'y' : coor['y'], 'angle' : coor['angle']}
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
        temp = input('Enter imp motor '+motor+ ': ' )
        return np.uint16(temp)
        if motor=='L': return 0#getmotorimpulses[motor] (function from myserial.py)
        else: return 1          #getmotorimpulses[motor] (function from myserial.py)

    def _encoderConversion(self, motor):
        currentImp = self._getCurrentImpulses(motor)
        S = ( self._getDiff(currentImp, self._lastMeasure[motor]) / self._impulsesPerRevolution) * math.pi* self._wheelDiameter
        self._lastMeasure[motor] = currentImp
        return S
    #-------------HIGH LEVEL CONVERSION FUNCTIONS---------------------------------------------------------------
    def _localCoordinate(self,S1, S2):
        if S1 == S2 and S1 == 0: return {'x':0, 'y':0, 'fi': 0, 'R':0 }
        elif S1 == S2:
            return {'x':0, 'y':S1, 'fi': 0, 'R':0 }
        else:
            fi = ( S1 - S2 ) / self._wheelbase
            R = ( (S1 * self._wheelbase) /  (S1 - S2) ) - (0.5* self._wheelbase)
            y = R * math.sin( fi )
            x =  y/ math.tan( 0.5*( math.pi-fi) )
            fi *= -1
            return{'x':x, 'y':y,'fi': math.degrees(fi), 'R':R }


    def _globalCoordinate(self, x, y, angle, S1, S2):
        temp = self._localCoordinate(S1,S2)
        x_out = x + ( temp['x'] * math.cos( math.radians(angle) ) ) - (temp['y'] * math.sin( math.radians(angle) ) )
        y_out = y + ( temp['x'] * math.sin( math.radians(angle) ) ) + (temp['y'] * math.cos( math.radians(angle) ) )

        fi_out = temp['fi'] + angle 
        return{'x':x_out, 'y':y_out,'angle': fi_out, 'R':temp['R'], 'S1':S1, 'S2':S2 }
