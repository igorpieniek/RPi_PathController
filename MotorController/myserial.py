import serial
from construct import*
import struct
import threading
import time

class Parser:
    def __init__(self,serial):
        self.size=0
        self.state=0
        self.ptr=0
        self.data= bytearray(0)
        self.rpm_a=0
        self.rpm_b=0
        self.enc_a=0
        self.enc_b=0
        self.serial=serial
        self.frame_id_1 = Struct("rpm_a" / Float32l ,"rpm_b" / Float32l ,"enc_a" / Int16ul,"enc_b" / Int16ul)
        self.thread_run=1
        self.thread = threading.Thread(target=self.receiver_thread, args=(self,))
        self.notifier = threading.Condition()
        self.thread.start()
    def addByte(self,byte):
        ret=0
        #print('parsuje: ',byte)
        if self.state == 0:
            if byte == 0x12:
                self.state=1
        elif self.state == 1:
            if byte == 0x34:
                self.state=2
            else:
                self.state=0
                print('error')
        elif self.state == 2:
            self.size = byte
            self.ptr = 0
            self.state=3
            self.data = bytearray(0) # change from clear()
           # print('Rozmiar: ',self.size)
        elif self.state == 3:
            if self.ptr<self.size:
                self.data.append(byte)
                self.ptr+=1;
            else:
                if byte == 0xAA or byte == -86:
                    ret = self.data[0]
                    #print('crc poprawne')
                self.state=0
        else:
            self.state=0
        return ret
    def addByteArray(self,bytes):
        r=0
        for i in range(len(bytes)):
            ret=self.addByte(bytes[i])
            if ret==2:
                frame=self.frame_id_1.parse(self.data[1:len(self.data)])
                self.rpm_a=frame.rpm_a
                self.rpm_b=frame.rpm_b
                self.enc_a=frame.enc_a
                self.enc_b=frame.enc_b
                r = self.data[0]
                #print(frame)
        return r
    def receiver_thread(self,dd):
        print('thred start')
        import binascii
        while self.thread_run>0:
            #pre = self.serial.read(17)
            pre =  bytearray(self.serial.read(17))
            #raw = [str(self.serial.read(1).encode('hex')) for a in range(17)]
            #pre = [binascii.unhexlify(el)  for el in raw]
            #pre = [int( struct.unpack('<b',  binascii.unhexlify(el))[0] ) for el in raw]
            #print(pre)
            if len(pre)>0:
                ret=self.addByteArray(pre);
                if ret>0:
                    with self.notifier:
                        self.notifier.notify()
                    print(self.rpm_a,self.rpm_b,self.enc_a,self.enc_b)
        print('thread stop')
    def stop_receiver(self):
        self.thread_run=0
        self.thread.join()
    def get_data(self):
        return (self.rpm_a,self.rpm_b,self.enc_a,self.enc_b)
    def wait_on_data(self,timeout):
        no_timeout = True
        with self.notifier:
            no_timeout=self.notifier.wait(timeout)
        return (no_timeout,self.rpm_a,self.rpm_b,self.enc_a,self.enc_b)
    

class MotorControler:
    def __init__(self,serial_name):
        #otwieram polaczenie szeregowe
        self.serial=serial.Serial(port = serial_name, baudrate=115200, bytesize=8, parity='N')
        print(self.serial)
        #tworze obiekt parsujacy dane
        self.parser = Parser(self.serial)
        #tworze wzorzec ramki danych
        self.frame_pwm = Struct(Const(b"\x12"),Const(b"\x34"),Const(b"\x03"),Const(b"\x03"),"pwm_a" / Int8sl, "pwm_b" / Int8sl,Const(b"\xAA"))
        print("MotorControler On")
    def __del__(self):
        #zatrzymuje watek parsera
        self.parser.stop_receiver()
        #zamykam polaczenie
        self.serial.close();
        print("MotorControler Off")
    def SetPWMControl(self,left_duty,right_duty):
        #wprowadzam ograniczenie na sterowanie
        if left_duty > 100:
            left_duty = 100
        elif left_duty< -100:
            left_duty = -100
        if right_duty > 100:
            right_duty = 100
        elif right_duty< -100:
            right_duty = -100
        frame = self.frame_pwm.build(dict(pwm_a=left_duty,pwm_b=right_duty))
        self.serial.write(frame);
        print("Wyslano dane: ",frame)
    def GetMeasurements(self):
        return self.parser.get_data()
    def WaitOnMeasurement(self,timeout):
        #funkja oczekuje na odbior nowych danych z parsera i zwraca je
        # funkcja zwraca w pierwszym argumencie info o odczycie nowych danych w zadanym czasie: False - przekroczono timeout.
        return self.parser.wait_on_data(timeout)

    
    
