"""    Module for receiving IR data from IR receiver.  """
import machine
"""Importamos la libreria de machine"""
import utime
""" Importamos la libreria de utime"""
import micropython
""" Importamos la libreria de micropython"""

class irGetCMD(object):
    """ Clase que representa un sensor IR"""
    def __init__(self, gpioNum):
        """ Constructor de la clase irGetCMD
        :param gpioNum: Pin de entrada del sensor
        """

        self.irRecv = machine.Pin(gpioNum, machine.Pin.IN, machine.Pin.PULL_UP)
        self.irRecv.irq(
            trigger=machine.Pin.IRQ_RISING | machine.Pin.IRQ_FALLING,
            handler=self.__logHandler)
        self.logList = []
        self.index = 0
        self.start = 0
        self.dictKeyNum = 0
        self.irDict = {}

    def __logHandler(self, source):
        """ Funcion que maneja la interrupcion del sensor IR
        :param source: Pin de entrada del sensor
        """
        thisComeInTime = utime.ticks_us()
        if self.start == 0:
            self.start = thisComeInTime
            self.index = 0
            return
        self.logList.append(utime.ticks_diff(thisComeInTime, self.start))
        self.start = thisComeInTime
        self.index += 1

    def ir_read(self):
        """ Funcion que lee el valor del sensor IR
        """
        utime.sleep_ms(200) 
        if utime.ticks_diff(
                utime.ticks_us(),
                self.start) > 800000 and self.index > 0:
            ir_buffer=[]
            for i in range(3,66,2):
                if self.logList[i]>800:
                    ir_buffer.append(1)
                else:
                    ir_buffer.append(0)
            irValue=0x00000000
            for i in range(0,4):
                for j in range(0,8):
                    if ir_buffer[i*8+j]==1:
                        irValue=irValue<<1
                        irValue |= 0x01
                    else:
                        irValue=irValue<<1
                        irValue &= 0xfffffffe                    
            # reset 
            self.logList = []
            self.index = 0
            self.start = 0
            return hex(irValue)
        
        


       