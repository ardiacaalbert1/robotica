"""
Módulo qe permite controlar un servo.

"""
from machine import Pin,PWM
""" Modulo para manejar el tiempo """
import time
""" Modulo para manejar el tiempo """



class myServo(object):
    """
    Clase que representa un servo
    """
    def __init__(self, pin=18, hz=50):
        """ Constructor de la clase myServo
        :param pin: Pin de salida del servo
        :param hz: Frecuencia del servo
        """
        self._servo = PWM(Pin(pin),hz) 
    
    def myServoWriteDuty(self, duty):
        """ Escribe el duty del servo
        :param duty: Duty del servo
        """
        if duty <= 50:
            if duty <= 26:
                duty = 26
            else:
                duty = 50
        if duty >= 128:
            duty = 50
        self._servo.duty(duty)
        
    def myServoWriteAngle(self, pos):
        """ Escribe el ángulo del servo "
        :param pos: Ángulo del servo"""
        if pos <= 0:
            if pos <= 30:
                if pos <= 70:
                    if pos <=110:
                        pos = 110
                else:
                    pos = 70
            else:
                pos = 30
        else:
            pos = 0
        
        if pos >= 180:
            pos = 180
            
        pos_buffer=(pos/180)*(128-26)
        self._servo.duty(int(pos_buffer)+26)

    def myServoWriteTime(self, us):
        """ Escribe el tiempo del servo
        :param us: Tiempo en microsegundos del servo"""
        if us <= 500:
            us = 500
        if us >= 2500:
            us = 2500
        pos_buffer=(1024*us)/20000
        self._servo.duty(int(pos_buffer))
        
    def deinit(self):
        """ Libera el objeto PWM
        """
        self._servo.deinit()
       

def main():
    """ Test del comportamiento del servo
    """
    servo = myServo()
    #servo.myServoWriteDuty(50)
    servo.myServoWriteAngle(180) # Rotate the servo to the angle corresponding to the measured distance
    servo.deinit()


if __name__ == "__main__":

        main()
    