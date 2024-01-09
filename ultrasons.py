"""
Módulo que proporciona las funcionalidades basicas para el uso de sensores de ultrasonidos usando MicroPython. El sensor SR04 se utiliza para medir distancias mediante la emisión y recepeción de ultrasonidos 

"""

from machine import Pin
""" Modulo para manejar el tiempo """
import time
""" Modulo para manejar el tiempo """


soundVelocity = 340
""" Velocidad del sonido  """
distance = 0
""" Distancia en cm """




class SR04(object):
    """ Clase que representa un sensor de ultrasonidos SR04
    """

    def __init__(self, trig, echo):
        """ Constructor de la clase SR04
        :param trig: Pin de salida del sensor
        :param echo: Pin de entrada del sensor
        """
        self._trigPin = Pin(trig, Pin.OUT, 0)
        self._echoPin = Pin(echo, Pin.IN, 0)

    def distanceCM(self):
        """ Devuelve la distancia en cm"""
        self._trigPin.value(1)
        time.sleep_us(10)
        self._trigPin.value(0)
        while (self._echoPin.value() == 0):
            pass
        pingStart = time.ticks_us()
        while (self._echoPin.value() == 1):
            pass
        pingStop = time.ticks_us()
        pingTime = time.ticks_diff(pingStop, pingStart)
        distance = pingTime * soundVelocity // 2 // 10000
        return distance

    def distanceMM(self):
        """ Devuelve la distancia en mm"""
        self._trigPin.value(1)
        time.sleep_us(10)
        self._trigPin.value(0)
        while (self._echoPin.value() == 0):
            pass
        pingStart = time.ticks_us()
        while (self._echoPin.value() == 1):
            pass
        pingStop = time.ticks_us()
        pingTime = time.ticks_diff(pingStop, pingStart)
        distance = pingTime * soundVelocity // 2 // 1000
        return distance

    def distance(self):
        """ Devuelve la distancia en m"""
        self._trigPin.value(1)
        time.sleep_us(10)
        self._trigPin.value(0)
        while (self._echoPin.value() == 0):
            pass
        pingStart = time.ticks_us()
        while (self._echoPin.value() == 1):
            pass
        pingStop = time.ticks_us()
        pingTime = time.ticks_diff(pingStop, pingStart)
        distance = pingTime * soundVelocity / 2 / 10000
        return distance


def main():
    """ Test del comportamiento del sensor de ultrasonidos
        Se muestra por pantalla la distancia en centímetros, milímetros y metros
        """
    sr04 = SR04(4, 5)

    cm = sr04.distanceCM()
    mm = sr04.distanceMM()
    m = sr04.distance()

    print("Distance in centimeters: ", cm)
    print("Distance in millimeters: ", mm)
    print("Distance in meters: ", m)


if __name__ == "__main__":

    while True:
        main()
