"""
Módulo que proporciona las funcionalidades basicas para el uso del sensor IR
"""
from irrecvdata import irGetCMD
"""Modulo para manejar el tiempo"""

recvPin = irGetCMD(21)
"""Pin de entrada del sensor"""

try:
    """Bucle infinito"""
    while True:
        """Devuelve el valor del sensor IR"""
        irValue = recvPin.ir_read()
        if irValue:
            print(irValue)
except:
    """Si se produce un error"""
    pass
