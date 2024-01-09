# This example demonstrates a UART periperhal.

import bluetooth
""" Importe de la librería bluetooth """
import random
""" Importe de la librería random """
import struct
""" Importe de la librería struct """
import time
""" Importe de la librería time """
from ble_advertising import advertising_payload
""" Importe de la librería advertising_payload de ble_advertising """

from micropython import const
""" Importe de la librería const de micropython """

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_READ | _FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE,
)
_UART_SERVICE = (
    _UART_UUID,
    (_UART_TX, _UART_RX),
)


class BLESimplePeripheral:
    """ Clase que representa un periférico BLE """
    def __init__(self, ble, name="Turmo"):
        """ Constructor de la clase BLESimplePeripheral
        :param ble: Objeto BLE
        :param name: Nombre del periférico
        """
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_tx, self._handle_rx),) = self._ble.gatts_register_services((_UART_SERVICE,))
        self._connections = set()   
        self._write_callback = None
        self._payload = advertising_payload(name=name, services=[_UART_UUID])
        self._advertise()

    def _irq(self, event, data):
        """ Funcion que maneja las interrupciones del BLE
        :param event: Evento del BLE
        :param data: Datos del BLE
        """

        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("New connection", conn_handle)
            print("\nThe BLE connection is successful.")
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Disconnected", conn_handle)
            self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            value = self._ble.gatts_read(value_handle)
            if value_handle == self._handle_rx and self._write_callback:
                self._write_callback(value)

    def send(self, data):
        """ Funcion que envia datos por BLE
        :param data: Datos a enviar
        """
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_tx, data)

    def is_connected(self):
        """ Funcion que comprueba si hay una conexión BLE
        """
        return len(self._connections) > 0

    def _advertise(self, interval_us=500000):
        """ Funcion que anuncia el BLE
        :param interval_us: Intervalo de anuncio
        """
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def on_write(self, callback):
        """ Funcion que maneja la escritura por BLE
        :param callback: Funcion de callback
        """
        self._write_callback = callback


def demo():
        """" Test que muestra el funcionamiento del BLE
        Al ejecutar el test, se muestra por pantalla el mensaje "Please use LightBlue to connect to ESP32."
        Al conectarse un dispositivo BLE, se muestra por pantalla el mensaje "New connection"
        Al desconectarse un dispositivo BLE, se muestra por pantalla el mensaje "Disconnected"
        Al introducir un dato por teclado, se muestra por pantalla el mensaje "Send: "

        """
        ble = bluetooth.BLE()
        p = BLESimplePeripheral(ble)

        def on_rx(rx_data):
            print("RX", rx_data)

        p.on_write(on_rx)
        
        print("Please use LightBlue to connect to ESP32.")

        while True:
            if p.is_connected():
                # Short burst of queued notifications.
                tx_data = input("Enter anything: ")
                print("Send: ", tx_data)
                p.send(tx_data)


if __name__ == "__main__":
        demo()
