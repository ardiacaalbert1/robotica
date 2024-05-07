import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from turtlesim.srv import Kill
from turtlesim.srv import SetPen

import math
import time
import threading


class PaisatjePaint(Node):
    def __init__(self):
        super().__init__('paisatje_paint')
        self.publisher1 = self.create_publisher(Twist, 'turtle1/cmd_vel',
                                                20)  # nodes envian missatge al tema/topico cmd_vel
        self.publisher2 = self.create_publisher(Twist, 'turtle2/cmd_vel', 10)
        self.publisher3 = self.create_publisher(Twist, 'turtle3/cmd_vel', 10)

        self.srv = self.create_service(Trigger, 'start_drawing', self.start_drawing_callback)

        self.cli_kill = self.create_client(Kill, 'kill')  # Servei Kill

    def start_drawing_callback(self, request, response):
        self.get_logger().info("Dibuixem el paisatge")

        # Threads
        sol_thread = threading.Thread(target=self.sol)
        terra_thread = threading.Thread(target=self.terra)
        casa_thread = threading.Thread(target=self.casa)

        # Start threads
        sol_thread.start()
        terra_thread.start()
        casa_thread.start()

        # Joins esperar
        sol_thread.join()
        terra_thread.join()
        casa_thread.join()

        response = Trigger.Response()
        response.success = True
        response.message = "Començant a dibuixar el paisatge"
        return response

    def sol(self):
        self.get_logger().info("Pintant el sol")

        vel_ang = 2 * math.pi / 10

        radiMax = 0.5
        radiDec = 0.1
        radi = radiMax
        while radi > 0:
            vel = radi * vel_ang

            twist = Twist()
            twist.linear.x = vel
            twist.angular.z = vel_ang

            for _ in range(20):
                self.publisher1.publish(twist)

                time.sleep(1 / vel_ang)  # sleep per evitar forats al cercle

            radi -= radiDec

            # Sleep per evitar forats al cercle
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher1.publish(twist)
            time.sleep(1)

    def terra(self):
        self.get_logger().info("Pintant el terra")
        # El limite en el eje x es 11.168889

        # Dibujar una línea horizontal de 0 a 11.168889
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.0

        for _ in range(11):
            self.publisher2.publish(twist)
            time.sleep(1)

        twist.linear.x = 0.0
        self.publisher2.publish(twist)
        time.sleep(1)



    def casa(self):
        self.get_logger().info('Iniciando a dibujar el cuadrado')

        vel_msg = Twist()
        # Definir la velocidad lineal y angular
        velocidad_lineal = 2.0  # La distancia que quieres recorrer (2 metros)
        velocidad_angular = 1.57  # Pi/2 radianes para girar 90 grados

        # Dibujar los cuatro lados del cuadrado
        for i in range(4):
            # Moverse en línea recta
            if i == 0:
                vel_msg.linear.x = velocidad_lineal * 2
            elif i == 2:
                vel_msg.linear.x = velocidad_lineal * 2
            else:
                vel_msg.linear.x = velocidad_lineal

            vel_msg.angular.z = 0.0
            self.publisher3.publish(vel_msg)
            time.sleep(1)  #

            # Detener el movimiento lineal antes de girar
            vel_msg.linear.x = 0.0
            self.publisher3.publish(vel_msg)
            time.sleep(0.1)  #  pausa para asegurar que la tortuga se detiene completamente

            # Girar
            vel_msg.angular.z = velocidad_angular
            self.publisher3.publish(vel_msg)
            time.sleep(1)  # Tiempo para completar el giro

            # Detener el giro
            vel_msg.angular.z = 0.0
            self.publisher3.publish(vel_msg)
            time.sleep(0.1)

        # Hacer triangulo - estoy mirando hacia arriba
        velocidad_angular = -0.785  # 120 grados para triángulo equilátero
        for i in range(3):
            if i == 0:  # Ir a esquina superior
                vel_msg.angular.z = 1.60
                self.publisher3.publish(vel_msg)
                time.sleep(1)  # Tiempo para completar el giro

                # Detener el giro
                vel_msg.angular.z = 0.0
                self.publisher3.publish(vel_msg)
                time.sleep(0.1)

                vel_msg.linear.x = velocidad_lineal
                self.publisher3.publish(vel_msg)
                time.sleep(1)

            # Detener el movimiento lineal antes de girar
            vel_msg.linear.x = 0.0
            self.publisher3.publish(vel_msg)
            time.sleep(0.1)

            # Girar
            vel_msg.angular.z = velocidad_angular
            self.publisher3.publish(vel_msg)
            time.sleep(1)  # Tiempo para completar el giro

            # Detener el giro
            vel_msg.angular.z = 0.0
            self.publisher3.publish(vel_msg)
            time.sleep(0.1)

            if i == 1:
                vel_msg.linear.x = 1.3
            else:
                vel_msg.linear.x = velocidad_lineal
            # Mover

            self.publisher3.publish(vel_msg)
            time.sleep(1)

        # Casa dibujada
        self.get_logger().info('Casa dibujada')




def main():
    rclpy.init()
    painter = PaisatjePaint()
    try:
        rclpy.spin(painter)
    except KeyboardInterrupt:
        painter.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
