import time

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill, SetPen
from turtlesim.msg import Pose
from std_srvs.srv import Trigger, Empty
from geometry_msgs.msg import Twist
import math




class ControllerTortuga(Node):
    def __init__(self):
        super().__init__('controller_tortuga')
        self.cli_spawn = self.create_client(Spawn, 'spawn') # Servei Spawn
        self.cli_kill = self.create_client(Kill, 'kill')    #Servei Kill
        self.cli_draw= self.create_client(Trigger, 'start_drawing')  # Cliente para servicio de dibujo

        self.sub_pose = self.create_subscription(Pose, 'turtle3/pose', self.pose_callback, 10)
        self.publisher3 = self.create_publisher(Twist, 'turtle3/cmd_vel', 10)


        # Configurar las tortugas
        self.borrarTortuga('turtle1')
        # sol
        self.configurarTortuga(8.0, 8.0, 0.0, 'turtle1', True)
        # línea
        self.configurarTortuga(0.0, 1.0, 0.0, 'turtle2', False)

        self.configurarTortuga(5.0, 5.0, 0.0, 'turtle3', False)



    def borrarTortuga(self, name):
        kill_request = Kill.Request()
        kill_request.name = name
        future_kill = self.cli_kill.call_async(kill_request)
        rclpy.spin_until_future_complete(self, future_kill)
    def configurarTortuga(self, x, y, theta, name, es_turtle1):

        # Coloca la  tortuga
        spawn_request = Spawn.Request(x=x,y=y,theta=theta,name=name)
        future_spawn = self.cli_spawn.call_async(spawn_request)
        rclpy.spin_until_future_complete(self, future_spawn) # Esperar spwan fet

        # Configurar color/grosor
        configPen = SetPen.Request()      # Servei SetPen
        configPen.r = 255 if es_turtle1 else 165  # Amarillo (255, 255, 0) o Marrón (165, 42, 42)
        configPen.g = 255 if es_turtle1 else 42
        configPen.b = 0 if es_turtle1 else 42
        configPen.width = 8
        configPen.off = False

        if es_turtle1:
            service_name = 'turtle1/set_pen'
        else:
            service_name = 'turtle2/set_pen'

        setPen = self.create_client(SetPen, service_name) # crida servei setPen corresponent

        callSetPen = setPen.call_async(configPen)   # Enviar solicitud per la pluma

        rclpy.spin_until_future_complete(self, callSetPen) # Esperar a ser procesada

    def pose_callback(self, posicio):
        self.get_logger().info(f'Posicio de la tortuga - X: {posicio.x}, Y: {posicio.y}')

        # Ejemplo: si la tortuga está lejos de un punto objetivo, muévela hacia allí
        x2, y2 = 1.0, 1.0
        posX = posicio.x - x2
        posY = posicio.y - y2

        distancia = math.sqrt(posX ** 2 + posY ** 2) # Distancia euclidiana

        # Si la tortuga está dentro del umbral, empieza a dibujar
        if distancia <= 0.45:
            self.get_logger().info('Tortuga posicionada.') # Mensaje de depuración
            self.rotate_to()
        else:

            # Si no, sigue moviéndose hacia el objetivo
            self.move_toward(x2, y2, posicio)

    def rotate_to(self):
        velAngular = 2.65  # Velocidad angular

        vel_msg = Twist()
        vel_msg.angular.z = velAngular
        self.publisher3.publish(vel_msg)
        # Espera a que se complete la rotación (esto es solo un ejemplo, necesitas una condición de parada más robusta)
        time.sleep(1)

        vel_msg.angular.z = 0.0
        self.publisher3.publish(vel_msg)


        self.startDibuix()

    def move_toward(self, xFinal, yFinal, posicioActual):
        # Calcular la dirección hacia el objetivo
        dx = xFinal - posicioActual.x
        dy = yFinal - posicioActual.y
        angle = math.atan2(dy, dx)

        # Calcular la velocidad lineal y angular
        distance = math.sqrt(dx ** 2 + dy ** 2)
        vel_msg = Twist()
        vel_msg.linear.x = min(distance, 1.0)  # Limita la velocidad para evitar movimientos bruscos
        vel_msg.angular.z = 2.0 * (angle - posicioActual.theta)  # Ajustar la orientación

        # Publicar el mensaje de velocidad
        self.publisher3.publish(vel_msg)


    def startDibuix(self):
        if self.cli_draw.wait_for_service(timeout_sec=1.0):
                request = Trigger.Request()
                future = self.cli_draw.call_async(request)
                rclpy.spin_until_future_complete(self, future)



def main(args=None):
    rclpy.init(args=args)
    controller = ControllerTortuga()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
