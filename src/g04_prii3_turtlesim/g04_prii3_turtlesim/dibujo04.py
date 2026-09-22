import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

from turtlesim.msg import Pose
from turtlesim.srv import SetPen


class Dibujo04(Node):

    def __init__(self):
        super().__init__('dibujo04')

        # ==========================================================
        # PUBLISHER
        # ==========================================================
        # Publicamos mensajes Twist para mover la tortuga.
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # ==========================================================
        # SUBSCRIBER
        # ==========================================================
        # Nos suscribimos a /turtle1/pose para saber:
        # - posición x
        # - posición y
        # - orientación theta
        #
        # Esto nos permite hacer los giros de forma precisa.
        self.pose = None

        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.actualizar_pose,
            10
        )

        # ==========================================================
        # SERVICIOS QUE PIDE EL SPRINT
        # ==========================================================

        self.create_service(
            Empty,
            'detener_dibujo',
            self.detener
        )

        self.create_service(
            Empty,
            'reanudar_dibujo',
            self.reanudar
        )

        self.create_service(
            Empty,
            'reiniciar_dibujo',
            self.reiniciar
        )

        # ==========================================================
        # CLIENTES DE SERVICIOS DE TURTLESIM
        # ==========================================================

        # Para levantar y bajar el lápiz
        self.lapiz = self.create_client(
            SetPen,
            '/turtle1/set_pen'
        )

        # Para reiniciar turtlesim
        self.reset = self.create_client(
            Empty,
            '/reset'
        )

        # ==========================================================
        # VARIABLES
        # ==========================================================

        # Indica si el dibujo está pausado
        self.pausado = False

        # Acción que estamos ejecutando
        self.paso = 0

        # Ciclos restantes de los movimientos rectos
        self.contador = 0

        # ==========================================================
        # ACCIONES PARA DIBUJAR "04"
        # ==========================================================
        #
        # move:
        #   velocidad y número de ciclos
        #
        # turn:
        #   orientación exacta a la que queremos llegar
        #
        # pen:
        #   1 = levantar
        #   0 = bajar
        # ==========================================================

        self.acciones = [

            # ======================================================
            # DIBUJAR EL 0
            # ======================================================

            # Avanzar y girar al mismo tiempo para hacer un círculo
            ('circle', 1.0, (2 * math.pi) / 6.0, 60),

            ('stop', 3),

            # ======================================================
            # COLOCARNOS PARA DIBUJAR EL 4
            # ======================================================

            # Levantamos el lápiz
            ('pen', 1),

            # Ponemos la tortuga mirando exactamente hacia la derecha
            ('turn', 0.0),

            # Nos desplazamos hacia la derecha
            ('move', 1.0, 22),

            # Miramos exactamente hacia arriba
            ('turn', math.pi / 2),

            # Subimos
            ('move', 1.0, 15),

            # Miramos exactamente hacia abajo
            ('turn', -math.pi / 2),

            # Bajamos el lápiz
            ('pen', 0),

            # ======================================================
            # DIBUJAR EL 4
            # ======================================================

            # Línea vertical izquierda
            ('move', 1.0, 15),

            # Miramos exactamente hacia la derecha
            ('turn', 0.0),

            # Barra horizontal
            ('move', 1.0, 17),

            # Levantamos el lápiz
            ('pen', 1),

            # Miramos hacia arriba
            ('turn', math.pi / 2),

            # Subimos hasta arriba del palo derecho
            ('move', 1.0, 15),

            # Miramos hacia abajo
            ('turn', -math.pi / 2),

            # Bajamos el lápiz
            ('pen', 0),

            # Dibujamos el palo largo del 4
            ('move', 1.0, 32),

            # Paramos
            ('stop', 5)
        ]

        # Cada 0.1 segundos ejecutamos controlar_dibujo()
        self.timer = self.create_timer(
            0.1,
            self.controlar_dibujo
        )


    # ==============================================================
    # GUARDAR LA POSICIÓN ACTUAL
    # ==============================================================

    def actualizar_pose(self, pose):

        # Guardamos la información recibida de turtlesim
        self.pose = pose


    # ==============================================================
    # CONTROL PRINCIPAL DEL DIBUJO
    # ==============================================================

    def controlar_dibujo(self):

        # Esperamos hasta conocer la posición de la tortuga
        if self.pose is None:
            return

        # Si está pausado, paramos
        if self.pausado:
            self.parar()
            return

        # Si hemos terminado todas las acciones
        if self.paso >= len(self.acciones):
            self.parar()
            return

        accion = self.acciones[self.paso]

        tipo = accion[0]

        # ==========================================================
        # CONTROL DEL LÁPIZ
        # ==========================================================

        if tipo == 'pen':

            self.cambiar_lapiz(accion[1])

            self.paso += 1

            return

        # ==========================================================
        # GIRO EXACTO
        # ==========================================================

        if tipo == 'turn':

            # Ángulo al que queremos llegar
            objetivo = accion[1]

            # Diferencia entre:
            # ángulo deseado - ángulo actual
            error = objetivo - self.pose.theta

            # Convertimos el error al intervalo -pi / +pi
            error = math.atan2(
                math.sin(error),
                math.cos(error)
            )

            # Si ya estamos prácticamente en el ángulo correcto
            if abs(error) < 0.01:

                self.parar()

                self.paso += 1

                return

            # Creamos el Twist para girar
            msg = Twist()

            # La velocidad depende del error.
            # Cuanto más cerca estamos, más despacio giramos.
            msg.angular.z = 2.0 * error

            # Limitamos la velocidad máxima de giro
            if msg.angular.z > 1.0:
                msg.angular.z = 1.0

            if msg.angular.z < -1.0:
                msg.angular.z = -1.0

            self.publisher.publish(msg)

            return

        # ==========================================================
        # MOVIMIENTO EN CÍRCULO
        # ==========================================================

        if tipo == 'circle':

            if self.contador == 0:
                self.contador = accion[3]

            msg = Twist()

            msg.linear.x = accion[1]
            msg.angular.z = accion[2]

            self.publisher.publish(msg)

            self.contador -= 1

            if self.contador == 0:

                self.parar()

                self.paso += 1

            return

        # ==========================================================
        # MOVIMIENTO RECTO
        # ==========================================================

        if tipo == 'move':

            if self.contador == 0:
                self.contador = accion[2]

            msg = Twist()

            # Solo velocidad lineal.
            # No giramos.
            msg.linear.x = accion[1]
            msg.angular.z = 0.0

            self.publisher.publish(msg)

            self.contador -= 1

            if self.contador == 0:

                self.parar()

                self.paso += 1

            return

        # ==========================================================
        # PARADA
        # ==========================================================

        if tipo == 'stop':

            if self.contador == 0:
                self.contador = accion[1]

            self.parar()

            self.contador -= 1

            if self.contador == 0:

                self.paso += 1


    # ==============================================================
    # CONTROL DEL LÁPIZ
    # ==============================================================

    def cambiar_lapiz(self, estado):

        peticion = SetPen.Request()

        # Color blanco
        peticion.r = 255
        peticion.g = 255
        peticion.b = 255

        # Grosor de la línea
        peticion.width = 3

        # 1 = levantado
        # 0 = dibujando
        peticion.off = estado

        self.lapiz.call_async(peticion)


    # ==============================================================
    # PARAR LA TORTUGA
    # ==============================================================

    def parar(self):

        # Twist vacío:
        # linear = 0
        # angular = 0
        msg = Twist()

        self.publisher.publish(msg)


    # ==============================================================
    # SERVICIO DETENER
    # ==============================================================

    def detener(self, request, response):

        self.pausado = True

        self.parar()

        self.get_logger().info('Dibujo detenido')

        return response


    # ==============================================================
    # SERVICIO REANUDAR
    # ==============================================================

    def reanudar(self, request, response):

        self.pausado = False

        self.get_logger().info('Dibujo reanudado')

        return response


    # ==============================================================
    # SERVICIO REINICIAR
    # ==============================================================

    def reiniciar(self, request, response):

        self.pausado = True

        self.parar()

        # Volvemos a la primera acción
        self.paso = 0
        self.contador = 0

        # Reiniciamos turtlesim
        peticion = Empty.Request()

        futuro = self.reset.call_async(peticion)

        futuro.add_done_callback(
            self.reset_terminado
        )

        self.get_logger().info('Dibujo reiniciado')

        return response


    def reset_terminado(self, futuro):

        # Volvemos a empezar el dibujo
        self.pausado = False


# ==============================================================
# MAIN
# ==============================================================

def main(args=None):

    # Inicializamos ROS2
    rclpy.init(args=args)

    # Creamos el nodo
    nodo = Dibujo04()

    # Mantenemos el nodo ejecutándose
    rclpy.spin(nodo)

    # Cerramos
    nodo.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
