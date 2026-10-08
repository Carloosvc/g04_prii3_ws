import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


def normalizar_angulo(angulo):
    return math.atan2(
        math.sin(angulo),
        math.cos(angulo)
    )


def limitar(valor, minimo, maximo):
    return max(min(valor, maximo), minimo)


class DrawNumber(Node):

    def __init__(self):
        super().__init__('draw_number')

        # =====================================================
        # PUBLICADOR Y SUSCRIPTOR
        # =====================================================

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # =====================================================
        # SERVICIOS
        # =====================================================

        self.servicio_detener = self.create_service(
            Empty,
            'detener_dibujo',
            self.detener_callback
        )

        self.servicio_reanudar = self.create_service(
            Empty,
            'reanudar_dibujo',
            self.reanudar_callback
        )

        self.servicio_reiniciar = self.create_service(
            Empty,
            'reiniciar_dibujo',
            self.reiniciar_callback
        )

        # =====================================================
        # TIMER DE CONTROL
        # =====================================================

        self.timer = self.create_timer(
            0.05,
            self.control_movimiento
        )

        # =====================================================
        # POSICION ACTUAL
        # =====================================================

        self.x = None
        self.y = None
        self.theta = None

        self.x_inicio = None
        self.y_inicio = None

        self.theta_objetivo = None

        # =====================================================
        # VELOCIDADES
        # =====================================================

        self.velocidad_lineal = 0.18
        self.velocidad_angular_max = 0.6

        self.k_giro = 2.0
        self.k_recta = 1.5

        self.tolerancia_angular = math.radians(0.5)

        # =====================================================
        # CONTROL DEL DIBUJO
        # =====================================================

        self.indice = 0
        self.accion_iniciada = False

        # False -> dibujando
        # True  -> detenido
        self.pausado = False

        # =====================================================
        # TRAYECTORIA DEL NUMERO 04
        # =====================================================

        self.acciones = [

            # =========================
            # NUMERO 0
            # =========================

            ('move', 0.8),
            ('turn', 90),

            ('move', 1.2),
            ('turn', 90),

            ('move', 0.8),
            ('turn', 90),

            ('move', 1.2),
            ('turn', 90),

            # =========================
            # DESPLAZAMIENTO A DERECHA
            # =========================

            ('turn', -90),
            ('move', 1.4),
            ('turn', 90),

            # =========================
            # NUMERO 4
            # =========================

            ('move', 1.2),

            ('turn', 90),
            ('turn', 90),

            ('move', 0.6),

            ('turn', 90),
            ('move', 0.8),

            ('turn', 90),
            ('move', 0.6),

            ('turn', 90),
            ('turn', 90),

            ('move', 1.2),
        ]

        self.get_logger().info(
            'Nodo draw_number iniciado. Dibujando 04.'
        )

        self.get_logger().info(
            'Servicios disponibles: '
            '/detener_dibujo, '
            '/reanudar_dibujo, '
            '/reiniciar_dibujo'
        )


    # =========================================================
    # ODOMETRIA
    # =========================================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        self.theta = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        if self.theta_objetivo is None:

            self.theta_objetivo = self.theta

            self.get_logger().info(
                f'Orientacion inicial: '
                f'{math.degrees(self.theta):.2f} grados'
            )


    # =========================================================
    # SERVICIO: DETENER
    # =========================================================

    def detener_callback(self, request, response):

        self.pausado = True

        self.parar()

        self.get_logger().info(
            'Dibujo detenido'
        )

        return response


    # =========================================================
    # SERVICIO: REANUDAR
    # =========================================================

    def reanudar_callback(self, request, response):

        self.pausado = False

        self.get_logger().info(
            'Dibujo reanudado'
        )

        return response


    # =========================================================
    # SERVICIO: REINICIAR
    # =========================================================

    def reiniciar_callback(self, request, response):

        # Paramos primero el robot
        self.parar()

        # Volvemos a la primera acción
        self.indice = 0
        self.accion_iniciada = False

        # El dibujo deja de estar pausado
        self.pausado = False

        # La orientación actual pasa a ser
        # la nueva orientación de referencia
        if self.theta is not None:
            self.theta_objetivo = self.theta

        self.x_inicio = None
        self.y_inicio = None

        self.get_logger().info(
            'Dibujo reiniciado'
        )

        return response


    # =========================================================
    # INICIAR ACCION
    # =========================================================

    def iniciar_accion(self):

        tipo, valor = self.acciones[self.indice]

        if tipo == 'move':

            self.x_inicio = self.x
            self.y_inicio = self.y

        elif tipo == 'turn':

            self.theta_objetivo = normalizar_angulo(
                self.theta_objetivo +
                math.radians(valor)
            )

        self.accion_iniciada = True

        self.get_logger().info(
            f'Accion {self.indice + 1}: '
            f'{tipo} {valor}'
        )


    # =========================================================
    # TERMINAR ACCION
    # =========================================================

    def terminar_accion(self):

        self.parar()

        self.indice += 1
        self.accion_iniciada = False


    # =========================================================
    # CONTROL PRINCIPAL
    # =========================================================

    def control_movimiento(self):

        # Si no tenemos odometría, esperamos
        if (
            self.x is None or
            self.y is None or
            self.theta is None or
            self.theta_objetivo is None
        ):
            return

        # Si está pausado, no hacemos nada
        if self.pausado:

            self.parar()
            return

        # Si hemos terminado el dibujo
        if self.indice >= len(self.acciones):

            self.parar()

            if self.indice == len(self.acciones):

                self.get_logger().info(
                    'Dibujo 04 terminado'
                )

                self.indice += 1

            return

        if not self.accion_iniciada:
            self.iniciar_accion()

        tipo, valor = self.acciones[self.indice]

        if tipo == 'move':

            self.mover(valor)

        elif tipo == 'turn':

            self.girar()


    # =========================================================
    # MOVIMIENTO RECTO
    # =========================================================

    def mover(self, distancia_objetivo):

        distancia = math.sqrt(
            (self.x - self.x_inicio) ** 2 +
            (self.y - self.y_inicio) ** 2
        )

        restante = (
            abs(distancia_objetivo) -
            distancia
        )

        mensaje = Twist()

        if restante > 0.0:

            if restante < 0.10:

                velocidad = 0.08

            else:

                velocidad = self.velocidad_lineal

            if distancia_objetivo >= 0:

                mensaje.linear.x = velocidad

            else:

                mensaje.linear.x = -velocidad

            # Corrección de orientación mientras avanza
            error_angulo = normalizar_angulo(
                self.theta_objetivo -
                self.theta
            )

            correccion = (
                self.k_recta *
                error_angulo
            )

            mensaje.angular.z = limitar(
                correccion,
                -0.25,
                0.25
            )

            self.publisher.publish(mensaje)

        else:

            self.get_logger().info(
                f'Distancia completada: '
                f'{distancia:.2f} m'
            )

            self.terminar_accion()


    # =========================================================
    # GIRO
    # =========================================================

    def girar(self):

        error = normalizar_angulo(
            self.theta_objetivo -
            self.theta
        )

        mensaje = Twist()

        if abs(error) > self.tolerancia_angular:

            mensaje.linear.x = 0.0

            velocidad = (
                self.k_giro *
                error
            )

            mensaje.angular.z = limitar(
                velocidad,
                -self.velocidad_angular_max,
                self.velocidad_angular_max
            )

            self.publisher.publish(mensaje)

        else:

            self.parar()

            self.get_logger().info(
                f'Giro completado. '
                f'Error angular: '
                f'{math.degrees(error):.2f} grados'
            )

            self.terminar_accion()


    # =========================================================
    # PARAR ROBOT
    # =========================================================

    def parar(self):

        mensaje = Twist()

        mensaje.linear.x = 0.0
        mensaje.angular.z = 0.0

        self.publisher.publish(mensaje)


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = DrawNumber()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.parar()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()