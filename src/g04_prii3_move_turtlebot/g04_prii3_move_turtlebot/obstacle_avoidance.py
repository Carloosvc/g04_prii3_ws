import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def normalizar_angulo(angulo):
    return math.atan2(
        math.sin(angulo),
        math.cos(angulo)
    )


def limitar(valor, minimo, maximo):
    return max(minimo, min(valor, maximo))


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # =====================================================
        # ROS
        # =====================================================

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.timer = self.create_timer(
            0.05,
            self.control
        )

        # =====================================================
        # POSE REAL DEL ROBOT
        # =====================================================

        self.x = None
        self.y = None
        self.theta = None

        # =====================================================
        # LIDAR
        # =====================================================

        self.scan_points = []
        self.scan_ready = False

        self.front_distance = math.inf
        self.left_space = math.inf
        self.right_space = math.inf

        # =====================================================
        # TRAYECTORIA NOMINAL
        # =====================================================

        self.plan_x = None
        self.plan_y = None
        self.plan_theta = None

        self.segmento_inicio_x = None
        self.segmento_inicio_y = None

        self.segmento_objetivo_x = None
        self.segmento_objetivo_y = None

        self.segmento_ux = 0.0
        self.segmento_uy = 0.0

        self.segmento_longitud = 0.0

        self.theta_objetivo = None

        # =====================================================
        # SECUENCIA
        # =====================================================

        self.indice = 0
        self.accion_iniciada = False

        # =====================================================
        # TRAYECTORIA DEL 04
        # =====================================================

        self.acciones = [

            # =================================================
            # NUMERO 0
            #
            # ESTA PARTE NO SE TOCA.
            # YA FUNCIONA CORRECTAMENTE.
            # =================================================

            ('move', 0.8),
            ('turn', 90),

            ('move', 1.2),
            ('turn', 90),

            ('move', 0.8),
            ('turn', 90),

            ('move', 1.2),
            ('turn', 90),

            # =================================================
            # IR A LA ZONA DONDE DIBUJAREMOS EL 4
            # =================================================
            #
            # Al terminar el 0 estamos en su esquina
            # inferior derecha y con la orientación inicial.
            #
            # Giramos 90 grados y subimos directamente
            # por el hueco que se ve en tu captura.
            #
            # NO nos vamos primero hacia un lateral.
            #
            # Esto coloca el 4 justo encima del 0.
            # =================================================

            ('turn', 90),

            ('move', 1.65),

            ('turn', -90),

            # =================================================
            # NUMERO 4
            # =================================================
            #
            # Se dibuja tumbado en la vista superior.
            #
            # Cuando gires la cámara 90 grados se verá
            # como un 4 normal a la derecha del 0.
            #
            # Dimensiones aproximadas:
            #
            # largo: 0.90 m
            # ancho: 0.45 m
            #
            # Lo hacemos compacto para que permanezca
            # entre las columnas.
            # =================================================

            # -----------------------------------------
            # Línea larga principal
            # -----------------------------------------

            ('move', 0.90),

            # -----------------------------------------
            # Volvemos hasta la mitad
            # -----------------------------------------

            ('turn', 90),
            ('turn', 90),

            ('move', 0.45),

            # -----------------------------------------
            # Rama perpendicular del 4
            # -----------------------------------------

            ('turn', 90),

            ('move', 0.45),

            # -----------------------------------------
            # Segunda parte horizontal
            # -----------------------------------------

            ('turn', 90),

            ('move', 0.45),

            # -----------------------------------------
            # Damos media vuelta
            # -----------------------------------------

            ('turn', 90),
            ('turn', 90),

            # -----------------------------------------
            # Línea larga final
            # -----------------------------------------

            ('move', 0.90),
        ]

        # =====================================================
        # PLANIFICADOR LOCAL
        # =====================================================

        self.radio_colision = 0.23

        self.rango_lidar_planificacion = 1.60

        self.horizonte = 2.0

        self.dt_sim = 0.10

        self.velocidad_max = 0.18
        self.velocidad_min = 0.05

        self.angular_max = 1.00

        # =====================================================
        # TOLERANCIAS
        # =====================================================

        self.tolerancia_punto = 0.07

        self.tolerancia_angular = math.radians(
            0.5
        )

        self.k_giro = 2.0

        # =====================================================
        # COMANDO ANTERIOR
        # =====================================================

        self.ultimo_v = 0.0
        self.ultimo_w = 0.0

        self.comando_v = 0.0
        self.comando_w = 0.0

        # =====================================================
        # PLANIFICACION
        # =====================================================

        self.contador_plan = 0

        self.plan_cada = 2

        self.en_desvio = False

        self.get_logger().info(
            'Nodo obstacle_avoidance iniciado. '
            'Dibujando 04 con evitacion de obstaculos.'
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

        # Primera pose recibida.

        if self.plan_x is None:

            self.plan_x = self.x
            self.plan_y = self.y
            self.plan_theta = self.theta

            self.get_logger().info(
                f'Referencia inicial: '
                f'x={self.x:.2f}, '
                f'y={self.y:.2f}, '
                f'theta='
                f'{math.degrees(self.theta):.1f} grados'
            )


    # =========================================================
    # LIDAR
    # =========================================================

    def scan_callback(self, msg):

        puntos = []

        frente = math.inf

        izquierda = []
        derecha = []

        # Utilizamos un rayo de cada tres.

        for i in range(
            0,
            len(msg.ranges),
            3
        ):

            r = msg.ranges[i]

            angulo = normalizar_angulo(
                msg.angle_min +
                i * msg.angle_increment
            )

            # =================================================
            # ESPACIO DISPONIBLE
            # =================================================

            if math.isinf(r):

                valor_espacio = (
                    msg.range_max
                )

            elif (
                msg.range_min <= r <=
                msg.range_max
            ):

                valor_espacio = r

            else:

                continue

            # Frente

            if abs(angulo) <= math.radians(18):

                frente = min(
                    frente,
                    valor_espacio
                )

            # Izquierda

            if (
                math.radians(20)
                <= angulo <=
                math.radians(100)
            ):

                izquierda.append(
                    valor_espacio
                )

            # Derecha

            if (
                math.radians(-100)
                <= angulo <=
                math.radians(-20)
            ):

                derecha.append(
                    valor_espacio
                )

            # =================================================
            # PUNTOS DE OBSTACULO
            # =================================================

            if (
                math.isfinite(r)
                and
                msg.range_min <= r <=
                min(
                    msg.range_max,
                    self.rango_lidar_planificacion
                )
            ):

                puntos.append(
                    (
                        r * math.cos(angulo),
                        r * math.sin(angulo)
                    )
                )

        self.scan_points = puntos

        self.scan_ready = True

        self.front_distance = frente

        if izquierda:

            self.left_space = (
                sum(izquierda) /
                len(izquierda)
            )

        else:

            self.left_space = (
                msg.range_max
            )

        if derecha:

            self.right_space = (
                sum(derecha) /
                len(derecha)
            )

        else:

            self.right_space = (
                msg.range_max
            )


    # =========================================================
    # INICIAR ACCION
    # =========================================================

    def iniciar_accion(self):

        tipo, valor = (
            self.acciones[self.indice]
        )

        # =================================================
        # MOVIMIENTO RECTO
        # =================================================

        if tipo == 'move':

            self.segmento_inicio_x = (
                self.plan_x
            )

            self.segmento_inicio_y = (
                self.plan_y
            )

            # Punto final NOMINAL.

            self.segmento_objetivo_x = (
                self.plan_x +
                valor *
                math.cos(self.plan_theta)
            )

            self.segmento_objetivo_y = (
                self.plan_y +
                valor *
                math.sin(self.plan_theta)
            )

            dx = (
                self.segmento_objetivo_x -
                self.segmento_inicio_x
            )

            dy = (
                self.segmento_objetivo_y -
                self.segmento_inicio_y
            )

            self.segmento_longitud = (
                math.hypot(dx, dy)
            )

            if self.segmento_longitud > 0.0:

                self.segmento_ux = (
                    dx /
                    self.segmento_longitud
                )

                self.segmento_uy = (
                    dy /
                    self.segmento_longitud
                )

            self.contador_plan = (
                self.plan_cada
            )

            self.en_desvio = False

        # =================================================
        # GIRO
        # =================================================

        else:

            self.theta_objetivo = (
                normalizar_angulo(
                    self.plan_theta +
                    math.radians(valor)
                )
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

        tipo, _ = (
            self.acciones[self.indice]
        )

        if tipo == 'move':

            # Seguimos siempre desde la posición
            # NOMINAL del dibujo.

            self.plan_x = (
                self.segmento_objetivo_x
            )

            self.plan_y = (
                self.segmento_objetivo_y
            )

        else:

            self.plan_theta = (
                self.theta_objetivo
            )

        self.publicar_velocidad(
            0.0,
            0.0
        )

        self.indice += 1

        self.accion_iniciada = False

        self.en_desvio = False


    # =========================================================
    # CONTROL PRINCIPAL
    # =========================================================

    def control(self):

        if (
            self.x is None
            or self.y is None
            or self.theta is None
            or self.plan_x is None
            or not self.scan_ready
        ):

            return

        # =================================================
        # DIBUJO TERMINADO
        # =================================================

        if self.indice >= len(
            self.acciones
        ):

            self.publicar_velocidad(
                0.0,
                0.0
            )

            if (
                self.indice ==
                len(self.acciones)
            ):

                self.get_logger().info(
                    'Dibujo 04 terminado'
                )

                self.indice += 1

            return

        # =================================================
        # INICIAR NUEVA ACCION
        # =================================================

        if not self.accion_iniciada:

            self.iniciar_accion()

        tipo, _ = (
            self.acciones[self.indice]
        )

        if tipo == 'move':

            self.control_segmento()

        else:

            self.control_giro()


    # =========================================================
    # CONTROL DE TRAMO RECTO
    # =========================================================

    def control_segmento(self):

        distancia_objetivo = math.hypot(

            self.segmento_objetivo_x -
            self.x,

            self.segmento_objetivo_y -
            self.y
        )

        # =================================================
        # OBJETIVO ALCANZADO
        # =================================================

        if (
            distancia_objetivo <=
            self.tolerancia_punto
        ):

            self.get_logger().info(
                f'Punto alcanzado. '
                f'Error='
                f'{distancia_objetivo:.2f} m'
            )

            self.terminar_accion()

            return

        # =================================================
        # REPLANIFICAR
        # =================================================

        self.contador_plan += 1

        if (
            self.contador_plan >=
            self.plan_cada
        ):

            self.contador_plan = 0

            resultado = (
                self.buscar_mejor_comando(
                    distancia_objetivo
                )
            )

            # =================================================
            # NO HAY CAMINO SEGURO
            # =================================================

            if resultado is None:

                (
                    self.comando_v,
                    self.comando_w
                ) = (
                    self.comando_emergencia()
                )

            else:

                (
                    self.comando_v,
                    self.comando_w,
                    desvio
                ) = resultado

                # =================================================
                # EMPIEZA LA ESQUIVA
                # =================================================

                if (
                    desvio
                    and
                    not self.en_desvio
                ):

                    self.en_desvio = True

                    self.get_logger().warn(
                        'Obstaculo bloqueando el tramo. '
                        'Rodeando sin detener el movimiento.'
                    )

                # =================================================
                # RECUPERA TRAYECTORIA
                # =================================================

                elif (
                    not desvio
                    and
                    self.en_desvio
                ):

                    self.en_desvio = False

                    self.get_logger().info(
                        'Trayectoria nominal recuperada.'
                    )

        self.publicar_velocidad(
            self.comando_v,
            self.comando_w
        )


    # =========================================================
    # BUSCAR MEJOR COMANDO
    # =========================================================

    def buscar_mejor_comando(
        self,
        distancia_objetivo
    ):

        # =================================================
        # VELOCIDADES LINEALES
        # =================================================

        if distancia_objetivo < 0.16:

            velocidades = [
                0.03,
                0.05,
                0.07,
                0.08
            ]

        elif distancia_objetivo < 0.30:

            velocidades = [
                0.05,
                0.08,
                0.11,
                0.13
            ]

        else:

            velocidades = [
                0.06,
                0.10,
                0.14,
                0.18
            ]

        # =================================================
        # VELOCIDADES ANGULARES
        # =================================================

        angulares = [

            -1.00,
            -0.90,
            -0.80,
            -0.70,
            -0.60,
            -0.50,
            -0.40,
            -0.30,
            -0.20,
            -0.10,

            0.00,

            0.10,
            0.20,
            0.30,
            0.40,
            0.50,
            0.60,
            0.70,
            0.80,
            0.90,
            1.00,
        ]

        mejor = None

        mejor_score = math.inf

        # =================================================
        # PROBAR TODAS LAS TRAYECTORIAS
        # =================================================

        for v in velocidades:

            for w in angulares:

                evaluacion = (
                    self.evaluar_trayectoria(
                        v,
                        w
                    )
                )

                # Si choca, descartamos.

                if evaluacion is None:

                    continue

                (
                    score,
                    desvio
                ) = evaluacion

                # =================================================
                # SUAVIDAD
                # =================================================

                score += (
                    0.30 *
                    abs(
                        w -
                        self.ultimo_w
                    )
                )

                score += (
                    0.20 *
                    abs(
                        v -
                        self.ultimo_v
                    )
                )

                # =================================================
                # EVITAR OSCILACIONES
                # =================================================

                if (
                    abs(self.ultimo_w) >
                    0.25
                    and
                    abs(w) > 0.25
                    and
                    self.ultimo_w * w < 0.0
                ):

                    score += 0.80

                # =================================================
                # MEJOR OPCION
                # =================================================

                if score < mejor_score:

                    mejor_score = score

                    mejor = (
                        v,
                        w,
                        desvio
                    )

        return mejor


    # =========================================================
    # EVALUAR TRAYECTORIA
    # =========================================================

    def evaluar_trayectoria(
        self,
        v,
        w
    ):

        # Pose simulada en coordenadas
        # locales del robot.

        lx = 0.0
        ly = 0.0
        ltheta = 0.0

        pasos = int(
            self.horizonte /
            self.dt_sim
        )

        radio2 = (
            self.radio_colision *
            self.radio_colision
        )

        distancia_minima = math.inf

        coste_linea_acumulado = 0.0

        cos_theta = math.cos(
            self.theta
        )

        sin_theta = math.sin(
            self.theta
        )

        # =================================================
        # SIMULAR LOS SIGUIENTES 2 SEGUNDOS
        # =================================================

        for _ in range(pasos):

            lx += (
                v *
                math.cos(ltheta) *
                self.dt_sim
            )

            ly += (
                v *
                math.sin(ltheta) *
                self.dt_sim
            )

            ltheta = normalizar_angulo(
                ltheta +
                w *
                self.dt_sim
            )

            # =================================================
            # DISTANCIA A OBSTACULOS
            # =================================================

            min_d2 = math.inf

            for ox, oy in self.scan_points:

                dx = ox - lx
                dy = oy - ly

                d2 = (
                    dx * dx +
                    dy * dy
                )

                if d2 < min_d2:

                    min_d2 = d2

            # =================================================
            # COLISION
            # =================================================

            if min_d2 < radio2:

                return None

            if min_d2 < math.inf:

                distancia_superficie = (
                    math.sqrt(min_d2)
                )

                distancia_minima = min(
                    distancia_minima,
                    distancia_superficie
                )

            # =================================================
            # PASAR A COORDENADAS GLOBALES
            # =================================================

            gx = (
                self.x +
                cos_theta * lx -
                sin_theta * ly
            )

            gy = (
                self.y +
                sin_theta * lx +
                cos_theta * ly
            )

            coste_linea_acumulado += (
                self.distancia_a_linea(
                    gx,
                    gy
                )
            )

        # =================================================
        # ESTADO FINAL DEL HORIZONTE
        # =================================================

        gx = (
            self.x +
            cos_theta * lx -
            sin_theta * ly
        )

        gy = (
            self.y +
            sin_theta * lx +
            cos_theta * ly
        )

        gtheta = normalizar_angulo(
            self.theta +
            ltheta
        )

        # =================================================
        # DISTANCIA AL OBJETIVO
        # =================================================

        distancia_goal = math.hypot(

            self.segmento_objetivo_x -
            gx,

            self.segmento_objetivo_y -
            gy
        )

        # =================================================
        # DISTANCIA A LA LINEA
        # =================================================

        distancia_linea = (
            self.distancia_a_linea(
                gx,
                gy
            )
        )

        # =================================================
        # PROGRESO
        # =================================================

        progreso = (
            self.progreso_segmento(
                gx,
                gy
            )
        )

        # =================================================
        # ORIENTACION HACIA EL OBJETIVO
        # =================================================

        angulo_goal = math.atan2(

            self.segmento_objetivo_y -
            gy,

            self.segmento_objetivo_x -
            gx
        )

        error_heading = abs(
            normalizar_angulo(
                angulo_goal -
                gtheta
            )
        )

        # =================================================
        # PENALIZAR PASARSE DEL FINAL
        # =================================================

        exceso = max(
            0.0,

            progreso -
            self.segmento_longitud
        )

        # =================================================
        # COSTE DE OBSTACULOS
        # =================================================

        coste_obstaculo = 0.0

        if distancia_minima < 0.60:

            margen = max(

                0.03,

                distancia_minima -
                self.radio_colision
            )

            coste_obstaculo = (
                1.0 /
                margen
            )

        # =================================================
        # PUNTUACION TOTAL
        # =================================================

        score = (

            4.5 *
            distancia_goal

            +

            3.0 *
            distancia_linea

            +

            0.10 *
            coste_linea_acumulado

            +

            0.80 *
            error_heading

            +

            0.08 *
            coste_obstaculo

            +

            0.35 *
            abs(w)

            +

            4.0 *
            exceso

            -

            1.30 *
            progreso

            -

            0.60 *
            v
        )

        # =================================================
        # ¿ESTAMOS HACIENDO UNA ESQUIVA?
        # =================================================

        desvio = (

            distancia_linea >
            0.12

            or

            (
                coste_linea_acumulado /
                pasos
            ) >
            0.08
        )

        return (
            score,
            desvio
        )


    # =========================================================
    # DISTANCIA A LA LINEA NOMINAL
    # =========================================================

    def distancia_a_linea(
        self,
        x,
        y
    ):

        dx = (
            x -
            self.segmento_inicio_x
        )

        dy = (
            y -
            self.segmento_inicio_y
        )

        nx = (
            -self.segmento_uy
        )

        ny = (
            self.segmento_ux
        )

        return abs(
            dx * nx +
            dy * ny
        )


    # =========================================================
    # PROGRESO SOBRE EL SEGMENTO
    # =========================================================

    def progreso_segmento(
        self,
        x,
        y
    ):

        dx = (
            x -
            self.segmento_inicio_x
        )

        dy = (
            y -
            self.segmento_inicio_y
        )

        return (

            dx *
            self.segmento_ux

            +

            dy *
            self.segmento_uy
        )


    # =========================================================
    # MANIOBRA DE EMERGENCIA
    # =========================================================

    def comando_emergencia(self):

        # En obstacle_avoidance no esperamos
        # detenidos.
        #
        # Seguimos avanzando lentamente.

        v = 0.03

        if (
            self.left_space >=
            self.right_space
        ):

            w = 0.90

        else:

            w = -0.90

        self.get_logger().warn(
            'Sin trayectoria segura en el horizonte. '
            'Aplicando maniobra de emergencia.'
        )

        return (
            v,
            w
        )


    # =========================================================
    # GIROS
    # =========================================================

    def control_giro(self):

        error = normalizar_angulo(
            self.theta_objetivo -
            self.theta
        )

        if (
            abs(error) <=
            self.tolerancia_angular
        ):

            self.get_logger().info(
                f'Giro completado. '
                f'Error angular='
                f'{math.degrees(error):.2f} grados'
            )

            self.terminar_accion()

            return

        w = limitar(

            self.k_giro *
            error,

            -0.60,

            0.60
        )

        self.publicar_velocidad(
            0.0,
            w
        )


    # =========================================================
    # PUBLICAR VELOCIDAD
    # =========================================================

    def publicar_velocidad(
        self,
        v,
        w
    ):

        mensaje = Twist()

        mensaje.linear.x = float(v)

        mensaje.angular.z = float(w)

        self.publisher.publish(
            mensaje
        )

        self.ultimo_v = float(v)
        self.ultimo_w = float(w)


    # =========================================================
    # PARAR
    # =========================================================

    def parar(self):

        try:

            self.publicar_velocidad(
                0.0,
                0.0
            )

        except Exception:

            pass


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = ObstacleAvoidance()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.parar()

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == '__main__':

    main()