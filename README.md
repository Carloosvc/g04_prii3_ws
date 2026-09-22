# Sprint 1 - Proyecto RII 3 - Grupo 04

Proyecto realizado para la asignatura Proyecto RII 3.

El programa utiliza ROS2 Humble y turtlesim para dibujar automáticamente
el número del grupo, 04.

## Requisitos

- Ubuntu 22.04
- ROS2 Humble
- turtlesim
- Python 3
- colcon

## Descargar el proyecto

Clonar el repositorio:

```bash
git clone https://github.com/Carloosvc/g04_prii3_ws.git
```

Entrar en el workspace:

```bash
cd g04_prii3_ws
```

## Compilar

Compilar el workspace:

```bash
colcon build
```

Cargar el entorno del workspace:

```bash
source install/setup.bash
```

## Ejecutar

El proyecto se inicia desde un único fichero launch:

```bash
ros2 launch g04_prii3_turtlesim dibujo04.launch.py
```

Al ejecutarlo:

- Se inicia turtlesim.
- Se inicia el nodo `dibujo04`.
- La tortuga dibuja automáticamente el número 04.

## Servicios

El dibujo se puede controlar mediante tres servicios.

### Detener el dibujo

```bash
ros2 service call /detener_dibujo std_srvs/srv/Empty "{}"
```

### Reanudar el dibujo

```bash
ros2 service call /reanudar_dibujo std_srvs/srv/Empty "{}"
```

### Reiniciar el dibujo

```bash
ros2 service call /reiniciar_dibujo std_srvs/srv/Empty "{}"
```

## Funcionamiento

El nodo `dibujo04` publica mensajes de tipo `Twist` en el topic:

```text
/turtle1/cmd_vel
```

De esta forma controla la velocidad lineal y angular de la tortuga.

También se suscribe al topic:

```text
/turtle1/pose
```

para conocer la posición y orientación de la tortuga y realizar los giros correctamente.

## Grupo 04
