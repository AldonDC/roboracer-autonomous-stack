# 🏁 RoboRacer — Documentación Técnica y Plan de Trabajo

> **Alfonso D. — Tecnológico de Monterrey**
> Materia: Assesment
> Asesores: Dr. Daniel Sosa-Ceron · Dr. Jorge A. Reyes-Avendaño
> Última actualización: Abril 2026

---

## 📋 1. Ojetivo del Proyecto
Diseñar y desplegar un **software stack completo** para carreras autónomas de alta velocidad. El vehículo debe integrar chasis mecánico, electrónica de potencia y cómputo de alto rendimiento (NVIDIA Jetson). Las tareas incluyen: Time Trials, evasión de obstáculos dinámicos, y carreras *head-to-head*.

---

## 🏛️ 2. Arquitectura del Sistema (Gazebo + RViz)

### 2.1. Migración de la Pista a Gazebo
Para tener un entorno de pruebas hiperrealista, se recreó la pista oficial de la competencia dentro de **Gazebo Harmonic**. 

* **Mesh 3D (`qcar_track.obj`)**: Se importó el modelo 3D exacto de la pista con sus texturas mapeadas (`SDCS_MapLayout.png`).
* **Mundo de Gazebo (`test_world.sdf`)**: Se configuró utilizando los plugins físicos nativos (`dartsim`). La pista se carga estáticamente junto a barreras perimetrales (walls) para contener al carro.
* **Sincronización:** Gazebo procesa las físicas a alta frecuencia (1000Hz) y expone la odometría del QCar a ROS 2 vía `ros_gz_bridge`.

### 2.2. Visualización Profesional en RViz
Para evitar usar GUIs externas lentas (como Matplotlib) y mantener un workflow estándar de robótica, toda la interacción ocurre en **RViz**:
* `odom_tf_broadcaster.py`: Traduce la odometría plana de Gazebo en transformaciones TF formales (`world` → `base_link`), moviendo al carro en RViz en tiempo real.
* `track_visualizer.py`: Extrae los vértices del `.obj` de la pista de Gazebo y los transmite como `Marker` a RViz, permitiendo ver la pista real como plantilla base.

---

## 🧮 3. Controlador Matemático: Pure Pursuit Adaptativo

El seguimiento autónomo de ruta (Waypoint Navigation) utiliza una variante mejorada del **Algoritmo Pure Pursuit**, adaptado específicamente para esquemas de dirección tipo Ackermann (como el QCar).

### 3.1. Teoría Pura (Pure Pursuit)
El objetivo de Pure Pursuit es encontrar el ángulo de giro (steering angle $\delta$) que mantenga al carro sobre un arco circular que intersecte un objetivo a una distancia $L_d$ (Lookahead Distance).

**Ecuación de Dirección:**

$$
\delta = \arctan\left(\frac{2L \sin(\alpha)}{L_d}\right)
$$

Donde:
* $L$ = Distancia entre ejes del carro (0.256m).
* $\alpha$ = Ángulo entre el heading actual del carro y el waypoint.
* $L_d$ = Lookahead distance (dinámico basado en la distancia al punto).

### 3.2. Mejoras Implementadas (Adaptive Speed)
Nuestro controlador no mantiene velocidad constante, en su lugar, analiza la agudeza del giro para frenar antes de curvas cerradas:

1. **Clip de Dirección**: El ángulo $\delta$ jamás supera `max_steer` (0.5 rad).
2. **Curve Factor**: Calculamos qué tanto está girando el volante relativo al límite.

$$
C_f = 1.0 - 0.4 \left|\frac{\delta}{\delta_{max}}\right|
$$

3. **Velocidad de Salida**: Reducimos el `v_ref` (velocidad objetivo) proporcionalmente al *Curve Factor*. Si el carro va recto, asume el 100% de vel; si gira brusco, cae al 60%.
4. **Aceleración Ramp-up**: Perfil de aceleración/frenado escalonado (`+0.02 m/s` acelerando, `-0.04 m/s` frenando) eliminando jalones mecánicos.

---

## ⚙️ 4. Nodos de Software (`roboracer_racing`)

El core de lógica customizada habita en el paquete `roboracer_racing`.

| Nodo (Python) | Descripción |
|:--------------|:------------|
| `multi_goal_navigator.py` | **Manejador Maestro**. Lee los clicks del "Publish Point" en RViz, acumula los waypoints, calcula la cinemática Pure Pursuit en tiempo real, y expone un CLI asíncrono para dar la orden de inicio (`[g] Go`). |
| `odom_tf_broadcaster.py` | **Puente Espacial**. Lee `/qcar_sim/odom` y emite el *Transform Tree* oficial. |
| `track_visualizer.py` | **Pintor 3D**. Inserta el mapa de Gazebo dentro de RViz. |
| `keyboard_teleop.py` | **Conducción Mánual**. Script WASD de precisión para dominar la pista. |

---

## 📂 5. Estructura del Proyecto

A continuación se muestra cómo está organizado el código dentro del repositorio, separando claramente lo que es configuración de simulación y lo que es lógica de carreras:

```text
roboracer-autonomous-stack/
├── docs/                             # Documentación extendida
├── scripts/
│   ├── build.sh                      # Script para compilar el código rápidamente
│   └── launch_sim.sh                 # Máster script para arrancar Gazebo, RViz y Bridges
├── src/
│   ├── racing_logic/                 # 🧠 AQUÍ VIVE LA INTELIGENCIA
│   │   └── roboracer_racing/         # Nuestro paquete principal de Python
│   │       ├── multi_goal_navigator.py # CLI Planner y Lógica Pure Pursuit
│   │       ├── odom_tf_broadcaster.py  # Sincronizador de coordenadas
│   │       ├── track_visualizer.py     # Dibuja la pista en RViz
│   │       └── keyboard_teleop.py      # Conducción manual
│   ├── roboracer/                    # 🚗 SIMULACIÓN Y ROBOT
│   │   ├── roboracer_description/    # Modelos 3D (URDF), Sensores y config de RViz
│   │   ├── roboracer_gazebo/         # El Mundo de Gazebo (Pistas, Muros) y el Bridge
│   │   └── roboracer_interfaces/     # Mensajes customizados de ROS 2
│   └── support/                      # Nodos helpers y librerías externas de cámaras
└── README.md                         # Este documento
```

---

## 🚀 6. Manual Rápido (Comandos de Uso)

### Paso 1: Lanzar la Simulación (Terminal 1)
```bash
cd ~/Documents/Assesment-Auto
./scripts/launch_sim.sh
```
*Esto arranca Gazebo con el QCar, la pista, inicializa los puentes ROS y abre RViz listo.*

### Paso 2: Preparar Control y CLI (Terminal 2)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/Documents/Assesment-Auto
colcon build --symlink-install
source install/setup.bash

# Lanzamos el sincronizador de RViz y la pista (corriendo de fondo)
ros2 run roboracer_racing odom_tf &
ros2 run roboracer_racing track_viz &

# Lanzamos el Controlador Multipunto CLI
ros2 run roboracer_racing multi_goal
```

### Paso 3: Interactuar
1. En RViz (que se abrió solo en el Paso 1), usa la herramienta **Publish Point** (arriba a la derecha).
2. Da **todos los clicks que quieras** a lo largo de la pista. (Verás aparecer esferas numeradas).
3. Ve a la Terminal 2 y escribe **`g`**, luego `Enter` para que el carro arranque la carrera, siguiendo la línea de path.
4. Escribe **`s`** si quieres guardar la ruta perfecta, o **`c`** para limpiar y volver a intertarlo.

---

## 📊 7. Plan de Trabajo Restante

**✅ Fase 1: Simulación y RViz Completa** (Gazebo y Visualización estables).
**✅ Fase 2: Control Time-Trial Básico** (Pure pursuit + GUI/CLI).

**⏳ Fase 3: Evasión (Siguiente paso lógico)**
* Activar el escaneo LiDAR (`/qcar_sim/scan`).
* Integrar lógica que evite el estancamiento y haga desvíos laterales cuando detecte una obstrucción en el path de waypoints.

**⏳ Fase 4: Head-to-Head**
* Modificar `test_world.sdf` para *spawnear* dos QCars e implementar detección adversaria.

---
*"El que no arriesga, no gana la carrera."* 🏁
