<div align="center">

# 🏎️ RoboRacer · QCar 1 — Despliegue Físico

### Integración de Percepción, Planeación y Control en un vehículo autónomo a escala 1:10

**Tecnológico de Monterrey, Campus Puebla** · Bloque de Integración Final

Alfonso Solís Díaz · Emmanuel Lechuga Arreola
Profesores: A. Daniel Sosa-Cerón, Ph.D. · Jorge A. Reyes-Avendaño, Ph.D.

![Demo del QCar siguiendo el carril y evadiendo obstáculos](docs/assets/demo_qcar.gif)

*El QCar sigue el carril por visión, detecta los obstáculos y los bordea sin colisión, retomando la línea (×6).*

</div>

---

## 📑 Índice

1. [Resumen](#-1-resumen)
2. [El problema](#-2-el-problema)
3. [Objetivos](#-3-objetivos)
4. [Arquitectura del sistema](#-4-arquitectura-del-sistema)
5. [Plataforma y modelo cinemático](#-5-plataforma-y-modelo-cinemático)
6. [Percepción](#-6-percepción-del-pixel-al-a-dónde-ir)
7. [Control — Pure Pursuit](#-7-control--pure-pursuit)
8. [Decisión — FSM de evasión](#-8-decisión--máquina-de-estados-de-evasión)
9. [Fusión visión–LIDAR](#-9-fusión-visiónlidar)
10. [Puesta en marcha (red + lanzamiento)](#-10-puesta-en-marcha)
11. [Mapa de topics ROS 2](#-11-mapa-de-topics-ros-2)
12. [Estructura del paquete](#-12-estructura-del-paquete)
13. [Módulos del stack](#-13-módulos-del-stack)
14. [Dashboard de monitoreo](#-14-dashboard-de-monitoreo)
15. [Problemas y soluciones](#-15-problemas-y-soluciones)
16. [Resultados](#-16-resultados)
17. [Conclusiones y trabajo futuro](#-17-conclusiones-y-trabajo-futuro)
18. [Rebuild y sincronización](#-18-rebuild-y-sincronización)

---

## 🎯 1. Resumen

Stack de software en **ROS 2** que conduce un **Quanser QCar 1** de forma autónoma sobre
cómputo embebido (NVIDIA Jetson): **sigue el carril por visión**, **detecta y evade
obstáculos** con LIDAR y profundidad, y mantiene un lazo de **control geométrico a 50 Hz**.
La arquitectura es **modular** (nodos ROS 2 independientes), prioriza **robustez y
simplicidad**, y fue validada en pista cerrada con obstáculos estáticos.

| | |
|---|---|
| **Plataforma** | Quanser QCar 1 — LIDAR 2D, cámara RGB-D Intel RealSense D435, Jetson |
| **Middleware** | ROS 2 · DDS (FastRTPS), red distribuida Jetson ↔ laptop |
| **Percepción** | Visión clásica (BEV + ajuste polinomial) + LIDAR + profundidad |
| **Control** | Pure Pursuit adaptativo, 50 Hz |
| **Decisión** | Máquina de estados (FSM) de evasión + fusión visión–LIDAR |
| **Seguridad** | Tope de velocidad, freno por proximidad, distancia representativa anti-ruido |

---

## ❓ 2. El problema

**RoboRacer** es una competencia de carreras autónomas a escala **1:10**, cabeza a cabeza.
No basta con ir rápido: el vehículo debe **evadir obstáculos y no chocar**. El reto real es
**integrar percepción + planeación + control en tiempo real** sobre cómputo embebido,
con sensores ruidosos y energía limitada.

> **En una frase:** el carro debe **ver**, **decidir** y **moverse** solo, rápido y sin golpear nada.

---

## 🎯 3. Objetivos

**General** — Desplegar un *stack* en ROS 2 que conduzca el QCar de forma autónoma,
siguiendo el carril y evadiendo obstáculos sin colisión.

**Específicos**
- Seguir el carril por **visión** y completar el circuito.
- **Detectar y evadir** obstáculos estáticos con **LIDAR** y **profundidad**.
- Arquitectura **modular** (nodos ROS 2) con control a **50 Hz**.
- **Seguridad**: tope de velocidad y paro ante riesgo inminente.

---

## 🏛️ 4. Arquitectura del sistema

Flujo **sensores → percepción → planeación → control → actuación**. Cada bloque es un nodo
ROS 2 independiente: si uno falla, los demás siguen; cada uno se prueba y reemplaza por separado.

```
   SENSORES            PERCEPCIÓN            PLANEACIÓN          CONTROL        ACTUACIÓN
┌────────────┐      ┌──────────────┐      ┌──────────────┐                  ┌────────────┐
│  LIDAR 2D  │─────▶│lidar_processor│────▶│ gap_follower │──┐               │ user_cmd   │
├────────────┤      ├──────────────┤      ├──────────────┤  │  ┌─────────┐  ├────────────┤
│RealSense   │─────▶│depth_processor│──┐  │ FSM evasión  │──┴─▶│  Pure   │─▶│ Motor +    │
│  D435      │      ├──────────────┤  └─▶│  + fusión    │     │ Pursuit │  │ Servo      │
├────────────┤      │              │     └──────────────┘     └─────────┘  │  (v, δ)    │
│  Cámara    │─────▶│ lane_detector │────────────▲                          └────────────┘
└────────────┘      └──────────────┘                                       
```

**¿Por qué ROS 2?** Cada función es un nodo pequeño que hace una sola tarea (leer LIDAR,
ver el carril, controlar el motor…) y los nodos se comunican por **topics** (canales con
nombre). Es como una cocina donde cada cocinero hace un paso y pasa el platillo por una banda.

### Decisiones de diseño

| Subsistema | Alternativas | **Elección** y porqué |
|---|---|---|
| Percepción | un sensor *vs.* fusión | **Fusión** (cámara + LIDAR + profundidad): más robusta |
| Planeación | reactiva pura *vs.* trayectoria pregrabada | **Híbrida**: rápida y segura ante obstáculos |
| Control | PID / Pure Pursuit *vs.* MPC | **Pure Pursuit**: ligero, cabe a 50 Hz en el Jetson |
| Decisión | FSM *vs.* árbol de comportamiento | **FSM**: más predecible y fácil de depurar |

> Criterio transversal: **viabilidad · robustez · costo de cómputo** sobre el hardware embebido.

---

## 🧮 5. Plataforma y modelo cinemático

El QCar usa **dirección Ackermann**. Su movimiento se modela con la **bicicleta cinemática**:

$$\dot\psi = \frac{v}{L}\tan\delta \qquad R = \frac{L}{\tan\delta}$$

donde $v$ es la velocidad, $\delta$ el giro de las llantas, $\psi$ la orientación y $R$ el radio de
giro. **Más velocidad o más giro ⇒ curva más cerrada** (menor $R$).

### Restricciones clave (medibles)

| Parámetro | Valor |
|---|---|
| Distancia entre ejes $L$ | **0.256 m** |
| Ángulo de giro máx. $\delta_{\max}$ | **≈ 30°** (0.30 rad) |
| LIDAR | cono **90°**, válido desde **0.15 m**, máx. **5 m** |
| Cámara RealSense D435 | FOV **69°**, profundidad en metros |
| Velocidad de operación | **0.08 – 0.18 m/s** |
| Zona muerta del motor | vel. mínima **0.12** para que arranque |
| Cómputo | visión **320×240**, control **50 Hz**, visión **clásica** |

---

## 🧠 6. Percepción: del pixel al "a dónde ir"

Pipeline de visión en `lane_detector.py`:

```
Imagen cámara → CLAHE → Máscara amarilla (HSV/HLS) → Vista de pájaro (BEV) → Punto objetivo
```

- **CLAHE** — mejora el contraste para que la línea resalte aunque cambie la luz.
- **Máscara amarilla (HSV/HLS)** — deja solo los píxeles del color del carril.
- **Vista de pájaro (BEV)** — *endereza* la imagen como vista cenital, libre de perspectiva.
- **Ajuste polinomial** — parábola de 2° grado por carril, suavizada con un **filtro EMA**.
- **Punto objetivo** — una coordenada simple $(x, y)$ en metros de a dónde ir.

> La **línea amarilla** central es el carril a seguir.

<details>
<summary><b>Parámetros de calibración (CSI 320×240)</b></summary>

```python
# Cámara
hfov = 160°, vfov = 120°
fx = 160 / tan(rad(80)) ≈ 90.4 px
fy = 120 / tan(rad(60)) ≈ 69.3 px
distCoefs = [-0.2646, 0.01563, 6.53e-5, 5.4e-3, 0.0822]

# Bird's-Eye View
src = [[100,140],[220,140],[320,230],[0,230]]   # imagen original
dst = [[60,0],  [260,0],  [260,240],[60,240]]    # vista cenital
```

| Canal | Parámetro | Default |
|---|---|---|
| Amarillo H | `h_min / h_max` | 8 / 55 |
| Amarillo S | `s_min` | 20 |
| Amarillo V | `v_min` | 40 |
| Blanco L (HLS) | — | > 160 |
| Blanco S (HLS) | — | < 80 |

</details>

### Percepción de distancia: LIDAR + profundidad

- **LIDAR** (`lidar_processor.py`) — mide qué tan lejos está lo más cercano y en qué ángulo
  dentro de un cono frontal de **90°**. Se usa un **percentil**, no el mínimo, para no frenar
  por un punto aislado y erróneo.
- **Profundidad** (`depth_processor.py`) — cuenta píxeles cercanos al frente (RealSense D435);
  si pasan un umbral, hay obstáculo. Confirma lo que ve el LIDAR.
- **Círculo de seguridad** — si algo entra muy cerca, **paro inmediato**.

> Tres fuentes (visión + LIDAR + profundidad) ⇒ si una falla, las otras ayudan.

<details>
<summary><b>Matemática del LIDAR</b></summary>

Ángulo por índice (con flip y offset de calibración):

$$\theta_i = (-1)^{\text{flip}}\,(\theta_{\min} + i\,\Delta\theta) + \theta_{\text{offset}}$$

Cono frontal de seguridad y obstáculo más cercano:

$$\text{front}_i = \text{valid}_i \wedge |\theta_i| < \phi/2 \qquad i^\* = \arg\min_{i \in \text{front}} r_i$$

Zonas de seguridad: **DANGER** `< 0.6 m` 🟥 · **WARNING** `0.6–1.5 m` 🟨 · **SAFE** `> 1.5 m` 🟦

</details>

---

## 🎯 7. Control — Pure Pursuit

**Idea (como un conductor):** fija la vista en un punto del carril más adelante y gira hacia él.

$$\delta = \text{atan2}\big(2L\sin\alpha,\; l_d\big)$$

donde $\delta$ es el giro de llantas, $\alpha$ el ángulo al punto objetivo, $L = 0.256$ m el
wheelbase y $l_d$ el **lookahead** dinámico:

$$l_d = 0.5\,S_{\text{base}} + 0.4\,v$$

- Más rápido ⇒ **mira más lejos** (trayectoria más suave).
- En curva cerrada **baja la velocidad** automáticamente.
- **Tope de seguridad** y **compensación de la zona muerta** del motor.

| Parámetro | Default | | Parámetro | Default |
|---|---|---|---|---|
| `lookahead_min` | 0.16 m | | `speed_straight` | 0.15 m/s |
| `lookahead_max` | 0.34 m | | `speed_curve` | 0.0775 m/s |
| `lookahead_base` | 0.20 m | | `curve_threshold` | 0.20 rad |
| `lookahead_speed_gain` | 0.80 | | `max_steering_angle` | 0.30 rad |

**Safety LIDAR integrado** en el follower:

```
d < 0.6 m          →  speed × 0.0   (freno total)
d < 1.5 m          →  speed × 0.4   (slow zone)
sin datos > 0.5 s  →  sin freno     (sensor offline, ignora)
```

---

## 🔁 8. Decisión — Máquina de estados de evasión

Cuando hay obstáculo, el follower entra en una **FSM** de pasos claros y predecibles
(fácil de seguir y depurar):

```
            ┌──────── sin obst. ────────┐
            ▼                           │
        ┌────────┐  obst.   ┌───────────┐
        │ NORMAL │─────────▶│ REVERSING │
        └────────┘          └───────────┘
            ▲                      │ libre
       listo│                      ▼
        ┌──────────┐  rebasa  ┌────────────┐
        │ SETTLING │◀─────────│ STEER_AWAY │
        └──────────┘          └────────────┘
```

| Estado | Qué hace |
|---|---|
| **NORMAL** | Sigue el carril. |
| **REVERSING** | Retrocede girando al lado opuesto. |
| **STEER_AWAY** | Avanza esquivando (rebasa). |
| **SETTLING** | Se estabiliza y vuelve al carril. |

---

## ➕ 9. Fusión visión–LIDAR

El giro final **mezcla** dos sugerencias según qué tan cerca está el obstáculo:

$$\delta = w\,\delta_{\text{cámara}} + (1-w)\,\delta_{\text{LIDAR}}$$

El peso $w \in [0,1]$ lo decide la **distancia al obstáculo**:

- Obstáculo **lejos** → manda la **cámara** (seguir carril).
- Obstáculo **cerca** → manda el **LIDAR** (esquivar).

El cambio es **suave** (transición gradual, sin brincos), se **adapta a la distancia** (no un
50/50 fijo) y es **ligero y predecible** frente a un MPC o una red neuronal. El nodo
`gap_follower.py` (Disparity Extender vectorizado) provee $\delta_{\text{LIDAR}}$ junto con su
confianza.

---

## 🚀 10. Puesta en marcha

### 10.0 — Conexión remota (SSHFS, opcional)

Edita los archivos de la Jetson desde tu laptop como si fueran locales:

```bash
# Montar (laptop)
sshfs -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3 \
  nvidia@192.168.2.12:/home/nvidia ~/Documents/qcar_remoto

# Desmontar si la conexión se traba
fusermount -u ~/Documents/qcar_remoto
```

### 10.1 — Variables de red (CRÍTICO — en todas las terminales)

La Jetson y la laptop usan el **mismo `ROS_DOMAIN_ID`** en la misma red; con ROS 2/DDS se
descubren solos y comparten los topics.

```bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### 10.2 — Lanzamiento (Opción A: recomendada)

Un solo comando levanta **lane_detector + lidar_processor + lane_follower_pp** en la Jetson.
El follower arranca **automáticamente** al detectar que no hay TTY (modo launch).

```bash
# Terminal 1 — Jetson: hardware base (sensores + actuadores + TF)
cd ~/Assesment_qcar_irs && source install/setup.bash
ros2 launch roboracer_racing physical_racing.launch.py

# Terminal 2 — Jetson: pipeline autónoma completa
cd ~/Assesment_qcar_irs && source install/setup.bash
ros2 launch roboracer_racing lane_pursuit.launch.py

# Terminal 3 — Laptop: dashboard de monitoreo
cd ~/Documents/qcar_remoto/Assesment_qcar_irs && source install/setup.bash
ros2 run roboracer_racing physical_dashboard
```

Con dashboard incluido en el mismo launch:
```bash
ros2 launch roboracer_racing lane_pursuit.launch.py launch_dashboard:=true
```

Con parámetros personalizados:
```bash
ros2 launch roboracer_racing lane_pursuit.launch.py \
  speed_straight:=0.18 speed_curve:=0.09 use_lidar_safety:=true
```

### 10.3 — Lanzamiento (Opción B: nodos individuales, para debug)

```bash
ros2 launch roboracer_racing physical_racing.launch.py   # hardware base
ros2 run roboracer_racing lane_detector                  # detector de carril
ros2 run roboracer_racing lidar_processor                # procesador LIDAR
ros2 run roboracer_racing lane_follower_pp               # follower (TTY: ENTER=arranca, Q=para)
ros2 run roboracer_racing keyboard_teleop                # teleop manual (opcional)
ros2 run roboracer_racing physical_dashboard             # dashboard (laptop)
```

> **TTY detection:** el mismo binario `lane_follower_pp` espera **ENTER/SPACE** para arrancar
> en `ros2 run`, pero arranca **solo** en `ros2 launch`. El estado se publica en
> `/lane/auto_active` → el dashboard muestra **AUTÓNOMO** / **MANUAL**.

---

## 🗺️ 11. Mapa de topics ROS 2

| Topic | Tipo | Publica | Consumido por |
|-------|------|---------|---------------|
| `/qcar/csi_front` | `CompressedImage` | `csi_node` | `lane_detector`, dashboard |
| `/qcar/scan` | `LaserScan` | `lidar_node` | `lidar_processor` |
| `/qcar/velocity` | `Vector3Stamped` | `qcar_node` | dashboard, plots |
| `/qcar/stateBattery` | `BatteryState` | `qcar_node` | dashboard |
| `/qcar/user_command` | `Vector3Stamped` | `lane_follower_pp` | hardware QCar |
| `/lane_target_point_m` | `Float32MultiArray` | `lane_detector` | `lane_follower_pp` |
| `/lane/center_offset` | `Float32` | `lane_detector` | dashboard, plots |
| `/lane/confidence` | `Float32` | `lane_detector` | dashboard, plots |
| `/lane/auto_active` | `Bool` | `lane_follower_pp` | dashboard (AUTÓNOMO/MANUAL) |
| `/lidar/image_debug` | `CompressedImage` | `lidar_processor` | dashboard (radar) |
| `/lidar/min_distance` | `Float32` | `lidar_processor` | dashboard, `lane_follower_pp` |
| `/lidar/closest_angle` | `Float32` | `lidar_processor` | dashboard |
| `/lidar/obstacle` | `Bool` | `lidar_processor` | dashboard, `lane_follower_pp` |
| `/depth/obstacle_detected` | `Bool` | `depth_processor` | `lane_follower_pp`, dashboard |
| `/depth/closest_angle` | `Float32` | `depth_processor` | dashboard |
| `/gap/steer_cmd` | `Float32` | `gap_follower` | `lane_follower_pp` (fusión) |

---

## 🧱 12. Estructura del paquete

```
src/roboracer_racing/
├── launch/
│   ├── physical_racing.launch.py      ← qcar + lidar + csi + tf (hardware base)
│   ├── lane_pursuit.launch.py         ← pipeline autónoma completa ⭐
│   ├── lane_detector_only.launch.py   ← solo percepción de carril (debug)
│   └── competition.launch.py
├── roboracer_racing/
│   ├── lane_detector.py               ← BEV + poly fit + EMA + debug pro ⭐
│   ├── lane_follower_pp.py            ← Pure Pursuit + FSM evasión + fusión + safety ⭐
│   ├── lidar_processor.py             ← LaserScan → radar + métricas
│   ├── depth_processor.py             ← profundidad RealSense → distancia frontal
│   ├── gap_follower.py                ← Disparity Extender vectorizado (LIDAR)
│   ├── physical_dashboard.py          ← Dashboard v4 (Qt + pyqtgraph)
│   ├── keyboard_teleop.py             ← Teleop v4 SNAP mode
│   ├── yellow_line_tracker.py         ← seguimiento directo de línea amarilla
│   ├── lane_obstacle_avoider.py       ← evasión por cambio de carril
│   ├── qcar_state_estimator.py        ← odometría fusionada (IMU + tach + TF)
│   ├── imu_bno055.py                  ← publisher IMU BNO055 (serial)
│   └── ...
└── setup.py
```

---

## 🧩 13. Módulos del stack

### Núcleo (pipeline autónoma)

| Nodo | Rol |
|---|---|
| `lane_detector` | Percepción de carril: BEV + ajuste polinomial + EMA + debug 840×240. |
| `lidar_processor` | LaserScan → radar top-down, cono frontal, distancia/ángulo del obstáculo. |
| `lane_follower_pp` | Cerebro de control: Pure Pursuit + FSM de evasión + fusión + safety LIDAR. |
| `physical_dashboard` | Monitoreo en vivo (cámaras, radar, profundidad, telemetría). |
| `keyboard_teleop` | Teleoperación manual con modo SNAP (llantas visibles al instante). |

### Complementarios (fusión, sensores, estado)

Opcionales: el pipeline mínimo funciona sin ellos, pero añaden fusión sensorial, evasión y
estimación de estado.

| Nodo | Rol | Publica |
|---|---|---|
| `gap_follower` | Disparity Extender vectorizado (LIDAR), propuesta de steering. | `/gap/steer_cmd`, `/gap/confidence`, `/gap/distance` |
| `depth_processor` | Proximidad por profundidad RealSense D435 (no YOLO, no color). | `/depth/obstacle_detected`, `/depth/closest_angle`, `/depth/image_debug` |
| `yellow_line_tracker` | Segundo "ojo" enfocado solo en la línea amarilla central. | `/yellow_line/target_point_m`, `/yellow_line/found`, `/yellow_line/image_debug` |
| `lane_obstacle_avoider` | Evasión por cambio de carril (reescribe el target con sesgo lateral). | `/lane_target_point_m_safe` |
| `qcar_state_estimator` | Odometría fusionada IMU BNO055 + tach + dead reckoning. | `/qcar/odom_fused`, `/qcar/trajectory`, TF |
| `imu_bno055` | Publisher serial del IMU BNO055. | `/imu/data`, `/imu/accel_raw` |

---

## 🖥️ 14. Dashboard de monitoreo

Construimos un **dashboard interactivo** (Qt + `pyqtgraph`) para ver en tiempo real estado,
carril, LIDAR, profundidad, cámaras y telemetría. La **Jetson solo publica datos** y la
visualización (pesada) corre en la **laptop**, así no se satura la Jetson y queda más cómputo
para percepción y control.

```
┌──────────────────────┬──────────────────────────────────────┐
│ System Status        │  🎯 LIDAR RADAR  /lidar/image_debug  │
│  ROS · Piloto · Bat. │  CÁMARA FRONTAL  /qcar/csi_front     │
│ Lane Detector        │  YELLOW TRACKER  /yellow_line/...     │
│  Modo · Offset · Conf│  ZED PROFUNDIDAD /depth/image_debug   │
│ LIDAR /qcar/scan     │  LANE DEBUG  /lane/image_debug        │
│ 📈 Telemetría        │      840×240 [raw | BEV | stats]      │
└──────────────────────┴──────────────────────────────────────┘
```

| Indicador | Estado | Color |
|---|---|---|
| Lane conf ≥ 0.70 | DETECTADO | 🟢 |
| Lane conf 0.25–0.70 | PARCIAL | 🟡 |
| Lane conf < 0.25 / sin señal | CIEGO | 🔴 |
| `auto_active=True` | **AUTÓNOMO** | 🔵 |
| `auto_active=False` | **MANUAL** | ⚪ |

---

## 🛠️ 15. Problemas y soluciones

| Problema | Solución |
|---|---|
| Zona muerta del motor/servo (no arranca a baja señal) | Velocidad mínima de motor y signo de servo calibrado. |
| IMU con deriva/ruido y complejidad extra | **No usar IMU** en el lazo base: control sobre el punto de visión (marco local). |
| Visión sensible a la iluminación | **CLAHE** + fusión con LIDAR ponderada por confianza. |
| Cómputo limitado del Jetson | **NumPy vectorizado**, *downscale* de profundidad, **sin DNN**. |
| Frenado de más por una lectura aislada del LIDAR | Distancia **representativa** (percentil) que ignora puntos sueltos. |

---

## 📊 16. Resultados

**Validación cualitativa en pista cerrada** con obstáculos estáticos (torres tipo edificio,
muro y caja) colocados sobre o junto al carril para forzar la evasión:

| Escenario | Evasión | Observación |
|---|---|---|
| Seguimiento de carril | — | Sigue la línea de forma estable. |
| Evasión de obstáculo | ✅ | Bordea sin colisión y retoma el carril. |

El QCar completó el recorrido, **detectó los obstáculos y los esquivó sin chocar**, volviendo
al carril — como se ve en el GIF de arriba (la cámara cenital es solo registro; el carro
percibe a bordo).

---

## 🏁 17. Conclusiones y trabajo futuro

- Una arquitectura **modular en ROS 2** con percepción multimodal y un control geométrico
  ligero **funciona** sobre cómputo embebido.
- La **fusión visión–LIDAR** logra una transición estable entre seguir el carril y evadir
  obstáculos.
- Decisiones de ingeniería (no IMU en el lazo base, visión clásica) priorizaron **robustez y
  simplicidad**.

**Trabajo futuro:** localización con IMU/EKF para un modo de carrera global (ver
`qcar_state_estimator`), control predictivo (MPC) en hardware más potente, y percepción con
aprendizaje.

---

## 🔧 18. Rebuild y sincronización

```bash
# En la Jetson (o vía SSHFS desde la laptop)
cd ~/Assesment_qcar_irs
colcon build --packages-select roboracer_racing --symlink-install
source install/setup.bash
```

Si `colcon build` falla por permisos de un build previo con root:
```bash
sudo chmod 666 build/roboracer_racing/colcon_build.rc
colcon build --packages-select roboracer_racing
```

Sincronizar los cambios del QCar físico al repositorio (desde la laptop):
```bash
cd ~/Documents/Assesment-Auto
./sync_qcar.sh
```

---

<div align="center">

**RoboRacer · QCar 1** — Tecnológico de Monterrey, Campus Puebla
*Bloque de Integración Final · Integración de Percepción, Planeación y Control*

</div>
