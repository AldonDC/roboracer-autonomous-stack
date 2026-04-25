# 🏎️ QCar: Del Simulador al Físico (RoboRacer Deployment v4)

Guía definitiva para correr el stack `roboracer_racing` en el **hardware físico del QCar** (Quanser + Jetson Nano). Cubre red, secuencia de lanzamiento, arquitectura, **matemática del LIDAR y del Lane Detector**, y la nueva versión **v4 del Dashboard** (Radar + Lane + Telemetría en 2 columnas).

---

## 🔌 0. Conexión de Red Remota (SSHFS)

Para editar los archivos físicos del QCar como si estuvieran en tu computadora (sin lag visual). **SIEMPRE en tu Laptop:**

**Conectar / Montar la carpeta del QCar:**
```bash
sshfs -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3 nvidia@192.168.2.12:/home/nvidia ~/Documents/qcar_remoto
```
*(Te pedirá la contraseña del usuario `nvidia`)*

**Si se traba la conexión (el WiFi parpadeó):**
```bash
fusermount -u ~/Documents/qcar_remoto
```

---

## 🌐 1. Configuración de Red (CRÍTICO)

En **TODAS** las terminales (Jetson y Laptop):

```bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 🚀 2. Secuencia de Lanzamiento Profesional (4 Terminales)

### 🔴 Terminal 1: Encendido del Hardware (SSH Jetson)
*Levanta `qcar_node`, `lidar_node`, `csi_node`, `lidar_processor` y el radar visual.*
```bash
cd ~/Assesment_qcar_irs
colcon build --packages-select roboracer_racing --symlink-install
source install/setup.bash
ros2 launch roboracer_racing physical_racing.launch.py
```

### 🔵 Terminal 2: Motor de Percepción de Carril (SSH Jetson)
*Detecta la línea central con BEV + ajuste polinomial.*
```bash
cd ~/Assesment_qcar_irs
colcon build --packages-select roboracer_racing --symlink-install
source install/setup.bash
ros2 run roboracer_racing lane_detector
```

### 🟢 Terminal 3: Inteligencia Autónoma (SSH Jetson)
*Fusión Lane + LIDAR (Pure Pursuit architecture) con seguridad activa.*
```bash
cd ~/Assesment_qcar_irs
colcon build --packages-select roboracer_racing --symlink-install
source install/setup.bash
ros2 run roboracer_racing autonomous_lane_follower
```

### 🟡 Terminal 4: Teleoperación Activa (SSH Jetson)
*Mueve el QCar físicamente por la pista y dispara la misión autónoma.*
```bash
cd ~/Assesment_qcar_irs
source install/setup.bash
ros2 run roboracer_racing keyboard_teleop  # <--- Presiona 'M' para START
```

### 🖥️ Terminal 5: Estación de Monitoreo (Laptop Local)
*Dashboard visual en tiempo real.*
```bash
# En tu laptop (asegura ROS_DOMAIN_ID=115)
cd ~/Documents/qcar_remoto/Assesment_qcar_irs
source install/setup.bash
ros2 run roboracer_racing physical_dashboard
```

---

## 🗺️ 3. Mapa de Topics ROS 2

| Topic | Tipo | Publica | Consumido por |
|-------|------|---------|---------------|
| `/qcar/csi_front` | `CompressedImage` | `csi_node` | `lane_detector`, dashboard |
| `/qcar/scan` | `LaserScan` | `lidar_node` | `lidar_processor` |
| `/qcar/velocity` | `Vector3Stamped` | `qcar_node` | dashboard, plots |
| `/qcar/stateBattery` | `BatteryState` | `qcar_node` | dashboard (System Status) |
| `/qcar/imu` | `Imu` | `qcar_node` | (futuro: odometría) |
| `/lane/image_debug` | `CompressedImage` | `lane_detector` | dashboard |
| `/lane/center_offset` | `Float32` | `lane_detector` | dashboard, plots |
| `/lane/confidence` | `Float32` | `lane_detector` | dashboard, plots |
| `/lane/stop_sign` | `Bool` | `lane_detector` | dashboard |
| `/lidar/image_debug` | `CompressedImage` | `lidar_processor` | dashboard (radar hero) |
| `/lidar/min_distance` | `Float32` | `lidar_processor` | dashboard, plots |
| `/lidar/closest_angle` | `Float32` | `lidar_processor` | dashboard |
| `/lidar/obstacle` | `Bool` | `lidar_processor` | dashboard |

---

## 🧱 4. Estructura del Paquete `roboracer_racing`

```
src/roboracer_racing/
├── launch/
│   ├── physical_racing.launch.py     ← qcar + lidar + csi + tf
│   └── competition.launch.py
├── roboracer_racing/
│   ├── lane_detector.py              ← BEV + polynomial fit + EMA
│   ├── lidar_processor.py            ← LaserScan → radar + métricas  ⭐ NUEVO
│   ├── physical_dashboard.py         ← Dashboard v4 (Qt + pyqtgraph)
│   ├── pure_pursuit_node.py          ← (futuro) navegación autónoma
│   ├── keyboard_teleop.py            ← Control manual
│   └── ...
└── setup.py                          ← entry points (incluye lidar_processor)
```

**Entry points relevantes** (`setup.py`):
```python
'lane_detector    = roboracer_racing.lane_detector:main'
'lidar_processor  = roboracer_racing.lidar_processor:main'
'physical_dashboard = roboracer_racing.physical_dashboard:main'
```

---

## 🧠 5. Arquitectura de Percepción

### 5.1 Lane Detector (`lane_detector.py`)

- **Bird's-Eye View (BEV)**: homografía 4 puntos → vista cenital sin distorsión perspectiva.
- **Segmentación dual HSV + HLS**: dos espacios de color en paralelo, fusionados con `AND` lógico → ignora reflejos y sombras.
- **Ajuste polinomial de 2° grado**: parábola $x = ay^2 + by + c$ por mínimos cuadrados sobre los píxeles amarillos.
- **Filtro EMA temporal**: $\hat{o}_t = \alpha \, o_t + (1 - \alpha) \, \hat{o}_{t-1}$ , con $\alpha = 0.3$ → suaviza saltos cuando la línea se oculta.

Salidas: `/lane/center_offset` ∈ $[-1, 1]$, `/lane/confidence` ∈ $[0, 1]$, `/lane/stop_sign`.

### 5.2 LIDAR Processor (`lidar_processor.py`) ⭐

Suscribe a `/qcar/scan` (`sensor_msgs/LaserScan`) y produce un radar top-down + métricas frontales.

#### 📐 Matemática del LIDAR

**a) Ángulos por índice.** El `LaserScan` entrega un arreglo `ranges[i]` y dos escalares `angle_min`, `angle_increment`. Aplicamos inversión de eje (`flip_scan`) y calibración de orientación (`angle_offset`):

$$\theta_i = (-1)^{\text{flip}} \cdot (\theta_{\min} + i \cdot \Delta\theta) + \theta_{\text{offset}}, \qquad i = 0, 1, \dots, N-1$$

Normalizado siempre al rango $[-\pi,\, \pi]$.

**b) Filtrado de lecturas válidas.** Descartamos NaN, ∞ y lecturas fuera del rango físico del sensor:

$$\text{valid}_i = \text{isfinite}(r_i) \;\wedge\; r_{\min} < r_i < r_{\max}$$

**c) Cono frontal de seguridad (60°).** Definimos la máscara del cono frontal de semi-apertura $\phi = 30°$:

$$\text{front}_i = \text{valid}_i \;\wedge\; \left|\theta_i\right| < \frac{\phi_{\text{cono}}}{2}$$

**d) Distancia mínima y ángulo del obstáculo más cercano:**

$$i^* = \arg\min_{i \,\in\, \text{front}} r_i, \qquad d_{\min} = r_{i^*}, \qquad \theta^* = \theta_{i^*}$$

**e) Flag de obstáculo:**

$$\text{obstacle} = \mathbb{1}\!\left[\,d_{\min} < d_{\text{th}}\,\right], \qquad d_{\text{th}} = 0.6 \;\text{m}$$

**f) Conversión polar → cartesiana (top-down).** Para pintar cada punto en la imagen, con el frente del carro hacia arriba:

$$x_{\text{px}} = c_x + r_i \sin(\theta_i) \cdot s$$

$$y_{\text{px}} = c_y - r_i \cos(\theta_i) \cdot s$$

donde $s = \dfrac{0.45 \cdot S}{r_{\max}}$ es la **escala** (px/m) para un canvas cuadrado de tamaño $S$, y $(c_x, c_y) = (S/2,\; S/2)$ es el centro = posición del carro.

> El signo negativo en $y$ es porque el eje Y de la imagen **crece hacia abajo**; queremos que el frente del carro ($\theta = 0$) apunte **hacia arriba**.

**g) Coloreado por distancia (estilo RViz).** Cada punto se renderiza como un cuadrado con color basado en su distancia $d = r_i$:

| Rango | Color |
|-------|-------|
| $d < 0.3$ m | 🟥 Rojo |
| $0.3 \le d < 0.6$ m | 🟨 Amarillo |
| $0.6 \le d < 0.8$ m | 🟩 Verde |
| $d \ge 0.8$ m | 🟦 Azul/Cyan |

El fondo incluye una **grilla de 1 m × 1 m** y anillos circulares tenues.

**h) Zonas de seguridad** (umbrales del dashboard):

| Zona | Rango (m) | Color |
|------|-----------|-------|
| 🟥 DANGER | $d < 0.6$ | Rojo |
| 🟨 WARNING | $0.6 \le d < 1.5$ | Amarillo |
| 🟩 SAFE | $d \ge 1.5$ | Verde |

#### 📤 Salidas del nodo

| Topic | Tipo | Contenido |
|-------|------|-----------|
| `/lidar/image_debug` | `CompressedImage` | Render top-down JPEG (radar) |
| `/lidar/min_distance` | `Float32` | $d_{\min}$ en metros (-1 si no hay datos) |
| `/lidar/closest_angle` | `Float32` | $\theta^*$ en radianes |
| `/lidar/obstacle` | `Bool` | `True` si $d_{\min} < 0.6$ m |

#### ⚙️ Parámetros configurables
```bash
ros2 run roboracer_racing lidar_processor \
  --ros-args \
  -p scan_topic:=/qcar/scan \
  -p max_range:=5.0 \
  -p front_cone_deg:=60.0 \
  -p obstacle_thresh:=0.6 \
  -p angle_offset:=-1.5708 \
  -p flip_scan:=True
```

---

## 🖥️ 6. Dashboard v4 — Layout 2 Columnas

```
┌────────────────────┬──────────────────────────────────┐
│ IZQUIERDA (status) │ DERECHA (visual)                 │
├────────────────────┼──────────────────────────────────┤
│ System Status      │ 🎯 LIDAR RADAR        (hero)     │
│ ── ROS / Batería   │     /lidar/image_debug            │
│ ── Vel / FPS       │                                  │
│                    │                                  │
│ Lane Detector      │ CAMARA FRONTAL                   │
│ ── Modo/Off/Conf   │     /qcar/csi_front              │
│ ── STOP + barra    │                                  │
│                    │                                  │
│ LIDAR /qcar/scan   │ LANE DEBUG                       │
│ ── Estado/Min/Ang  │     /lane/image_debug            │
│ ── Obst + barra    │                                  │
│                    │                                  │
│ 📈 Telemetría      │                                  │
│ ── Velocidad       │                                  │
│ ── Offset          │                                  │
│ ── Confianza       │                                  │
│ ── LIDAR Min Dist  │                                  │
└────────────────────┴──────────────────────────────────┘
```

### Paneles definitivos

| # | Panel | Topic | Descripción |
|---|-------|-------|-------------|
| 1 | **System Status** | `/qcar/stateBattery`, `/qcar/velocity` | Heartbeat ROS, voltaje (V), velocidad (m/s), FPS |
| 2 | **Lane Detector** | `/lane/center_offset`, `/confidence`, `/stop_sign` | Modo (DETECTADO/PARCIAL/CIEGO), offset, conf, STOP, barra de offset |
| 3 | **LIDAR /qcar/scan** | `/lidar/min_distance`, `/closest_angle`, `/obstacle` | Estado (LIBRE/CERCA/OBSTÁCULO), dist mín, ángulo, barra de proximidad |
| 4 | **Telemetría** | velocidad, offset, conf, min-dist | 4 gráficas pyqtgraph estilo MATLAB, ventana 15 s, con líneas de umbral |
| 5 | **🎯 LIDAR RADAR** (hero) | `/lidar/image_debug` | Render top-down de la Jetson — el componente más importante |
| 6 | **Cámara Frontal** | `/qcar/csi_front` | Vista cruda CSI |
| 7 | **Lane Debug** | `/lane/image_debug` | BEV + polinomio |

### Arquitectura interna

- **Backend ROS**: clase `ROSBackend(Node)` corriendo en thread daemon con `rclpy.spin`. Todas las callbacks escriben a un estado compartido protegido por `threading.Lock`.
- **Frontend Qt**: `QMainWindow` con `QSplitter` horizontal (2 columnas redimensionables). Un `QTimer` a 33 ms (≈30 FPS) lee el estado y repinta paneles.
- **Imágenes** llegan como `CompressedImage` JPEG → `cv2.imdecode` → `QPixmap` escalado.
- **QoS**: `BEST_EFFORT` con `depth=2` en imágenes y scan → si la red se atrasa, se descarta lo viejo.

---

## 🛠️ 7. Resumen de Implementación Profesional

Hemos transformado el QCar en un vehículo de carreras autónomo con las siguientes capacidades de vanguardia:

1.  **Arquitectura de Control Pure Pursuit**: No es un seguimiento de línea básico; es un controlador geométrico profesional que proyecta un *lookahead* dinámico y calcula la curvatura ideal de giro.
2.  **Fusión Sensorial de Seguridad**: El sistema integra el LIDAR para frenado de emergencia y la confianza de visión para evitar comportamientos erráticos.
3.  **Dashboard de Misión v4.2**: Visualización en tiempo real del radar LIDAR (estilo RViz), cámara frontal, métricas de batería y estado del piloto (MANUAL vs. AUTÓNOMO).
4.  **Sistema de Disparo Remoto**: Activación segura mediante un canal dedicado para evitar interferencias de red.

---

## 🛠️ 8. Progreso y Retos Superados

**Completado (✅):**
- [x] **Pure Pursuit Lane Follower**: Arquitectura profesional de seguimiento de carril. ⭐
- [x] **Fusión Lane + LIDAR**: Seguridad activa ante obstáculos.
- [x] **Radar LIDAR RViz-Style**: Visualización técnica de alta fidelidad.
- [x] **Dashboard Telemetry**: Monitoreo de batería, velocidad y confianza.
- [x] **Control Maestro**: Activación autónoma segura desde Teleop.

---

## 🔄 9. Sincronización con el Repositorio

Para respaldar tus cambios del QCar físico en GitHub:
```bash
cd ~/Documents/Assesment-Auto
./sync_qcar.sh
```

---
> *RoboRacer Deployment System — QCar 2026 — Dashboard v4 (Radar Edition)*
