# 🏁 RoboRacer — Documentación Técnica y Plan de Trabajo

> **Alfonso D. — Tecnológico de Monterrey**
> Materia: Assesment
> Asesores: Dr. Daniel Sosa-Ceron · Dr. Jorge A. Reyes-Avendaño
> Versión Actual: **v13.0 — Vision-Fused Intelligent Racing**
> Última actualización: Abril 2026

[**Ver Video Demostrativo del QCar (Navegación Pure Pursuit)**](https://github.com/AldonDC/roboracer-autonomous-stack/raw/main/docs/assets/demo_rviz.webm)

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

### 3.2. Perfiles de Velocidad Inteligentes (v12)
El controlador implementa un esquema de **Velocidad de Curvatura Crítica** para asegurar la estabilidad lateral en giros cerrados:

1. **Límite por Curvatura**: La velocidad se reduce inversamente al ángulo de dirección $\delta$.

$$V(\delta) = \frac{V_{ref}}{1 + k|\delta|}$$

Donde $k = 1.25$ es el coeficiente de agresividad de frenado.

2. **Perfil de Arribo (S-Curve)**: Desaceleración suave basada en la raíz cuadrada de la distancia al objetivo, evitando el comportamiento oscilatorio cerca del waypoint.

$$V_{dist} = V_{ref} \cdot \sqrt{\max\left(0.2, \frac{d}{d_{range}}\right)}$$

3. **Filtro de Jerk**: El perfil de aceleración está limitado a $0.08\,m/s^2$ para proteger los actuadores mecánicos y evitar el deslizamiento de neumáticos.

---

## 🛰️ 4. Evasión de Obstáculos: Artificial Potential Fields (APF)

Para la navegación reactiva, el QCar utiliza un esquema de **Campos de Potencial**, donde el entorno se modela como un terreno de energías.

### 4.1. Formalismo Matemático
El potencial total $U(\mathbf{q})$ es la suma de una superficie atractiva y una repulsiva. La fuerza resultante es el gradiente negativo de dicho potencial:

$$\mathbf{F}_{net} = -\nabla U_{att}(\mathbf{q}) - \nabla U_{rep}(\mathbf{q})$$

Calculamos la fuerza emitida por cada cluster de obstáculos detectados por el láser:

$$\mathbf{F}_{rep} = \begin{cases} \eta \left( \frac{1}{\rho(\mathbf{q})} - \frac{1}{\rho_0} \right) \frac{1}{\rho^2(\mathbf{q})} \frac{\mathbf{q} - \mathbf{q}_{obs}}{\rho(\mathbf{q})} & \text{si } \rho(\mathbf{q}) \leq \rho_0 \\ 0 & \text{si } \rho(\mathbf{q}) > \rho_0 \end{cases}$$

Donde $\rho_0 = 1.1m$ es el radio de influencia y $\eta$ es el factor de escala de evasión.

### 4.2. Evasión Dinámica (2D Nav Goal)
Desde RViz, el operador puede colocar **objetos físicos reales** en Gazebo. El sistema intercepta el clic de navegación, envía un comando al simulador para `spawnear` un cubo rojo, y el carro lo detecta inmediatamente tanto por LiDAR como por Cámara.

---

## 👁️ 5. Visión Artificial Activa — Lane Detection (v13)

A partir de v13 el carro no solo "ve" las cámaras — las **interpreta**. El nodo `lane_detector.py` procesa la cámara `csi_front` con OpenCV para detectar las líneas de carril y fusiona el resultado con el APF y Pure Pursuit para formar un **control de tres capas**.

### 5.1. Semántica de la Pista

La pista RoboRacer sigue la convención vial estándar:
* **Línea amarilla** = divisoria central de la carretera (borde izquierdo del carril propio).
* **Borde blanco/gris** = borde derecho del carril (curb exterior).

El objetivo del detector es estimar el **centro del carril** $c_{lane}$ a partir de ambos bordes.

### 5.2. Pipeline de Visión (OpenCV)

El algoritmo ejecuta un pipeline clásico de 5 etapas a 30 Hz.

**Etapa 1 — Conversión de espacio de color.** Se convierte la imagen de $\text{BGR} \rightarrow \text{HSV}$ para aislar color independientemente de iluminación.

**Etapa 2 — Doble máscara HSV.** Se generan dos máscaras binarias en paralelo, una por color objetivo:

$$M_{yellow}(p) = 1 \cdot [ H(p) \in [18, 38] \land S(p) \in [80, 255] \land V(p) \in [80, 255] ]$$

$$M_{white}(p) = 1 \cdot [ S(p) \leq 80 \land V(p) \geq 140 ]$$

**Etapa 3 — Apertura + Cierre morfológico.** Elimina ruido de sal/pimienta y conecta segmentos fragmentados con un kernel rectangular de $5 \times 5$.

**Etapa 4 — Region of Interest (ROI) trapezoidal.** Se aplica una máscara poligonal que conserva solo la mitad inferior de la imagen, descartando cielo y horizonte:

$$\text{ROI} = \{ (0.02w, h), (0.20w, 0.55h), (0.80w, 0.55h), (0.98w, h) \}$$

**Etapa 5 — Detección de líneas.** Canny + `HoughLinesP` extrae segmentos; luego se ajusta una recta $x = my + b$ por mínimos cuadrados en cada lado.

### 5.3. Modelo de Centro de Carril (Fusión)

Sea $x_Y$ el $x$ del ajuste de la línea amarilla en la base de la imagen, $x_W$ el de la blanca, y $w_{lane}$ el ancho medio de carril asumido (28% del ancho de imagen). El centro de carril se calcula según las detecciones disponibles:

$$
c_{lane} = \begin{cases} 
\frac{x_Y + x_W}{2} & \text{si ambas líneas detectadas} \\ 
x_Y + w_{lane} & \text{solo amarillo} \\ 
x_W - w_{lane} & \text{solo blanco} \\ 
\text{ninguna} & \text{sin estimación} 
\end{cases}
$$

con niveles de confianza asociados:

| Caso | Detecciones | $\text{conf}$ |
|:-----|:------------|:--------------|
| Completo | Amarillo + Blanco | $1.00$ |
| Parcial A | Solo amarillo | $0.70$ |
| Parcial B | Solo blanco | $0.55$ |
| Blind | Ninguna | $0.00$ |

El **offset normalizado** publicado en `/lane/center_offset` es:

$$o = \text{clip}\left( \frac{c_{lane} - w/2}{w/2},\; -1,\; +1 \right)$$

Con suavizado exponencial $o_t = 0.7\, o_{t-1} + 0.3\, \tilde{o}_t$ para eliminar jitter.

### 5.4. Fusión en el Controlador (Lane-Assist)

El `multi_goal_navigator` suscribe `/lane/center_offset` y `/lane/confidence` e inyecta un **término de corrección lateral** en el steering, pero **solo cuando el APF no está en modo emergencia** (para que LiDAR siempre mande):

$$\delta_{final} = \delta_{PP} + \delta_{APF} + \delta_{lane}$$

donde el término de visión se activa condicionalmente:

$$\delta_{lane} = \begin{cases} k_{lane} \cdot o \cdot \text{conf} & \text{si } \text{conf} > 0.25 \\ 0 & \text{otros casos} \end{cases}$$

Con $k_{lane} = 0.18$ como ganancia de lane-assist (intencionalmente baja para afinar, no comandar).

### 5.5. Calibración de Cámaras (v13)

Durante la integración se detectaron y corrigieron varios problemas en el URDF de sensores:

| Cámara | Problema original | Corrección v13 |
|--------|-------------------|---------------|
| `csi_front` | FOV 160° (fisheye extremo) | **FOV 70° (1.22 rad)** + pitch 8.6° hacia abajo |
| `csi_right` | FOV 160°, `y=-0.068` asimétrico | **FOV 80°**, `y=-0.060` |
| `csi_left`  | FOV 160°, `y=+0.052`, nombre "front" | **FOV 80°**, `y=+0.060`, nombre "left" |
| `csi_back`  | FOV 160°, nombre "front" | **FOV 86°**, nombre "back" |
| `RGB` (RealSense) | `y=0.0315` off-center | `y=0.0` centrada |

Resultado: **líneas rectas en la imagen se ven rectas** (sin distorsión de barril), amarillo y blanco conservan sus gradientes reales en HSV, y las side-cams tienen una posición simétrica como espejo.

---

## ⚙️ 6. Nodos de Software (`roboracer_racing`)

El *core* de nuestra lógica customizada habita en el paquete `roboracer_racing` y se divide en módulos modulares y de responsabilidad única:

| Nodo (Python) | Descripción Técnica y Funcionalidad Destacada |
|:--------------|:----------------------------------------------|
| `multi_goal_navigator.py` | **Planner & Controller (v13).** <br>✅ **Gazebo Spawner**: Interfaz directa para inyectar modelos 3D físicos en el simulador vía RViz.<br>✅ **Goal-Biased Gap Following**: Selección inteligente de huecos de escape alineada con la meta.<br>✅ **Unified APF Control**: Fusión de Pure Pursuit con gradientes de repulsión a 50Hz.<br>✅ **Lane-Assist Fusion (v13)**: Suma un término de corrección lateral desde `lane_detector`, ponderado por confianza, sin comprometer la prioridad de LiDAR/APF en emergencia. |
| `lane_detector.py` | **Visión Artificial Activa (v13, NUEVO).** <br>✅ **Dual HSV Masking**: Segmentación paralela amarillo (divisoria central) + blanco (curb derecho).<br>✅ **Hough Line Fit**: Ajuste lineal robusto $x = my + b$ por mínimos cuadrados en cada lado.<br>✅ **Fallback por ancho fijo**: Estima centro de carril aun con una sola línea detectada.<br>✅ **Publicación dual**: `/lane/center_offset` (Float32) + `/lane/confidence` (Float32) + `/lane/image_debug` (Image anotada con overlay). |
| `telemetry_dashboard.py` | **Engineering Station (v12).** <br>✅ **Blueprint Aesthetic**: Interfaz de alta densidad con paleta de colores profesional blanca/azul.<br>✅ **Perception Fusion**: Overlay de LiDAR sobre las cámaras y detección de objetos clusterizada.<br>✅ **Engineering Metrics**: Gráficas de aceleración Gx, fuerza APF e histéresis de dirección.<br>✅ **Lane HUD (v13)**: Badge centrado sobre la cámara frontal con estado `FULL / YELLOW-ONLY / WHITE-ONLY / BLIND` + barra horizontal de offset en vivo. |
| `odom_tf_broadcaster.py` | **Puente Espacial.** Lee `/qcar_sim/odom` originado por Gazebo y expone el *Transform Tree* (`world` → `base_link`) para que RViz acople el modelo 3D del carro perfectamente con la física real. |
| `track_visualizer.py` | **Pintor 3D de RViz.** Extrapola los vértices del `.obj` de la pista de Gazebo y transmite un `visualization_msgs/Marker` gigante a RViz, permitiendo ver la pista real como plantilla y referencia topológica. |
| `keyboard_teleop.py` | **Conducción Manual.** Script WASD de precisión para validación de hardware y trazado empírico inicial. |

---

## 📂 7. Estructura del Proyecto

A continuación se muestra cómo está organizado el código dentro del repositorio. Notarás la estricta separación de responsabilidades: *Simulators, Algorithms, and Support*.

```text
roboracer-autonomous-stack/
├── docs/                             # Documentación Técnica Formal
├── scripts/
│   ├── build.sh                      # Helper builder para ROS 2
│   └── launch_sim.sh                 # Máster Node Launcher para Gazebo
├── src/
│   ├── racing_logic/                 # 🧠 AQUÍ VIVE LA INTELIGENCIA ARTIFICIAL (ALGORITMOS)
│   │   └── roboracer_racing/         # Nuestro paquete principal de Python
│   │       ├── routes/               # Archivos JSON autogenerados de "perfect laps"
│   │       ├── multi_goal_navigator.py # Loop principal (CLI + Pure Pursuit + APF + Lane-Assist)
│   │       ├── lane_detector.py        # 👁️  Visión Artificial — detección amarillo/blanco (v13)
│   │       ├── telemetry_dashboard.py  # GUI de Instrumentación Analítica MATLAB
│   │       ├── odom_tf_broadcaster.py  # Sistema de Referencia Geométrico
│   │       └── track_visualizer.py     # RViz Mesh Exporter
│   ├── roboracer/                    # 🚗 HARDWARE / ROBOT / MUNDOS
│   │   ├── roboracer_description/    # Specs físicas del QCar 3D, Sensores, RViz config
│   │   ├── roboracer_gazebo/         # Escenarios de Gazebo Harmonic y Puentes físicos
│   │   └── roboracer_interfaces/     # Serialización de Mensajes dedicados de ROS 2
│   └── support/                      # Librerías heredadas (cámaras, modelos del profe)
└── README.md                         # Documentación Root
```

---

## 🚀 8. Manual Rápido (Comandos de Uso)

### Paso 1: Lanzar la Simulación (Terminal 1)
```bash
cd ~/Documents/Assesment-Auto
./scripts/launch_sim.sh
```
*Esto arranca Gazebo con el QCar, la pista, inicializa los puentes ROS y abre RViz listo.*

### Paso 2: Preparar Control, Telemetría y CLI (Terminal 2)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/Documents/Assesment-Auto
colcon build --symlink-install
source install/setup.bash

# Lanzamos el sincronizador de RViz, pista, telemetría y visión (corriendo de fondo)
ros2 run roboracer_racing odom_tf &
ros2 run roboracer_racing track_viz &
ros2 run roboracer_racing telemetry &
ros2 run roboracer_racing lane_detector &

# Lanzamos el Controlador Multipunto CLI
ros2 run roboracer_racing multi_goal
```

### (Opcional) Terminal 3 — Visualización del Lane Detector
```bash
ros2 run rqt_image_view rqt_image_view /lane/image_debug
# Verás la cámara frontal con overlay amarillo/blanco, líneas fit,
# centro de carril estimado y cuadro HUD de estado.
```

### Paso 3: Interactuar
1. En RViz (que se abrió solo en el Paso 1), usa la herramienta **Publish Point** (arriba a la derecha).
2. Da **todos los clicks que quieras** a lo largo de la pista. (Verás aparecer esferas numeradas).
3. Ve a la Terminal 2 y escribe **`g`**, luego `Enter` para que el carro arranque.
4. **Obstáculos**: Selecciona **2D Nav Goal** en RViz y da click en la pista para que aparezca un cubo físico en Gazebo frente al carro.
5. Escribe **`s`** para guardar la ruta, o **`c`** para limpiar (esto borrará también los cubos de Gazebo).

---

## 📊 9. Plan de Trabajo Restante

**✅ Fase 1: Simulación y RViz Completa.**
**✅ Fase 2: Control Time-Trial Básico.**
**✅ Fase 3: Evasión Inteligente (Autonomous Racing).**
   * Campos de fuerza (APF) y frenado curvo adaptativo.
   * Interacción física Gazebo ↔ RViz (Spawning dinámico).
   * Telemetría de alta densidad (Gx, Clearance, AI Vision).

**✅ Fase 3.5: Visión Artificial Activa (v13).**
   * Detector OpenCV dual-color (amarillo/blanco) con ROI trapezoidal y Hough fit.
   * Modelo de centro de carril con fallback por ancho fijo.
   * Fusión `k_lane · o · conf` en el controlador, subordinada al APF.
   * Recalibración completa de cámaras: FOV, pitch y simetría lateral.
   * Dashboard con HUD de confianza en vivo.

**⏳ Fase 4: Head-to-Head (Siguiente paso lógico)**
   * Nodo `ghost_car.py` con modos `static` y `route` (seguimiento de waypoints).
   * Detección adversaria vía `/ghost/odom` → obstáculo dinámico en el APF.
   * Overtake logic agresivo-seguro: usar `sector_clearance` para elegir carril de rebase.
   * Nuevo estado visual `OVERTAKE` en el dashboard.

**⏳ Fase 5: Launch Unificado**
   * `competition.launch.py` que arranca odom_tf + track_viz + telemetry + lane_detector + multi_goal + ghost (opcional) de un solo comando.

---
*"El que no arriesga, no gana la carrera."* 🏁
