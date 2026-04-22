# 🏎️ QCar: Del Simulador al Físico (RoboRacer Deployment)

Bienvenido a la guía definitiva para correr el algoritmo `roboracer_racing` en el **hardware físico del QCar**. Este documento explica el flujo de trabajo, la configuración de red y las capas de ejecución.

---

## 🔌 0. Conexión de Red Remota (SSHFS)
Para editar los archivos físicos del QCar como si estuvieran en tu computadora (y sin lag visual), usamos un puente SSHFS. **Estos comandos se ejecutan SIEMPRE en tu Laptop (Terminal Local):**

**Conectar / Montar la carpeta del QCar:**
```bash
sshfs -o reconnect,ServerAliveInterval=15,ServerAliveCountMax=3 nvidia@192.168.2.12:/home/nvidia ~/Documents/qcar_remoto
```
*(Te pedirá la contraseña del usuario `nvidia`)*

**Si se traba la conexión (El WiFi parpadeó):**
Forza la limpieza antes de volver a conectarte:
```bash
fusermount -u ~/Documents/qcar_remoto
# ó si falla: sudo umount -l ~/Documents/qcar_remoto
```

---

## 🛑 1. Arquitectura del Sistema
Para que el QCar sea autónomo, el código físico se divide en "Capas" que usamos de forma simultánea.

1. **Capa Sensorial (El Hardware)**: Encendemos los ojos y la fuerza (*Cámara, LiDAR, Motores*).
2. **Capa Analítica (El Cerebro Visión)**: Extrae carriles de las imágenes en vivo (*Lane Detector*).
3. **Capa de Control (El Volante)**: Manda voltajes a los motores dependiendo de los algoritmos (*Teleop o Pure Pursuit*).
4. **Capa Visual (El Conductor)**: Transmite video y Lidar a la Laptop (*RViz / Foxglove*).

---

## 🚀 2. Secuencia de Lanzamiento
Ejecuta los siguientes comandos **exactamente en este orden**. Cada capa necesita una terminal (Pestaña) distinta.

### 🔴 Terminal 1: El Encendido del Hardware (SSH Jetson)
*No la cierres nunca. Es el puente entre el mundo físico y ROS 2.*
```bash
# Entrar al workspace físico
cd ~/Assesment_qcar_irs
# Cargar entorno ROS
source install/setup.bash
# Encender el Coche y Sensores
ros2 launch roboracer_racing physical_racing.launch.py
```

### 🔵 Terminal 2: El Analista de Visión (SSH Jetson)
*Activa el pipeline del Lane Detector que analiza los píxeles de la cámara frontal.*
```bash
# Entrar al workspace
cd ~/Assesment_qcar_irs
source install/setup.bash
# Analizar carril físico (línea amarilla/blanca)
ros2 run roboracer_racing lane_detector
```

### 🟢 Terminal 3: Visualización Remota Integral (Foxglove)
*Evitamos problemas de compatibilidad ROS Jazzy vs Dashing usando WebSockets.*
```bash
# En el coche (Terminal SSH extra):
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
Luego **en tu laptop**:
1. Entra a `studio.foxglove.dev`
2. Conexión Rosbridge a `ws://192.168.2.12:9090`
3. Monitorea cámaras, lidar y comandos sin lag.

> *Alternativa rápida para ver sólo cámaras en tu Laptop:* `ros2 run rqt_image_view rqt_image_view`

### 🎮 Terminal 4: Teleoperación Activa (SSH Jetson)
*Mueve el QCar físicamente por la pista.*
```bash
cd ~/Assesment_qcar_irs
source install/setup.bash
ros2 run roboracer_racing keyboard_teleop
```
> **Controles:** `W` (Acelerar), `S` (Reversa), `A`/`D` (Volante), `Espacio` (Freno de Emergencia).

---

## 🛠️ Progreso y Próximos Retos Físicos

**Completado (✅):**
- [x] Migración del Launch de Simulación al Hardware Quanser (Dashing compatible).
- [x] Corrección de Hardware (HIL Reset y Quanser Daemon).
- [x] Traductor de Teclado a `/qcar/user_command`.
- [x] **Motor de Percepción Pro**: BEV + Polynomial Fit para línea central.
- [x] **Dashboard v3**: Monitor de telemetría y gráficas de velocidad (30 FPS).
- [x] Optimización de red vía `CompressedImage` (JPEG).

**Siguientes Tareas Pendientes (🚀):**
1. **Odometría Matemática:** Escribir un script que convierta velocidades de engrane del topic `/qcar/velocity` en posiciones `x, y, yaw` para simular el `/odom`.
2. **SLAM Lidar:** Activar un mapa 2D usando el sensor `lidar_qos` para precisión en la pista.
3. **Migración Pure Pursuit:** Enlazar el control robótico autónomo a la odometría para competir.

---
> *Desplegado para Assesment_qcar_irs (RoboRacer Físico)*
