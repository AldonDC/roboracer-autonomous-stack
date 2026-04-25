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

## 🚀 2. Secuencia de Lanzamiento Profesional

Para correr el sistema de carreras completo, abre 4 terminales siguiendo este orden exacto:

### 🔴 Terminal 1: Hardware y Radar (SSH Jetson)
*Levanta sensores, actuadores y el procesador de Radar LIDAR.*
```bash
cd ~/Assesment_qcar_irs
colcon build --packages-select roboracer_racing --symlink-install
source install/setup.bash
ros2 launch roboracer_racing physical_racing.launch.py
```

### 🔵 Terminal 2: Percepción de Carril (SSH Jetson)
*Detecta la línea amarilla con Visión Artificial.*
```bash
cd ~/Assesment_qcar_irs
source install/setup.bash
ros2 run roboracer_racing lane_detector
```

### 🟢 Terminal 3: Inteligencia Autónoma (SSH Jetson)
*Fusión Lane + LIDAR (Pure Pursuit architecture).*
```bash
cd ~/Assesment_qcar_irs
source install/setup.bash
ros2 run roboracer_racing autonomous_lane_follower
```

### 🟡 Terminal 4: Dashboard y Control (Laptop Local)
*Visualización en tiempo real y disparador de misión.*
```bash
# En Terminal A: Dashboard de Telemetría
cd ~/Documents/qcar_remoto/Assesment_qcar_irs
source install/setup.bash
ros2 run roboracer_racing physical_dashboard

# En Terminal B: Teleoperación y START
cd ~/Documents/qcar_remoto/Assesment_qcar_irs
source install/setup.bash
ros2 run roboracer_racing keyboard_teleop  # <--- Presiona 'M' para START
```

---

## 🛠️ Progreso y Retos Físicos

**Completado (✅):**
- [x] **Pure Pursuit Lane Follower**: Arquitectura profesional de seguimiento. ⭐
- [x] **Fusión Lane + LIDAR**: Seguridad activa ante obstáculos.
- [x] **Dashboard v4.2**: Monitor de telemetría y Radar (30 FPS).
- [x] **Motor de Percepción Pro**: BEV + Polynomial Fit.

Para una explicación técnica detallada sobre la matemática y los topics, consulta el:
👉 **[README_FISICO.md](file:///home/alfonsd/Documents/qcar_remoto/Assesment_qcar_irs/README_FISICO.md)**

---
> *Desplegado para Assesment_qcar_irs (RoboRacer Físico)*
