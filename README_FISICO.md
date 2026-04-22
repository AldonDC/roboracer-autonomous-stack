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
```bash
fusermount -u ~/Documents/qcar_remoto
```

---

## 🌐 1. Configuración de Red (CRÍTICO)
Para que tu Laptop y el QCar se hablen, deben compartir estas variables de entorno en **TODAS** las terminales:

```bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 🚀 2. Secuencia de Lanzamiento Profesional

### 🔴 Terminal 1: El Encendido del Hardware (SSH Jetson)
```bash
cd ~/Assesment_qcar_irs
source install/setup.bash
ros2 launch roboracer_racing physical_racing.launch.py
```

### 🔵 Terminal 2: Motor de Percepción PRO (SSH Jetson)
*Analiza la línea central usando Bird's-Eye View y ajuste polinomial.*
```bash
cd ~/Assesment_qcar_irs
source install/setup.bash
# Rebuild si es necesario:
colcon build --packages-select roboracer_racing --symlink-install
ros2 run roboracer_racing lane_detector
```

### 🖥️ Terminal 3: Physical Dashboard v3 (Laptop Local)
*Visualización de 30 FPS y gráficas en vivo.*
```bash
cd ~/Documents/Assesment-Auto
python3 src/racing_logic/roboracer_racing/roboracer_racing/physical_dashboard.py
```

---

## 🧠 3. Arquitectura de Percepción (Implementada)
El `lane_detector.py` ha sido actualizado a un nivel de ingeniería senior:
*   **Bird's-Eye View (BEV)**: Homografía para vista cenital libre de distorsión.
*   **Ajuste Polinomial**: Ajuste de parábola ($ax^2 + bx + c$) para curvas suaves.
*   **Segmentación Dual**: Fusión de HSV + HLS para ignorar reflejos y sombras.
*   **Filtro EMA**: Estabilidad temporal que evita saltos en el offset si la línea se oculta.

---

## 🛠️ 4. Progreso y Próximos Retos Físicos

**Completado (✅):**
- [x] Migración del Launch de Simulación al Hardware Quanser.
- [x] **Motor de Percepción Pro**: BEV + Polynomial Fit para línea central.
- [x] **Dashboard v3**: Monitor de telemetría y gráficas de velocidad (30 FPS).
- [x] Optimización de red vía `CompressedImage` (JPEG).
- [x] Control por teclado y telemetría de batería/velocidad.

**Siguientes Tareas Pendientes (🚀):**
1. **Odometría Matemática:** Integrar `/qcar/velocity` para estimar `x, y, yaw`.
2. **Navegación Autónoma:** Enlazar el offset del polinomio al Pure Pursuit.

---

## 🔄 5. Sincronización con el Repositorio (Assesment-Auto)

Para respaldar tus cambios del QCar físico en GitHub, usa el script automático en tu Laptop:
```bash
cd ~/Documents/Assesment-Auto
./sync_qcar.sh
```

---
> *RoboRacer Deployment System — QCar 2026*
