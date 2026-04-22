# 🏎️ QCar: Del Simulador al Físico (RoboRacer Deployment)

Bienvenido a la guía definitiva para correr el algoritmo `roboracer_racing` en el **hardware físico del QCar**. Este documento explica el flujo de trabajo profesional para la detección de carril y el uso del Dashboard de telemetría.

---

## 🌐 0. Configuración de Red (CRÍTICO)
Para que tu Laptop y el QCar se hablen, deben estar en la misma red y compartir estas variables de entorno.

**Ejecuta esto en TODAS tus terminales (tanto en Laptop como en Jetson):**
```bash
export ROS_DOMAIN_ID=115
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 🚀 Secuencia de Lanzamiento Profesional

### 🔴 Terminal 1: Encendido del Hardware (SSH Jetson)
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
# Rebuild si es la primera vez tras los cambios:
colcon build --packages-select roboracer_racing --symlink-install
ros2 run roboracer_racing lane_detector
```

### 🖥️ Terminal 3: Physical Dashboard v3 (Laptop Local)
*Visualización de 30 FPS, gráficas de velocidad y telemetría estilo MATLAB.*
```bash
cd ~/Documents/qcar_remoto/Assesment_qcar_irs
python3 src/roboracer_racing/roboracer_racing/physical_dashboard.py
```

---

## 🧠 Arquitectura de Percepción (Implementada)
El `lane_detector.py` ha sido actualizado a un nivel de ingeniería senior:
*   **Bird's-Eye View (BEV)**: Homografía para vista cenital libre de distorsión.
*   **Ajuste Polinomial**: Ajuste de parábola ($ax^2 + bx + c$) para curvas suaves.
*   **Segmentación Dual**: Fusión de HSV + HLS para ignorar reflejos y sombras.
*   **Filtro EMA**: Estabilidad temporal que evita saltos en el offset si la línea se oculta.

---

## 📊 Dashboard de Telemetría
El dashboard local permite monitorizar:
*   **Cámara Frontal**: Streaming JPEG optimizado a 320x240.
*   **Lane Debug**: Mapa de calor BEV + Polinomio detectado.
*   **Gráficas en Vivo**: Velocidad (m/s), Offset lateral y Confianza.
*   **Calibración HSV**: Sliders para ajustar el amarillo en tiempo real sin reiniciar nodos.

---

## 🛠️ Progreso del Proyecto Físico

**Completado (✅):**
- [x] Migración de tópicos de Simulación a Hardware Real (`/qcar/csi_front`).
- [x] Soporte para `CompressedImage` (JPEG) para evitar lag de red.
- [x] **Motor de Percepción Pro**: BEV + Polynomial Fit.
- [x] **Dashboard v3**: Gráficas de alta frecuencia con `PyQtGraph`.
- [x] Control por teclado y telemetría de batería/velocidad.

**Siguientes Tareas (🚀):**
1. **Odometría Matemática:** Integrar `/qcar/velocity` para estimar `x, y, yaw`.
2. **Navegación Autónoma:** Enlazar el offset del polinomio directamente al Pure Pursuit.

---
> *RoboRacer Deployment System — QCar 2026*
