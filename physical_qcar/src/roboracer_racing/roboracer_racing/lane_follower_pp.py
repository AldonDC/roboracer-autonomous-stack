"""
Lane Follower Pure Pursuit — QCar Physical
==========================================
Sigue el carril usando el target point en metros publicado por lane_detector
(/lane_target_point_m). Aplica Pure Pursuit clasico sobre frame eje trasero:
    delta = atan2(2 * L * x_target, lookahead^2)

Soporta 'qcar' (Vector3Stamped → /qcar/user_command) y 'qcar2' (MotorCommands).
Selecciona velocidad en recta vs curva segun el angulo de steering resultante.
"""

import json
import math
import sys
import time
import threading
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from std_msgs.msg import Float32MultiArray, Float32, Bool
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu


class LaneFollowerPP(Node):
    def __init__(self):
        super().__init__('lane_follower_pp')

        # Plataforma: 'qcar' (este repo, hardware fisico) o 'qcar2'
        self.declare_parameter('platform', 'qcar')

        # FASE 2: default cambiado a /lane_target_point_m (lane_detector).
        # Esto es coherente con la fusion + curvatura, que vienen del mismo
        # pipeline de vision. ATENCION: los modos del launcher que usen
        # yellow_line_tracker deben actualizarse para arrancar lane_detector
        # en su lugar — el follower ya no escucha /yellow_line/target_point_m
        # por default. Si necesitas el comportamiento viejo, edita este
        # default a '/yellow_line/target_point_m'.
        self.declare_parameter('target_point_topic', '/yellow_line/target_point_m')
        self.declare_parameter('cmd_topic', '/qcar/user_command')

        # Geometria y limites
        self.declare_parameter('wheelbase', 0.256)
        self.declare_parameter('max_steering_angle', 0.30)  # rango util QCar

        # Lookahead dinamico
        self.declare_parameter('lookahead_min', 0.30)
        self.declare_parameter('lookahead_max', 0.80)
        self.declare_parameter('lookahead_base', 0.35)
        self.declare_parameter('lookahead_speed_gain', 0.80)

        # Perfil de velocidad — MUY BAJO para que el controlador tenga
        # margen de procesamiento y el carro reaccione a tiempo en curvas.
        # Subir manualmente con `ros2 param set` cuando este bien tuneado.
        self.declare_parameter('curve_threshold', 0.20)
        self.declare_parameter('speed_straight', 0.08)   # restaurado a 0.08
        self.declare_parameter('speed_curve',    0.06)   # aumentado a 0.06 para curvas más veloces

        # Signo del steering segun montaje del servo (-1 o +1)
        self.declare_parameter('steering_sign', -1.0)

        # Ganancia del steering: amplifica el comando de Pure Pursuit.
        # Util cuando el servo tiene zona muerta o el target_x es muy pequeño.
        # 1.0 = sin amplificar (default Pure Pursuit clasico)
        # 2.0-3.0 = mas reactivo (recomendado si carro casi no gira)
        self.declare_parameter('steering_gain', 1.0)

        # ── Safety LiDAR ─────────────────────────────────────────────────────
        # Defaults menos paranoicos: solo frenar fuerte si esta MUY cerca.
        # En pista interior con paredes a ~1m los anteriores frenaban siempre.
        self.declare_parameter('use_lidar_safety', True)
        # Stop a 45 cm: deja margen para frenado real (caja u otro QCar).
        # Mejora 1: zona intermedia activa para frenar suave en vez de en seco.
        # Entre slow_distance y stop_distance se aplica slow_factor a la speed.
        # Defaults anteriores (stop binario): slow_distance=0.50, slow_factor=1.0
        self.declare_parameter('lidar_stop_distance', 0.35)   # antes 0.45 — stop a 35 cm
        self.declare_parameter('lidar_slow_distance', 0.80)   # antes 0.50
        self.declare_parameter('lidar_slow_factor',   0.5)    # antes 1.0
        # Zona de slow LATERAL separada: mucho mas corta que la frontal para
        # que muros laterales (paredes de pista a ~0.7 m) NO disparen slow falso.
        # Solo afecta velocidad si un lateral ve algo MUY cerca (objeto real).
        self.declare_parameter('lidar_slow_distance_side', 0.45)
        # STOP TRASERO: si un objeto en el cono trasero (60° por default,
        # configurado en lidar_processor) esta a menos de back_stop_distance,
        # frenar. Util como proteccion contra colision por reversa o si
        # alguien se acerca por detras durante el rebase.
        self.declare_parameter('lidar_back_stop_distance', 0.45)
        # ── CIRCULOS DE SEGURIDAD (omnidireccionales en capas) ──────────────
        # Dos burbujas concentricas 360°. NO respetan is_wall (son absolutos).
        #  - lidar_safety_circle_slow_radius (exterior, 35 cm): SLOW — reduce
        #    velocidad al detectar objeto cercano. Aviso temprano.
        #  - lidar_safety_circle_radius (interior, 30 cm): STOP duro. Es la
        #    red de seguridad final, validada AL ULTIMO en _lidar_safety_factor.
        # Para apagar cualquiera: poner 0.0.
        self.declare_parameter('lidar_safety_circle_radius',      0.30)
        self.declare_parameter('lidar_safety_circle_slow_radius', 0.35)
        self.declare_parameter('lidar_timeout_s',     0.5)

        # ── Evasion lateral (retroceso + rebase + settle) ────────────────────
        # Maquina de estados:
        #   NORMAL    → sigue linea
        #   REVERSING → retrocede con giro que aleja el frente del obstaculo
        #               (mismo lado del obstaculo). Sale cuando el lado
        #               afectado supera (evade_clear_distance + extra) sostenido.
        #   STEER_AWAY → avanza giro al lado OPUESTO al obstaculo durante
        #                evade_steer_away_time_s. Durante este estado, la
        #                deteccion lateral se IGNORA (modo rebase) — solo el
        #                STOP frontal queda activo. Asi el obstaculo puede
        #                pasar al lado sin re-disparar la evasion.
        #   SETTLING  → quieto evade_settle_time_s y vuelve a NORMAL.
        self.declare_parameter('evade_enable',                  True)
        self.declare_parameter('evade_reverse_speed',           0.05)
        self.declare_parameter('evade_reverse_steering',        0.20)
        self.declare_parameter('evade_clear_distance',          0.65)
        # Margen extra DESPUES del clear, para retroceder un poco mas y darse
        # espacio antes de avanzar.
        self.declare_parameter('evade_reverse_extra_distance',  0.10)
        # Rebase: avanza con giro al lado opuesto.
        # STEER_AWAY se divide en 3 sub-fases:
        #   Fase 1 (rodea): gira al lado OPUESTO al obstaculo. Duracion:
        #                   evade_steer_away_time_s * phase1_ratio
        #   Fase 2 (corrige): gira al lado DEL obstaculo para volver al carril.
        #                     Duracion: time_s * (1 - phase1_ratio - straighten_ratio)
        #                     Con steering reducido por phase2_steer_factor.
        #   Fase 3 (endereza): los ultimos straighten_ratio del tiempo avanza
        #                      recto (steer=0) para no quedar torcido.
        self.declare_parameter('evade_steer_away_time_s',       2.0)
        self.declare_parameter('evade_steer_away_speed',        0.09)
        self.declare_parameter('evade_steer_away_steering',     0.20)
        # ── Ignorar pared envolvente (vallas) ──────────────────────────────
        # Si /lidar/is_wall == True, ignorar preempt y evasion: el lidar esta
        # viendo la pared/vallas alrededor, no un obstaculo real.
        self.declare_parameter('ignore_walls', True)

        # ── Esquiva PREEMPTIVA (zona lejana, antes de entrar a evasion) ──────
        # Si el lidar ve un obstaculo entre stop_d y preempt_trigger_distance
        # (todavia lejos), se aplica un sesgo lateral al target_x del Pure
        # Pursuit para desplazar el carro suavemente sin frenar ni cambiar
        # de estado. Solo actua si evade_state == 'NORMAL'.
        # Si el objeto se acerca a < stop_d, la maquina de estados existente
        # toma control (REVERSING/SWERVE).
        # Preempt ACTIVADO por default (antes False). Razon: --ros-args -p
        # NO sobreescribe declare_parameter en ROS 2 Dashing, asi que la
        # unica forma de tenerlo encendido al arrancar es como default.
        # Para apagarlo en runtime: ros2 param set /lane_follower_pp preempt_enable False
        self.declare_parameter('preempt_enable',              True)
        self.declare_parameter('preempt_trigger_distance',    1.30)   # m — antes 1.20
        self.declare_parameter('preempt_bias_max_m',          0.10)   # m
        # Si True, invierte el signo del bias (compatible con steering_sign=-1).
        # En el QCar fisico el preempt se desviaba HACIA el obstaculo: arreglado
        # invirtiendo aqui. Si tu setup tiene steering_sign=+1, pon False.
        self.declare_parameter('preempt_invert_sign',         False)
        # Sectores a vigilar (front + ambos laterales por default)
        self.declare_parameter('preempt_use_front',           True)
        self.declare_parameter('preempt_use_sides',           True)
        # Cooldown despues de una evasion: bloquear el preempt N segundos
        # para que el carro retome la linea sin re-desviarse por el mismo
        # obstaculo que acaba de rebasar.
        self.declare_parameter('preempt_cooldown_after_evade_s', 2.5)

        # ── Fusion con CAMARA DE PROFUNDIDAD (depth_processor / "ZED") ───────
        # El depth_processor publica un obstaculo frontal visto por la camara
        # RealSense (apodada ZED en la UI): /depth/obstacle_detected (Bool),
        # /depth/min_distance_center_10deg (m) y /depth/closest_angle (rad).
        # Cuando depth_preempt_enable=True, esa deteccion se inyecta como una
        # fuente FRONTAL adicional dentro de _preempt_bias, exactamente como el
        # sector frontal del lidar: el lado de evasion se decide por el carril
        # mas despejado (lidar_left vs lidar_right), o por el signo del angulo
        # del obstaculo si el lidar lateral no esta disponible.
        # Util cuando el obstaculo es bajo / de poco perfil para el lidar
        # (cajas, conos) pero claramente visible en profundidad.
        self.declare_parameter('depth_preempt_enable',        True)
        # Solo se considera el obstaculo de la camara si su distancia cae en la
        # zona preemptiva [stop_d, depth_trigger_distance]. Default 2.5 m: la
        # VISION (YOLO) ve mucho mas lejos que la profundidad, asi que la evasion
        # arranca DESDE ANTES. Subido aqui (no via -p) porque en ROS Dashing el
        # --ros-args -p no siempre sobreescribe el default de declare_parameter.
        # Solo afecta cuando el detector de vision esta publicando (modo C).
        self.declare_parameter('depth_trigger_distance',      2.50)   # m
        # Timeout de los datos de la camara (mas lento que el lidar: ~15 Hz).
        self.declare_parameter('depth_timeout_s',             0.7)
        # Flag para invertir el angulo de la camara de profundidad si se detecta al reves
        self.declare_parameter('depth_invert_angle',          False)
        # ── Gate de consistencia ZED↔LiDAR (ground-truth ESTRICTO) ───────────
        # El LiDAR es la verdad de terreno. La deteccion de la ZED solo activa
        # la evasion si el LiDAR la confirma: MISMO lado y distancia parecida
        # (±depth_lidar_dist_tol, compensando la separacion fisica camara-lidar).
        # Si el LiDAR no ve nada en ese lado -> la ZED NO dispara (estricto).
        self.declare_parameter('depth_require_lidar_confirm',  True)
        self.declare_parameter('depth_lidar_dist_tol',         0.30)  # m
        self.declare_parameter('depth_lidar_forward_offset_m', 0.0)   # m (camara adelante del lidar)
        self.declare_parameter('depth_lidar_confirm_frames',   2)     # debounce (frames seguidos)
        self.declare_parameter('depth_lidar_sector_check',     True)  # exigir mismo lado
        # Tiempo (en segundos) que se mantiene el sesgo preemptivo despues de
        # perder el obstaculo de vista, para asegurar rebase completo.
        self.declare_parameter('preempt_hold_time_s',         1.5)

        # ── Estricto en CURVAS (fix vallas externas) ────────────────────────
        # En curvas, la camara depth ve la valla exterior dentro de su ROI
        # (HFOV ancho). Para evitar evadir "fantasmas":
        # 1) Detectamos curva por |kappa_smoothed| del lane_detector o por
        #    el ultimo steering >= curve_threshold.
        # 2) En curva, exigimos confirmacion CRUZADA: solo evadir si LIDAR
        #    frontal Y depth ven obstaculo en zona preemptiva.
        # 3) En recta, comportamiento sin cambio (cualquier sensor dispara).
        # Default True; flag para apagarlo en runtime si genera problemas.
        self.declare_parameter('curve_strict_fusion_enable',  True)
        # Umbral de curvatura (1/m) sobre kappa_smoothed para considerar curva.
        # Para pista 4x4 con radios ~1.0 m -> kappa ~1.0. Threshold de 0.30
        # captura curvas evidentes sin disparar en rectas con polyfit ruidoso.
        self.declare_parameter('curve_strict_kappa_thresh',   0.30)
        # Distancia LIDAR maxima (en zona preemptiva) para considerar que
        # el LIDAR "confirma" el obstaculo en curva. Igual al preempt_trigger.
        # Si lidar_front >= esto, NO confirma -> en curva no evadimos.
        self.declare_parameter('curve_strict_lidar_max',      1.50)   # m

        # Early exit del swerve: si durante fase >= min_phase se ve la linea
        # amarilla N frames consecutivos Y el frente esta despejado, saltar
        # directo a NORMAL (sin SETTLING ni SEARCHING). Util cuando el carro
        # vuelve a alinearse con la linea antes de terminar la S.
        self.declare_parameter('evade_early_exit_enable',       True)
        # Solo permite abortar en fase 3 (calibrado). En fase 2 detectaba la
        # linea prematuramente y reentraba al follower antes de tiempo.
        self.declare_parameter('evade_early_exit_min_phase',    3)      # 1, 2 o 3
        self.declare_parameter('evade_early_exit_frames',       3)

        self.declare_parameter('evade_swerve_phase1_ratio',     0.40)   # rodeo (calibrado)
        # Factor de fase 2. Signo + = gira hacia el carril (corrige); signo -
        # = sigue alejandose del obstaculo (mantiene desvio). Calibrado: -0.17
        # para evitar que la S se cierre y el carro vuelva a chocar.
        self.declare_parameter('evade_swerve_phase2_steer_factor', -0.17)
        self.declare_parameter('evade_swerve_straighten_ratio', 0.15)   # duracion fase 3
        # Factor de fase 3. Signo + = corrige hacia el carril, signo - = sigue
        # alejandose. Calibrado: -0.70 para que la fase 3 mantenga el alejamiento
        # y enderece sin cerrar la S.
        self.declare_parameter('evade_swerve_phase3_steer_factor', -0.70)
        # Si en fase 3 NO se encuentra la linea (caso curvas con obstaculo)
        # y se conoce el ultimo lado donde aparecio (last_yellow_side), girar
        # mas hacia ese lado para no perderla. Factor multiplica away_steer
        # y va al lado de la linea (no del obstaculo).
        self.declare_parameter('evade_phase3_no_line_enable',      True)
        self.declare_parameter('evade_phase3_no_line_steer_factor', 0.80)
        self.declare_parameter('evade_settle_time_s',           0.30)
        # Busqueda de la linea amarilla tras el rebase.
        self.declare_parameter('evade_search_time_s',           10.0)   # s — timeout
        self.declare_parameter('evade_search_speed',            0.05)  # m/s
        self.declare_parameter('evade_search_steering',         0.20)  # rad
        self.declare_parameter('evade_search_frames_required',  3)     # consecutivos
        self.declare_parameter('evade_invert_sign',             True)  # invierte el steer de esquiva reactiva

        # ── Ventana de gracia tras perder la linea ──────────────────────────
        # Cuando el tracker pierde la linea (target_y<=0), en vez de frenar
        # de golpe, sigue avanzando line_loss_grace_s segundos manteniendo el
        # ultimo steering conocido a velocidad reducida (line_loss_speed_factor).
        # Util en curvas donde la camara pierde la amarilla por un instante.
        # Si se recupera la linea durante la gracia → vuelve a NORMAL.
        # Si pasa el timeout → STOP.
        # Ajustado a 8.0s para dar la vuelta completa a ciegas si se pierde la línea.
        # Mantiene el último steering y si no encuentra la línea se detiene.
        self.declare_parameter('line_loss_grace_s',        8.0)  # 8 s
        self.declare_parameter('line_loss_speed_factor',   0.7)   # 70% de la velocidad

        # ── Feedback IMU (PI sobre velocidad angular) ────────────────────────
        # Cuando esta activo, compara el rate de giro DESEADO (calculado por
        # Pure Pursuit) con el rate de giro REAL (medido por IMU.gyro_z).
        # Si el carro no gira como pedido (slip, sub-viraje), corrige el
        # steering con un PI controller. Compensa pérdida de tracción y
        # zona muerta del servo.
        self.declare_parameter('use_imu_feedback', False)    # default OFF para compatibilidad
        self.declare_parameter('imu_kp', 0.15)               # proporcional sobre error omega
        self.declare_parameter('imu_ki', 0.05)               # integral sobre error omega
        self.declare_parameter('imu_max_correction', 0.10)   # rad — limite de correccion
        self.declare_parameter('imu_min_speed', 0.05)        # m/s — minimo para activar feedback
        self.declare_parameter('imu_timeout_s', 0.3)         # si no llega IMU, desactiva
        self.declare_parameter('imu_integral_limit', 0.5)    # anti-windup del integrador

        # ── FASE 2: Contribuciones A (fusion) y C (curvature speed) ──────────
        # Ambos flags default FALSE. Regresion cero: con ambos en False, el
        # comportamiento del nodo es funcionalmente identico al actual.
        # Activacion en runtime con `ros2 param set` (Dashing: -p al launch
        # NO sobreescribe, hay que set despues).
        self.declare_parameter('fusion_enable',             False)
        self.declare_parameter('curvature_speed_enable',    False)

        # Contribucion A: parametros de fusion vision-LIDAR
        # Sigmoid del peso w_vision: w_vision = lane_conf * sigmoid(soft*(lidar - thresh))
        # soft alto → transicion abrupta; bajo → mas suave.
        self.declare_parameter('fusion_sigmoid_soft',       5.0)
        self.declare_parameter('fusion_sigmoid_thresh',     1.0)   # m
        # Histeresis sobre w_vision para evitar chattering en la frontera.
        # w_vision_state baja a "lidar-dominado" si cae por debajo de hyst_lo,
        # vuelve a "vision-dominado" si sube por encima de hyst_hi.
        self.declare_parameter('fusion_hysteresis_lo',      0.30)
        self.declare_parameter('fusion_hysteresis_hi',      0.50)
        # Filtro EMA sobre lidar_min para suavizar ruido de rayos espurios.
        # alpha=1.0 = sin filtro; alpha bajo = mas suavizado.
        self.declare_parameter('lidar_min_ema_alpha',       0.30)
        # Timeouts de inputs de fusion: si pasan estos segundos sin datos
        # frescos del gap o de la vision, el peso correspondiente cae.
        self.declare_parameter('fusion_gap_timeout_s',      0.30)
        self.declare_parameter('fusion_lane_conf_timeout_s', 0.30)
        # Cooldown post-evasion: al volver evade_state a NORMAL, forzar
        # w_lidar >= floor durante este tiempo para evitar ping-pong.
        self.declare_parameter('fusion_evasion_cooldown_s', 1.5)
        self.declare_parameter('fusion_evasion_cooldown_w_lidar_floor', 0.60)

        # Contribucion C: parametros de curvature-adaptive speed
        # Formula: v = clip(v_max / (1 + alpha*|kappa|), v_min, v_max)
        # kappa en 1/m (proviene de /lane/curvature).
        self.declare_parameter('curv_v_max',                0.08)
        self.declare_parameter('curv_v_min',                0.04)
        self.declare_parameter('curv_alpha',                1.0)
        self.declare_parameter('curv_kappa_ema_alpha',      0.20)
        self.declare_parameter('curv_timeout_s',            0.30)
        # Blend con distancia LIDAR: v_final = v_target*dist_factor + v_min*(1-dist_factor)
        # dist_factor = clip((lidar_min - lo) / (hi - lo), 0, 1)
        self.declare_parameter('curv_dist_blend_lo',        0.30)   # m
        self.declare_parameter('curv_dist_blend_hi',        1.50)   # m
        self.declare_parameter('curv_v_low_dist',           0.05)   # m/s en lo

        # Telemetry: /follower/telemetry_json rate-limited
        self.declare_parameter('telemetry_rate_hz',         20.0)

        # ── Cache parametros ─────────────────────────────────────────────────
        self.platform              = self.get_parameter('platform').value
        target_topic               = self.get_parameter('target_point_topic').value
        cmd_topic                  = self.get_parameter('cmd_topic').value
        self.wheelbase             = float(self.get_parameter('wheelbase').value)
        self.max_steering_angle    = float(self.get_parameter('max_steering_angle').value)
        self.lookahead_min         = float(self.get_parameter('lookahead_min').value)
        self.lookahead_max         = float(self.get_parameter('lookahead_max').value)
        self.lookahead_base        = float(self.get_parameter('lookahead_base').value)
        self.lookahead_speed_gain  = float(self.get_parameter('lookahead_speed_gain').value)
        self.curve_threshold       = float(self.get_parameter('curve_threshold').value)
        self.speed_straight        = float(self.get_parameter('speed_straight').value)
        self.speed_curve           = float(self.get_parameter('speed_curve').value)
        self.steering_sign         = float(self.get_parameter('steering_sign').value)
        self.steering_gain         = float(self.get_parameter('steering_gain').value)

        self.use_lidar_safety      = bool(self.get_parameter('use_lidar_safety').value)
        self.lidar_stop_distance   = float(self.get_parameter('lidar_stop_distance').value)
        self.lidar_slow_distance   = float(self.get_parameter('lidar_slow_distance').value)
        self.lidar_slow_factor     = float(self.get_parameter('lidar_slow_factor').value)
        self.lidar_timeout_s       = float(self.get_parameter('lidar_timeout_s').value)

        # Cache IMU feedback
        self.use_imu_feedback      = bool(self.get_parameter('use_imu_feedback').value)
        self.imu_kp                = float(self.get_parameter('imu_kp').value)
        self.imu_ki                = float(self.get_parameter('imu_ki').value)
        self.imu_max_correction    = float(self.get_parameter('imu_max_correction').value)
        self.imu_min_speed         = float(self.get_parameter('imu_min_speed').value)
        self.imu_timeout_s         = float(self.get_parameter('imu_timeout_s').value)
        self.imu_integral_limit    = float(self.get_parameter('imu_integral_limit').value)

        self.last_speed_command = 0.0
        # Diagnostico para HUD
        self.last_target_x  = 0.0   # target lateral en metros recibido
        self.last_steer_raw = 0.0   # steering del Pure Pursuit (antes IMU)
        self.last_steer_cmd = 0.0   # steering final que se publica

        # Estado LiDAR (global + por sector)
        self.lidar_min_dist     = float('inf')
        self.lidar_obstacle     = False
        self.last_lidar_time    = 0.0
        self.lidar_front        = float('inf')
        self.lidar_left         = float('inf')
        self.lidar_right        = float('inf')
        self.lidar_back         = float('inf')
        self.lidar_min_360      = float('inf')
        self.lidar_center_10deg  = float('inf')
        self.lidar_closest_angle = 0.0
        # EMA del lidar_min para suavizar la sigmoid de fusion
        self.lidar_min_smoothed = float('inf')

        # Estado CAMARA DE PROFUNDIDAD (depth_processor / "ZED")
        self.depth_obstacle      = False         # /depth/obstacle_detected
        self.depth_center_10deg  = float('inf')  # /depth/min_distance_center_10deg
        self.depth_closest_angle = 0.0           # /depth/closest_angle (rad)
        self.last_depth_time     = 0.0

        # FASE 2: Estado de fusion (Contribucion A)
        self.gap_steer          = 0.0           # rad, del gap_follower
        self.gap_conf           = 0.0           # [0..1]
        self.gap_dist           = float('inf')  # m
        self.last_gap_time      = 0.0
        # Estado historico de w_vision para la histeresis (1.0=visión-dominado)
        self.w_vision_state     = 1.0
        # Cooldown post-evasion: hasta este timestamp, w_lidar se fuerza
        # al floor configurado para evitar ping-pong fusion vs maquina.
        self.evasion_cooldown_until = 0.0
        # Rastreo de transicion de evade_state a NORMAL
        self._last_evade_state  = 'NORMAL'

        # FASE 2: Estado de curvatura (Contribucion C)
        self.lane_curvature     = 0.0           # 1/m, raw del topic
        self.kappa_smoothed     = 0.0           # 1/m, EMA
        self.last_curvature_time = 0.0
        # /lane/confidence ya existe como subscription en otro lado? no.
        # La agregamos aqui — es input a la sigmoid.
        self.lane_confidence    = 0.0
        self.last_lane_conf_time = 0.0

        # Telemetry rate limit
        self._telemetry_tick    = 0

        # Estado evasion: 'NORMAL' | 'REVERSING' | 'STEER_AWAY' | 'SETTLING' | 'SEARCHING'
        self.evade_state        = 'NORMAL'
        self.evade_side         = None     # 'left' / 'right' / 'both' — lado original
        self.steer_away_start_t = 0.0
        self.settle_start_t     = 0.0
        self.search_start_t     = 0.0      # inicio SEARCHING
        self.search_found_count = 0        # frames consecutivos con linea encontrada
        self.early_exit_count   = 0        # frames consecutivos con linea durante STEER_AWAY
        self._depth_confirm_count = 0      # frames consecutivos en que el LiDAR confirma a la ZED
        # Yellow line tracking
        self.last_yellow_side   = None     # 'left' / 'right' — donde apareci antes
        self.yellow_found       = False    # /yellow_line/found mas reciente
        # Tracking de pérdida de línea: timestamp del ultimo target valido.
        # Usado por la ventana de gracia (line_loss_grace_s) en curvas.
        self.last_valid_target_t = 0.0
        # Wall flag del lidar
        self.lidar_is_wall      = False
        self.wall_loss_counter  = 0        # contador para debounce de perdida de vallas
        self.obstacle_persist_counter = 0  # contador para persistencia del obstaculo antes de evadir
        # Cooldown del preempt: timestamp en el que se reactiva el preempt
        self.preempt_blocked_until_t = 0.0
        # Cache y latch de esquiva preemptiva
        self.preempt_latched_bias = 0.0
        self.preempt_last_detection_t = 0.0
        self.preempt_latched_side = None

        # Estado IMU feedback (PI controller)
        self.imu_omega_z        = 0.0       # gyro_z mas reciente del IMU
        self.last_imu_time      = 0.0
        self.imu_integral       = 0.0       # acumulador integral del PI
        self.last_correction    = 0.0       # ultima correccion aplicada (para logs)
        self.last_omega_error   = 0.0       # ultimo error (para logs)
        self.last_control_time  = 0.0       # para calcular dt en el PI

        # Arranque manual: espera ENTER para activarse (SPACE no arranca aqui)
        self.running = False

        # Variables de estado del target de la cámara y watchdog
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_valid = False
        self.last_target_time = 0.0

        # ── ROS ──────────────────────────────────────────────────────────────
        self.subscription = self.create_subscription(
            Float32MultiArray, target_topic, self.target_callback, 10,
        )

        if self.use_lidar_safety:
            self.create_subscription(Float32, '/lidar/min_distance',
                                     self._lidar_min_cb, 10)
            self.create_subscription(Bool, '/lidar/obstacle',
                                     self._lidar_obs_cb, 10)
            # Sectores: front (centro) + lateral izquierdo + lateral derecho + trasero
            self.create_subscription(Float32, '/lidar/min_distance_front',
                                     self._lidar_front_cb, 10)
            self.create_subscription(Float32, '/lidar/min_distance_left',
                                     self._lidar_left_cb,  10)
            self.create_subscription(Float32, '/lidar/min_distance_right',
                                     self._lidar_right_cb, 10)
            self.create_subscription(Float32, '/lidar/min_distance_back',
                                     self._lidar_back_cb,  10)
            self.create_subscription(Float32, '/lidar/min_distance_360',
                                     self._lidar_360_cb,   10)
            self.create_subscription(Bool, '/lidar/is_wall',
                                     self._lidar_wall_cb, 10)
            self.create_subscription(Float32, '/lidar/min_distance_center_10deg',
                                     self._lidar_center_10deg_cb, 10)
            self.create_subscription(Float32, '/lidar/closest_angle',
                                     self._lidar_closest_angle_cb, 10)

        # Suscripciones a la CAMARA DE PROFUNDIDAD (depth_processor / "ZED").
        # Se crean SIEMPRE (sin depender de use_lidar_safety) para que la
        # evasion por vision funcione aunque el lidar este apagado. Si el
        # depth_processor no esta corriendo, los callbacks nunca se invocan y
        # el timeout deja depth_obstacle en False — sin coste.
        self.create_subscription(Bool, '/depth/obstacle_detected',
                                 self._depth_obs_cb, 10)
        self.create_subscription(Float32, '/depth/min_distance_center_10deg',
                                 self._depth_center_cb, 10)
        self.create_subscription(Float32, '/depth/closest_angle',
                                 self._depth_angle_cb, 10)

        # Suscripcion a /yellow_line/found para detectar fin de SEARCHING
        self.create_subscription(Bool, '/yellow_line/found',
                                 self._yellow_found_cb, 10)

        # Suscripcion a /lane/enable_auto: permite al TELEOP detener al
        # follower con SPACE (parada de emergencia) o reactivarlo con M.
        # SPACE publica False -> running=False + STOP. M publica True.
        self.create_subscription(Bool, '/lane/enable_auto',
                                 self._enable_auto_cb, 10)

        # IMU subscriber — solo si feedback activado. QoS RELIABLE matchea
        # con el publisher de imu_bno055 (que tambien es RELIABLE).
        if self.use_imu_feedback:
            imu_qos = QoSProfile(
                reliability = QoSReliabilityPolicy.RELIABLE,
                durability  = QoSDurabilityPolicy.VOLATILE,
                history     = QoSHistoryPolicy.KEEP_LAST,
                depth       = 10,
            )
            self.create_subscription(Imu, '/imu/data', self._imu_cb, imu_qos)
            self.get_logger().info(
                f'IMU feedback ACTIVADO | Kp={self.imu_kp:.2f} Ki={self.imu_ki:.2f} '
                f'max_corr={self.imu_max_correction:.2f} rad'
            )

        # Para que el dashboard sepa el estado
        self.active_pub = self.create_publisher(Bool, '/lane/auto_active', 10)
        self.active_timer = self.create_timer(0.2, self._publish_active)

        # FASE 2: subscripciones de fusion + curvatura. Se crean SIEMPRE
        # (no detras de flag) para permitir flip de flags en runtime sin
        # reiniciar el nodo. El overhead es despreciable cuando no hay
        # publishers (callbacks no se invocan).
        from std_msgs.msg import String
        self.create_subscription(Float32, '/gap/steer_cmd',  self._gap_steer_cb,  10)
        self.create_subscription(Float32, '/gap/confidence', self._gap_conf_cb,   10)
        self.create_subscription(Float32, '/gap/distance',   self._gap_dist_cb,   10)
        self.create_subscription(Float32, '/lane/curvature', self._curvature_cb,  10)
        self.create_subscription(Float32, '/lane/confidence', self._lane_conf_cb, 10)
        # Publisher de telemetria para experimentos del paper. Rate-limited
        # en el control_loop a self.telemetry_rate_hz (default 20 Hz).
        self.telemetry_pub = self.create_publisher(String, '/follower/telemetry_json', 10)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  Presiona  ENTER  para ARRANCAR')
        self.get_logger().info('  Presiona  Q  o  Ctrl+C    para PARAR')
        self.get_logger().info('=' * 50)

        if self.platform == 'qcar':
            self.cmd_pub = self.create_publisher(Vector3Stamped, cmd_topic, 10)
        elif self.platform == 'qcar2':
            # Import diferido para no romper si el msg no esta instalado
            from qcar2_interfaces.msg import MotorCommands  # type: ignore
            self._MotorCommands = MotorCommands
            self.cmd_pub = self.create_publisher(MotorCommands, cmd_topic, 10)
        else:
            raise ValueError(f"platform desconocida: {self.platform}")

        self.get_logger().info(
            f'Lane Follower PP iniciado | platform={self.platform} | '
            f'cmd={cmd_topic} | target={target_topic}'
        )

        # Timer de control a 50Hz (bucle principal de control con watchdog)
        self.control_timer = self.create_timer(0.02, self.control_loop)

    # ─────────────────────────────────────────────────────────────────────────
    def _lidar_min_cb(self, msg):
        self.lidar_min_dist = float(msg.data)
        self.last_lidar_time = time.time()

    def _lidar_obs_cb(self, msg):
        self.lidar_obstacle = bool(msg.data)
        self.last_lidar_time = time.time()

    def _lidar_front_cb(self, msg):
        d = float(msg.data)
        # lidar_processor publica -1.0 cuando no hay datos validos → inf
        self.lidar_front = d if d >= 0.0 else float('inf')
        self.last_lidar_time = time.time()

    def _lidar_left_cb(self, msg):
        d = float(msg.data)
        self.lidar_left = d if d >= 0.0 else float('inf')
        self.last_lidar_time = time.time()

    def _lidar_right_cb(self, msg):
        d = float(msg.data)
        self.lidar_right = d if d >= 0.0 else float('inf')
        self.last_lidar_time = time.time()

    def _lidar_back_cb(self, msg):
        d = float(msg.data)
        self.lidar_back = d if d >= 0.0 else float('inf')
        self.last_lidar_time = time.time()

    def _lidar_360_cb(self, msg):
        """Distancia minima omnidireccional (360°). Si entra dentro del
        circulo de seguridad → STOP duro absoluto."""
        d = float(msg.data)
        self.lidar_min_360 = d if d >= 0.0 else float('inf')
        self.last_lidar_time = time.time()

    def _lidar_center_10deg_cb(self, msg):
        d = float(msg.data)
        self.lidar_center_10deg = d if d >= 0.0 else float('inf')
        self.last_lidar_time = time.time()

    def _lidar_closest_angle_cb(self, msg):
        self.lidar_closest_angle = float(msg.data)
        self.last_lidar_time = time.time()

    # ── Callbacks CAMARA DE PROFUNDIDAD (depth_processor / "ZED") ─────────────
    def _depth_obs_cb(self, msg):
        self.depth_obstacle = bool(msg.data)
        self.last_depth_time = time.time()

    def _depth_center_cb(self, msg):
        # depth_processor publica -1.0 cuando no hay lectura valida → inf
        d = float(msg.data)
        self.depth_center_10deg = d if d >= 0.0 else float('inf')
        self.last_depth_time = time.time()

    def _depth_angle_cb(self, msg):
        # Convencion ROS: angulo > 0 = obstaculo a la IZQUIERDA, < 0 = derecha.
        self.depth_closest_angle = float(msg.data)
        self.last_depth_time = time.time()

    # FASE 2: callbacks de fusion + curvatura. Trivial (solo guardan valor).
    def _gap_steer_cb(self, msg):
        self.gap_steer = float(msg.data)
        self.last_gap_time = time.time()

    def _gap_conf_cb(self, msg):
        self.gap_conf = float(msg.data)
        self.last_gap_time = time.time()

    def _gap_dist_cb(self, msg):
        self.gap_dist = float(msg.data)
        self.last_gap_time = time.time()

    def _curvature_cb(self, msg):
        self.lane_curvature = float(msg.data)
        self.last_curvature_time = time.time()

    def _lane_conf_cb(self, msg):
        self.lane_confidence = float(msg.data)
        self.last_lane_conf_time = time.time()

    def _yellow_found_cb(self, msg):
        self.yellow_found = bool(msg.data)

    def _enable_auto_cb(self, msg):
        """Recibe /lane/enable_auto del teleop.
        False (SPACE en TELEOP) -> parada de emergencia: running=False + STOP.
        True  (M en TELEOP)     -> reactiva el follower sin necesidad de ENTER.
        """
        enable = bool(msg.data)
        if not enable and self.running:
            self.running = False
            self.publish_stop()
            self.get_logger().warn(
                'STOP de emergencia recibido desde TELEOP (SPACE). '
                'Pulsa ENTER en el panel del follower o M en el teleop para reanudar.'
            )
        elif enable and not self.running:
            self.running = True
            self.get_logger().info(
                'Reactivacion remota desde TELEOP (M). Follower arrancando.'
            )

    def _lidar_wall_cb(self, msg):
        wall_val = bool(msg.data)
        if wall_val:
            self.lidar_is_wall = True
            self.wall_loss_counter = 0
        else:
            # Si era True y ahora es False, demorar la transicion para evitar oscilaciones en curvas
            if self.lidar_is_wall:
                self.wall_loss_counter += 1
                if self.wall_loss_counter >= 5:  # requiere 5 ticks (100 ms) seguidos de False
                    self.lidar_is_wall = False
                    self.wall_loss_counter = 0
            else:
                self.lidar_is_wall = False
                self.wall_loss_counter = 0
        self.last_lidar_time = time.time()

    def _imu_cb(self, msg):
        """Recibe IMU: extrae solo angular_velocity.z (rate de giro real)."""
        self.imu_omega_z = float(msg.angular_velocity.z)
        self.last_imu_time = time.time()

    # ─────────────────────────────────────────────────────────────────────────
    def _imu_correction(self, delta_target, speed):
        """
        PI controller sobre el error de velocidad angular (omega).

        El modelo bicycle dice que:  omega_deseado = speed * tan(delta) / L
        Si el carro realmente girara como pedido, omega_imu == omega_deseado.
        Cualquier diferencia es slip/sub-viraje → corregimos el steering.

        Retorna correccion (rad) a sumar al delta_target.
        Devuelve 0.0 si:
          - feedback IMU desactivado
          - datos IMU desactualizados
          - velocidad demasiado baja (no hay slip a 0 m/s)
        """
        if not self.use_imu_feedback:
            return 0.0
        if (time.time() - self.last_imu_time) > self.imu_timeout_s:
            return 0.0
        if abs(speed) < self.imu_min_speed:
            # A baja velocidad reset integral para no acumular ruido
            self.imu_integral = 0.0
            return 0.0

        # 1. Velocidad angular DESEADA segun bicycle model
        omega_desired = speed * math.tan(delta_target) / self.wheelbase

        # 2. Error contra la medicion REAL del IMU
        omega_error = omega_desired - self.imu_omega_z
        self.last_omega_error = omega_error

        # 3. dt para integracion
        now = time.time()
        if self.last_control_time == 0.0:
            self.last_control_time = now
            return 0.0
        dt = now - self.last_control_time
        self.last_control_time = now
        if dt <= 0.0 or dt > 0.2:   # ignora gaps largos
            return 0.0

        # 4. Acumulador integral con anti-windup
        self.imu_integral += omega_error * dt
        self.imu_integral = self.clamp(
            self.imu_integral,
            -self.imu_integral_limit,
            self.imu_integral_limit,
        )

        # 5. PI: correccion = Kp·error + Ki·integral
        correction = self.imu_kp * omega_error + self.imu_ki * self.imu_integral

        # 6. Clamp al maximo permitido
        correction = self.clamp(
            correction,
            -self.imu_max_correction,
            self.imu_max_correction,
        )

        self.last_correction = correction
        return correction

    def _maybe_publish_telemetry(self, now, steer_pp, steer_gap, steer_final,
                                  w_vision, w_lidar, v_target, v_final, safety,
                                  fusion_enable, curvature_speed_enable):
        """Publica /follower/telemetry_json a `telemetry_rate_hz` (default 20).
        Sub-muestrea el control_loop (50 Hz) con un contador modulo.
        Falla silenciosa si la serializacion ocurre y se aborta el tick."""
        rate_hz = float(self.get_parameter('telemetry_rate_hz').value)
        if rate_hz <= 0.0:
            return
        # Control loop a 50 Hz. Sub-muestreamos cada N ticks.
        skip = max(1, int(round(50.0 / rate_hz)))
        self._telemetry_tick += 1
        if (self._telemetry_tick % skip) != 0:
            return
        try:
            payload = {
                't': float(now),
                'w_vision': float(w_vision),
                'w_lidar': float(w_lidar),
                'steer_pp_rad': float(steer_pp),
                'steer_gap_rad': float(steer_gap),
                'steer_final_rad': float(steer_final),
                'v_target': float(v_target),
                'v_final': float(v_final),
                'safety_factor': float(safety),
                'lane_conf': float(self.lane_confidence),
                'gap_conf': float(self.gap_conf),
                'lidar_min': float(self.lidar_min_360),
                'lidar_min_smoothed': float(self.lidar_min_smoothed),
                'curvature': float(self.lane_curvature),
                'kappa_smoothed': float(self.kappa_smoothed),
                'evade_state': str(self.evade_state),
                'fusion_enable': bool(fusion_enable),
                'curvature_speed_enable': bool(curvature_speed_enable),
                'evasion_cooldown_active': bool(now < self.evasion_cooldown_until),
            }
            from std_msgs.msg import String
            msg = String()
            msg.data = json.dumps(payload)
            self.telemetry_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(
                f'telemetry publish failed: {e}',
                throttle_duration_sec=5.0,
            )

    def _publish_active(self):
        self.active_pub.publish(Bool(data=self.running))
        if self.running:
            imu_info = ''
            if self.use_imu_feedback:
                imu_age = time.time() - self.last_imu_time
                if imu_age < self.imu_timeout_s:
                    imu_info = (f' IMU:ω{self.imu_omega_z:+.2f} '
                                f'corr{self.last_correction:+.3f}')
                else:
                    imu_info = ' IMU:OFFLINE'
            print(
                f'\r  [RUNNING] spd:{self.last_speed_command:.3f} '
                f'tx:{self.last_target_x:+.3f}m '
                f'steer:{math.degrees(self.last_steer_cmd):+5.1f}° '
                f'lidar:{self.lidar_min_dist:.2f}m{imu_info}    ',
                end='', flush=True,
            )
        else:
            print('\r  [PARADO]  Presiona ENTER para arrancar...          ',
                  end='', flush=True)

    def _lidar_confirms_depth(self, depth_angle, depth_dist):
        """Ground-truth: ¿el LiDAR confirma la deteccion de la ZED?

        Exige que el LiDAR vea el obstaculo en el MISMO lado (centrado / izq /
        der, segun depth_angle) y a distancia parecida (±depth_lidar_dist_tol),
        compensando la separacion fisica camara-lidar (depth_lidar_forward_offset_m).
        Si el LiDAR no ve nada en ese lado -> NO confirma (ground-truth estricto).
        """
        tol    = float(self.get_parameter('depth_lidar_dist_tol').value)
        offset = float(self.get_parameter('depth_lidar_forward_offset_m').value)
        sector_check = bool(self.get_parameter('depth_lidar_sector_check').value)

        # Mapeo preciso de los limites de los sectores del LiDAR (coincidiendo con lidar_processor.py)
        # El sector frontal del LiDAR cubre un semi-angulo de 15.0 grados (front_center_deg = 30.0)
        half_front = math.radians(15.0)

        # ¿Donde ve la ZED el obstaculo? centrado / izquierda / derecha
        if abs(depth_angle) <= half_front:
            # El obstaculo esta en el sector frontal del LiDAR
            lidar_d = self.lidar_front
            # Si esta extremadamente centrado, intentamos usar el cono ultra-estrecho de 10deg
            if abs(depth_angle) < math.radians(5.0) and math.isfinite(self.lidar_center_10deg):
                lidar_d = self.lidar_center_10deg
        else:
            # El obstaculo esta en los sectores laterales
            obs_left = depth_angle > 0.0      # mismo criterio que depth_side
            if sector_check:
                lidar_d = self.lidar_left if obs_left else self.lidar_right
            else:
                lidar_d = min(self.lidar_left, self.lidar_right)

        # El LiDAR no ve nada en ese lado -> ground-truth estricto: no confirma
        if not math.isfinite(lidar_d):
            return False

        # Distancia coherente (compensando offset fisico camara-lidar)
        return abs(depth_dist - (lidar_d - offset)) <= tol

    def _preempt_bias(self, stop_d):
        """Calcula un sesgo lateral (en metros) para el target_x cuando un
        obstaculo esta en la zona PREEMPTIVA (entre stop_d y trigger_d).
        Usa la camara ZED para validar y determinar la direccion, y el LIDAR
        como ground truth de distancia de alta precision.
        """
        if not bool(self.get_parameter('preempt_enable').value):
            return 0.0
        if self.evade_state != 'NORMAL':
            return 0.0
        # Cooldown despues de evadir: no reactivar preempt enseguida
        if time.time() < self.preempt_blocked_until_t:
            return 0.0
        if (time.time() - self.last_lidar_time) > float(
                self.get_parameter('lidar_timeout_s').value):
            return 0.0
        trigger_d = float(self.get_parameter('preempt_trigger_distance').value)
        bias_max  = float(self.get_parameter('preempt_bias_max_m').value)
        use_front = bool(self.get_parameter('preempt_use_front').value)
        use_sides = bool(self.get_parameter('preempt_use_sides').value)

        # ── Procesar CAMARA DE PROFUNDIDAD (depth_processor / "ZED") ──────────
        depth_active = False
        depth_side = None
        depth_dist = None

        if bool(self.get_parameter('depth_preempt_enable').value):
            depth_trigger_d = float(self.get_parameter('depth_trigger_distance').value)
            depth_fresh = (time.time() - self.last_depth_time) <= float(
                self.get_parameter('depth_timeout_s').value)
            if (depth_fresh and self.depth_obstacle
                    and stop_d <= self.depth_center_10deg < depth_trigger_d):
                depth_active = True
                
                # Obtener angulo (opcionalmente invertido)
                depth_angle = self.depth_closest_angle
                if bool(self.get_parameter('depth_invert_angle').value):
                    depth_angle = -depth_angle

                # Umbral de "centrado": ~6°
                centered = abs(depth_angle) < math.radians(6.0)
                if centered and (math.isfinite(self.lidar_left)
                                 or math.isfinite(self.lidar_right)):
                    depth_side = 'right' if self.lidar_left > self.lidar_right else 'left'
                else:
                    depth_side = 'left' if depth_angle > 0.0 else 'right'
                
                # Ground truth de distancia: si el lidar frontal ve un obstaculo,
                # usamos la distancia del lidar por ser mas precisa. Sino, la de camara.
                if math.isfinite(self.lidar_front) and stop_d <= self.lidar_front < trigger_d:
                    depth_dist = self.lidar_front
                else:
                    depth_dist = self.depth_center_10deg

            # ── GATE de consistencia ZED↔LiDAR (ground-truth ESTRICTO) ───────
            # Solo aceptamos la deteccion de la ZED si el LiDAR la confirma en
            # el MISMO lado y a distancia parecida (±tol). Debounce por frames.
            # Si el LiDAR no ve nada ahi, la ZED NO cuenta.
            if depth_active and bool(
                    self.get_parameter('depth_require_lidar_confirm').value):
                if self._lidar_confirms_depth(depth_angle, self.depth_center_10deg):
                    self._depth_confirm_count += 1
                else:
                    self._depth_confirm_count = 0
                if self._depth_confirm_count < int(
                        self.get_parameter('depth_lidar_confirm_frames').value):
                    depth_active = False   # sin confirmacion del lidar -> ignorar ZED
            else:
                self._depth_confirm_count = 0

        candidates = []

        # 1. Si la camara detecto un obstaculo, tiene prioridad maxima para definir el lado.
        if depth_active and depth_side is not None and depth_dist is not None:
            candidates.append((depth_dist, depth_side))
            self.preempt_latched_side = depth_side

        # 2. Si no hay camara activa pero estabamos en una evasion latente,
        #    usamos el lidar (frontal y del lado del obstaculo) como ground truth
        #    de distancia de seguimiento. Usamos el rango de la camara (depth_trigger_distance)
        #    para continuar rastreando el obstaculo con el LiDAR mientras lo rebasamos.
        elif self.preempt_latched_side is not None:
            max_track_d = max(trigger_d, float(self.get_parameter('depth_trigger_distance').value))
            lidar_d = float('inf')
            if self.preempt_latched_side == 'left':
                lidar_d = min(self.lidar_front, self.lidar_left)
            elif self.preempt_latched_side == 'right':
                lidar_d = min(self.lidar_front, self.lidar_right)
            
            if math.isfinite(lidar_d) and lidar_d < max_track_d:
                candidates.append((lidar_d, self.preempt_latched_side))

        # 3. Si no hay deteccion de camara ni latch, procesamos lidar normal (legacy fallback)
        if not candidates:
            if use_front and stop_d <= self.lidar_front < trigger_d:
                side = 'right' if self.lidar_left > self.lidar_right else 'left'
                candidates.append((self.lidar_front, side))
            
            # Ignorar las paredes laterales de la pista
            ignore_walls_val = bool(self.get_parameter('ignore_walls').value)
            if use_sides and not (ignore_walls_val and self.lidar_is_wall):
                if stop_d <= self.lidar_left < trigger_d:
                    candidates.append((self.lidar_left, 'left'))
                if stop_d <= self.lidar_right < trigger_d:
                    candidates.append((self.lidar_right, 'right'))

        # ── Estricto en CURVAS: exigir confirmacion cruzada LIDAR+depth ─────
        if bool(self.get_parameter('curve_strict_fusion_enable').value):
            kappa_thr = float(
                self.get_parameter('curve_strict_kappa_thresh').value)
            lidar_max = float(
                self.get_parameter('curve_strict_lidar_max').value)
            in_curve = (abs(self.kappa_smoothed) > kappa_thr
                        or abs(self.last_steer_raw) > self.curve_threshold)
            if in_curve and candidates:
                # ¿LIDAR frontal confirma obstaculo cercano?
                lidar_confirms = (math.isfinite(self.lidar_front)
                                  and self.lidar_front < lidar_max)
                if not lidar_confirms:
                    # Ignorar detecciones de la camara o basadas en el latch de camara
                    candidates = [
                        c for c in candidates
                        if c[1] not in (depth_side, self.preempt_latched_side)
                    ]
                    if depth_active:
                        self.preempt_latched_side = None

        now = time.time()
        hold_time = float(self.get_parameter('preempt_hold_time_s').value)

        if not candidates:
            # Si no hay candidatos actuales, pero estabamos evadiendo recientemente
            if self.preempt_latched_side is not None and (now - self.preempt_last_detection_t < hold_time):
                elapsed = now - self.preempt_last_detection_t
                decay = 1.0 - (elapsed / max(1e-3, hold_time))
                decay = max(0.0, min(1.0, decay))
                return self.preempt_latched_bias * decay
            else:
                self.preempt_latched_bias = 0.0
                self.preempt_latched_side = None
                return 0.0

        # El obstaculo mas cercano manda
        d_obs, side = min(candidates, key=lambda x: x[0])

        # Definir la distancia de activacion para el calculo del bias
        active_trigger_d = trigger_d
        if self.preempt_latched_side is not None:
            active_trigger_d = max(trigger_d, float(self.get_parameter('depth_trigger_distance').value))

        # Proporcional: lejos (active_trigger_d) → 0; cerca (stop_d) → bias_max
        rng = max(active_trigger_d - stop_d, 1e-3)
        prox = (active_trigger_d - d_obs) / rng        # 0 a 1
        prox = max(0.0, min(1.0, prox))

        # Signo del bias
        sign = -1.0 if side == 'right' else +1.0
        if bool(self.get_parameter('preempt_invert_sign').value):
            sign = -sign

        current_bias = sign * bias_max * prox

        # Guardar en cache/latch
        self.preempt_latched_bias = current_bias
        self.preempt_latched_side = side
        self.preempt_last_detection_t = now

        return current_bias

    def _evade_action(self, stop_d):
        """Maquina de estados de evasion lateral con REBASE.

        Estados:
          NORMAL     → si hay obstaculo lateral o frontal, pasa a REVERSING
          REVERSING  → retrocede girando al lado OPUESTO del obstaculo
                       (cola del carro se aleja, frente queda mirando al
                       lado contrario para rebasar despues). Sale cuando el
                       sector del obstaculo muestra distancia >=
                       evade_clear_distance + evade_reverse_extra_distance
                       (objeto fuera del cono de colision lateral).
          STEER_AWAY → avanza girando al MISMO lado del obstaculo (rebase
                       por el costado libre). La deteccion lateral se IGNORA
                       (modo rebase) pero el STOP frontal sigue activo.
          SETTLING   → quieto evade_settle_time_s; despues vuelve a NORMAL.

        Convenciones:
          - Steering positivo = giro a la izquierda
          - Steering negativo = giro a la derecha
          - Al retroceder el carro pivotea: la COLA se va al lado del giro
          - Al avanzar: el FRENTE se va al lado del giro

        Caso obstaculo a la DERECHA:
          REVERSING : steer +rev_steer (izquierda) → cola se va a la izq,
                       frente queda apuntando al lado contrario (izquierdo)
          STEER_AWAY: steer -away_steer (derecha) → avanza por la derecha,
                       rebasa pasando por el lado del obstaculo
        Caso obstaculo a la IZQUIERDA (simetrico):
          REVERSING : steer -rev_steer (derecha)
          STEER_AWAY: steer +away_steer (izquierda)

        Si hay obstaculo en el sector FRONTAL, se trata como lateral del lado
        donde haya MAS espacio (lidar_right vs lidar_left).

        Retorna (speed, steering_angle) si esta en algun estado de evasion;
        (None, None) si NORMAL (deja pasar el control normal del follower).
        """
        if not bool(self.get_parameter('evade_enable').value):
            self.evade_state = 'NORMAL'
            return None, None
        if not bool(self.get_parameter('use_lidar_safety').value):
            self.evade_state = 'NORMAL'
            return None, None
        # Datos del lidar viejos → salir de cualquier estado de evasion
        if (time.time() - self.last_lidar_time) > float(self.get_parameter('lidar_timeout_s').value):
            self.evade_state = 'NORMAL'
            return None, None
        # Si detectamos pared envolvente (vallas) y ignore_walls está activo:
        # - Si vemos la línea amarilla (yellow_found=True), ignoramos la evasión por completo
        #   (ya que es sólo la pared de la curva y el seguidor la contornea).
        # - Si NO vemos la línea amarilla (yellow_found=False) y hay un obstáculo inminente al frente,
        #   sí permitimos la evasión.
        ignore_active = bool(self.get_parameter('ignore_walls').value) and self.lidar_is_wall
        if ignore_active and self.evade_state == 'NORMAL':
            if self.yellow_found:
                return None, None
            elif not (self.lidar_front < stop_d):
                return None, None

        # Lectura en vivo de parametros
        clear_d    = float(self.get_parameter('evade_clear_distance').value)
        extra_d    = float(self.get_parameter('evade_reverse_extra_distance').value)
        settle_s   = float(self.get_parameter('evade_settle_time_s').value)
        rev_speed  = float(self.get_parameter('evade_reverse_speed').value)
        rev_steer  = float(self.get_parameter('evade_reverse_steering').value)
        away_time  = float(self.get_parameter('evade_steer_away_time_s').value)
        away_speed = float(self.get_parameter('evade_steer_away_speed').value)
        away_steer = float(self.get_parameter('evade_steer_away_steering').value)

        # Ajuste adaptativo: en recta somos agresivos y anticipamos; en curva somos conservadores
        # para evitar falsos positivos con las vallas exteriores.
        is_straight = abs(self.last_steer_cmd) < 0.06
        if is_straight:
            front_trigger_d = 1.10
            away_steer = away_steer * 1.4
        else:
            front_trigger_d = 0.60

        now = time.time()

        # Helpers para steering segun lado del obstaculo
        # Convencion: steer + = giro a la izquierda; steer - = giro a la derecha
        def _reverse_steer():
            # Al retroceder: la cola del carro se va al lado del giro.
            # Queremos que la cola se vaya al lado OPUESTO del obstaculo.
            # obstaculo derecho → cola a la izquierda → steer +
            # obstaculo izquierdo → cola a la derecha → steer -
            if self.evade_side == 'right':
                return +rev_steer
            elif self.evade_side == 'left':
                return -rev_steer
            return 0.0

        def _away_steer_phase1():
            # Fase 1 del swerve: gira al lado OPUESTO al obstaculo (rodea por
            # el lado libre).
            # obstaculo derecha → giro izquierda → steer +
            # obstaculo izquierda → giro derecha → steer -
            if self.evade_side == 'right':
                return +away_steer
            elif self.evade_side == 'left':
                return -away_steer
            return 0.0

        # ── STEER_AWAY (SWERVE en 3 sub-fases): rodea → corrige → endereza ──
        # Durante todo STEER_AWAY se IGNORA la deteccion lateral. El STOP
        # frontal sigue activo por seguridad.
        if self.evade_state == 'STEER_AWAY':
            elapsed = now - self.steer_away_start_t
            if elapsed >= away_time:
                self.evade_state = 'SETTLING'
                self.settle_start_t = now
                self.early_exit_count = 0
                return 0.0, 0.0

            phase1_ratio   = float(self.get_parameter('evade_swerve_phase1_ratio').value)
            straight_ratio = float(self.get_parameter('evade_swerve_straighten_ratio').value)
            phase2_factor  = float(self.get_parameter('evade_swerve_phase2_steer_factor').value)
            phase3_factor  = float(self.get_parameter('evade_swerve_phase3_steer_factor').value)

            t_phase1_end   = away_time * phase1_ratio
            t_phase3_start = away_time * (1.0 - straight_ratio)

            # Decidir fase
            if elapsed < t_phase1_end:
                phase = 1   # rodear
            elif elapsed < t_phase3_start:
                phase = 2   # corregir
            else:
                phase = 3   # enderezar (o corregir suave si phase3_factor > 0)

            # Visual Servoing Adaptativo en Recta (Control en lazo cerrado por cámara):
            # Si estamos en recta, y en teoría pasaríamos a fase 2 o 3, pero la línea amarilla
            # aún no se ha desplazado a los extremos de la pantalla (división de 3 sectores),
            # forzamos al coche a seguir en Fase 1 (volantazo de esquiva) para apartarlo físicamente.
            is_straight = abs(self.last_steer_cmd) < 0.06
            if is_straight and phase > 1 and self.yellow_found and self.target_valid:
                if self.evade_side == 'right' and self.target_x < 0.12:
                    phase = 1  # Forzar volantazo a la izq (la línea debe pegarse a la derecha)
                elif self.evade_side == 'left' and self.target_x > -0.12:
                    phase = 1  # Forzar volantazo a la der (la línea debe pegarse a la izquierda)

            # ── EARLY EXIT: si vemos la linea y el frente esta despejado,
            # saltamos a NORMAL sin esperar al SETTLING/SEARCHING. Solo se
            # permite desde fase >= min_phase (fase 1 puede dar falsos
            # positivos al pasar sobre el divisor amarillo).
            if bool(self.get_parameter('evade_early_exit_enable').value):
                min_phase = int(self.get_parameter('evade_early_exit_min_phase').value)
                ee_frames = int(self.get_parameter('evade_early_exit_frames').value)
                front_clear = self.lidar_front >= stop_d
                if phase >= min_phase and self.yellow_found and front_clear:
                    self.early_exit_count += 1
                else:
                    self.early_exit_count = 0
                if self.early_exit_count >= ee_frames:
                    self.get_logger().warn(
                        f'EVADE EARLY_EXIT — linea detectada en fase {phase} '
                        f'(L:{self.lidar_left:.2f} F:{self.lidar_front:.2f} '
                        f'R:{self.lidar_right:.2f}) → vuelve a NORMAL'
                    )
                    self.evade_state = 'NORMAL'
                    self.evade_side = None
                    self.early_exit_count = 0
                    self.preempt_blocked_until_t = now + float(
                        self.get_parameter('preempt_cooldown_after_evade_s').value)
                    return None, None    # cede control al follower normal

            # Direccion del giro segun fase y lado del obstaculo
            #   Fase 1: gira al OPUESTO del obstaculo
            #   Fase 2: gira al MISMO lado del obstaculo (steering * phase2_factor)
            #   Fase 3: gira al MISMO lado del obstaculo (steering * phase3_factor)
            #           CASO ESPECIAL: si en fase 3 no se ve la linea y se
            #           conoce last_yellow_side, girar mas hacia ese lado.
            if phase == 1:
                # Swerve Proporcional: esquiva mas agresivamente si el objeto esta mas cerca.
                prox_factor = max(0.6, min(1.4, (stop_d / (self.lidar_front + 1e-3))))
                dynamic_steer = away_steer * prox_factor
                if self.evade_side == 'right':
                    steer = +dynamic_steer
                elif self.evade_side == 'left':
                    steer = -dynamic_steer
                else:
                    steer = 0.0
            elif phase == 3 and (not self.yellow_found) and \
                 bool(self.get_parameter('evade_phase3_no_line_enable').value) and \
                 self.last_yellow_side in ('left', 'right'):
                # Linea perdida en fase 3: gira mas hacia el ultimo lado conocido
                # de la linea (no del obstaculo). Util en curvas con obstaculo.
                no_line_factor = float(
                    self.get_parameter('evade_phase3_no_line_steer_factor').value)
                if self.last_yellow_side == 'left':
                    steer = +away_steer * no_line_factor   # gira a la izq
                else:
                    steer = -away_steer * no_line_factor   # gira a la der
            else:
                factor = phase2_factor if phase == 2 else phase3_factor
                # Correccion proporcional
                prox_factor = max(0.6, min(1.4, (stop_d / (self.lidar_front + 1e-3))))
                dynamic_steer = away_steer * prox_factor
                if self.evade_side == 'right':
                    steer = -dynamic_steer * factor
                elif self.evade_side == 'left':
                    steer = +dynamic_steer * factor
                else:
                    steer = 0.0

            # Safety frontal durante rebase
            if self.lidar_front < stop_d:
                return 0.0, steer
            return +abs(away_speed), steer

        # ── SETTLING: quieto, luego pasa a SEARCHING ─────────────────────────
        if self.evade_state == 'SETTLING':
            if (now - self.settle_start_t) >= settle_s:
                self.evade_state = 'SEARCHING'
                self.search_start_t = now
                self.search_found_count = 0
                # Cae al bloque SEARCHING abajo
            else:
                # Mantener el carro rodando suavemente en lugar de frenar en seco a 0.0
                return float(self.get_parameter('evade_search_speed').value), 0.0

        # ── SEARCHING: avanza girando al lado donde estaba la linea hasta
        # encontrarla (yellow_found=True por search_frames_required frames)
        # o hasta el timeout. Si timeout, STOP. ──────────────────────────────
        if self.evade_state == 'SEARCHING':
            search_time   = float(self.get_parameter('evade_search_time_s').value)
            search_speed  = float(self.get_parameter('evade_search_speed').value)
            search_steer  = float(self.get_parameter('evade_search_steering').value)
            frames_req    = int(self.get_parameter('evade_search_frames_required').value)

            # Lado donde girar buscando la linea:
            #   1) si recordamos donde estaba la linea (last_yellow_side), ahi
            #   2) si no, hacia el lado del obstaculo original (cerca del
            #      divisor central, que es la linea amarilla)
            if self.last_yellow_side == 'left':
                steer = +search_steer    # girar a la izquierda
            elif self.last_yellow_side == 'right':
                steer = -search_steer
            elif self.evade_side == 'right':
                steer = -search_steer    # obstaculo a la derecha → girar al lado del obstaculo
            elif self.evade_side == 'left':
                steer = +search_steer
            else:
                steer = 0.0

            # Salida 1: encontre linea N frames seguidos
            if self.yellow_found:
                self.search_found_count += 1
            else:
                self.search_found_count = 0

            if self.search_found_count >= frames_req:
                self.evade_state = 'NORMAL'
                self.evade_side = None
                self.preempt_blocked_until_t = now + float(
                    self.get_parameter('preempt_cooldown_after_evade_s').value)
                return None, None

            # Salida 2: timeout — para totalmente
            if (now - self.search_start_t) >= search_time:
                self.get_logger().warn(
                    'SEARCHING timeout — no se encontro linea amarilla. '
                    'Carro detenido. Reinicia con Q + ENTER en el follower.')
                # Quedarse aqui hasta que el usuario haga algo: publica STOP
                # y NO cambia de estado. running queda True, pero envia 0s.
                self.evade_state = 'NORMAL'
                self.evade_side = None
                self.preempt_blocked_until_t = now + float(
                    self.get_parameter('preempt_cooldown_after_evade_s').value)
                self.running = False    # fuerza al usuario a re-arrancar
                return 0.0, 0.0

            return +abs(search_speed), steer

        # ── Deteccion (NORMAL o REVERSING) ───────────────────────────────────
        # Para la frontal reactiva usamos un umbral adaptivo (front_trigger_d)
        # que cambia segun si estamos en recta o en curva.
        left_blocked   = self.lidar_left   < stop_d
        right_blocked  = self.lidar_right  < stop_d
        front_blocked  = self.lidar_front  < front_trigger_d
        center_blocked = self.lidar_center_10deg < front_trigger_d

        # ── NORMAL → REVERSING o STEER_AWAY directo ──────────────────────────
        if self.evade_state == 'NORMAL':
            if left_blocked or right_blocked or front_blocked or center_blocked:
                self.obstacle_persist_counter += 1
            else:
                self.obstacle_persist_counter = 0

            # Requiere 3 ticks consecutivos (~60 ms) de confirmación del obstáculo para filtrar ruidos
            if self.obstacle_persist_counter < 3:
                return None, None

            # Caso A: obstaculo frontal (en cono amplio o el cono central de 4°)
            # → esquiva DIRECTO con STEER_AWAY (sin reversa).
            if (front_blocked or center_blocked) and not (left_blocked or right_blocked):
                # Decidimos el lado libre alejándonos del obstáculo usando su ángulo exacto
                # lidar_closest_angle > 0.05 significa que el objeto está a la izquierda (en ROS std)
                # Por ende, debemos evadir hacia la DERECHA -> evade_side='left' (steer negativo)
                # lidar_closest_angle < -0.05 significa que el objeto está a la derecha
                # Por ende, debemos evadir hacia la IZQUIERDA -> evade_side='right' (steer positivo)
                if abs(self.lidar_closest_angle) > 0.05:
                    if self.lidar_closest_angle > 0.0:
                        self.evade_side = 'left'    # esquiva a la derecha (steer -)
                    else:
                        self.evade_side = 'right'   # esquiva a la izquierda (steer +)
                else:
                    # Si está centrado en ±3 grados, desempatamos usando la línea amarilla en la cámara
                    if self.yellow_found and self.target_valid:
                        if self.target_x > 0.0:
                            # Línea a la derecha -> carril libre a la izquierda -> esquiva izq (steer +)
                            self.evade_side = 'right'
                        else:
                            # Línea a la izquierda -> carril libre a la derecha -> esquiva der (steer -)
                            self.evade_side = 'left'
                    else:
                        # Fallback final: decidir por el sector de LIDAR más despejado
                        if self.lidar_left > self.lidar_right:
                            self.evade_side = 'right'   # esquiva por la izq
                        else:
                            self.evade_side = 'left'    # esquiva por la der

                self.evade_state = 'STEER_AWAY'
                self.steer_away_start_t = now
                # Caera al bloque STEER_AWAY arriba en el proximo ciclo.
                # Devuelve un comando inicial de fase 1 (rodeo).
                if self.evade_side == 'right':
                    return +abs(away_speed), +away_steer
                else:
                    return +abs(away_speed), -away_steer

            # Caso B: obstaculos a ambos lados (callejon sin salida)
            # → STOP duro. El usuario rescata con teleop.
            # REVERSING desactivado: no retrocedemos nunca.
            if left_blocked and right_blocked:
                return 0.0, 0.0

            # Caso C: obstaculo solo en un lado (o lado + frente)
            # → REVERSING desactivado. En su lugar: STEER_AWAY directo
            # en movimiento, esquivando hacia el lado libre.
            # El lado del obstaculo es el bloqueado; el rebase pasa por
            # el lado contrario (que esta libre).
            if left_blocked:
                self.evade_side = 'left'
            else:
                self.evade_side = 'right'

            self.evade_state = 'STEER_AWAY'
            self.steer_away_start_t = now
            return +abs(away_speed), _away_steer_phase1()

        return None, None

    def _lidar_safety_factor(self):
        """Devuelve factor multiplicador de velocidad segun LiDAR.
        1.0 = OK, slow_factor = lento por proximidad, 0.0 = freno total.

        FRONTAL: aplica slow + stop completos (slow_d=0.80, stop_d=0.35).
        LATERALES: aplican SOLO slow con su propia zona MUCHO mas corta
        (slow_d_side=0.45). Muros de pista (~0.7 m) no disparan; solo
        objetos MUY cercanos al lado.
        TRASERO: STOP duro si algo entra en el cono trasero a menos de
        back_stop_d (default 0.45m). Proteccion anticolision por detras.

        Si is_wall=True (lidar viendo pared/vallas envolventes), se
        ignora todo (factor=1.0).

        Las zonas NO designadas (fuera de los conos front/left/right/back)
        se IGNORAN completamente — no influyen en la velocidad.

        Los umbrales se leen del parametro en CADA llamada para que
        `ros2 param set` haga efecto en runtime."""
        use_safety    = bool(self.get_parameter('use_lidar_safety').value)
        stop_d        = float(self.get_parameter('lidar_stop_distance').value)
        slow_d        = float(self.get_parameter('lidar_slow_distance').value)
        slow_d_side   = float(self.get_parameter('lidar_slow_distance_side').value)
        back_stop_d   = float(self.get_parameter('lidar_back_stop_distance').value)
        safety_r      = float(self.get_parameter('lidar_safety_circle_radius').value)
        safety_slow_r = float(self.get_parameter('lidar_safety_circle_slow_radius').value)
        slow_factor   = float(self.get_parameter('lidar_slow_factor').value)
        timeout_s     = float(self.get_parameter('lidar_timeout_s').value)

        if not use_safety:
            return 1.0
        if (time.time() - self.last_lidar_time) > timeout_s:
            return 1.0

        # Factor acumulado (1.0 = sin freno). Cada chequeo lo baja con min().
        factor = 1.0

        # Si detectamos pared envolvente (vallas) y ignore_walls esta activo, o si estamos
        # tomando una curva cerrada (girando fuerte), o si hemos perdido la línea y estamos
        # navegando en modo de gracia (ciegos), desactivamos/omitimos el LIDAR por completo.
        is_turning_hard = abs(self.last_steer_cmd) > 0.15
        line_lost_grace = self.running and not self.target_valid
        ignore_active = bool(self.get_parameter('ignore_walls').value) and (
            self.lidar_is_wall or is_turning_hard or line_lost_grace
        )

        if not ignore_active:
            # Frontal STOP
            if self.lidar_front < stop_d:
                return 0.0
            # Frontal SLOW
            if self.lidar_front < slow_d:
                factor = min(factor, slow_factor)

            # Trasero STOP
            if self.lidar_back < back_stop_d:
                return 0.0
            # Laterales SLOW
            if self.lidar_left < slow_d_side or self.lidar_right < slow_d_side:
                factor = min(factor, slow_factor)

            # ── CIRCULOS DE SEGURIDAD (validados solo si no estamos ignorando vallas/curvas) ──
            if safety_slow_r > 0.0 and self.lidar_min_360 < safety_slow_r:
                factor = min(factor, slow_factor)
            if safety_r > 0.0 and self.lidar_min_360 < safety_r:
                return 0.0

        return factor

    @staticmethod
    def clamp(value, lo, hi):
        return max(min(value, hi), lo)

    def _publish_motion(self, speed, steering_angle):
        """Publica un comando de motor con (speed, steering_angle) ya calculados.
        Aplica el steering_sign y registra los valores para el HUD."""
        steering_angle = self.clamp(
            steering_angle, -self.max_steering_angle, self.max_steering_angle)
        steering_cmd = self.steering_sign * steering_angle
        self.last_steer_cmd = steering_cmd
        self.last_speed_command = speed
        if self.platform == 'qcar':
            cmd = Vector3Stamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.vector.x = float(speed)
            cmd.vector.y = float(steering_cmd)
            self.cmd_pub.publish(cmd)
        else:
            cmd = self._MotorCommands()
            cmd.motor_names = ["steering_angle", "motor_throttle"]
            cmd.values = [float(steering_cmd), float(speed)]
            self.cmd_pub.publish(cmd)

    def publish_stop(self):
        if self.platform == 'qcar':
            cmd = Vector3Stamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.vector.x = 0.0
            cmd.vector.y = 0.0
            self.cmd_pub.publish(cmd)
        else:
            cmd = self._MotorCommands()
            cmd.motor_names = ["steering_angle", "motor_throttle"]
            cmd.values = [0.0, 0.0]
            self.cmd_pub.publish(cmd)
        self.last_speed_command = 0.0

    # ─────────────────────────────────────────────────────────────────────────
    def target_callback(self, msg):
        data = msg.data
        tx = float(data[0]) if len(data) >= 2 else -1.0
        ty = float(data[1]) if len(data) >= 2 else -1.0
        self.target_x = tx
        self.target_y = ty
        self.target_valid = (len(data) >= 2) and (ty > 0.0)
        self.last_target_time = time.time()
        self.last_target_x = tx

        if self.target_valid:
            self.last_yellow_side = 'right' if tx > 0 else 'left'
            self.last_valid_target_t = self.last_target_time

    # ─────────────────────────────────────────────────────────────────────────
    def control_loop(self):
        """Bucle principal de control que corre a 50Hz."""
        now = time.time()

        # DEBUG LOGGER (para diagnóstico remoto de variables)
        if not hasattr(self, '_last_debug_log_t'):
            self._last_debug_log_t = 0.0
        if now - self._last_debug_log_t > 0.5:
            self._last_debug_log_t = now
            try:
                last_valid_dt = now - self.last_valid_target_t if self.last_valid_target_t > 0.0 else 999.0
                with open('/home/nvidia/roboracer_debug.log', 'a') as f_log:
                    f_log.write(
                        f"[{now:.3f}] running={self.running} "
                        f"target_valid={self.target_valid} "
                        f"target_x={self.target_x:.3f} "
                        f"target_y={self.target_y:.3f} "
                        f"last_target_dt={now - self.last_target_time:.3f}s "
                        f"last_valid_dt={last_valid_dt:.3f}s "
                        f"evade_state={self.evade_state} "
                        f"lidar_front={self.lidar_front:.3f}\n"
                    )
            except Exception:
                pass

        if not self.running:
            return

        # FASE 2: EMA sobre lidar_min para suavizar la sigmoid de fusion.
        # Se actualiza siempre (cuesta nada) — la fusion lo lee solo si flag.
        ema_a = float(self.get_parameter('lidar_min_ema_alpha').value)
        ema_a = max(0.0, min(1.0, ema_a))
        if math.isinf(self.lidar_min_smoothed) and math.isfinite(self.lidar_min_360):
            self.lidar_min_smoothed = self.lidar_min_360
        elif math.isfinite(self.lidar_min_360):
            self.lidar_min_smoothed = (ema_a * self.lidar_min_360 +
                                       (1.0 - ema_a) * self.lidar_min_smoothed)
        # EMA sobre kappa (Contribucion C)
        k_ema_a = float(self.get_parameter('curv_kappa_ema_alpha').value)
        k_ema_a = max(0.0, min(1.0, k_ema_a))
        curv_to = float(self.get_parameter('curv_timeout_s').value)
        if (self.last_curvature_time > 0.0 and
                (now - self.last_curvature_time) < curv_to):
            self.kappa_smoothed = (k_ema_a * self.lane_curvature +
                                   (1.0 - k_ema_a) * self.kappa_smoothed)
        else:
            # Sin curvatura fresca → asumir recta (decay a 0)
            self.kappa_smoothed *= (1.0 - k_ema_a)

        # Watchdog: si no hemos recibido ningún target o el último es muy viejo (> 0.60s)
        # y no estamos en un estado de evasion (donde ignoramos la camara temporalmente)
        is_evading = (self.evade_state != 'NORMAL')
        if (self.last_target_time == 0.0 or (now - self.last_target_time) > 0.60) and not is_evading:
            self.publish_stop()
            self.get_logger().error(
                f"WATCHDOG TRIGGERED: No camera target received for {((now - self.last_target_time) if self.last_target_time > 0.0 else 0.0)*1000:.0f} ms. Stopping car!",
                throttle_duration_sec=1.0
            )
            return

        stop_d_check = float(self.get_parameter('lidar_stop_distance').value)

        # ── EVASION: tiene prioridad sobre el seguimiento de carril
        # FASE 2: detectamos transicion de evasion a NORMAL para arrancar el
        # cooldown que evita ping-pong entre fusion y maquina vieja.
        prev_evade = self.evade_state
        ev_speed, ev_steer = self._evade_action(stop_d=stop_d_check)
        if ev_speed is not None:
            if bool(self.get_parameter('evade_invert_sign').value) and ev_steer is not None:
                ev_steer = -ev_steer
            self.get_logger().warn(
                f'EVADE {self.evade_state} (side={self.evade_side}) '
                f'yellow_side={self.last_yellow_side} '
                f'L:{self.lidar_left:.2f} F:{self.lidar_front:.2f} '
                f'R:{self.lidar_right:.2f} found={self.yellow_found} '
                f'→ spd={ev_speed:+.3f} '
                f'steer={math.degrees(ev_steer):+.1f}°',
                throttle_duration_sec=0.5,
            )
            self._publish_motion(ev_speed, ev_steer)
            self._last_evade_state = self.evade_state
            return
        # Evasion devolvio (None, None) — estamos en NORMAL ahora.
        # Si veniamos de evasion, arrancar cooldown.
        if prev_evade != 'NORMAL' and self.evade_state == 'NORMAL':
            cooldown_s = float(self.get_parameter('fusion_evasion_cooldown_s').value)
            self.evasion_cooldown_until = now + cooldown_s
        self._last_evade_state = self.evade_state

        # Si no hay target válido en este instante (pero el watchdog no ha expirado todavía,
        # o estamos dentro de la ventana de gracia)
        target_valid = self.target_valid
        target_x = self.target_x
        target_y = self.target_y

        if not target_valid:
            # Ventana de gracia: si perdimos la linea hace menos de line_loss_grace_s,
            # seguir avanzando con el ULTIMO steering conocido a velocidad reducida.
            grace_s = float(self.get_parameter('line_loss_grace_s').value)
            elapsed = now - self.last_valid_target_t
            if grace_s > 0.0 and elapsed < grace_s and self.last_valid_target_t > 0.0:
                factor = float(self.get_parameter('line_loss_speed_factor').value)
                speed_straight = float(self.get_parameter('speed_straight').value)
                safety = self._lidar_safety_factor()
                grace_speed = speed_straight * factor * safety
                # Deshacer el steering_sign para pasarlo a _publish_motion
                last_angle = self.last_steer_cmd / self.steering_sign \
                    if self.steering_sign != 0.0 else self.last_steer_cmd
                self.get_logger().warn(
                    f'LINE LOSS GRACE — sin linea hace {elapsed*1000:.0f} ms, '
                    f'manteniendo steer={math.degrees(last_angle):+.1f}° '
                    f'spd={grace_speed:+.3f}',
                    throttle_duration_sec=0.2,
                )
                self._publish_motion(grace_speed, last_angle)
                return
            # Fuera de ventana: STOP
            self.publish_stop()
            return

        # ── Esquiva PREEMPTIVA: si hay obstaculo en zona lejana, sesga el target_x
        # FASE 2: cuando fusion_enable=True, deshabilitamos el preempt para
        # evitar doble correccion lateral (preempt sobre target_x + fusion
        # sobre steering). La fusion ya integra la senal del LIDAR.
        fusion_enable = bool(self.get_parameter('fusion_enable').value)
        if not fusion_enable:
            stop_d_pp = float(self.get_parameter('lidar_stop_distance').value)
            preempt_bias = self._preempt_bias(stop_d=stop_d_pp)
            if preempt_bias != 0.0:
                target_x = target_x + preempt_bias
                self.last_target_x = target_x   # actualizar HUD
                depth_fresh = (time.time() - self.last_depth_time) <= float(
                    self.get_parameter('depth_timeout_s').value)
                depth_tag = ' [ZED]' if (depth_fresh and self.depth_obstacle) else ''
                self.get_logger().info(
                    f'PREEMPT bias={preempt_bias:+.3f}{depth_tag} '
                    f'L:{self.lidar_left:.2f} F:{self.lidar_front:.2f} '
                    f'R:{self.lidar_right:.2f} '
                    f'ZEDd:{self.depth_center_10deg:.2f}',
                    throttle_duration_sec=1.0,
                )

        # Lectura en vivo de parámetros para permitir tuneo en caliente (hot-tuning)
        lookahead_min = float(self.get_parameter('lookahead_min').value)
        lookahead_max = float(self.get_parameter('lookahead_max').value)
        lookahead_base = float(self.get_parameter('lookahead_base').value)
        lookahead_speed_gain = float(self.get_parameter('lookahead_speed_gain').value)

        # Lookahead efectivo: combina distancia geometrica y velocidad
        lookahead_raw = math.hypot(target_x, target_y)
        lookahead_speed = self.clamp(
            lookahead_base + lookahead_speed_gain * abs(self.last_speed_command),
            lookahead_min,
            lookahead_max,
        )
        lookahead = self.clamp(
            min(lookahead_raw, lookahead_speed),
            lookahead_min,
            lookahead_max,
        )

        # Pure Pursuit (frame eje trasero: x lateral, y forward)
        steer_pp = math.atan2(
            2.0 * self.wheelbase * target_x,
            lookahead * lookahead,
        )
        steering_angle = steer_pp
        self.last_steer_raw = steering_angle   # para HUD

        # ── FASE 2 (Contribucion A): FUSION continua vision-LIDAR ───────────
        # Solo se aplica si fusion_enable=True (default False = regresion cero).
        # w_vision = lane_conf * sigmoid(soft * (lidar_smoothed - thresh))
        # w_lidar  = 1 - w_vision (con histeresis y cooldown post-evasion)
        # steer    = w_vision * steer_pp + w_lidar * steer_gap
        steer_gap_used = 0.0
        w_vision = 1.0
        w_lidar = 0.0
        if fusion_enable:
            soft = float(self.get_parameter('fusion_sigmoid_soft').value)
            thresh = float(self.get_parameter('fusion_sigmoid_thresh').value)
            hyst_lo = float(self.get_parameter('fusion_hysteresis_lo').value)
            hyst_hi = float(self.get_parameter('fusion_hysteresis_hi').value)
            gap_to = float(self.get_parameter('fusion_gap_timeout_s').value)
            lane_to = float(self.get_parameter('fusion_lane_conf_timeout_s').value)
            cd_floor = float(
                self.get_parameter('fusion_evasion_cooldown_w_lidar_floor').value)

            # Confianzas frescas? Si no, ponderar a 0 ese canal.
            lane_conf = self.lane_confidence if (
                self.last_lane_conf_time > 0.0 and
                (now - self.last_lane_conf_time) < lane_to) else 0.0
            gap_conf = self.gap_conf if (
                self.last_gap_time > 0.0 and
                (now - self.last_gap_time) < gap_to) else 0.0

            # Sigmoid sobre lidar_min_smoothed. Cuando lidar << thresh →
            # sigmoid(neg) ~ 0 → w_vision baja → toma control el LIDAR.
            d = self.lidar_min_smoothed if math.isfinite(self.lidar_min_smoothed) else 5.0
            x = soft * (d - thresh)
            # clamp para evitar overflow en exp
            x = max(-50.0, min(50.0, x))
            sig = 1.0 / (1.0 + math.exp(-x))
            w_vision_raw = float(max(0.0, min(1.0, lane_conf * sig)))

            # Histeresis: estado binario suave usando los thresholds
            if self.w_vision_state >= hyst_hi:
                # Estabamos dominados por vision; bajar solo si cae bajo lo
                if w_vision_raw < hyst_lo:
                    self.w_vision_state = w_vision_raw
                else:
                    self.w_vision_state = max(hyst_hi, w_vision_raw)
            else:
                # Estabamos en zona LIDAR; subir solo si sube sobre hi
                if w_vision_raw > hyst_hi:
                    self.w_vision_state = w_vision_raw
                else:
                    self.w_vision_state = min(hyst_lo, w_vision_raw)
            w_vision = float(self.w_vision_state)
            w_lidar = 1.0 - w_vision

            # Cooldown post-evasion: forzar piso w_lidar
            if now < self.evasion_cooldown_until:
                if w_lidar < cd_floor:
                    w_lidar = cd_floor
                    w_vision = 1.0 - w_lidar

            # Steering del gap (clampearlo por su propio limite no necesario,
            # gap_follower ya clampea). Si no hay gap fresco, no aporta.
            steer_gap_used = self.gap_steer if gap_conf > 0.0 else 0.0

            steering_angle = w_vision * steer_pp + w_lidar * steer_gap_used

        # ── FASE 2 (Contribucion C): CURVATURE-ADAPTIVE SPEED ───────────────
        # Solo si curvature_speed_enable=True. Formula:
        #   v = clip(v_max / (1 + alpha*|kappa|), v_min, v_max)
        # Luego blend con distancia LIDAR para frenar si hay obstaculo cerca:
        #   dist_factor = clip((lidar_smoothed - lo) / (hi - lo), 0, 1)
        #   v_final = v_target * dist_factor + v_low_dist * (1 - dist_factor)
        curvature_speed_enable = bool(
            self.get_parameter('curvature_speed_enable').value)
        v_target = 0.0
        if curvature_speed_enable:
            v_max = float(self.get_parameter('curv_v_max').value)
            v_min = float(self.get_parameter('curv_v_min').value)
            alpha = float(self.get_parameter('curv_alpha').value)
            blend_lo = float(self.get_parameter('curv_dist_blend_lo').value)
            blend_hi = float(self.get_parameter('curv_dist_blend_hi').value)
            v_low_dist = float(self.get_parameter('curv_v_low_dist').value)

            kappa_abs = abs(self.kappa_smoothed)
            v_target = v_max / (1.0 + alpha * kappa_abs)
            v_target = max(v_min, min(v_max, v_target))

            d_for_blend = (self.lidar_min_smoothed
                           if math.isfinite(self.lidar_min_smoothed) else 5.0)
            if blend_hi > blend_lo:
                dist_factor = (d_for_blend - blend_lo) / (blend_hi - blend_lo)
                dist_factor = max(0.0, min(1.0, dist_factor))
            else:
                dist_factor = 1.0
            speed = v_target * dist_factor + v_low_dist * (1.0 - dist_factor)
        else:
            # Perfil clasico: mas lento en curvas.
            curve_thr     = float(self.get_parameter('curve_threshold').value)
            speed_curve   = float(self.get_parameter('speed_curve').value)
            speed_straight= float(self.get_parameter('speed_straight').value)
            speed = speed_curve if abs(steering_angle) > curve_thr else speed_straight

        steering_gain = float(self.get_parameter('steering_gain').value)

        # Safety LiDAR: aplica factor de freno por proximidad (override final)
        safety = self._lidar_safety_factor()
        speed *= safety
        if safety == 0.0:
            self.get_logger().warn(
                f'LiDAR STOP — F:{self.lidar_front:.2f} B:{self.lidar_back:.2f} '
                f'360:{self.lidar_min_360:.2f} m',
                throttle_duration_sec=1.0,
            )

        # FEEDBACK IMU: PI sobre velocidad angular
        omega_correction = self._imu_correction(steering_angle, speed)
        steering_angle = steering_angle + omega_correction

        # GANANCIA: amplifica steering
        steering_angle = steering_angle * steering_gain

        # Clamp final
        steering_angle = self.clamp(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle,
        )

        # Publicar movimiento
        self._publish_motion(speed, steering_angle)

        # ── FASE 2: Telemetria JSON (rate-limited a telemetry_rate_hz) ──────
        # Se publica SIEMPRE (no detras de flag) para que el experimento E1
        # (sistema actual) tambien quede registrado. Pero a 20 Hz, no 50.
        self._maybe_publish_telemetry(
            now=now,
            steer_pp=steer_pp,
            steer_gap=steer_gap_used,
            steer_final=steering_angle,
            w_vision=w_vision,
            w_lidar=w_lidar,
            v_target=v_target,
            v_final=speed,
            safety=safety,
            fusion_enable=fusion_enable,
            curvature_speed_enable=curvature_speed_enable,
        )


def _stdin_is_tty():
    return sys.stdin.isatty()


def _get_key(timeout=0.1):
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def _keyboard_loop(node):
    """Loop de teclado — solo corre si hay TTY real (ros2 run, no launch).

    Teclas en el panel del FOLLOWER:
      ENTER   ARRANCAR
      Q       PARAR (segunda Q = salir del proceso)

    SPACE NO arranca el follower aqui — esta reservado al panel del TELEOP
    como "freno + auto OFF" (parada de emergencia remota).
    """
    print('\n  Presiona ENTER para ARRANCAR  |  Q para PARAR\n',
          flush=True)
    try:
        while rclpy.ok():
            key = _get_key(timeout=0.2)
            if key is None:
                continue
            if key in ('\r', '\n'):
                node.running = True
                print('\n  ▶  ARRANCANDO...', flush=True)
            elif key in ('q', 'Q', '\x03'):
                if node.running:
                    node.running = False
                    node.publish_stop()
                    print('\n  ■  PARADO. Q de nuevo para salir.', flush=True)
                else:
                    break
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerPP()

    if _stdin_is_tty():
        # Correr desde terminal real (ros2 run): habilita control por teclado
        ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        ros_thread.start()
        try:
            _keyboard_loop(node)
        except KeyboardInterrupt:
            pass
    else:
        # Correr desde launch (sin TTY): arranca automaticamente sin teclado
        node.running = True
        node.get_logger().info('Sin TTY — arrancando automaticamente (modo launch)')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.running = False
    node.publish_stop()
    print('\n  Coche parado.', flush=True)
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
