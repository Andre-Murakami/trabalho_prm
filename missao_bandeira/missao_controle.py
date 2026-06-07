#!/usr/bin/env python3

import math
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

LABEL_BANDEIRA_AZUL = 25
VEL_LINEAR          = 0.18
VEL_ANGULAR         = 0.50
LARGURA_IMAGEM      = 320
DISTANCIA_PARADA    = 0.45
AREA_MINIMA_PARADA  = 1500   #original 800
DISTANCIA_OBSTACULO = 0.55   # detecta obstáculo a 0.55m
# Durações das fases de contorno
T_RE        = 0.35 / 0.10                      # ré 0.55m a 0.10m/s = 5.5s
T_GIRO_80   = math.radians(90) / VEL_ANGULAR   # tempo para girar 80°
T_GIRO_90   = math.radians(102) / VEL_ANGULAR   # tempo para girar 90°
T_AVANCO    = 2.0 / VEL_LINEAR                # tempo para andar 1.70m


class MissaoControle(Node):

    def __init__(self):
        super().__init__('missao_controle')

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Image,     '/robot_cam/labels_map', self.cb_camera, 10)
        self.create_subscription(LaserScan, '/scan',                 self.cb_lidar,  10)
        self.create_subscription(Imu,        '/imu',                  self.cb_imu,    10)
        self.bridge = CvBridge()

        # Câmera
        self.bandeira_visivel = False
        self.offset           = 0.0
        self.area_bandeira    = 0.0

        # LiDAR
        self.frente   = 999.0
        self.esquerda = 999.0
        self.direita  = 999.0

        # IMU — detecção de tombamento
        self.roll  = 0.0
        self.pitch = 0.0
        self.em_tombamento = False
        self.t_tombamento  = None
        LIMITE_TOMBAMENTO_GRAUS = 20.0
        self.LIMITE_TOMBAMENTO = math.radians(LIMITE_TOMBAMENTO_GRAUS)
        T_RE_TOMBAMENTO = 0.65 / 0.10   # ré 0.65m a 0.10m/s
        self.T_RE_TOMBAMENTO = T_RE_TOMBAMENTO

        # Contorno por fases
        self.fase_contorno       = 0
        self.t_fase              = None
        self.dir_desvio          = 1.0
        self.estado_anterior     = 'LINHA_RETA'
        self.tentativas_contorno = 0    # reencontros de obstáculo no avanço
        self.graus_extra         = 0    # graus extras a partir da 3ª tentativa

        self.estado = 'LINHA_RETA'
        self.get_logger().info('ESTADO: LINHA_RETA — andando em frente')

        self.create_timer(0.1, self.tick)

    def cb_lidar(self, msg):
        ranges = list(msg.ranges)
        n = len(ranges)
        if n == 0:
            return
        def setor(g0, g1):
            vals = [ranges[int((g % 360) * n / 360)] for g in range(g0, g1)]
            vals = [d for d in vals if not math.isinf(d) and not math.isnan(d) and d > 0.05]
            return min(vals) if vals else 999.0
        self.frente   = min(setor(335, 360), setor(0, 25))  # Alterado - original 330,360 e 0,30
        self.esquerda = setor(240, 300)  # LiDAR anti-horário: esquerda física = 240-300°
        self.direita  = setor(60, 120)   # LiDAR anti-horário: direita física  = 60-120°

    def cb_imu(self, msg):
        # Extrai roll e pitch do quaternion
        q = msg.orientation
        # roll (rotação em X)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self.roll = math.atan2(sinr, cosr)
        # pitch (rotação em Y)
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self.pitch = math.asin(sinp)

    def cb_camera(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        canal = frame[:, :, 0]
        mask  = cv2.inRange(canal, LABEL_BANDEIRA_AZUL, LABEL_BANDEIRA_AZUL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.bandeira_visivel = False
        self.area_bandeira    = 0.0
        if not contours:
            return
        maior = max(contours, key=cv2.contourArea)
        area  = cv2.contourArea(maior)
        if area < 30:
            return
        M = cv2.moments(maior)
        if M['m00'] <= 0:
            return
        cx = int(M['m10'] / M['m00'])
        self.offset        = (cx - LARGURA_IMAGEM / 2) / (LARGURA_IMAGEM / 2)
        self.area_bandeira = area
        self.bandeira_visivel = True

    def iniciar_contorno(self):
        agora = self.get_clock().now()

        # Diagnóstico: print dos três setores do LiDAR
        self.get_logger().info(
            f'[LIDAR] frente={self.frente:.2f}m  '
            f'esquerda={self.esquerda:.2f}m  '
            f'direita={self.direita:.2f}m'
        )

        # Decide o lado do obstáculo e gira para o lado OPOSTO
        if self.direita < self.esquerda:
            self.dir_desvio = 1.0   # obstáculo à DIREITA → gira ESQUERDA
            lado_str = 'ESQUERDA  ← (obstáculo detectado à DIREITA)'
        else:
            self.dir_desvio = -1.0  # obstáculo à ESQUERDA → gira DIREITA
            lado_str = 'DIREITA  → (obstáculo detectado à ESQUERDA)'

        self.get_logger().info(
            f'OBSTACULO DETECTADO a {self.frente:.2f}m — '
            f'girando para {lado_str}'
        )
        self.fase_contorno   = 1
        self.t_fase          = agora
        self.estado_anterior = self.estado
        self.estado          = 'CONTORNANDO'

    def tick(self):
        twist = Twist()
        agora = self.get_clock().now()

        # ── FINALIZADO ────────────────────────────────────────────────
        if self.estado == 'FINALIZADO':
            self.pub_cmd.publish(Twist())
            return

        # ── TOMBAMENTO (segunda prioridade) ──────────────────────────
        if (
            abs(self.roll)  > self.LIMITE_TOMBAMENTO
            or abs(self.pitch) > self.LIMITE_TOMBAMENTO
        ) and not self.em_tombamento and self.estado != 'FINALIZADO':
            self.em_tombamento = True
            self.t_tombamento  = agora
            self.pub_cmd.publish(Twist())
            self.get_logger().info(
                f'TOMBAMENTO DETECTADO! '
                f'roll={math.degrees(self.roll):.1f}° '
                f'pitch={math.degrees(self.pitch):.1f}° — dando ré'
            )

        if self.em_tombamento:
            dt = (agora - self.t_tombamento).nanoseconds / 1e9
            if dt < self.T_RE_TOMBAMENTO:
                twist.linear.x  = -0.10
                twist.angular.z =  0.0
                self.get_logger().info(
                    f'TOMBAMENTO: ré ({dt:.1f}/{self.T_RE_TOMBAMENTO:.1f}s)',
                    throttle_duration_sec=0.5
                )
                self.pub_cmd.publish(twist)
                return
            else:
                self.get_logger().info('Ré concluída — iniciando contorno')
                self.em_tombamento = False
                self.iniciar_contorno()
                return

        # ── PARADA FINAL (prioridade máxima) ──────────────────────────
        if (
            self.bandeira_visivel
            and self.frente        <= DISTANCIA_PARADA
            and self.area_bandeira >= AREA_MINIMA_PARADA
            and abs(self.offset)   <  0.25
        ):
            self.pub_cmd.publish(Twist())
            self.get_logger().info(
                f'PARADO NA FRENTE DA BANDEIRA! '
                f'dist={self.frente:.2f}m area={self.area_bandeira:.0f}px'
            )
            self.estado = 'FINALIZADO'
            return

        # ── BANDEIRA SUSPENDE CONTORNO NAS FASES 1 E 2 ──────────────
        if (
            self.bandeira_visivel
            and self.estado == 'CONTORNANDO'
            and self.fase_contorno in (1, 2)
        ):
            self.get_logger().info('Bandeira durante contorno fases 1/2 → NAVEGANDO')
            self.fase_contorno = 0; self.tentativas_contorno = 0; self.graus_extra = 0
            self.estado = 'NAVEGANDO'

        # ── ACIONA CONTORNO ───────────────────────────────────────────
        # Só aciona se NÃO for a bandeira grande na frente
        e_a_bandeira = self.bandeira_visivel and self.area_bandeira >= AREA_MINIMA_PARADA
        if (
            self.frente < DISTANCIA_OBSTACULO
            and not e_a_bandeira
            and self.estado != 'CONTORNANDO'
        ):
            self.iniciar_contorno()

        # ── LINHA_RETA → NAVEGANDO ────────────────────────────────────
        if self.bandeira_visivel and self.estado == 'LINHA_RETA':
            self.estado = 'NAVEGANDO'
            self.get_logger().info(
                f'BANDEIRA AZUL IDENTIFICADA! '
                f'offset={self.offset:.2f} dist={self.frente:.2f}m '
                f'area={self.area_bandeira:.0f}px — ESTADO: NAVEGANDO'
            )

        # ══════════════════════════════════════════════════════════════
        # ESTADOS
        # ══════════════════════════════════════════════════════════════

        if self.estado == 'LINHA_RETA':
            twist.linear.x  = VEL_LINEAR
            twist.angular.z = 0.0
            self.get_logger().info(
                'LINHA_RETA — andando para frente',
                throttle_duration_sec=2.0
            )

        elif self.estado == 'CONTORNANDO':
            dt = (agora - self.t_fase).nanoseconds / 1e9

            if self.fase_contorno == 1:
                # Fase 1: ré 0.55m
                if dt < T_RE:
                    twist.linear.x  = -0.10
                    twist.angular.z = 0.0
                    self.get_logger().info(
                        f'CONTORNO fase 1/4: ré 0.55m '
                        f'({dt:.1f}/{T_RE:.1f}s)',
                        throttle_duration_sec=0.5
                    )
                else:
                    self.fase_contorno = 2
                    self.t_fase = agora
                    self.get_logger().info('CONTORNO fase 2/4: girando 80°')

            elif self.fase_contorno == 2:
                # Fase 2: gira 80° + graus_extra adaptativo
                t_giro_adaptativo = T_GIRO_80 + math.radians(self.graus_extra) / VEL_ANGULAR
                graus_total = 90 + self.graus_extra
                if dt < t_giro_adaptativo:
                    twist.linear.x  = 0.0
                    twist.angular.z = self.dir_desvio * VEL_ANGULAR
                    self.get_logger().info(
                        f'CONTORNO fase 2/4: girando {graus_total}° '
                        f'({dt:.1f}/{t_giro_adaptativo:.1f}s)',
                        throttle_duration_sec=0.5
                    )
                else:
                    self.fase_contorno = 3
                    self.t_fase = agora
                    self.get_logger().info('CONTORNO fase 3/4: avançando 1.70m')

            elif self.fase_contorno == 3:
                # Fase 3: anda 1.70m — bandeira suspende, obstáculo reinicia adaptativamente
                if self.bandeira_visivel:
                    self.get_logger().info('Bandeira durante avanço → NAVEGANDO')
                    self.fase_contorno = 0; self.tentativas_contorno = 0; self.graus_extra = 0
                    self.estado = 'NAVEGANDO'
                elif self.frente < DISTANCIA_OBSTACULO:
                    self.tentativas_contorno += 1
                    if self.tentativas_contorno >= 3:
                        self.graus_extra += 90    #evita loop iterativo de colisão em parede! Andre
                        self.get_logger().info(f'Loop! tentativa {self.tentativas_contorno} — +30° (total extra: {self.graus_extra}°)')
                    else:
                        self.get_logger().info(f'Obstáculo no avanço (tentativa {self.tentativas_contorno}) — reiniciando')
                    self.iniciar_contorno()
                elif dt < T_AVANCO:
                    twist.linear.x  = VEL_LINEAR
                    twist.angular.z = 0.0
                    self.get_logger().info(
                        f'CONTORNO fase 3/4: avançando '
                        f'({dt:.1f}/{T_AVANCO:.1f}s)',
                        throttle_duration_sec=0.5
                    )
                else:
                    self.fase_contorno = 4
                    self.t_fase = agora
                    self.get_logger().info('CONTORNO fase 4/4: girando 90° de volta')

            elif self.fase_contorno == 4:
                # Fase 4: gira 90° no sentido contrário
                if dt < T_GIRO_90:
                    twist.linear.x  = 0.0
                    twist.angular.z = -self.dir_desvio * VEL_ANGULAR
                    self.get_logger().info(
                        f'CONTORNO fase 4/4: girando 90° de volta '
                        f'({dt:.1f}/{T_GIRO_90:.1f}s)',
                        throttle_duration_sec=0.5
                    )
                else:
                    # Contorno concluído com sucesso — reseta contadores
                    self.fase_contorno       = 0
                    self.tentativas_contorno = 0
                    self.graus_extra         = 0
                    if self.bandeira_visivel:
                        self.estado = 'NAVEGANDO'
                        self.get_logger().info('Contorno concluído → NAVEGANDO')
                    else:
                        self.estado = 'LINHA_RETA'
                        self.get_logger().info('Contorno concluído → LINHA_RETA')

        elif self.estado == 'NAVEGANDO':
            # Diagnóstico contínuo dos setores laterais
            self.get_logger().info(
                f'[LIDAR] frente={self.frente:.2f}m  '
                f'esquerda={self.esquerda:.2f}m  '
                f'direita={self.direita:.2f}m',
                throttle_duration_sec=1.0
            )
            if not self.bandeira_visivel:
                twist.linear.x  = VEL_LINEAR * 0.4
                twist.angular.z = 0.0
                self.get_logger().info(
                    'Bandeira perdida — desacelerando',
                    throttle_duration_sec=1.0
                )
            else:
                twist.angular.z = -self.offset * VEL_ANGULAR
                twist.linear.x  = VEL_LINEAR * max(0.3, 1.0 - abs(self.offset))
                self.get_logger().info(
                    f'NAVEGANDO — offset={self.offset:.2f} '
                    f'dist={self.frente:.2f}m area={self.area_bandeira:.0f}px',
                    throttle_duration_sec=0.5
                )

        self.pub_cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MissaoControle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
