#!/usr/bin/env python3

"""
missao_controle.py
SSC0712 - Programação de Robôs Móveis
Pacote: missao_bandeira

VERSÃO RECONSTRUÍDA:
- Arquivo único
- Máquina de estados inteligente
- Navegação heurística
- Uso da semântica do SOLO
- Busca focada no campo azul
- Anti-loop
- Recovery
- Aproximação visual robusta
- Sem SLAM pesado
- Sem exploração aleatória burra
"""

import math
from enum import Enum, auto

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan


# ============================================================
# CONFIGURAÇÕES
# ============================================================

LABEL_BANDEIRA_AZUL = 25

VEL_LINEAR = 0.22
VEL_LINEAR_LENTA = 0.10

VEL_ANGULAR = 0.8

DIST_CRITICA = 0.42
DIST_SEGURA = 0.80

KP_CAMERA = 1.5

AREA_MIN_BANDEIRA = 120

IMG_W = 640
IMG_H = 480

TICK = 0.1

TEMPO_SWEEP = 45

AREA_CHAO_AZUL_MIN = 14000

# ============================================================
# ESTADOS
# ============================================================


class Estado(Enum):

    INDO_PARA_AREA_AZUL = auto()

    BUSCANDO_BANDEIRA = auto()

    CENTRALIZANDO_BANDEIRA = auto()

    APROXIMANDO_BANDEIRA = auto()

    POSICIONADO = auto()

    RECOVERY = auto()


# ============================================================
# NÓ PRINCIPAL
# ============================================================


class MissaoControle(Node):

    def __init__(self):

        super().__init__('missao_controle')

        # ====================================================
        # ROS
        # ====================================================

        self.pub_cmd = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.cb_lidar,
            10
        )

        self.create_subscription(
            Image,
            '/robot_cam/labels_map',
            self.cb_camera,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.cb_odom,
            10
        )

        self.bridge = CvBridge()

        # ====================================================
        # ESTADO
        # ====================================================

        self.estado = Estado.INDO_PARA_AREA_AZUL

        # ====================================================
        # LIDAR
        # ====================================================

        self.frente = 999.0
        self.esq = 999.0
        self.dir = 999.0
        self.fesq = 999.0
        self.fdir = 999.0

        # ====================================================
        # CÂMERA
        # ====================================================

        self.bandeira_visivel = False
        self.bandeira_offset = 0.0
        self.bandeira_area = 0.0

        self.area_chao_azul = 0

        # ====================================================
        # ODOM
        # ====================================================

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # ====================================================
        # CONTROLE
        # ====================================================

        self.sweep_ticks = 0

        self.recovery_ticks = 0

        self.ultimo_lado = 1

        self.loop_sem_progresso = 0

        # ====================================================
        # TIMER
        # ====================================================

        self.create_timer(
            TICK,
            self.tick
        )

        self.get_logger().info(
            'MISSAO CONTROLE INICIADO'
        )

    # ========================================================
    # CALLBACK LIDAR
    # ========================================================

    def cb_lidar(self, msg):

        ranges = list(msg.ranges)

        n = len(ranges)

        if n == 0:
            return

        def setor(a, b):

            ai = int(a * n / 360)
            bi = int(b * n / 360)

            if ai <= bi:
                idx = range(ai, bi)
            else:
                idx = list(range(ai, n)) + list(range(0, bi))

            vals = []

            for i in idx:

                d = ranges[i]

                if math.isnan(d):
                    continue

                if math.isinf(d):
                    continue

                if d < 0.05:
                    continue

                vals.append(d)

            if not vals:
                return 999.0

            return min(vals)

        self.frente = setor(345, 15)

        self.fesq = setor(15, 70)

        self.fdir = setor(290, 345)

        self.esq = setor(70, 120)

        self.dir = setor(240, 290)

    # ========================================================
    # CALLBACK CAMERA
    # ========================================================

    def cb_camera(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='mono8'
            )

        except Exception:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            frame = frame[:, :, 0]

        # ====================================================
        # DETECÇÃO DA BANDEIRA
        # ====================================================

        mask_flag = cv2.inRange(
            frame,
            LABEL_BANDEIRA_AZUL,
            LABEL_BANDEIRA_AZUL
        )

        contours, _ = cv2.findContours(
            mask_flag,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        self.bandeira_visivel = False

        if contours:

            maior = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(maior)

            if area > AREA_MIN_BANDEIRA:

                M = cv2.moments(maior)

                if M['m00'] > 0:

                    cx = int(M['m10'] / M['m00'])

                    self.bandeira_offset = (
                        cx - IMG_W / 2
                    ) / (IMG_W / 2)

                    self.bandeira_area = area

                    self.bandeira_visivel = True

        # ====================================================
        # DETECÇÃO DO SOLO AZUL
        # ====================================================

        metade_inferior = frame[
            IMG_H // 2:,
            :
        ]

        mask_chao_azul = cv2.inRange(
            metade_inferior,
            LABEL_BANDEIRA_AZUL,
            LABEL_BANDEIRA_AZUL
        )

        self.area_chao_azul = cv2.countNonZero(
            mask_chao_azul
        )

    # ========================================================
    # CALLBACK ODOM
    # ========================================================

    def cb_odom(self, msg):

        p = msg.pose.pose

        self.x = p.position.x
        self.y = p.position.y

        q = p.orientation

        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (
                q.y * q.y + q.z * q.z
            )
        )

    # ========================================================
    # UTIL
    # ========================================================

    def mudar_estado(self, novo):

        if novo != self.estado:

            self.get_logger().info(
                f'{self.estado.name} -> {novo.name}'
            )

            self.estado = novo

    # ========================================================
    # RECOVERY
    # ========================================================

    def estado_recovery(self):

        t = Twist()

        self.recovery_ticks += 1

        if self.recovery_ticks < 15:

            t.linear.x = -0.12

        else:

            t.angular.z = (
                VEL_ANGULAR * self.ultimo_lado
            )

        if self.recovery_ticks > 35:

            self.recovery_ticks = 0

            self.mudar_estado(
                Estado.INDO_PARA_AREA_AZUL
            )

        return t

    # ========================================================
    # INDO PARA AREA AZUL
    # ========================================================

    def estado_indo_area_azul(self):

        t = Twist()

        # ====================================================
        # RECOVERY
        # ====================================================

        if self.frente < DIST_CRITICA:

            self.ultimo_lado = (
                1 if self.esq > self.dir else -1
            )

            self.mudar_estado(
                Estado.RECOVERY
            )

            return Twist()

        # ====================================================
        # DETECTOU AREA AZUL
        # ====================================================

        if self.area_chao_azul > AREA_CHAO_AZUL_MIN:

            self.get_logger().info(
                'AREA AZUL DETECTADA'
            )

            self.mudar_estado(
                Estado.BUSCANDO_BANDEIRA
            )

            return Twist()

        # ====================================================
        # NAVEGAÇÃO INTELIGENTE
        # ====================================================

        t.linear.x = VEL_LINEAR

        erro = self.fdir - self.fesq

        t.angular.z = 0.9 * erro

        # ====================================================
        # LIMITES
        # ====================================================

        t.angular.z = max(
            -VEL_ANGULAR,
            min(VEL_ANGULAR, t.angular.z)
        )

        # ====================================================
        # PAREDE MUITO PERTO
        # ====================================================

        if self.fesq < 0.55:

            t.angular.z -= 0.6

        if self.fdir < 0.55:

            t.angular.z += 0.6

        return t

    # ========================================================
    # BUSCANDO BANDEIRA
    # ========================================================

    def estado_busca_bandeira(self):

        t = Twist()

        # ====================================================
        # ENCONTROU BANDEIRA
        # ====================================================

        if self.bandeira_visivel:

            self.mudar_estado(
                Estado.CENTRALIZANDO_BANDEIRA
            )

            return Twist()

        # ====================================================
        # EVITA OBSTÁCULO
        # ====================================================

        if self.frente < DIST_CRITICA:

            self.ultimo_lado = (
                1 if self.esq > self.dir else -1
            )

            self.mudar_estado(
                Estado.RECOVERY
            )

            return Twist()

        # ====================================================
        # SWEEP CONTROLADO
        # ====================================================

        self.sweep_ticks += 1

        t.linear.x = 0.06

        if self.sweep_ticks < TEMPO_SWEEP:

            t.angular.z = 0.5

        else:

            t.angular.z = -0.5

        if self.sweep_ticks > TEMPO_SWEEP * 2:

            self.sweep_ticks = 0

        return t

    # ========================================================
    # CENTRALIZA
    # ========================================================

    def estado_centraliza(self):

        t = Twist()

        if not self.bandeira_visivel:

            self.mudar_estado(
                Estado.BUSCANDO_BANDEIRA
            )

            return Twist()

        erro = self.bandeira_offset

        t.angular.z = -erro * KP_CAMERA

        if abs(erro) < 0.08:

            self.mudar_estado(
                Estado.APROXIMANDO_BANDEIRA
            )

        return t

    # ========================================================
    # APROXIMA
    # ========================================================

    def estado_aproxima(self):

        t = Twist()

        if not self.bandeira_visivel:

            self.mudar_estado(
                Estado.BUSCANDO_BANDEIRA
            )

            return Twist()

        # ====================================================
        # CHEGOU
        # ====================================================

        if (
            self.frente < 0.65 and
            abs(self.bandeira_offset) < 0.12
        ):

            self.mudar_estado(
                Estado.POSICIONADO
            )

            return Twist()

        # ====================================================
        # EVITA OBSTÁCULO
        # ====================================================

        if self.frente < DIST_CRITICA:

            self.ultimo_lado = (
                1 if self.esq > self.dir else -1
            )

            self.mudar_estado(
                Estado.RECOVERY
            )

            return Twist()

        # ====================================================
        # APROXIMAÇÃO VISUAL
        # ====================================================

        t.linear.x = VEL_LINEAR_LENTA

        t.angular.z = (
            -self.bandeira_offset * KP_CAMERA
        )

        return t

    # ========================================================
    # POSICIONADO
    # ========================================================

    def estado_posicionado(self):

        t = Twist()

        return t

    # ========================================================
    # TICK
    # ========================================================

    def tick(self):

        if self.estado == Estado.INDO_PARA_AREA_AZUL:

            cmd = self.estado_indo_area_azul()

        elif self.estado == Estado.BUSCANDO_BANDEIRA:

            cmd = self.estado_busca_bandeira()

        elif self.estado == Estado.CENTRALIZANDO_BANDEIRA:

            cmd = self.estado_centraliza()

        elif self.estado == Estado.APROXIMANDO_BANDEIRA:

            cmd = self.estado_aproxima()

        elif self.estado == Estado.POSICIONADO:

            cmd = self.estado_posicionado()

        elif self.estado == Estado.RECOVERY:

            cmd = self.estado_recovery()

        else:

            cmd = Twist()

        self.pub_cmd.publish(cmd)


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = MissaoControle()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
