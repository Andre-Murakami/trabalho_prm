#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

import cv2
import math
import numpy as np


# =========================================================
# CONFIGURAÇÕES
# =========================================================

LABEL_BANDEIRA_AZUL = 25

VEL_LINEAR_MAX = 0.22
VEL_LINEAR_CURVA = 0.12

VEL_ANGULAR_MAX = 0.75

DISTANCIA_SEGURA = 1.00
DISTANCIA_CRITICA = 0.50
DISTANCIA_ESCAPE = 0.70

DISTANCIA_PARADA = 0.70

LARGURA_IMAGEM = 640

KP_CAMERA = 1.2

AREA_MINIMA = 40

# quantidade de ciclos preso para ativar escape
LIMITE_LOOP = 18

# =========================================================


class MissaoControle(Node):

    def __init__(self):

        super().__init__('missao_controle')

        # =================================================
        # ROS
        # =================================================

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

        self.bridge = CvBridge()

        # =================================================
        # VISÃO
        # =================================================

        self.bandeira_visivel = False
        self.bandeira_offset = 0.0

        # =================================================
        # LIDAR
        # =================================================

        self.dist_frente = 999.0
        self.dist_fesq = 999.0
        self.dist_fdir = 999.0

        self.dist_esquerda = 999.0
        self.dist_direita = 999.0

        # =================================================
        # ANTI LOOP
        # =================================================

        self.ticks_preso = 0

        self.ultimo_lado = 1

        self.modo_escape = False
        self.escape_ticks = 0

        # =================================================

        self.create_timer(0.1, self.tick)

        self.get_logger().info("MISSAO INICIADA")

    # =====================================================
    # LIDAR
    # =====================================================

    def cb_lidar(self, msg):

        ranges = list(msg.ranges)

        n = len(ranges)

        def setor(a, b):

            if a <= b:
                idx = range(a, b)
            else:
                idx = list(range(a, n)) + list(range(0, b))

            vals = []

            for i in idx:

                d = ranges[i]

                if math.isinf(d):
                    continue

                if math.isnan(d):
                    continue

                if d < 0.05:
                    continue

                vals.append(d)

            if len(vals) == 0:
                return 999.0

            # =================================================
            # INFLAÇÃO DO OBSTÁCULO
            # protege rodas e laterais
            # =================================================

            return min(vals) - 0.22

        # =====================================================
        # SETORES MAIS LARGOS
        # footprint mais realista
        # =====================================================

        self.dist_frente = setor(330, 30)

        self.dist_fesq = setor(20, 80)
        self.dist_fdir = setor(280, 340)

        self.dist_esquerda = setor(80, 140)
        self.dist_direita = setor(220, 280)

    # =====================================================
    # CAMERA
    # =====================================================

    def cb_camera(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='mono8'
            )

        except:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            frame = frame[:, :, 0]

        mask = cv2.inRange(
            frame,
            LABEL_BANDEIRA_AZUL,
            LABEL_BANDEIRA_AZUL
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        self.bandeira_visivel = False

        if len(contours) == 0:
            return

        maior = max(contours, key=cv2.contourArea)

        area = cv2.contourArea(maior)

        if area < AREA_MINIMA:
            return

        M = cv2.moments(maior)

        if M["m00"] <= 0:
            return

        cx = int(M["m10"] / M["m00"])

        self.bandeira_offset = (
            (cx - LARGURA_IMAGEM / 2)
            / (LARGURA_IMAGEM / 2)
        )

        self.bandeira_visivel = True

    # =====================================================
    # ESCOLHE MELHOR LADO
    # =====================================================

    def melhor_lado(self):

        margem = 0.15

        if self.dist_direita > self.dist_esquerda + margem:
            return -1

        elif self.dist_esquerda > self.dist_direita + margem:
            return 1

        return self.ultimo_lado

    # =====================================================
    # DETECTA LOOP
    # =====================================================

    def detectar_loop(self):

        corredor_esq = self.dist_fesq > DISTANCIA_ESCAPE
        corredor_dir = self.dist_fdir > DISTANCIA_ESCAPE

        if not corredor_esq and not corredor_dir:

            self.ticks_preso += 1

        else:

            self.ticks_preso = 0

        return self.ticks_preso > LIMITE_LOOP

    # =====================================================
    # ESCAPE
    # =====================================================

    def executar_escape(self):

        t = Twist()

        self.escape_ticks += 1

        # recua primeiro
        if self.escape_ticks < 8:

            t.linear.x = -0.10
            t.angular.z = 0.0

        # depois faz curva larga
        else:

            t.linear.x = 0.08
            t.angular.z = 0.75 * self.ultimo_lado

        # finaliza escape
        if self.escape_ticks > 22:

            self.escape_ticks = 0
            self.modo_escape = False
            self.ticks_preso = 0

        return t

    # =====================================================
    # LOOP PRINCIPAL
    # =====================================================

    def tick(self):

        t = Twist()

        # =================================================
        # MODO ESCAPE
        # =================================================

        if self.modo_escape:

            t = self.executar_escape()

            self.pub_cmd.publish(t)

            return

        # =================================================
        # DETECTA LOOP
        # =================================================

        if self.detectar_loop():

            self.get_logger().info("ESCAPE MODE")

            self.modo_escape = True

            self.pub_cmd.publish(Twist())

            return

        # =================================================
        # EMERGÊNCIA
        # =================================================

        if self.dist_frente < DISTANCIA_CRITICA:

            lado = self.melhor_lado()

            self.ultimo_lado = lado

            t.linear.x = -0.08
            t.angular.z = 0.7 * lado

            self.pub_cmd.publish(t)

            return

        # =================================================
        # CORREDORES
        # =================================================

        corredor_esq = self.dist_fesq > DISTANCIA_SEGURA
        corredor_dir = self.dist_fdir > DISTANCIA_SEGURA

        # =================================================
        # CAMINHO LIVRE
        # =================================================

        if corredor_esq and corredor_dir:

            t.linear.x = VEL_LINEAR_MAX

            # objetivo global:
            # seguir reto para região inimiga

            objetivo = 0.0

            # se vê bandeira, corrige suavemente
            if self.bandeira_visivel:

                objetivo = -self.bandeira_offset * KP_CAMERA

            # steering suave
            t.angular.z = objetivo

        # =================================================
        # BLOQUEIO ESQUERDA
        # =================================================

        elif not corredor_esq and corredor_dir:

            self.ultimo_lado = -1

            t.linear.x = VEL_LINEAR_CURVA
            t.angular.z = -0.45

        # =================================================
        # BLOQUEIO DIREITA
        # =================================================

        elif corredor_esq and not corredor_dir:

            self.ultimo_lado = 1

            t.linear.x = VEL_LINEAR_CURVA
            t.angular.z = 0.45

        # =================================================
        # CORREDOR ESTREITO
        # =================================================

        else:

            lado = self.melhor_lado()

            self.ultimo_lado = lado

            t.linear.x = 0.06
            t.angular.z = 0.60 * lado

        # =================================================
        # LIMITADORES
        # =================================================

        t.angular.z = max(
            -VEL_ANGULAR_MAX,
            min(VEL_ANGULAR_MAX, t.angular.z)
        )

        self.pub_cmd.publish(t)


# =========================================================
# MAIN
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = MissaoControle()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
