#!/usr/bin/env python3

import math
import cv2

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image

from geometry_msgs.msg import Twist

from cv_bridge import CvBridge


# ==========================================================
# PARAMETROS
# ==========================================================

LABEL_BANDEIRA_AZUL = 25

VEL_LINEAR = 0.18
VEL_ANGULAR = 0.60

DISTANCIA_OBSTACULO = 0.55
DISTANCIA_CRITICA = 0.30

DISTANCIA_PARADA = 0.70

LARGURA_IMAGEM = 640


# ==========================================================
# NODE
# ==========================================================

class MissaoControle(Node):

    def __init__(self):

        super().__init__('missao_controle')

        # ==================================================
        # PUB
        # ==================================================

        self.pub_cmd = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ==================================================
        # SUBS
        # ==================================================

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

        # ==================================================
        # ESTADO
        # ==================================================

        self.estado = 'EXPLORANDO'

        # ==================================================
        # LIDAR
        # ==================================================

        self.frente = 999.0
        self.esquerda = 999.0
        self.direita = 999.0

        # ==================================================
        # VISAO
        # ==================================================

        self.bandeira_visivel = False
        self.offset = 0.0
        self.area = 0.0

        # ==================================================
        # TIMER
        # ==================================================

        self.create_timer(
            0.1,
            self.tick
        )

        self.get_logger().info(
            'MISSAO CONTROLE INICIADO'
        )

    # ======================================================
    # LIDAR
    # ======================================================

    def cb_lidar(self, msg):

        ranges = list(msg.ranges)

        n = len(ranges)

        if n == 0:
            return

        def setor(grau_inicio, grau_fim):

            vals = []

            for g in range(grau_inicio, grau_fim):

                i = int((g % 360) * n / 360)

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

            return min(vals)

        self.frente = min(
            setor(330, 360),
            setor(0, 30)
        )

        self.esquerda = setor(60, 120)

        self.direita = setor(240, 300)

    # ======================================================
    # CAMERA
    # ======================================================

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

        maior = max(
            contours,
            key=cv2.contourArea
        )

        area = cv2.contourArea(maior)

        if area < 40:
            return

        M = cv2.moments(maior)

        if M['m00'] <= 0:
            return

        cx = int(M['m10'] / M['m00'])

        self.offset = (
            (cx - (LARGURA_IMAGEM / 2))
            /
            (LARGURA_IMAGEM / 2)
        )

        self.area = area

        self.bandeira_visivel = True

    # ======================================================
    # LOOP
    # ======================================================

    def tick(self):

        twist = Twist()

        # ==================================================
        # ESTADO
        # ==================================================

        if self.bandeira_visivel:

            self.estado = 'NAVEGANDO'

        else:

            self.estado = 'EXPLORANDO'

        # ==================================================
        # EXPLORAR
        # ==================================================

        if self.estado == 'EXPLORANDO':

            # perigo real

            if self.frente < DISTANCIA_CRITICA:

                twist.linear.x = -0.05

                if self.esquerda > self.direita:

                    twist.angular.z = 0.7

                else:

                    twist.angular.z = -0.7

            # obstáculo moderado

            elif self.frente < DISTANCIA_OBSTACULO:

                twist.linear.x = 0.05

                if self.esquerda > self.direita:

                    twist.angular.z = 0.5

                else:

                    twist.angular.z = -0.5

            # livre

            else:

                twist.linear.x = VEL_LINEAR

        # ==================================================
        # NAVEGAR
        # ==================================================

        else:

            # muito perto obstáculo

            if self.frente < DISTANCIA_CRITICA:

                twist.linear.x = -0.05

                if self.esquerda > self.direita:

                    twist.angular.z = 0.7

                else:

                    twist.angular.z = -0.7

            # contorno obstáculo

            elif self.frente < DISTANCIA_OBSTACULO:

                twist.linear.x = 0.06

                if self.esquerda > self.direita:

                    twist.angular.z = 0.6

                else:

                    twist.angular.z = -0.6

            else:

                # mira visual

                twist.angular.z = -self.offset * 0.9

                # desaceleração alinhamento

                alinhamento = max(
                    0.25,
                    1.0 - abs(self.offset)
                )

                twist.linear.x = (
                    VEL_LINEAR * alinhamento
                )

                # parar perto

                if (
                    self.frente < DISTANCIA_PARADA
                    and
                    abs(self.offset) < 0.10
                ):

                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                    self.get_logger().info(
                        'BANDEIRA ALCANCADA',
                        throttle_duration_sec=5.0
                    )

        # ==================================================
        # PUBLICA
        # ==================================================

        self.pub_cmd.publish(twist)


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = MissaoControle()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()
