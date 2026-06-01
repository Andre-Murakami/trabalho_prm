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
# PARAMETROS CORRIGIDOS PARA O TAMANHO DA GARRA E ROBÔ
# ==========================================================
LABEL_BANDEIRA_AZUL = 25

VEL_LINEAR = 0.16
VEL_ANGULAR = 0.55

# Ajustados levemente para dar mais margem de manobra para a câmera
DISTANCIA_OBSTACULO = 0.80  
DISTANCIA_CRITICA = 0.50   
DISTANCIA_PARADA = 0.70    

LARGURA_IMAGEM = 640
TIMEOUT_BUSCA_CICLOS = 70 

# ==========================================================
# NODE
# ==========================================================
class MissaoControle(Node):

    def __init__(self):
        super().__init__('missao_controle')

        # Pub / Subs
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.cb_lidar, 10)
        self.create_subscription(Image, '/robot_cam/labels_map', self.cb_camera, 10)

        self.bridge = CvBridge()

        # Máquina de Estados
        self.estado = 'EXPLORANDO'

        # Setores do LiDAR (Mapeamento de 360 graus sem pontos cegos)
        self.frente = 999.0
        self.diag_esquerda = 999.0
        self.diag_direita = 999.0
        self.esquerda = 999.0
        self.direita = 999.0

        # Visão e Memória de Persistência
        self.bandeira_visivel = False
        self.offset = 0.0
        self.area = 0.0
        
        self.ultimo_offset = 0.0
        self.contador_busca = 0
        self.contador_desvio = 0  # NOVA VARIÁVEL: Evita a oscilação rápida entre estados

        # Loop de Controle (10Hz)
        self.create_timer(0.1, self.tick)
        self.get_logger().info('NÓ DE CONTROLE ATUALIZADO: PERSISTÊNCIA DE ESTADOS IMPLEMENTADA')

    # ======================================================
    # LIDAR SEM PONTOS CEGOS
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
                if i >= n:
                    continue
                d = ranges[i]
                if math.isinf(d) or math.isnan(d) or d < 0.05:
                    continue
                vals.append(d)
            return min(vals) if len(vals) > 0 else 999.0

        # Divisão matemática perfeita sem deixar nenhuma fresta de graus de fora
        self.frente = min(setor(340, 360), setor(0, 20))
        self.diag_esquerda = setor(20, 65)
        self.esquerda = setor(65, 115)
        self.direita = setor(245, 295)
        self.diag_direita = setor(295, 340)

    # ======================================================
    # PROCESSAMENTO VISUAL
    # ======================================================
    def cb_camera(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = frame[:, :, 0]

        mask = cv2.inRange(frame, LABEL_BANDEIRA_AZUL, LABEL_BANDEIRA_AZUL)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.bandeira_visivel = False

        if len(contours) == 0:
            return

        maior = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(maior)

        if area < 40:
            return

        M = cv2.moments(maior)
        if M['m00'] <= 0:
            return

        cx = int(M['m10'] / M['m00'])
        self.offset = ((cx - (LARGURA_IMAGEM / 2)) / (LARGURA_IMAGEM / 2))
        self.area = area
        self.bandeira_visivel = True

    # ======================================================
    # LOGICA DE CONTROLE (MÁQUINA DE ESTADOS)
    # ======================================================
    def tick(self):
        twist = Twist()

        # Menor distância detectada em todo o semicírculo frontal exposto
        menor_dist_frontal = min(self.frente, self.diag_esquerda, self.diag_direita)

        # --------------------------------------------------
        # ESTADO: EXPLORANDO
        # --------------------------------------------------
        if self.estado == 'EXPLORANDO':
            if self.bandeira_visivel:
                self.estado = 'NAVIGANDO_PARA_BANDEIRA'
                self.get_logger().info('Alvo visualizado! Transitando para aproximação.')
            else:
                # Desvio de obstáculos padrão com margem de segurança expandida
                if menor_dist_frontal < DISTANCIA_CRITICA:
                    twist.linear.x = -0.06
                    twist.angular.z = VEL_ANGULAR if self.esquerda > self.direita else -VEL_ANGULAR
                elif menor_dist_frontal < DISTANCIA_OBSTACULO:
                    twist.linear.x = 0.04
                    twist.angular.z = 0.5 if self.esquerda > self.direita else -0.5
                else:
                    twist.linear.x = VEL_LINEAR
                    twist.angular.z = 0.0

        # --------------------------------------------------
        # ESTADO: NAVEGANDO PARA A BANDEIRA
        # --------------------------------------------------
        elif self.estado == 'NAVIGANDO_PARA_BANDEIRA':
            if self.bandeira_visivel:
                self.ultimo_offset = self.offset  # Memoriza a posição da bandeira continuamente

            # Condição de parada segura baseada no comprimento do robô + garra
            if self.bandeira_visivel and self.frente < DISTANCIA_PARADA and abs(self.offset) < 0.18:
                self.estado = 'POSICIONANDO_PARA_COLETA'
                self.get_logger().info('Alinhamento concluído com sucesso em frente à bandeira azul!')
            
            # INTERRUPÇÃO DE SEGURANÇA: Bloqueio frontal detectado
            elif menor_dist_frontal < DISTANCIA_OBSTACULO:
                self.estado = 'DESVIANDO_OBSTACULO'
                self.contador_desvio = 0  # Inicializa o contador de persistência do desvio
                self.get_logger().warn('Interrupção por proximidade! Ativando persistência de desvio.')
            
            # Perda visual simples sem obstáculo iminente
            elif not self.bandeira_visivel:
                self.estado = 'BUSCANDO_BANDEIRA_PERDIDA'
                self.contador_busca = 0
            
            else:
                # Segue o alvo visual aplicando frenagem gradual ao se aproximar
                twist.angular.z = -self.offset * 0.85
                alinhamento = max(0.20, 1.0 - abs(self.offset))
                twist.linear.x = VEL_LINEAR * alinhamento

        # --------------------------------------------------
        # ESTADO: DESVIANDO DO OBSTÁCULO (FOGO CONTRA FOGO)
        # --------------------------------------------------
        elif self.estado == 'DESVIANDO_OBSTACULO':
            self.contador_desvio += 1  # Incrementa a cada ciclo (10Hz)

            # Só avalia a saída do estado se o robô já executou a manobra por pelo menos 1.2 segundos (12 ciclos)
            if self.contador_desvio > 12:
                # Se a bandeira reaparecer e TODO o semicírculo frontal estiver seguro (com margem de histerese de +0.15m)
                if self.bandeira_visivel and menor_dist_frontal > (DISTANCIA_OBSTACULO + 0.15):
                    self.estado = 'NAVIGANDO_PARA_BANDEIRA'
                    self.get_logger().info('Caminho completamente limpo e alvo readquirido!')
                
                # Se o obstáculo sumir completamente do campo frontal e diagonal, vai procurar a bandeira por perto
                elif menor_dist_frontal > (DISTANCIA_OBSTACULO + 0.25):
                    self.estado = 'BUSCANDO_BANDEIRA_PERDIDA'
                    self.contador_busca = 0
                    self.get_logger().info('Obstáculo superado. Iniciando varredura com memória angular.')
            
            # Ações de movimento focadas exclusivamente em limpar o obstáculo (ignora a câmera aqui)
            if menor_dist_frontal < DISTANCIA_CRITICA:
                twist.linear.x = -0.05
                twist.angular.z = VEL_ANGULAR if (self.esquerda + self.diag_esquerda) > (self.direita + self.diag_direita) else -VEL_ANGULAR
            else:
                # Contorna o cilindro movendo-se lateralmente com cuidado para não colidir a garra
                twist.linear.x = 0.06
                if (self.esquerda + self.diag_esquerda) > (self.direita + self.diag_direita):
                    twist.angular.z = 0.45  # Gira se esquivando do obstáculo à direita
                else:
                    twist.angular.z = -0.45 # Gira se esquivando do obstáculo à esquerda

        # --------------------------------------------------
        # ESTADO: BUSCANDO BANDEIRA PERDIDA (USO DA MEMÓRIA)
        # --------------------------------------------------
        elif self.estado == 'BUSCANDO_BANDEIRA_PERDIDA':
            if self.bandeira_visivel:
                self.estado = 'NAVIGANDO_PARA_BANDEIRA'
                self.get_logger().info('Alvo readequado no campo visual!')
            else:
                self.contador_busca += 1
                if self.contador_busca > TIMEOUT_BUSCA_CICLOS:
                    self.estado = 'EXPLORANDO'
                    self.get_logger().info('Tempo limite de busca esgotado. Retornando ao modo exploratório.')
                else:
                    # Gira sobre o próprio eixo focado na última direção salva antes de sumir
                    twist.linear.x = 0.0
                    if self.ultimo_offset > 0:
                        twist.angular.z = -0.40  # Gira à direita de forma controlada
                    else:
                        twist.angular.z = 0.40   # Gira à esquerda de forma controlada

        # --------------------------------------------------
        # ESTADO: POSICIONANDO PARA COLETA (SUCESSO FIM)
        # --------------------------------------------------
        elif self.estado == 'POSICIONANDO_PARA_COLETA':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('ROBÔ CORRETAMENTE POSICIONADO NA BANDEIRA AZUL!', throttle_duration_sec=15.0)

        # Publicação de comandos de movimento
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
