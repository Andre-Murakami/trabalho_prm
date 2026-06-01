#!/usr/bin/env python3

import cv2


LABEL_BANDEIRA_AZUL = 25


class FlagTracker:

    def __init__(self,
                 largura_imagem=640,
                 area_minima=150):

        self.largura_imagem = largura_imagem
        self.area_minima = area_minima

    def detect(self, frame):

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

        if not contours:
            return False, 0.0, 0.0

        maior = max(contours, key=cv2.contourArea)

        area = cv2.contourArea(maior)

        if area < self.area_minima:
            return False, 0.0, 0.0

        M = cv2.moments(maior)

        if M['m00'] <= 0:
            return False, 0.0, 0.0

        cx = int(M['m10'] / M['m00'])

        offset = (
            (cx - self.largura_imagem / 2)
            / (self.largura_imagem / 2)
        )

        return True, offset, area
