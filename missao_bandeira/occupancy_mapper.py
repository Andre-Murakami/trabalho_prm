#!/usr/bin/env python3

import math
import numpy as np


class OccupancyMapper:

    UNKNOWN = 127
    FREE = 0
    OCCUPIED = 255

    def __init__(self, size=300, resolution=0.10):

        self.size = size
        self.resolution = resolution

        self.grid = np.full(
            (size, size),
            self.UNKNOWN,
            dtype=np.uint8
        )

        self.center = size // 2

    def world_to_grid(self, x, y, x0, y0):

        gx = int((x - x0) / self.resolution) + self.center
        gy = int((y - y0) / self.resolution) + self.center

        return gx, gy

    def inside(self, gx, gy):
        return 0 <= gx < self.size and 0 <= gy < self.size

    def mark_free(self, gx, gy):

        if self.inside(gx, gy):

            if self.grid[gy, gx] != self.OCCUPIED:
                self.grid[gy, gx] = self.FREE

    def mark_occupied(self, gx, gy):

        if self.inside(gx, gy):
            self.grid[gy, gx] = self.OCCUPIED

    def update_from_scan(
        self,
        scan_msg,
        ranges,
        robot_x,
        robot_y,
        robot_yaw,
        x0,
        y0
    ):

        angle = scan_msg.angle_min

        for d in ranges:

            if math.isnan(d) or d < 0.05:
                angle += scan_msg.angle_increment
                continue

            global_angle = robot_yaw + angle

            max_d = d

            if math.isinf(d):
                max_d = min(scan_msg.range_max, 6.0)

            steps = int(max_d / self.resolution)

            for s in range(steps):

                r = s * self.resolution

                fx = robot_x + r * math.cos(global_angle)
                fy = robot_y + r * math.sin(global_angle)

                gx, gy = self.world_to_grid(
                    fx,
                    fy,
                    x0,
                    y0
                )

                self.mark_free(gx, gy)

            if not math.isinf(d):

                ox = robot_x + d * math.cos(global_angle)
                oy = robot_y + d * math.sin(global_angle)

                gx, gy = self.world_to_grid(
                    ox,
                    oy,
                    x0,
                    y0
                )

                self.mark_occupied(gx, gy)

            angle += scan_msg.angle_increment
