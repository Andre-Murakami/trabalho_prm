#!/usr/bin/env python3

import math


class NavigationManager:

    def __init__(self, mapper):
        self.mapper = mapper

    def grid_to_world(self,
                      gx,
                      gy,
                      x0,
                      y0):

        x = (gx - self.mapper.center) * self.mapper.resolution + x0
        y = (gy - self.mapper.center) * self.mapper.resolution + y0

        return x, y

    def calculate_goal_vector(self,
                              robot_x,
                              robot_y,
                              robot_yaw,
                              goal_x,
                              goal_y):

        dx = goal_x - robot_x
        dy = goal_y - robot_y

        desired_yaw = math.atan2(dy, dx)

        error = desired_yaw - robot_yaw

        while error > math.pi:
            error -= 2.0 * math.pi

        while error < -math.pi:
            error += 2.0 * math.pi

        distance = math.hypot(dx, dy)

        return error, distance
