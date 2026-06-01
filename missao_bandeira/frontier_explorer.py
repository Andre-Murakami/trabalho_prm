#!/usr/bin/env python3

import math


class FrontierExplorer:

    def __init__(self, mapper):
        self.mapper = mapper

    def find_frontiers(self):

        grid = self.mapper.grid

        frontiers = []

        for y in range(1, self.mapper.size - 1):
            for x in range(1, self.mapper.size - 1):

                if grid[y, x] != self.mapper.FREE:
                    continue

                unknown_neighbor = False

                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:

                        if dx == 0 and dy == 0:
                            continue

                        ny = y + dy
                        nx = x + dx

                        if grid[ny, nx] == self.mapper.UNKNOWN:
                            unknown_neighbor = True

                if unknown_neighbor:
                    frontiers.append((x, y))

        return frontiers

    def select_best_frontier(self,
                             robot_gx,
                             robot_gy,
                             frontiers):

        if len(frontiers) == 0:
            return None

        best = None
        best_score = 999999

        for fx, fy in frontiers:

            dist = math.hypot(fx - robot_gx,
                              fy - robot_gy)

            if dist < best_score:
                best_score = dist
                best = (fx, fy)

        return best
