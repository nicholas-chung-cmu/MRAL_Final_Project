import numpy as np

from .grid import Point

class Observer:
    def __init__(self, gt_grid):
        self.grid = gt_grid

    def observe_along_ray(self, ray, max_range): # returns end cell and traversed cells until an obstacle is reached
        success, cells = self.grid.traverse(ray.o, ray.point_at_dist(max_range))

        freeCells = []
        if success:
            for i in range(len(cells)):
                c = cells[i]
                found_occ = False
                if self.grid.occupiedQ(c):
                    found_occ = True
                    break
                freeCells.append(self.grid.cell_to_point(c))

            if found_occ:
                return self.grid.cell_to_point(c) + Point(self.grid.resolution / 2, self.grid.resolution / 2), freeCells
            else:
                return ray.point_at_dist(max_range), freeCells
        else:
            return None, None
