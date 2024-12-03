from data_structures.grid import Grid2D, Cell
from mapper import Mapper
from data_structures.sensor import Sensor
from data_structures.observer import Observer
from data_structures.sdf import SDF
import numpy as np
from .grid import Point
import time
#Nick's class for representing robot that navigates map via SDF

class Robot:
    """
    global_grid: map with all obstacles known
    local_grid: map with only observed obstacles known
    pos: Cell that robot is in 
    
    """
    def __init__(self, global_grid, local_grid, pos):
        self.pos = pos
        self.global_grid = global_grid
        self.local_grid = local_grid
        self.mapper = Mapper(self.local_grid, Sensor(max_range=(local_grid.resolution * local_grid.width / 3), 
                                                     num_rays=50), Observer(self.global_grid))
        self.sdf = SDF(self.mapper.grid) #grid objects are mutable so SDF updates as the mapper fills out its grid

    
    def traverse(self, goal):
        np.set_printoptions(threshold = np.inf)
        np.set_printoptions(linewidth = np.inf)
        
        # goal is a Cell
        path = list()
        while self.pos != goal:
            
            # observe and update map
            self.mapper.add_obs(self.local_grid.Cell_to_Point(self.pos))
            self.sdf.load_obs()
            self.sdf.update_sdf()
            print(self.sdf.distances)

            #move
            path.append(self.pos)
            print("Robot: ", str(self.pos.row), str(self.pos.col))
            self.pos = self.sdf.chooseNextCell(self.pos, goal)
            input("continue") #so you can watch the robot move
            print()
        return path




