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

    def CellToTuple(self, c):
        return (c.row, c.col)

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

            #move
            path.append(self.pos)
            self.pos = self.sdf.chooseNextCell(self.pos, goal)
            input("continue") #so you can watch the robot move
        return path

    def traverse_improved(self, end, borderCellGroups):
        '''
        inputs:
        end: tuple of robot's goal

        outputs:
        path: list of traversed Cells
        sdfs: list of SDF objects taken at each step in traversal
        '''

        np.set_printoptions(threshold = np.inf)
        np.set_printoptions(linewidth = np.inf)
        

        start = self.pos
        if not self.sdf.inGrid(start):
            raise Exception('Start not in grid.') 
        if not self.sdf.inGrid(end):
            raise Exception('End not in grid.') 
        
        (srow, scol) = (start[0], start[1])
        (erow, ecol) = (end[0], end[1])
        start_val = self.sdf.distances[srow, scol]
        end_val = self.sdf.distances[erow, ecol]

        if start_val == 0:
            raise Exception('Start in obstacle. Cannot compute traversal.')
        if end_val == 0:
            raise Exception('End in obstacle. Cannot compute traversal.')

        # goal is a tuple of form (row, col)
        path = list()
        sdfs = list() # to view local map progress later
        while self.pos != end:
            
            # observe and update map
            self.observe_surroundings()

            # record current map
            currSDF = self.sdf.copy()
            sdfs.append(currSDF)

            #move
            path.append(self.pos)
            self.pos = self.sdf.chooseNextCell_improved(self.pos, end, self.mapper.sensor.max_range, borderCellGroups)
        return path, sdfs
    
    def traverse3(self, end, borderCellGroups):
        '''
        uses new chooseNextCell3 function

        inputs:
        end: tuple of robot's goal

        outputs:
        path: list of traversed Cells
        sdfs: list of SDF objects taken at each step in traversal
        '''

        np.set_printoptions(threshold = np.inf)
        np.set_printoptions(linewidth = np.inf)
        

        start = self.pos
        if not self.sdf.inGrid(start):
            raise Exception('Start not in grid.') 
        if not self.sdf.inGrid(end):
            raise Exception('End not in grid.') 
        
        (srow, scol) = (start[0], start[1])
        (erow, ecol) = (end[0], end[1])
        start_val = self.sdf.distances[srow, scol]
        end_val = self.sdf.distances[erow, ecol]

        if start_val == 0:
            raise Exception('Start in obstacle. Cannot compute traversal.')
        if end_val == 0:
            raise Exception('End in obstacle. Cannot compute traversal.')

        # goal is a tuple of form (row, col)
        path = list()
        sdfs = list() # to view local map progress later
        while self.pos != end:
            
            # observe and update map
            self.observe_surroundings()
            #print(self.sdf.distances)

            # record current map
            currSDF = self.sdf.copy()
            sdfs.append(currSDF)

            #move
            path.append(self.pos)
            self.pos = self.sdf.chooseNextCell3(self.pos, end, self.mapper.sensor.max_range, borderCellGroups)
        return path, sdfs

    def observe_surroundings(self):
        self.mapper.add_obs(self.local_grid.Tuple_to_Point(self.pos))
        self.sdf.load_obs()
        self.sdf.populate_sdf()
        return


