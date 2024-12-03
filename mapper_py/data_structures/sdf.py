from data_structures.grid import Grid2D, Cell
import numpy as np
from .grid import Point
from copy import deepcopy

from .sensor import *

class SDF:
    """Signed Distance Field data structure

    Attributes:
        grid: Grid2D object primarily used for its cell data

    """
    def __init__(self, grid):
        """Initialize the SDF data structure.
        """
        self.grid = grid
        self.cols = int(grid.width)
        self.rows = int(grid.height)
        self.maxDist = 10
        self.distances = np.array([[-1]*self.cols]*self.rows)

        self.load_obs()
        self.populate_sdf()

        self.N = self.cols * self.rows

        # Initially all the logodds values are zero.
        # A logodds value of zero corresponds to an occupancy probability of 0.5.
        self.data = [0.0] * self.N

    def load_obs(self):
        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid.occupiedQ(Cell(row, col)):
                    self.distances[row][col] = 0

    def populate_sdf(self):
        dists = self.distances
        for row in range(self.rows):
            for col in range(self.cols):
                if dists[row][col] != 0: #open space
                    self.populate_sdf_local(row, col)
    
    def populate_sdf_local(self, row, col):
        ri = 0
        rf = self.rows
        ci = 0
        cf = self.cols
        ri, rf, ci, cf = self.sdfSearchLimit(row, col)

        shortestDist = self.maxDist
        for r in range(ri, rf):
            for c in range(ci, cf):
                if self.distances[r][c] == 0: #obstacle found
                    distToObs = np.linalg.norm(np.array([r - row, c - col]))
                    if distToObs < shortestDist:
                        shortestDist = distToObs
        self.distances[row][col] = shortestDist

    def sdfSearchLimit(self, row, col):
        rowRange = self.rows//5
        colRange = self.cols//5

        ri = row - rowRange
        if ri < 0:
            ri = 0
        rf = row + rowRange
        if rf > self.rows:
            rf = self.rows
        
        ci = col - colRange
        if ci < 0:
            ci = 0
        cf = col + colRange
        if cf > self.cols:
            cf = self.cols

        return ri, rf, ci, cf
        
    # def traverse(self, start, end):
    #     """start and end are Cell objects
    #     return a list of Cell objects, where each entry is the path traveled from start to end"""
    #     c = start

    #     while c != end:
    #         reward = [0]*4
    #         #add reward for correct direction
    #         dir = np.arctan2(end.y - c.y, end.x - c.x)

    def getAdjacentCells(self, cell):
        dirs = [(0, 1), (0, -1), (1, 0), (-1, 0)] # only 90 degree turns for now lol
        adjCells = list()
        (crow, ccol) = (cell.row, cell.col)
        for dir in dirs:
            (adjrow, adjcol) = (crow + dir[0], ccol + dir[1])
            adjCell = Cell(adjrow, adjcol)
            if self.inGrid(adjCell):
                adjCells.append(adjCell)
        return adjCells


    def chooseNextCell(self, curr_cell, end_cell):
        adjCells = self.getAdjacentCells(curr_cell)

        best_cell = curr_cell
        best_dist_from_end = 1e6
        best_dist_from_obs = 0
        
        for cell in adjCells:
            cell_dist_from_end = ((cell.row - end_cell.row)**2 + (cell.col - end_cell.col)**2)**(1/2)
            cell_dist_from_obs = self.distances[cell.row, cell.col]
            
            if (((cell_dist_from_end < best_dist_from_end) and (cell_dist_from_obs > 0)) 
             or ((cell_dist_from_end == best_dist_from_end) and (cell_dist_from_obs > best_dist_from_obs))):
                best_cell = cell
                best_dist_from_end = cell_dist_from_end
                best_dist_from_obs = cell_dist_from_obs
        return best_cell
    

    
    def traverse_dummy(self, start, end):
        """
        Return a list of Cell objects

        Args:
        - start:
        - end:
        """
        if not self.inGrid(start):
            raise Exception('Start not in grid.') 
        if not self.inGrid(end):
            raise Exception('End not in grid.') 
        
        start_val = self.distances[start.row, start.col]
        end_val = self.distances[end.row, end.col]

        if start_val == 0:
            raise Exception('Start in obstacle. Cannot compute traversal.')
        if end_val == 0:
            raise Exception('End in obstacle. Cannot compute traversal.')

        curr_cell = start
        cells_traversed = list()
        
        while curr_cell != end:
            cells_traversed.append(curr_cell)
            curr_cell = self.chooseNextCell(curr_cell, end)
        
        cells_traversed.append(end)

        return cells_traversed
    
    def getCellsAtRangeBorder_square(self, curr, range):
        base_dirs = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)] 
        dirs = [(range * x, range * y) for (x, y) in base_dirs]
        #print('dirs: ', dirs)
        borderCells = list()
        (crow, ccol) = (curr.row, curr.col)
        for dir in dirs:
            (nrow, ncol) = (crow + dir[0], ccol + dir[1])
            borderCell = Cell(nrow, ncol)
            if self.inGrid(borderCell):
                borderCells.append(borderCell)
        return borderCells
    
    def getCellsAtRangeBorder_sensor(self, curr, range):
        cells = list()

        sensor = Sensor(range, range*2)
        angles = np.linspace(0, 2.0 * np.pi, sensor.num_rays, False)
        rays = list()
        (x, y) = (curr.col, curr.row)
        pos = Point(x, y)
        for angle in angles:
            rays.append(Ray(pos, Point(np.cos(angle), np.sin(angle))))
        
        for ray in rays:
            point = ray.point_at_dist(float(range))
            row = int(point.y) # assuming resolution of 1
            col = int(point.x) 
            #print(f'point: ({point.x},{point.y})')
            cell = Cell(row, col)
            if self.inGrid(cell):
                cells.append(cell)
        return cells

    
    def obstacleInPath(self, curr, next):
        #print(f'curr point: ({curr.row}, {curr.col})')
        traversal = self.traverse_cells(curr, next)
        cellsTraversed = traversal[1]

        if cellsTraversed == None:
            raise Exception('huh???')
        
        for cell in cellsTraversed:
            crow, ccol = cell.row, cell.col
            if self.distances[crow, ccol] == 0:
                return True
        return False

    def chooseNextCell_improved(self, curr, end, range, borderCellGroups):
        curr_point = (curr.row, curr.col)
        best_cell = curr
        best_dist_from_end = 1e6
        best_dist_from_obs = 0
        
        curr_dist_from_end = int(((curr.row - end.row)**2 + (curr.col - end.col)**2)**(1/2))
        range = min(range, curr_dist_from_end)

        # i.e. pointing at the same address if just best_cell = curr) ??
        while (best_cell.row == curr.row and best_cell.col == curr.col) and range > 0: 
            print('range: ', range)
            borderCells = self.getCellsAtRangeBorder_square(curr, range)
            if curr_point in borderCellGroups:
                borderCellGroups[curr_point].append(deepcopy(borderCells))
            else:
                borderCellGroups[curr_point] = [deepcopy(borderCells)]

            for next_cell in borderCells:
                if (not self.obstacleInPath(curr, next_cell)) and (not self.obstacleInPath(next_cell, end)):
                    cell_dist_from_end = ((next_cell.row - end.row)**2 + (next_cell.col - end.col)**2)**(1/2)
                    cell_dist_from_obs = self.distances[next_cell.row, next_cell.col]
                    
                    if (((cell_dist_from_end < best_dist_from_end) and (cell_dist_from_obs > 0)) 
                    or ((cell_dist_from_end == best_dist_from_end) and (cell_dist_from_obs > best_dist_from_obs))):
                        best_cell = next_cell
                        best_dist_from_end = cell_dist_from_end
                        best_dist_from_obs = cell_dist_from_obs
            # if we get through all the cells and they all have obstacle in path, best_cell should still be set to curr_cell
            range -= 1
        if range <= 0:
            raise Exception('No next step found.')
        return best_cell
        

    def traverse_dummy_improved(self, start, end, range, borderCellGroups):
        """
        Return a list of Cell objects

        Args:
        - start:
        - end:
        - borderCellGroups: mutable dictionary
        """
        if not self.inGrid(start):
            raise Exception('Start not in grid.') 
        if not self.inGrid(end):
            raise Exception('End not in grid.') 
        
        start_val = self.distances[start.row, start.col]
        end_val = self.distances[end.row, end.col]

        if start_val == 0:
            raise Exception('Start in obstacle. Cannot compute traversal.')
        if end_val == 0:
            raise Exception('End in obstacle. Cannot compute traversal.')

        curr_cell = start
        print(f'first curr cell: ({curr_cell.row}, {curr_cell.col})')
        cells_traversed = list()
        #print(f'SDF FIRST first cell: ({cells_traversed[0].row}, {cells_traversed[0].col})')
        counter = 0
        while curr_cell != end:
            cells_traversed.append(deepcopy(curr_cell))
            #print(f'next cell added: ({curr_cell.row},{curr_cell.col})')
            print(f'cell at idx {counter}: ({cells_traversed[counter].row},{cells_traversed[counter].col})')
            curr_cell = deepcopy(self.chooseNextCell_improved(curr_cell, end, range, borderCellGroups))
            counter += 1
        
        print(f'SDF first cell: ({cells_traversed[0].row}, {cells_traversed[0].col})')
        cells_traversed.append(end)

        
        return cells_traversed

        

    def to_numpy(self):
        """Export the grid in the form of a 2D numpy matrix.

        Each entry in this matrix is the probability of occupancy for the cell.
        """
        g = np.zeros((self.height, self.width))
        for row in range(self.height):
            for col in range(self.width):
                v = self.get_row_col(row, col)
                g[row][col] = self.probability(v)

        return g

    def to_index(self, cell):
        """Return the index into the data array (self.data) for the input cell.

        Args:
            cell: (Cell) The input cell for which the index in data array is requested.

        Returns:
            idx: (int) Index in the data array for the cell
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # self.data iterates through 2d grid row by row, left to right, top to bottom
        cellIndex = cell.row * self.width + cell.col
        return int(cellIndex)
        raise NotImplementedError

    def from_index(self, idx):
        """Return the cell in grid for the input index.

        Args:
            idx: (int) Index in the data array for which the cell is requested.

        Returns:
            cell: (Cell) Cell corresponding to the index.
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        r = idx // self.width
        c = idx %  self.width
        return Cell(r, c)
        raise NotImplementedError

    def get(self, idx):
        """Return the cell value corresponding to the input index.

        Args:
            idx: (int) Index in the data array for which the data is requested.

        Returns:
            val: (float) Value in the data array for idx
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        #print(idx)
        return self.data[idx]
        raise NotImplementedError

    def get_cell(self, cell):
        """Return the cell value corresponding to the input index."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use the `to_index` and `get` methods.
        return self.get(self.to_index(cell))
        raise NotImplementedError

    def get_row_col(self, row, col):
        """Return the cell value corresponding to the row and col."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use the `get_cell` method and the `Cell` constructor.
        return self.get_cell(Cell(row, col))
        raise NotImplementedError

    def set(self, idx, value):
        """Set the cell to value corresponding to the input idx."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        self.data[idx] = value
        return
        raise NotImplementedError

    def set_cell(self, cell, value):
        """Set the cell to value corresponding to the input cell."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use `to_index` and `set` methods.
        self.set(self.to_index(cell), value)
        return
        raise NotImplementedError

    def set_row_col(self, row, col, value):
        """Set the cell to value corresponding to the input row and col."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use the `set_cell` method and the `Cell` constructor.
        self.set_cell(Cell(row, col), value)
        return
        raise NotImplementedError

    def probability(self, logodds):
        """Convert input logodds to probability.

        Args:
            logodds: (float) Logodds representation of occupancy.

        Returns:
            prob: (float) Probability representation of occupancy.
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        P = np.exp(logodds) / (1 + np.exp(logodds))
        return P
        raise NotImplementedError

    def logodds(self, probability):
        """Convert input probability to logodds.

        Args:
            logodds: (float) Logodds representation of occupancy.

        Returns:
            prob: (float) Probability representation of occupancy.
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        logodds = np.log((probability) / (1 - probability))
        return logodds
        raise NotImplementedError

    def cell_to_point(self, cell):
        """Get the cell's lower-left corner in 2D point space.

        Args:
            cell: (Cell) Input cell.

        Returns:
            point: (Point) Lower-left corner in 2D space.
        """
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        return Point(cell.col * self.resolution, cell.row * self.resolution)
        raise NotImplementedError

    def cell_to_point_row_col(self, row, col):
        """Get the point for the lower-left corner of the cell represented by input row and col."""
        # TODO: Assignment 2, Problem 1.1 (test_data_structure)
        # Hint: Use the `cell_to_point` function and the `Cell` constructor.
        return self.cell_to_point(Cell(row, col))
        raise NotImplementedError

    def point_to_cell(self, point):
        """Get the cell position (i.e., bottom left hand corner) given the point.

        Args:
            point: (Point) Query point

        Returns:
            cell: (Cell) Cell in the grid corresponding to the query point.
        """
        # TODO: Assignment 2, Problem 1.1 (test_traversal)
        r = int(point.y / self.resolution)
        c = int(point.x / self.resolution)
        return Cell(r, c)
        raise NotImplementedError

    def inGrid(self, cell):
        return (cell.row >= 0) and (cell.col >= 0) and (cell.row < self.rows) and (cell.col < self.cols)
    
    def inQ(self, cell):
        """Is the cell inside this grid? Return True if yes, False otherwise."""
        # TODO: Assignment 2, Problem 1.1 (test_traversal)
        return (cell.row >= 0) and (cell.col >= 0) and (cell.row < self.height) and (cell.col < self.width)
        raise NotImplementedError

    def traverse_cells(self, start_cell, end_cell):
        # TODO: Assignment 2, Problem 1.1 (test_traversal)

        if ((start_cell.row == end_cell.row and start_cell.col == end_cell.col)):
            #print('ONE CELL')
            return (True, [start_cell])

        if not self.inGrid(start_cell):
            return (False, None)

        raycells = list()

        #raycells.append(Cell(start_cell.row, start_cell.col))
        curr_cell = start_cell

        x0 = start_cell.col + 0.5
        y0 = start_cell.row + 0.5
        x1 = end_cell.col + 0.5
        y1 = end_cell.row + 0.5
        o = np.array([x0, y0]).reshape(2, 1)
        e = np.array([x1, y1]).reshape(2, 1)
        diff = np.subtract(e, o)

        #print('e - o: ', diff)

        dir = diff * (1.0 / np.linalg.norm(diff)) 

        dx = dir[0][0]
        dy = dir[1][0]


        step_col = None
        step_row = None

        if dx > 0:
            step_col = 1
        elif dx == 0:
            step_col = 0
        else:
            step_col = -1

        if dy > 0:
            step_row = 1
        elif dy == 0:
            step_row = 0
        else:
            step_row = -1 

        #c_b = self.cell_to_point(start_cell)
        c_bx = start_cell.col
        c_by = start_cell.row

        alpha = 1

        o_x = o[0][0]
        o_y = o[1][0]


        tDeltaX = alpha * float(step_col) * (1.0/dx) if dx != 0 else 0
        tDeltaY = alpha * float(step_row) * (1.0/dy) if dy != 0 else 0


        tMaxX = None
        tMaxY = None

        if dx > 0.0:
            tMaxX = (c_bx + alpha - o_x) * (1.0 / dx)
        elif dx == 0:
            tMaxX = np.inf
        else:
            tMaxX = (c_bx - o_x) * (1.0 / dx)

        if dy > 0.0:
            tMaxY = (c_by + alpha - o_y) * (1.0 / dy)
        elif dy == 0:
            tMaxY = np.inf
        else:
            tMaxY = (c_by - o_y) * (1.0 / dy)

        while ((curr_cell.row != end_cell.row) or (curr_cell.col != end_cell.col)) and self.inGrid(curr_cell):
            #print(curr_cell)
            raycells.append(Cell(curr_cell.row, curr_cell.col))

            if tMaxX < tMaxY:
                curr_cell.col += step_col
                tMaxX = tMaxX + tDeltaX

            else:
                curr_cell.row += step_row
                tMaxY = tMaxY + tDeltaY

        if self.inGrid(end_cell):
            raycells.append(end_cell)

        return (True, raycells)


    def freeQ(self, cell):
        """Is the cell free? Return True if yes, False otherwise."""
        # TODO: Assignment 2, Problem 1.3
        # Hint: Use `get_cell` and `free_thres`
        return self.get_cell(cell) <= self.free_thres
        raise NotImplementedError

    def occupiedQ(self, cell):
        """Is the cell occupied? Return True if yes, False otherwise."""
        # TODO: Assignment 2, Problem 1.3
        # Hint: Use `get_cell` and `occ_thres`
        return self.get_cell(cell) >= self.occ_thres
        raise NotImplementedError

    def unknownQ(self, cell):
        """Is the cell unknown? Return True if yes, False otherwise."""
        # TODO: Assignment 2, Problem 1.3
        # Hint: Use `get_cell`, `occ_thres`, and `free_thres`
        value = self.get_cell(cell)
        return value > self.free_thres or value < self.occ_thres
        raise NotImplementedError
    
    ###################################################################
    ### MOST UP-TO-DATE FUNCTIONS FOR PATH PLANNING CODE BEGIN HERE ###
    ###################################################################

    # with tuple
    def inGrid(self, cell):
        """
        Determines whether the cell is in the grid bounds or not.

        Args:
        - cell: cell in tuple form (row, col)
        
        Returns:
        - Boolean that is True if the cell is in the grid and False if the cell is outside of grid boundaries.
        """
        (crow, ccol) = (cell[0], cell[1])
        return (crow >= 0) and (ccol >= 0) and (crow < self.rows) and (ccol < self.cols)
    
    def getCellsAtRangeBorder_sensor(self, curr, range):
        """
        Gets the cells (in tuple form) at the border of a circle with radius = range.

        Args:
        - curr: centered cell in tuple form (row, col)
        - range: range of border circle, i.e. the radius (will be cast to a float)

        Returns:
        - cells: set of tuple cells (row, col) that lie at the circular border of the range
        """
        cells = set()
        (crow, ccol) = (curr[0], curr[1])

        sensor = Sensor(1, max(20, range**2))
        angles = np.linspace(0, 2.0 * np.pi, sensor.num_rays, False)
        rays = list()
        (x, y) = (ccol + 0.5, crow + 0.5)
        pos = Point(x, y)
        for angle in angles:
            rays.append(Ray(pos, Point(np.cos(angle), np.sin(angle))))
        
        for ray in rays:
            point = ray.point_at_dist(float(range))
            row = int(point.y) # assuming resolution of 1
            col = int(point.x) 
            #print(f'point: ({point.x},{point.y})')
            cell = (row, col)
            if self.inGrid(cell):
                cells.add(cell)
        return cells

    
    def obstacleInPath(self, curr, next):
        """
        Determines if an obstacle is in the line-path from one cell to another.

        Args:
        - curr: starting cell in tuple form (row, col)
        - next: target cell in tuple form (row, col)

        Returns:
        - Boolean value - True if an obstacle is in the path, False if no obstacles in path
        """
        traversal = self.traverse_cells(curr, next)
        cellsTraversed = traversal[1]
        #print('cells traversed: ', cellsTraversed)

        if cellsTraversed == None:
            raise Exception('huh???')
        
        for cell in cellsTraversed:
            crow, ccol = (cell[0], cell[1])
            if self.distances[crow, ccol] == 0:
                #print('OBSTACLE FOUND???')
                return True
        return False

    def chooseNextCell_improved(self, curr, end, range, borderCellGroups):
        """
        Selects the next cell to traverse to in order to get closer to the end target.
        
        Args:
        - curr: current cell occupied in tuple form (row, col)
        - end: target ending cell in tuple form (row, col)
        - range: max range of the sensor (int or float)
        - borderCellGroups: mutable dictionary matching tuple cell keys to a list of groups of border cells referenced; 
                            e.g. [(row, col): [[borderCellsAtMaxRange], [borderCellsAtSmallerRange]...]]
        
        Returns:
        - best_cell: chosen cell to traverse to in tuple form (row, col)
        """
        (crow, ccol) = (curr[0], curr[1])
        (erow, ecol) = (end[0], end[1])
        
        best_cell = curr
        (brow, bcol) = (best_cell[0], best_cell[1])
        best_dist_from_end = 1e6
        best_dist_from_obs = 0
        
        curr_dist_from_end = int(((crow - erow)**2 + (ccol - ecol)**2)**(1/2))
        range = min(range, curr_dist_from_end) # if end is already within range, don't use the max range of the sensor.

        # Below while loop should continue iterating until a "best_cell" is found.
        # If no suitable cell is found at the current range, the range will decrease, 
        # and the while loop will try again. 
        while best_cell == curr and range > 0: 
            # print('range: ', range)
            # print('curr: ', curr)
            borderCells = self.getCellsAtRangeBorder_sensor(curr, range)
            #print('border cells: ', borderCells)
            if curr in borderCellGroups:
                borderCellGroups[curr].append(deepcopy(borderCells))
            else:
                borderCellGroups[curr] = [deepcopy(borderCells)]
            #print('num border cells: ', len(borderCells))
            for next_cell in borderCells: 
                # don't go to a cell that you've already been to?
                if next_cell not in borderCellGroups:
                    (nrow, ncol) = (next_cell[0], next_cell[1])

                    # if end is farther from range, dont worry about second not
                    if not self.obstacleInPath(curr, next_cell):
                    # print('no obstacle found yay')
                        cell_dist_from_end = ((nrow - erow)**2 + (ncol - ecol)**2)**(1/2)
                        cell_dist_from_obs = self.distances[nrow, ncol]

                        if (cell_dist_from_end <= range and not self.obstacleInPath(next_cell, end)) or cell_dist_from_end > range:
                            #print('cell dist from end: ', cell_dist_from_end)
                            if (((cell_dist_from_end < best_dist_from_end) and (cell_dist_from_obs > 0)) 
                            or ((cell_dist_from_end == best_dist_from_end) and (cell_dist_from_obs > best_dist_from_obs))):
                                #print('CHANGING BEST CELL')
                                best_cell = next_cell
                                best_dist_from_end = cell_dist_from_end
                                best_dist_from_obs = cell_dist_from_obs
            # if we get through all the cells and they all have obstacle in path, best_cell should still be set to curr_cell
            # at this point we know that there are no suitable cells at this range, so we try a smaller range. 
            range -= 1
    
        if best_cell == curr:
            raise Exception('No next step found.')
        return best_cell
        

    def traverse_dummy_improved(self, start, end, range, borderCellGroups):
        """
        Determines all cells traversed in path from starting cell to ending target cell.

        Args:
        - start: starting cell in tuple form (row, col)
        - end: target ending cell in tuple form (row, col)
        - range: max range of the sensor (int or float)
        - borderCellGroups: mutable dictionary matching tuple cell keys to a list of groups of border cells referenced; 
                            e.g. [(row, col): [[borderCellsAtMaxRange], [borderCellsAtSmallerRange]...]]
        
        Returns:
        - cells_traversed: list of all cells traversed in the path in tuple form;
                            e.g. [(row0, col0), (row1, col1), (row2, col2),...]
        
        """
        if not self.inGrid(start):
            raise Exception('Start not in grid.') 
        if not self.inGrid(end):
            raise Exception('End not in grid.') 
        
        (srow, scol) = (start[0], start[1])
        (erow, ecol) = (end[0], end[1])
        start_val = self.distances[srow, scol]
        end_val = self.distances[erow, ecol]

        if start_val == 0:
            raise Exception('Start in obstacle. Cannot compute traversal.')
        if end_val == 0:
            raise Exception('End in obstacle. Cannot compute traversal.')

        curr_cell = start
        #print(f'first curr cell: ({curr_cell[0]}, {curr_cell[1]})')
        cells_traversed = list()
        #print(f'SDF FIRST first cell: ({cells_traversed[0].row}, {cells_traversed[0].col})')
        counter = 0
        while curr_cell != end:
            cells_traversed.append(curr_cell)
            #print(f'next cell added: ({curr_cell.row},{curr_cell.col})')
            #print(f'cell at idx {counter}: ({cells_traversed[counter][0]},{cells_traversed[counter][1]})')
            curr_cell = self.chooseNextCell_improved(curr_cell, end, range, borderCellGroups)
            counter += 1
        
        #print(f'SDF first cell: ({cells_traversed[0][0]}, {cells_traversed[0][1]})')
        cells_traversed.append(end)
        
        return cells_traversed
    
    
    def traverse_cells(self, start_cell, end_cell):
        """
        Determines all cells passed through in the line from the center of a starting cell to the center of an ending cell.

        Args:
        - start_cell: starting cell in tuple form (row, col)
        - end_cell: ending cell in tuple form (row, col)

        Returns:
        - Tuple of a Boolean and a list (bool, raycells);
            - Boolean returns True if a traversal is successful and False if not
            - raycells: list of all cells traversed in a line from start to end cell in tuple form;
                        e.g. [(row0, col0), (row1, col1), (row2, col2),...]
        """

        (srow, scol) = (start_cell[0], start_cell[1])
        (erow, ecol) = (end_cell[0], end_cell[1])

        if ((srow == erow and scol == ecol)):
            #print('ONE CELL')
            return (True, [start_cell])

        if not self.inGrid(start_cell):
            return (False, None)

        raycells = list()

        #raycells.append(Cell(start_cell.row, start_cell.col))
        curr_cell = start_cell
        (crow, ccol) = (curr_cell[0], curr_cell[1])

        x0 = scol + 0.5
        y0 = srow + 0.5
        x1 = ecol + 0.5
        y1 = erow + 0.5
        o = np.array([x0, y0]).reshape(2, 1)
        e = np.array([x1, y1]).reshape(2, 1)
        diff = np.subtract(e, o)

        #print('e - o: ', diff)

        dir = diff * (1.0 / np.linalg.norm(diff)) 

        dx = dir[0][0]
        dy = dir[1][0]


        step_col = None
        step_row = None

        if dx > 0:
            step_col = 1
        elif dx == 0:
            step_col = 0
        else:
            step_col = -1

        if dy > 0:
            step_row = 1
        elif dy == 0:
            step_row = 0
        else:
            step_row = -1 

        #c_b = self.cell_to_point(start_cell)
        c_bx = scol
        c_by = srow

        alpha = 1 # assuming grid resolution = 1

        o_x = o[0][0]
        o_y = o[1][0]


        tDeltaX = alpha * float(step_col) * (1.0/dx) if dx != 0 else 0
        tDeltaY = alpha * float(step_row) * (1.0/dy) if dy != 0 else 0


        tMaxX = None
        tMaxY = None

        if dx > 0.0:
            tMaxX = (c_bx + alpha - o_x) * (1.0 / dx)
        elif dx == 0:
            tMaxX = np.inf
        else:
            tMaxX = (c_bx - o_x) * (1.0 / dx)

        if dy > 0.0:
            tMaxY = (c_by + alpha - o_y) * (1.0 / dy)
        elif dy == 0:
            tMaxY = np.inf
        else:
            tMaxY = (c_by - o_y) * (1.0 / dy)

        while ((crow != erow) or (ccol != ecol)) and self.inGrid(curr_cell):
            #print(curr_cell)
            raycells.append((crow, ccol))

            if tMaxX < tMaxY:
                ccol += step_col
                tMaxX = tMaxX + tDeltaX

            else:
                crow += step_row
                tMaxY = tMaxY + tDeltaY

        if self.inGrid(end_cell):
            raycells.append(end_cell)

        return (True, raycells)
