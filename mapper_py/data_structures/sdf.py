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
        if grid == None: #used with self.copy to save a little time
            return
        
        self.grid = grid
        self.cols = int(grid.width)
        self.rows = int(grid.height)
        self.maxDist = max(self.rows, self.cols)//5
        self.distances = np.array([[-1]*self.cols]*self.rows)

        self.load_obs()
        self.populate_sdf()

    def copy(self):
        '''
        used just for saving SDF snapshots during incremental traversal
        '''
        sdfCopy = SDF(None)
        sdfCopy.cols = self.cols
        sdfCopy.rows = self.rows
        sdfCopy.maxDist = self.maxDist
        sdfCopy.distances = self.distances.copy()
        return sdfCopy
        

    def load_obs(self):
        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid.occupiedQ(Cell(row, col)):
                    self.distances[row][col] = 0

    def populate_sdf(self):
        for row in range(self.rows):
            for col in range(self.cols):
                if self.distances[row][col] != 0: #open space
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
    
    ###################################################################
    ### MOST UP-TO-DATE FUNCTIONS FOR PATH PLANNING CODE BEGIN HERE ###
    ###################################################################

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
        angles = np.linspace(0, 2.0 * np.pi, int(sensor.num_rays), False)
        rays = list()
        (x, y) = (ccol + 0.5, crow + 0.5)
        pos = Point(x, y)
        for angle in angles:
            rays.append(Ray(pos, Point(np.cos(angle), np.sin(angle))))
        
        for ray in rays:
            point = ray.point_at_dist(float(range))
            row = int(point.y) # assuming resolution of 1
            col = int(point.x) 
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

        if cellsTraversed == None:
            raise Exception('huh???')
        
        for cell in cellsTraversed:
            crow, ccol = (cell[0], cell[1])
            if self.distances[crow, ccol] == 0:
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
            borderCells = self.getCellsAtRangeBorder_sensor(curr, range)
            if curr in borderCellGroups:
                borderCellGroups[curr].append(deepcopy(borderCells))
            else:
                borderCellGroups[curr] = [deepcopy(borderCells)]
            for next_cell in borderCells: 
                # don't go to a cell that you've already been to?
                if next_cell not in borderCellGroups:
                    (nrow, ncol) = (next_cell[0], next_cell[1])

                    # if end is farther from range, dont worry about second not
                    if not self.obstacleInPath(curr, next_cell):
                        cell_dist_from_end = ((nrow - erow)**2 + (ncol - ecol)**2)**(1/2)
                        cell_dist_from_obs = self.distances[nrow, ncol]

                        if (cell_dist_from_end <= range and not self.obstacleInPath(next_cell, end)) or cell_dist_from_end > range:
                            if (((cell_dist_from_end < best_dist_from_end) and (cell_dist_from_obs > 0)) 
                            or ((cell_dist_from_end == best_dist_from_end) and (cell_dist_from_obs > best_dist_from_obs))):
                                best_cell = next_cell
                                best_dist_from_end = cell_dist_from_end
                                best_dist_from_obs = cell_dist_from_obs
            # if we get through all the cells and they all have obstacle in path, best_cell should still be set to curr_cell
            # at this point we know that there are no suitable cells at this range, so we try a smaller range. 
            range -= 1
    
        if best_cell == curr:
            raise Exception('No next step found.')
        return best_cell
        
    def chooseNextCell3(self, curr, end, range, borderCellGroups):
        """
        Same as above, but with more heuristics
        ideas: 
         - add an obstacle to the visited cell to encourage moving around more
         - upon random chance, ignore heuristics and explore a random direction
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
            borderCells = self.getCellsAtRangeBorder_sensor(curr, range)
            if curr in borderCellGroups:
                borderCellGroups[curr].append(deepcopy(borderCells))
            else:
                borderCellGroups[curr] = [deepcopy(borderCells)]
            for next_cell in borderCells: 
                # don't go to a cell that you've already been to
                if next_cell not in borderCellGroups:
                    (nrow, ncol) = (next_cell[0], next_cell[1])

                    # if end is farther from range, dont worry about second not
                    if not self.obstacleInPath(curr, next_cell): # next_cell is not obstructed
                        cell_dist_from_end = ((nrow - erow)**2 + (ncol - ecol)**2)**(1/2)
                        cell_dist_from_obs = self.distances[nrow, ncol]

                        # check if goal is within reach or robot cannot reach goal from next_cell
                        if (cell_dist_from_end <= range and not self.obstacleInPath(next_cell, end)) or cell_dist_from_end > range:
                            if (np.random.random() > 0.9
                                or ((cell_dist_from_end == best_dist_from_end) and (cell_dist_from_obs > best_dist_from_obs))
                                or ((cell_dist_from_end < best_dist_from_end) and (cell_dist_from_obs > 0))):
                                best_cell = next_cell
                                best_dist_from_end = cell_dist_from_end
                                best_dist_from_obs = cell_dist_from_obs
            # if we get through all the cells and they all have obstacle in path, best_cell should still be set to curr_cell
            # at this point we know that there are no suitable cells at this range, so we try a smaller range. 
            range -= 1
    
        if best_cell == curr:
            raise Exception('No next step found.')
        
        # make current cell an obstacle, update distances
        if np.random.random() > 0.7:
            curr_idx = self.grid.to_index(Cell(curr[0], curr[1]))
            self.grid.data[curr_idx] = self.grid.occ_thres
            self.load_obs()
            self.populate_sdf()

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
        cells_traversed = list()
        counter = 0
        while curr_cell != end:
            cells_traversed.append(curr_cell)
            curr_cell = self.chooseNextCell_improved(curr_cell, end, range, borderCellGroups)
            counter += 1
        
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
            return (True, [start_cell])

        if not self.inGrid(start_cell):
            return (False, None)

        raycells = list()

        curr_cell = start_cell
        (crow, ccol) = (curr_cell[0], curr_cell[1])

        x0 = scol + 0.5
        y0 = srow + 0.5
        x1 = ecol + 0.5
        y1 = erow + 0.5
        o = np.array([x0, y0]).reshape(2, 1)
        e = np.array([x1, y1]).reshape(2, 1)
        diff = np.subtract(e, o)

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
