from data_structures.grid import Grid2D, Cell
import numpy as np
from .grid import Point

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
        self.maxDist = max(self.rows, self.cols)//5
        self.distances = np.array([[-1]*self.cols]*self.rows)

        self.load_obs()
        self.update_sdf()

        self.N = self.cols * self.rows


    def load_obs(self):
        for row in range(self.rows):
            for col in range(self.cols):
                if self.grid.occupiedQ(Cell(row, col)):
                    self.distances[row][col] = 0
                    #print('obs found')

    def update_sdf(self):
        dists = self.distances
        for row in range(self.rows):
            for col in range(self.cols):
                if dists[row][col] != 0: #open space
                    self.populate_sdf_local(row, col)
    
    def populate_sdf_local(self, row, col):
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

    def traverse(self, start, end): #nick: unfinished
        """start and end are Cell objects
        return a list of Cell objects, where each entry is the path traveled from start to end"""
        c = start

        while c != end:
            reward = [0]*4
            #add reward for correct direction
            dir = np.arctan2(end.y - c.y, end.x - c.x)

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

    #def traverse(self, start, end):
        """Figure out the cells that the ray from start to end traverses.

        Corner cases that must be accounted for:
        - If start and end points coincide, return (True, [start cell]).
        - Check that the start point is inside the grid. Return (False, None) otherwise.
        - End point can be outside the grid. The ray tracing must stop at the edges of the grid.
        - Perfectly horizontal and vertical rays.
        - Ray starts and ends in the same cell.

        Args:
            start: (Point) Start point of the ray
            end: (Point) End point of the ray

        Returns:
            success, raycells: (bool, list of Cell) If the traversal was successful, success is True
                                and raycells is the list of traversed cells (including the starting
                                cell). Otherwise, success is False and raycells is None.
        """
        # TODO: Assignment 2, Problem 1.1 (test_traversal)
        # corner cases
        if start == end:
            return (True, self.point_to_cell(start))
        
        print("Starting Point and Cell: ")
        print(start)
        sCell = self.point_to_cell(start)
        if not self.inQ(sCell):
            return (False, None)
        print(self.cell_to_point(sCell))

        print("Ending Point and Cell")
        print(end)
        eCell = self.point_to_cell(end)
        print(self.cell_to_point(eCell))
        dir = end - start
        print("Direction: ", dir)
        # determine what direction ray is going
        

        cCell = sCell
        p_relative = start - self.cell_to_point(self.point_to_cell(start))

        print('Resolution: ', self.resolution)
        cells = [sCell]
        while (cCell.row != eCell.row) or (cCell.col != eCell.col):
            # test for out of bounds
            if not self.inQ(cCell):
                break
            print('Current Cell: ', self.cell_to_point(cCell))
            print('End Cell: ', self.cell_to_point(eCell))
            print('P absolute: ', p_relative + self.cell_to_point(cCell))
            print('P relative: ', p_relative)

            if dir.x >= 0:
                print('right')
                step_col =  1
                dx = self.resolution - p_relative.x
            else:
                print('left')
                step_col = -1
                dx = -p_relative.x
            if dir.y >= 0:
                print('up')
                step_row =  1
                dy = self.resolution - p_relative.y
            else:
                print('down')
                step_row = -1
                dy = -p_relative.y

            # given direction and position, determine what side of cell is crossed
            cellCorner = self.cell_to_point(cCell)
            #print('tx, ty: ', dx/dir.x, ', ', dy/dir.y)
            if dir.x == 0:
                print('vertical')
                nextCell = Cell(cCell.row + step_row, cCell.col)
                p_relative = Point(p_relative.x + dy * dir.x / dir.y, 0.5 * (self.resolution - self.resolution * step_row))
            elif dir.y == 0:
                print('horizontal')
                nextCell = Cell(cCell.row, cCell.col + step_col)
                p_relative = Point(0.5 * (self.resolution - self.resolution * step_col), p_relative.y + dx * dir.y / dir.x)
            elif (dx/dir.x) < (dy/dir.y):    #will cross horizontally
                print('horizontal')
                nextCell = Cell(cCell.row, cCell.col + step_col)
                p_relative = Point(0.5 * (self.resolution - self.resolution * step_col), p_relative.y + dx * dir.y / dir.x)
            elif (dx/dir.x) >= (dy/dir.y): #will cross vertically
                print('vertical')
                nextCell = Cell(cCell.row + step_row, cCell.col)
                p_relative = Point(p_relative.x + dy * dir.x / dir.y, 0.5 * (self.resolution - self.resolution * step_row))
            cells.append(cCell)
            cCell = nextCell
            print("Input something to step to next cell")
            #x = input()
        cells.append(eCell)
        return (True, cells)
        raise NotImplementedError

    def traverse(self, start, end):
        """Figure out the cells that the ray from start to end traverses.

        Corner cases that must be accounted for:
        - If start and end points coincide, return (True, [start cell]).
        - Check that the start point is inside the grid. Return (False, None) otherwise.
        - End point can be outside the grid. The ray tracing must stop at the edges of the grid.
        - Perfectly horizontal and vertical rays.
        - Ray starts and ends in the same cell.

        Args:
            start: (Point) Start point of the ray
            end: (Point) End point of the ray

        Returns:
            success, raycells: (bool, list of Cell) If the traversal was successful, success is True
                                and raycells is the list of traversed cells (including the starting
                                cell). Otherwise, success is False and raycells is None.
        """
        # TODO: Assignment 2, Problem 1.1 (test_traversal)
        raycells = []

        c_start = self.point_to_cell(start)
        c_end = self.point_to_cell(end)
        #print('end point: ', end.x, end.y)
        #print('end cell: ', c_end)

        c = Cell()
        c = c_start

        # Comment: Make sure the start point is in the map
        if not self.inQ(c_start):
            print('Start point is not in the grid.')
            return False, None

        # Comment: Don't check the end point, we will terminate once the ray leaves the map.
        if start == end:
            return (True, [self.point_to_cell(start)])

        # Comment: Corner cases: Horizontal and vertical rays
        # Comment: Solution: Use two booleans to track whether to search along rows, cols, or both
        search_row = True
        if c_start.row == c_end.row: # Uncomment this and fill out if condition
            search_row = False
        search_col = True
        if c_start.col == c_end.col:
            search_col = False

        # Comment: Corner case: Ray is in only one cell
        # Comment Solution: Return the starting cell
        # TODO: fill me in
        if ((not search_row) and (not search_col)):
            return True, self.point_to_cell(start)

        mag = abs(end - start)
        dir = (end - start) / mag

        cb = self.cell_to_point(c)

        step_col = -1
        if dir.x > 0.0:
            step_col = 1

        step_row = -1
        if dir.y > 0.0:
            step_row = 1

        tmax = Point()
        tdelta = Point()

        if search_col:
            tdelta.x = self.resolution * float(step_col) * (1.0 / dir.x)
            if step_col == 1:
                tmax.x = (self.resolution - (start.x - cb.x)) * (1.0 / dir.x)
            elif step_col == -1:
                tmax.x = (cb.x - start.x) * (1.0 / dir.x) #will be negative to match negative tdelta.x
            else:
                print('smthn is wrong')

        if search_row:
            tdelta.y = self.resolution * float(step_row) * (1.0 / dir.y)
            if step_row == 1:
                tmax.y = (self.resolution - (start.y - cb.y)) * (1.0 / dir.y)
            elif step_row == -1:
                tmax.y = (cb.y - start.y) * (1.0 / dir.y) #will be negative to match negative tdelta.y
            else:
                print('smthn is wrong')

        #print("tmax's are: ", tmax.x, tmax.y)


        while True: # uncomment this
            #print(np.array([a.to_numpy() for a in raycells]))
            # Invalid cell reached; exit with the currently traversed cells.
            if not self.inQ(c):
                return True, raycells

            # Add c to the traversed cells
            # TODO: Fill me in
            #print('cell: ', c)
            raycells.append(Cell(c.row, c.col))


            # Escape after inserting the end cell once
            # TODO: Fill me in
            if c == c_end:
                break
            
            # Escape after detecting occupied cell
            if self.occupiedQ(c):
                break

            # Update type
            # ROW: Take a step along the row.
            # COL: Take a step along the column.
            # TODO: Fill me in -- how to decide whether to step along
            # row or step along column


            if search_col and search_row: #sloped ray

                if  (abs(tmax.x) < abs(tmax.y)): # crosses horizontally first
                    tmax.x += tdelta.x
                    update_row = False
                    update_col = True
                else:                 # crosses vertically first
                    tmax.y += tdelta.y
                    update_row = True
                    update_col = False
            elif search_col and not search_row: #horizontal ray
                tmax.x += tdelta.x
                update_row = False
                update_col = True
            elif search_row and not search_col:
                tmax.y += tdelta.y
                update_row = True
                update_col = False
            else:
                print('u fucked up')

            # Take a step and update the current cell
            if update_row:
                c.row = c.row + step_row
            elif update_col:
                c.col = c.col + step_col
            else:
                print('Undefined behavior during traversal.')
                return False, None
            
        return True, raycells

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