"""Cell, Point, and Grid classes for 16-362: Mobile Robot Algorithms Laboratory
"""

import numpy as np
from copy import copy


class Cell:
    """A single cell in the occupancy grid map.

    Attributes:
        row: Row number of the cell. Corresponds to Y-axis in 2D plane.
        col: Col number of the cell. Corresponds to X-axis in 2D plane.
    """

    def __init__(self, row=0, col=0):
        """Initializes the row and col for this cell to be 0."""
        self.row = row
        self.col = col

    def __str__(self):
        return f'Cell(row: {self.row}, col: {self.col})'

    def to_numpy(self):
        """Return a numpy array with the cell row and col."""
        return np.array([self.row, self.col], dtype=int)
    
    def __eq__(self, second):
        if isinstance(second, Cell):
            return ((self.col == second.col) and (self.row == second.row))
        else:
            raise TypeError('Argument type must be Cell.')


class Point:
    """A point in the 2D space.

    Attributes:
        x: A floating point value for the x coordinate of the 2D point.
        y: A floating point value for the y coordinate of the 2D point.
    """

    def __init__(self, x=0.0, y=0.0):
        """Initializes the x and y for this point to be 0.0"""
        self.x = x
        self.y = y

    def __abs__(self):
        return (self.x ** 2 + self.y ** 2) ** 0.5

    def __str__(self):
        return f'Point(x: {self.x}, y: {self.y})'

    def __eq__(self, second):
        if isinstance(second, Point):
            return ((self.x == second.x) and (self.y == second.y))
        else:
            raise TypeError('Argument type must be Point.')

    def __ne__(self, second):
        if isinstance(second, Point):
            return ((self.x != second.x) or (self.y != second.y))
        else:
            raise TypeError('Argument type must be Point.')

    def __add__(self, second):
        if isinstance(second, Point):
            return Point(self.x + second.x, self.y + second.y)
        elif isinstance(second, float):
            return Point(self.x + second, self.y + second)
        else:
            raise TypeError('Argument type must be either float or Point.')

    def __sub__(self, second):
        if isinstance(second, Point):
            return Point(self.x - second.x, self.y - second.y)
        elif isinstance(second, float):
            # when subtracting with a float, always post-subtract
            # YES: Point(1.2, 3.2) - 5.0
            # NO: 5.0 - Point(1.2, 3.2)
            return Point(self.x - second, self.y - second)
        else:
            raise TypeError('Argument type must be either float or Point.')

    def __mul__(self, second):
        if isinstance(second, Point):
            return (self.x * second.x + self.y * second.y)
        elif isinstance(second, float):
            # when multiplying with a float, always post-multiply
            # YES: Point(1.2, 3.2) * 5.0
            # NO: 5.0 * Point(1.2, 3.2)
            return Point(self.x * second, self.y * second)
        else:
            raise TypeError('Argument type must be either float or Point.')

    def __truediv__(self, second):
        if isinstance(second, float):
            # when dividing by a float, always post-divide
            # YES: Point(1.2, 3.2) / 5.0
            # NO: 5.0 / Point(1.2, 3.2)
            if np.abs(second - 0.0) < 1e-12:
                raise ValueError(
                    'Divide by zero error. Second argument is too close to zero.')
            else:
                return Point(self.x / second, self.y / second)
        else:
            raise TypeError('Argument type must be float.')

    def to_numpy(self):
        """Return a numpy array with the x and y coordinates."""
        return np.array([self.x, self.y], dtype=float)


class Grid2D:
    """Occupancy grid data structure.

    Attributes:
        resolution: (float) The size of each cell in meters.
        width: (int) Maximum number of columns in the grid.
        height: (int) Maximum number of rows in the grid.
        min_clamp: (float) Logodds corresponding to minimum possible probability
        (to ensure numerical stability).
        max_clamp: (float) Logodds corresponding to maximum possible probability
        (to ensure numerical stability).
        free_thres: (float) Logodds below which a cell is considered free
        occ_thres: (float) Logodds above which a cell is considered occupied
        N: (int) Total number of cells in the grid
        data: Linear array of containing the logodds of this occupancy grid
    """

    def __init__(self, res, W, H, min_clamp, max_clamp, free_thres=0.13, occ_thres=0.7):
        """Initialize the grid data structure.

        Note that min_clamp, max_clamp, free_thres, and occ_thres inputs to this constructor
        are probabilities. You have to convert them to logodds internally for numerical stability.
        """
        self.resolution = res

        self.width = int(W)
        self.height = int(H)

        self.min_clamp = self.logodds(min_clamp)
        self.max_clamp = self.logodds(max_clamp)
        self.free_thres = self.logodds(free_thres)
        self.occ_thres = self.logodds(occ_thres)

        self.N = self.width * self.height

        # Initially all the logodds values are zero.
        # A logodds value of zero corresponds to an occupancy probability of 0.5.
        self.data = [0.0] * self.N



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

    def Tuple_to_Point(self, c):
        return (Point(c[1] + 0.5, c[0] + 0.5) * self.resolution)

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
