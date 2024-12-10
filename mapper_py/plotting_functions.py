import matplotlib.pyplot as plt
import numpy as np
import time

# for testing purposes import sdf functions, etc.

# draw_grid function coded with aid from chatGPT
def draw_grid(grid, rows, cols, title, cmap_str='Greys'):
    """
    Draws a grid.

    Args:
    - rows: Number of rows in the grid.
    - cols: Number of columns in the grid.
    """
    fig, ax = plt.subplots()
    ax.imshow(grid, cmap= cmap_str, extent=[0, cols, 0, rows])

    #Set grid lines
    ax.set_xticks(np.arange(0, cols+1, 1))
    ax.set_yticks(np.arange(0, rows+1, 1))
    ax.grid(color='black', linestyle='-', linewidth=0.3)
    plt.title(title)

    #Disable ticks
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    return (fig, ax)

def draw_obstacles_from_SDF(sdf):
    rows = sdf.rows
    cols = sdf.cols
    obstacle_grid = np.array([[0]*cols]*rows)

    for row in range(0, rows):
        for col in range(0, cols):
            if np.flipud(sdf.distances)[row, col] == 0:
                obstacle_grid[row, col] = 1
    
    (fig, ax) = draw_grid(obstacle_grid, rows, cols, 'Path Traversal')
    return (fig, ax)

def draw_lines_on_grid_from_points(ax, points):
    """
    Draws a grid.

    Args:
    - points: list of tuples, in which each tuple is a point (x, y) for a cell in the grid
    """
    num_points = len(points)

    for point in range(num_points-1):
        (x0, y0) = points[point]
        (x1, y1) = points[point+1]
        ax.plot([x0, x1], [y0, y1], color='red')


def draw_lines_on_grid_incrementally(sdf, cells, steps=5): 
    points = convert_cells_to_points(cells)
    num_points = len(points)
    counter = 1

    step_size = (num_points-1)//steps

    while counter < num_points:
        fig, ax = draw_obstacles_from_SDF(sdf)
        for point in range(0, counter-1, 1):
            (x0, y0) = points[point]
            (x1, y1) = points[point+1]

            ax.plot([x0, x1], [y0, y1], color='red')
        
        plt.show()
        counter += step_size
        

def convert_cells_to_points(cells):
    """
    Converts a list of cells to a list of tuples such that each point is centered in each cell.

    Args:
    - cells: list of Cell objects 
    """
    points = list()
    for i in range(len(cells)):
        cell_to_tuple = (cells[i].col + 0.5, cells[i].row + 0.5)
        points.append(cell_to_tuple)

    return points

def convert_tuple_cells_to_points(cells):
    """
    Converts a set of cells to a list of tuples such that each point is centered in each cell.

    Args:
    - cells: list of tuple cells, [(row0, col0), (row1, col1), etc.]
    """
    points = list()

    for i in range(len(cells)):
        cell = cells[i]
        (crow, ccol) = (cell[0], cell[1])
        cell_to_tuple = (ccol + 0.5, crow + 0.5)
        points.append(cell_to_tuple)

    return points

def draw_lines_on_grid_from_cells(ax, cells):
    points = convert_cells_to_points(cells)
    draw_lines_on_grid_from_points(ax, points)

def highlight_referenced_cells_incrementally(sdf, cells_groups, steps):
    """
    Highlights cells at border of range used for traversal algorithm.

    Args:
    - sdf: sdf object
    - cells_groups: mutable dictionary matching tuple cell keys to a list of groups of border cells referenced; 
                    e.g. [(row, col): [[borderCellsAtMaxRange], [borderCellsAtSmallerRange]...]
    - steps: number of steps to divide path tracing for viewing; to view all steps: steps = len(points) - 1
    """
    num_groups = len(cells_groups)
    counter = 0

    step_size = (num_groups-1)//steps

    while counter < num_groups:
        fig, ax = draw_obstacles_from_SDF(sdf)
        cells = cells_groups[counter]
        cell_coords = list()
        for cell in cells:
            cell_coords.append((cell.row, cell.col))
        for row, col in cell_coords:
            # b/c x = col & y = row
            ax.add_patch(plt.Rectangle((col, row), 1, 1, color='yellow'))
        
        plt.show()
        counter += step_size


def trace_traversal_with_sensor(sdf, points, cells_with_sensor, steps):
    '''
    Draws steps of path traversal incrementally. Path traversal algorithm with initially known map.

    Args:
    - sdf: SDF object of known map
    - points: centers of traversed cells
    - cells_with_sensor: mutable dictionary matching tuple cell keys to a list of groups of border cells referenced; 
                        e.g. [(row, col): [[borderCellsAtMaxRange], [borderCellsAtSmallerRange]...]
    - steps: number of steps to divide path tracing for viewing; to view all steps: steps = len(points) - 1
    '''
    num_points = len(points)
    counter = 1

    step_size = (num_points-1)//steps

    (first_point_x, first_point_y) = points[0]
    (end_point_x, end_point_y) = points[num_points-1]

    while counter < num_points:

        prev_point = points[counter-1]
        curr_point = points[counter]
        
        # now draw sensor readings (if end hasn't been reached yet)
        prev_cell_tuple = (((int) (prev_point[1] - 0.5)), ((int) (prev_point[0] - 0.5)))
        print(cells_with_sensor)
        prev_cell_sensor_readings = cells_with_sensor[prev_cell_tuple]
        num_groups = len(prev_cell_sensor_readings)
        counter2 = 0

        # Below while loop should draw the path up until prev_point, 
        # and then highlight all border cells considered until the next point (curr_point) is selected.
        while counter2 < num_groups:
            fig, ax = draw_obstacles_from_SDF(sdf)

            # plot start and end points
            ax.plot(first_point_x, first_point_y, marker='o', color='red')
            ax.plot(end_point_x, end_point_y, marker='o', markerfacecolor='none', markeredgecolor='red')

            # draw all lines up to this cell
            for i in range(0, counter-1, 1):
                (x0, y0) = points[i]
                (x1, y1) = points[i+1]
                ax.plot([x0, x1], [y0, y1], color='red')
            cells = prev_cell_sensor_readings[counter2]
            cell_coords = list()
            for cell in cells:
                cell_coords.append(cell)
            for row, col in cell_coords:
                # b/c x = col & y = row
                ax.add_patch(plt.Rectangle((col, row), 1, 1, color='yellow'))
            plt.show()
            counter2 += 1

        # draw chosen point (curr)
        (prev_point_row, prev_point_col) = prev_point
        (curr_point_row, curr_point_col) = curr_point
        ax.plot([prev_point_row, curr_point_row], [prev_point_col, curr_point_col], color='red')      
        plt.show()
        counter += step_size
    
    # plot final trajectory
    fig_final, ax_final = draw_obstacles_from_SDF(sdf)
    ax_final.plot(first_point_x, first_point_y, marker='o', color='red')
    ax_final.plot(end_point_x, end_point_y, marker='o', markerfacecolor='none', markeredgecolor='red')
    for i in range(num_points-1):
        (x0, y0) = points[i]
        (x1, y1) = points[i+1]
        ax_final.plot([x0, x1], [y0, y1], color='red')
    plt.show()


def trace_traversal_with_sensor_savefigs(sdf, points, cells_with_sensor, steps, fig_path):
    """
    Serves same purpose as trace_traversal_with_sensor, but saves each generated figure (of each step in the path traversal) to fig_path.

    Args:
    - sdf: SDF object of known map
    - points: centers of traversed cells
    - cells_with_sensor: mutable dictionary matching tuple cell keys to a list of groups of border cells referenced; 
                        e.g. [(row, col): [[borderCellsAtMaxRange], [borderCellsAtSmallerRange]...]
    - steps: number of steps to divide path tracing for viewing; to view all steps: steps = len(points) - 1
    - fig_path: str path to which figures will be saved to
    """
    num_points = len(points)
    counter = 1

    step_size = (num_points-1)//steps

    (first_point_x, first_point_y) = points[0]
    (end_point_x, end_point_y) = points[num_points-1]

    while counter < num_points:

        prev_point = points[counter-1]
        curr_point = points[counter]
        
        # now draw sensor readings (if end hasn't been reached yet)
        prev_cell_tuple = (((int) (prev_point[1] - 0.5)), ((int) (prev_point[0] - 0.5)))
        prev_cell_sensor_readings = cells_with_sensor[prev_cell_tuple]
        num_groups = len(prev_cell_sensor_readings)
        counter2 = 0

        # Below while loop should draw the path up until prev_point, 
        # and then highlight all border cells considered until the next point (curr_point) is selected.
        while counter2 < num_groups:
            fig, ax = draw_obstacles_from_SDF(sdf)

            # plot start and end points
            ax.plot(first_point_x, first_point_y, marker='o', color='red')
            ax.plot(end_point_x, end_point_y, marker='o', markerfacecolor='none', markeredgecolor='red')

            # draw all lines up to this cell
            for i in range(0, counter-1, 1):
                (x0, y0) = points[i]
                (x1, y1) = points[i+1]
                ax.plot([x0, x1], [y0, y1], color='red')
            cells = prev_cell_sensor_readings[counter2]
            cell_coords = list()
            for cell in cells:
                cell_coords.append(cell)
            for row, col in cell_coords:
                # b/c x = col & y = row
                ax.add_patch(plt.Rectangle((col, row), 1, 1, color='yellow'))
            plt.savefig(fig_path + f'_step{counter}_substep{counter2}.png')
            counter2 += 1
        
        # draw chosen point (curr)
        (prev_point_row, prev_point_col) = prev_point
        (curr_point_row, curr_point_col) = curr_point
        ax.plot([prev_point_row, curr_point_row], [prev_point_col, curr_point_col], color='red')  
        plt.savefig(fig_path + f'_step{counter}_Z.png')    
        #plt.show()
        counter += step_size
    
    # plot final trajectory
    fig_final, ax_final = draw_obstacles_from_SDF(sdf)
    ax_final.plot(first_point_x, first_point_y, marker='o', color='red')
    ax_final.plot(end_point_x, end_point_y, marker='o', markerfacecolor='none', markeredgecolor='red')
    for i in range(num_points-1):
        (x0, y0) = points[i]
        (x1, y1) = points[i+1]
        ax_final.plot([x0, x1], [y0, y1], color='red')
    plt.savefig(fig_path + '_completePath.png')    

    
def trace_incremental_traversal_with_sensor(sdfs, points, cells_with_sensor, steps):
    '''
    Draws steps of path traversal incrementally. Path traversal algorithm with initially unknown map.

    Args:
    - sdf: list of SDF objects, each is the local map of each step
    - points: centers of traversed cells
    - cells_with_sensor: mutable dictionary matching tuple cell keys to a list of groups of border cells referenced; 
                        e.g. [(row, col): [[borderCellsAtMaxRange], [borderCellsAtSmallerRange]...]
    - steps: number of steps to divide path tracing for viewing; to view all steps: steps = len(points) - 1
    '''
    num_points = len(points)
    counter = 1

    step_size = (num_points-1)//steps

    (first_point_x, first_point_y) = points[0]
    (end_point_x, end_point_y) = points[num_points-1]

    while counter < num_points:

        prev_point = points[counter-1]
        curr_point = points[counter]
        
        # now draw sensor readings (if end hasn't been reached yet)
        #if counter < num_points:
        prev_cell_tuple = (((int) (prev_point[1] - 0.5)), ((int) (prev_point[0] - 0.5)))
        prev_cell_sensor_readings = cells_with_sensor[prev_cell_tuple]
        num_groups = len(prev_cell_sensor_readings)
        counter2 = 0

        # Below while loop should draw the path up until prev_point, 
        # and then highlight all border cells considered until the next point (curr_point) is selected.
        f = plt.figure()
        while counter2 < num_groups:
            #print(np.flipud(sdfs[counter2].distances))
            fig, ax = draw_obstacles_from_SDF(sdfs[counter - 1])

            # plot start and end points
            ax.plot(first_point_x, first_point_y, marker='o', color='red')
            ax.plot(end_point_x, end_point_y, marker='o', markerfacecolor='none', markeredgecolor='red')

            # draw all lines up to this cell
            for i in range(0, counter-1, 1):
                (x0, y0) = points[i]
                (x1, y1) = points[i+1]
                ax.plot([x0, x1], [y0, y1], color='red')
            cells = prev_cell_sensor_readings[counter2]
            #print('cells: ', cells)
            cell_coords = list()
            for cell in cells:
                cell_coords.append(cell)
            for row, col in cell_coords:
                # b/c x = col & y = row
                ax.add_patch(plt.Rectangle((col, row), 1, 1, color='yellow'))
            counter2 += 1

        # draw chosen point (curr)
        (prev_point_row, prev_point_col) = prev_point
        (curr_point_row, curr_point_col) = curr_point
        ax.plot([prev_point_row, curr_point_row], [prev_point_col, curr_point_col], color='red')      
        plt.show(block=False)
        print()
        plt.pause(0.35)
        plt.close("all")
        counter += step_size
    
    # plot final trajectory
    fig_final, ax_final = draw_obstacles_from_SDF(sdfs[-1])
    ax_final.plot(first_point_x, first_point_y, marker='o', color='red')
    ax_final.plot(end_point_x, end_point_y, marker='o', markerfacecolor='none', markeredgecolor='red')
    for i in range(num_points-1):
        (x0, y0) = points[i]
        (x1, y1) = points[i+1]
        ax_final.plot([x0, x1], [y0, y1], color='red')
    plt.show()

def trace_incremental_traversal_with_sensor_savefig(sdfs, points, cells_with_sensor, steps, fig_path):
    '''
    Serves same purpose as trace_incremental_traversal_with_sensor, but saves each generated figure (of each step in the path traversal) to fig_path.

    Args:
    - sdfs: list of SDF objects, each is the local map of each step
    - points: centers of traversed cells
    - cells_with_sensor: mutable dictionary matching tuple cell keys to a list of groups of border cells referenced; 
                        e.g. [(row, col): [[borderCellsAtMaxRange], [borderCellsAtSmallerRange]...]
    - steps: number of steps to divide path tracing for viewing; to view all steps: steps = len(points) - 1
    - fig_path: str path to which figures will be saved to
    '''

    num_points = len(points)
    counter = 1

    step_size = (num_points-1)//steps

    (first_point_x, first_point_y) = points[0]
    (end_point_x, end_point_y) = points[num_points-1]

    while counter < num_points:

        prev_point = points[counter-1]
        curr_point = points[counter]
        
        # now draw sensor readings (if end hasn't been reached yet)
        prev_cell_tuple = (((int) (prev_point[1] - 0.5)), ((int) (prev_point[0] - 0.5)))
        prev_cell_sensor_readings = cells_with_sensor[prev_cell_tuple]
        num_groups = len(prev_cell_sensor_readings)
        counter2 = 0

        # Below while loop should draw the path up until prev_point, 
        # and then highlight all border cells considered until the next point (curr_point) is selected.
        f = plt.figure()
        while counter2 < num_groups:
            fig, ax = draw_obstacles_from_SDF(sdfs[counter - 1])

            # plot start and end points
            ax.plot(first_point_x, first_point_y, marker='o', color='red')
            ax.plot(end_point_x, end_point_y, marker='o', markerfacecolor='none', markeredgecolor='red')

            # draw all lines up to this cell
            for i in range(0, counter-1, 1):
                (x0, y0) = points[i]
                (x1, y1) = points[i+1]
                ax.plot([x0, x1], [y0, y1], color='red')
            cells = prev_cell_sensor_readings[counter2]
            cell_coords = list()
            for cell in cells:
                cell_coords.append(cell)
            for row, col in cell_coords:
                # b/c x = col & y = row
                ax.add_patch(plt.Rectangle((col, row), 1, 1, color='yellow'))
            plt.savefig(fig_path + f'_step{counter}_substep{counter2}.png')
            counter2 += 1

        # draw chosen point (curr)
        (prev_point_row, prev_point_col) = prev_point
        (curr_point_row, curr_point_col) = curr_point
        ax.plot([prev_point_row, curr_point_row], [prev_point_col, curr_point_col], color='red')      

        plt.savefig(fig_path + f'_step{counter}_Z.png')    
        counter += step_size
    
    # plot final trajectory
    fig_final, ax_final = draw_obstacles_from_SDF(sdfs[-1])
    ax_final.plot(first_point_x, first_point_y, marker='o', color='red')
    ax_final.plot(end_point_x, end_point_y, marker='o', markerfacecolor='none', markeredgecolor='red')
    for i in range(num_points-1):
        (x0, y0) = points[i]
        (x1, y1) = points[i+1]
        ax_final.plot([x0, x1], [y0, y1], color='red')
    plt.savefig(fig_path + '_completePath.png')   