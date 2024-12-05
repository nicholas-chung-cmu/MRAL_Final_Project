import argparse
from cprint import cprint
import numpy as np
import sys
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator

from data_structures.grid import Grid2D, Point, Cell
from data_structures.sdf import SDF
from data_structures.robot import Robot
from utils import png_to_grid2d, visualize

from plotting_functions import *

def test(map_name, grid_visible=True):
    '''
    overall test file
    input:
    map_name: string of map png filename
    '''
    np.set_printoptions(threshold = np.inf)
    np.set_printoptions(linewidth = np.inf)
    
    #test_sdf(map_name)
    #test_robot(map_name, (31, 23), (37, 8))
    #test_robot(map_name, (31, 23), (37, 8))
    test_robot(map_name, (0, 4), (6, 23))
    test_robot(map_name, (0, 2), (37, 8))

    #rows = grid.height
    #cols = grid.width

def test_sdf(map_name):
    '''
    pathfinding with absolute knowledge
    '''

    # path for belle
    #png_map_path = rf'C:\Users\litin\OneDrive\Desktop\MRAL_Final_Project\mapper_py\test_data\{map_name}.png'
    
    # path for nick
    png_map_path = "test_data/" + map_name + ".png"
    grid = Grid2D(0.5, 30, 40, 0.001, 0.999)
    grid = png_to_grid2d(grid, png_map_path)

    sdf = SDF(grid)
    print(np.flipud(sdf.distances))
    print(np.count_nonzero(sdf.distances == -1))

    # drawing grids
    (fig1, ax1) = draw_grid(np.flipud(sdf.distances), sdf.rows, sdf.cols, 'SDF Grid', 'Greys_r')
    plt.show()
    #ax1.add_patch(plt.Rectangle((5,5), 1, 1, color='yellow')) #test
    (fig2, ax2) = draw_obstacles_from_SDF(sdf)
    plt.close()

    test_path_finding(sdf, (31, 23), (37, 8), 25)
    test_path_finding(sdf, (31, 23), (37, 8), 5)
    test_path_finding(sdf, (0, 4), (31, 23), 25)
    test_path_finding(sdf, (0, 2), (37, 8), 25)

def test_path_finding(sdf, start, end, sensor_range):
    """
    Tests and plots path finding algorithm, with each step plotted.
    
    Args:
    - start: start cell in tuple from (row, col)
    - end: end cell in tuple form (row, col)
    - sensor_range: max range of sensor
    """
    borderCellGroups = dict()
    traversedCells = sdf.traverse_dummy_improved(start, end, sensor_range, borderCellGroups)
    traversedPoints = convert_tuple_cells_to_points(traversedCells) 
    trace_traversal_with_sensor(sdf, traversedPoints, borderCellGroups, len(traversedPoints)-1)

def test_robot(map_name, start, end):
    '''
    pathfinding with incremental knowledge
    '''

    # path for belle
    #png_map_path = rf'C:\Users\litin\OneDrive\Desktop\MRAL_Final_Project\mapper_py\test_data\{map_name}.png'
    
    # path for nick
    png_map_path = "test_data/" + map_name + ".png"
    
    #initializing robot object
    local_grid = Grid2D(0.5, 30, 40, 0.001, 0.999)
    global_grid = Grid2D(0.5, 30, 40, 0.001, 0.999)
    global_grid = png_to_grid2d(global_grid, png_map_path)
    robot = Robot(global_grid, local_grid, start)

    global_sdf = SDF(global_grid)
    print(np.flipud(global_sdf.distances))
    print(np.count_nonzero(global_sdf.distances == -1))
    
    # drawing grids
    (fig1, ax1) = draw_grid(np.flipud(global_sdf.distances), global_sdf.rows, global_sdf.cols, 'SDF Grid', 'Greys_r')
    plt.show()
    #ax1.add_patch(plt.Rectangle((5,5), 1, 1, color='yellow')) #test
    (fig2, ax2) = draw_obstacles_from_SDF(global_sdf)
    plt.close()

    test_incremental_path_finding(robot, end)

def test_incremental_path_finding(r, end):
    '''
    inputs:
    start: tuple
    end: tuple
    '''

    borderCellGroups = dict()
    traversedCells, sdfs = r.traverse3(end, borderCellGroups)
    traversedPoints = convert_tuple_cells_to_points(traversedCells) 
    trace_incremental_traversal_with_sensor(sdfs, traversedPoints, borderCellGroups, len(traversedPoints)-1)


'''
def test_traversal(grid_ax, start=Point(1.2, 1.2), end=Point(2.2, 1.5), test_file='traced_cells_1',
                   c='navy', grid_visible=True):
    # Initialize an empty grid
    grid = Grid2D(0.1, 40, 40, 0.001, 0.999)

    # Helper function to highlight a cell
    def highlight_cell(x, y, res, ax=None, **kwargs):
        rect = plt.Rectangle((x, y), res, res, fill=True, **kwargs)
        ax = ax or plt.gca()
        ax.add_patch(rect)
        return rect

    # Helper function to plot the traced cells on the grid
    def plot_test_output(traced_cells, ax=None):
        for t in traced_cells:
            p = grid.cell_to_point(t)
            highlight_cell(p.x, p.y, grid.resolution, ax=ax,
                           color=c, alpha=0.2, linewidth=0)

    # Visualize the empty grid
    visualize(grid, ax=grid_ax)

    # Plot the ray from start to end
    grid_ax.plot([start.x, end.x], [start.y, end.y], color=c)

    # Get the traced cells
    success, traced_cells = grid.traverse(start, end)

    # First, the traverse function should succeed
    # If your code fails this check, check your traverse function implementation
    if success:
        cprint.info(f"traverse function succeeded, number of traced cells: {len(traced_cells)}")
    else:
        cprint.err("traverse function failed.", interrupt=True)

    # Then, check if the traced cells are correct
    # If your code fails this check, check your traverse function implementation
    traced_cells_np = np.array([a.to_numpy() for a in traced_cells])
    traced_cells_np_correct = np.load(f'test_data/{test_file}.npz')['traced_cells']

    #print('Correct Cells: ', traced_cells_np_correct)
    #print('Traced Cells: ', traced_cells_np)
    if (np.abs(traced_cells_np_correct - traced_cells_np) < 1e-6).all():
        cprint.info('test_traversal successful.')
    else:
        cprint.err('test_traversal failed.', interrupt=False)

    plot_test_output(traced_cells, ax=grid_ax)

    grid_ax.xaxis.set_major_locator(MultipleLocator(1.0))
    grid_ax.yaxis.set_major_locator(MultipleLocator(1.0))
    grid_ax.xaxis.set_minor_locator(MultipleLocator(grid.resolution))
    grid_ax.yaxis.set_minor_locator(MultipleLocator(grid.resolution))
    if grid_visible:
        grid_ax.grid(which='major', axis='both', linestyle='-')
        grid_ax.grid(which='minor', axis='both', linestyle='-')
    grid_ax.set_xlabel('Cell Space: Cols, Point Space: X (meters)')
    grid_ax.set_ylabel('Cell Space: Rows, Point Space: Y (meters)')
    grid_ax.set_aspect('equal')
    grid_ax.set_xlim([0.0, grid.resolution * grid.width])
    grid_ax.set_ylim([0.0, grid.resolution * grid.height])
'''

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='simple_obstacle')

    args = parser.parse_args()
    #test(args.map)
    test("obs1")

    '''png_map_path = "test_data/" + "obs1" + ".png"
    grid = Grid2D(0.5, 30, 40, 0.001, 0.999)
    grid = png_to_grid2d(grid, png_map_path)

    sdf = SDF(grid)
    sc = sdf.copy()

    sdf.distances[0][0] = 99

    print(sdf.distances)
    print(sc.distances)'''

    trav_fig, trav_ax = plt.subplots()
