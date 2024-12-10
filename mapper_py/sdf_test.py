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
    
    Args: 
    - map_name: string of map png filename
    '''
    np.set_printoptions(threshold = np.inf)
    np.set_printoptions(linewidth = np.inf)
    
    #test_sdf(map_name)
    #test_robot(map_name, (31, 23), (37, 8))
    #test_robot(map_name, (31, 23), (37, 8))
    test_robot(map_name, (0, 4), (6, 23))
    test_robot(map_name, (0, 2), (37, 8))

def test_sdf(map_name):
    '''
    pathfinding with absolute knowledge (assuming robot has knowledge of complete sdf grid at initialization)

    Args:
    - map_name: string of map png filename
    '''

    # path for belle
    # png_map_path = rf'C:\Users\litin\OneDrive\Desktop\MRAL_Final_Project\mapper_py\test_data\{map_name}.png'
    
    # path for nick
    png_map_path = "test_data/" + map_name + ".png"
    grid = Grid2D(0.5, 30, 40, 0.001, 0.999)
    grid = png_to_grid2d(grid, png_map_path)

    sdf = SDF(grid)
    #print(np.flipud(sdf.distances))
    #print(np.count_nonzero(sdf.distances == -1))

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
    - start: start cell in tuple form (row, col)
    - end: end cell in tuple form (row, col)
    - sensor_range: max range of sensor
    """
    borderCellGroups = dict()
    traversedCells = sdf.traverse_dummy_improved(start, end, sensor_range, borderCellGroups)
    traversedPoints = convert_tuple_cells_to_points(traversedCells) 
    trace_traversal_with_sensor(sdf, traversedPoints, borderCellGroups, len(traversedPoints)-1)

def test_robot(map_name, start, end):
    '''
    pathfinding with incremental knowledge (assuming robot updates local knowledge of sdf grid as path traversal progresses)

    Args:
    - map_name: string of map png filename
    - start: start cell in tuple form (row, col)
    - end: end cell in tuple form (row, col)
    '''

    # path for belle
    # png_map_path = rf'C:\Users\litin\OneDrive\Desktop\MRAL_Final_Project\mapper_py\test_data\{map_name}.png'
    
    # path for nick
    png_map_path = "test_data/" + map_name + ".png"
    
    #initializing robot object
    local_grid = Grid2D(0.5, 30, 40, 0.001, 0.999)
    global_grid = Grid2D(0.5, 30, 40, 0.001, 0.999)
    global_grid = png_to_grid2d(global_grid, png_map_path)
    robot = Robot(global_grid, local_grid, start)

    global_sdf = SDF(global_grid)
    
    # drawing grids
    (fig1, ax1) = draw_grid(np.flipud(global_sdf.distances), global_sdf.rows, global_sdf.cols, 'SDF Grid', 'Greys_r')
    plt.show()
    (fig2, ax2) = draw_obstacles_from_SDF(global_sdf)
    plt.close()

    test_incremental_path_finding(robot, end)

def test_incremental_path_finding(r, end):
    '''
    pathfinding with incremental knowledge (assuming robot updates local knowledge of sdf grid as path traversal progresses),
    helper function.

    Args:
    - r: Robot object
    - end: end cell in tuple form (row, col)
    '''

    borderCellGroups = dict()
    traversedCells, sdfs = r.traverse3(end, borderCellGroups)
    traversedPoints = convert_tuple_cells_to_points(traversedCells) 
    trace_incremental_traversal_with_sensor(sdfs, traversedPoints, borderCellGroups, len(traversedPoints)-1)


# version with figure saving
def test_savefigs(map_name, grid_visible=True):
    '''
    overall test file, but with added functionality of figure saving.
    
    Args: 
    - map_name: string of map png filename
    '''
    np.set_printoptions(threshold = np.inf)
    np.set_printoptions(linewidth = np.inf)

    image_folder_path = rf"C:\Users\litin\OneDrive\Desktop\MRAL_Final_Project-1\generated_plots\exploration_too\{map_name}"
    
    test_robot_savefigs(map_name, (0, 4), (6, 23), image_folder_path + '_test1')
    test_robot_savefigs(map_name, (0, 2), (37, 8), image_folder_path + '_test2')

def test_robot_savefigs(map_name, start, end, fig_path):
    '''
    pathfinding with incremental knowledge (assuming robot updates local knowledge of sdf grid as path traversal progresses),
    but with added functionality of figure saving.

    Args:
    - map_name: string of map png filename
    - start: start cell in tuple form (row, col)
    - end: end cell in tuple form (row, col)
    - fig_path: str path to which figures will be saved to
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
    
    # drawing grids
    (fig1, ax1) = draw_grid(np.flipud(global_sdf.distances), global_sdf.rows, global_sdf.cols, 'SDF Grid', 'Greys_r')
    plt.show()
    (fig2, ax2) = draw_obstacles_from_SDF(global_sdf)
    plt.close()

    test_incremental_path_finding_savefigs(robot, end, fig_path)

def test_incremental_path_finding_savefigs(r, end, fig_path):
    '''
    pathfinding with incremental knowledge (assuming robot updates local knowledge of sdf grid as path traversal progresses),
    but with added functionality of figure saving, helper function.

    Args:
    - r: Robot object
    - end: end cell in tuple form (row, col)
    - fig_path: str path to which figures will be saved to
    '''
    borderCellGroups = dict()
    traversedCells, sdfs = r.traverse3(end, borderCellGroups)
    traversedPoints = convert_tuple_cells_to_points(traversedCells) 
    trace_incremental_traversal_with_sensor_savefig(sdfs, traversedPoints, borderCellGroups, len(traversedPoints)-1, fig_path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='simple_obstacle')

    args = parser.parse_args()
    test(args.map)
    test("obs1")
    #test_savefigs("obs1")

    trav_fig, trav_ax = plt.subplots()
