import matplotlib.pyplot as plt
import numpy as np

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

    plt.show()

def draw_obstacles_from_SDF(sdf):
    rows = sdf.rows
    cols = sdf.cols
    obstacle_grid = np.array([[0]*cols]*rows)

    for row in range(0, rows):
        for col in range(0, cols):
            if np.flipud(sdf.distances)[row, col] == 0:
                obstacle_grid[row, col] = 1
    
    draw_grid(obstacle_grid, rows, cols, 'Obstacles from SDF')
