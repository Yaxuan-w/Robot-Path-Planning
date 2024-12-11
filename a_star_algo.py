# Author: Alice W.
# Usage: python3 a_star_algo.py <input_file_name> <output_file_name> <k_value>
import math
import heapq
import sys

# -------- Global variables and initialization --------
# Grid size (rows x columns = 30 x 50)
ROW = 50
COL = 30
# Actions: (direction): (degree, number)
DIRECTIONS = {
    (0, 1): (0, 0),    
    (1, 1): (45, 1),  
    (1, 0): (90, 2),     
    (1, -1): (135, 3), 
    (0, -1): (180, 4),    
    (-1, -1): (225, 5),  
    (-1, 0): (270, 6),     
    (-1, 1): (315, 7)   
}

# Initialize global variables to capture output details
path = []
path_directions = []
path_f_values = []
depth = 0
node_count = 0

class Cell:
    def __init__(self):
        self.parent_i = 0  # Parent cell's row index
        self.parent_j = 0  # Parent cell's column index
        self.f = float('inf')  # Total cost of the cell: f(x) = g(x) + h(x)
        self.g = float('inf')  # Actual cost from start to this cell
        self.h = 0  # Heuristic cost from this cell to destination
        self.depth = 0

# -------- Helper functions --------
# Check if the coordinate valid
def is_valid(row, col):
    return (0 <= row < ROW) and (0 <= col < COL)

# Check if the coordinate being blocked
def is_unblocked(grid, row, col):
    return grid[row][col] != 1

# Check if the coordinate is the destination
def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

# -------- Actual cose --------
def angle_difference(theta1, theta2):
    delta_theta = abs(theta1 - theta2)
    if delta_theta > 180:
        delta_theta = 360 - delta_theta
    return delta_theta

def calculate_step_cost(i, j, new_i, new_j, last_direction, k):
    direction = (new_i - i, new_j - j)
    theta_new, new_type = DIRECTIONS[direction]
    theta_old, _ = DIRECTIONS.get(last_direction, (theta_new, new_type))
    angle_cost = k * (angle_difference(theta_old, theta_new) / 180) if last_direction else 0
    distance_cost = 1 if direction in [(0, 1), (0, -1), (1, 0), (-1, 0)] else math.sqrt(2)
    return angle_cost + distance_cost, direction

# -------- Heuristic value --------
# Calculate the heuristic value which is Euclidian distance between the current position and the 
# goal position
def calculate_h_value(row, col, dest):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

# -------- A* Search algorithm --------
# If the target node is found, the path is traced through the trace_path function. This function uses 
# the parent node information of each node to trace back from the target node to the starting point, 
# generates the path coordinates and direction, and saves the f value of each node for output.
def trace_path(cell_details, dest):
    global path, path_directions, path_f_values
    row = dest[0]
    col = dest[1]
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        parent_row = cell_details[row][col].parent_i
        parent_col = cell_details[row][col].parent_j
        path_f_values.append(round(cell_details[row][col].f, 3)) # Keep only three decimal places in output

        # Calculate the direction from parent to current cell
        direction = (row - parent_row, col - parent_col)
        _, type = DIRECTIONS[direction]
        path_directions.append(type)
        row = parent_row
        col = parent_col

    path.append((row, col))
    path_f_values.append(cell_details[row][col].f)  # Add f-value for the start
    path.reverse()
    path_directions.reverse()
    path_f_values.reverse()

# - cell_details: Used to store the f, g, h values ​​of each grid node (the evaluation function in A*), 
# as well as the parent node position of the node (for path backtracking)
# - closed_list: Mark processed nodes
# - open_list: Used to store nodes that have not yet been processed
def a_star_search(grid, src, dest, k):
    global depth, path_f_values, path_directions, node_count
    # Check validation
    if not is_valid(src[0], src[1]) or not is_valid(dest[0], dest[1]):
        return "Invalid source or destination coordinates"
    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        return "Source or destination coordinate is blocked"
    if is_destination(src[0], src[1], dest):
        return "Already at destination"
    
    # Initialization
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]

    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]

    # Set the src coordinate as start point and set itself as parent node
    i, j = src
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].parent_i = i
    cell_details[i][j].parent_j = j
    
    # Initialize the open list (cells to be visited) with the start cell
    open_list = []
    heapq.heappush(open_list, (0.0, i, j, None)) # Push the start node
    node_count = 0 

    while open_list:
        p = heapq.heappop(open_list)
        i = p[1]
        j = p[2]
        last_direction = p[3]
        closed_list[i][j] = True
        node_count += 1
        # For each possible direction dir of the current node, calculate iterately the coordinates of the 
        # neighboring 
        # node: (new_i, new_j)
        for dir in DIRECTIONS:
            new_i = i + dir[0]
            new_j = j + dir[1]
            if is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                if is_destination(new_i, new_j, dest):
                    # Get depth directly
                    depth = cell_details[i][j].depth + 1
                    # Also calculate f/g/h values for the destination
                    g_new = cell_details[i][j].g + step_cost
                    h_new = calculate_h_value(new_i, new_j, dest)
                    f_new = g_new + h_new
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    # If the neighboring node is destination, use trace_path() to get the actual path
                    trace_path(cell_details, dest)
                    return
                
                step_cost, new_direction = calculate_step_cost(i, j, new_i, new_j, last_direction, k)
                g_new = cell_details[i][j].g + step_cost
                h_new = calculate_h_value(new_i, new_j, dest)
                f_new = g_new + h_new

                if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                    heapq.heappush(open_list, (f_new, new_i, new_j, new_direction))
                    cell_details[new_i][new_j].f = f_new
                    cell_details[new_i][new_j].g = g_new
                    cell_details[new_i][new_j].h = h_new
                    cell_details[new_i][new_j].parent_i = i
                    cell_details[new_i][new_j].parent_j = j
                    cell_details[new_i][new_j].depth = cell_details[i][j].depth + 1

    return "Failed to find the destination"

# -------- Loading input file --------
def load_grid_from_file(filename):
    grid = []
    src = None
    dest = None

    with open(filename, 'r') as file:
        # Get the src and dest coordinates
        first_line = file.readline().strip().split()
        src = (int(first_line[0]), int(first_line[1]))
        dest = (int(first_line[2]), int(first_line[3]))
        print("Source:", src)
        print("Destination:", dest)

        # Read grid from second line
        for line in file:
            row = list(map(int, line.strip().split()))
            grid.append(row)

    # Since The coordinates of the lower-left corner cell are (i, j) = (0,0), we need to reverse the input 
    # grid
    grid.reverse()
    grid = list(map(list, zip(*grid)))

    return grid, src, dest

# -------- Generate output file --------
def output_file(filename, grid):
    # Set the path to 4, src to 2, and dest to 5 in grid
    for row, col in path[1:-1]:
        grid[row][col] = 4
    grid[path[0][0]][path[0][1]] = 2
    grid[path[-1][0]][path[-1][1]] = 5

    # Reverse back
    grid = list(map(list, zip(*grid)))
    grid.reverse()
    
    with open(filename, 'w') as file:
        file.write(f"{depth}\n{node_count}\n{' '.join(map(str, path_directions))}\n{' '.join(map(str, path_f_values))}\n")
        for row in grid:
            file.write(" ".join(map(str, row)) + "\n")

# -------- Main function --------
# Usage: python3 a_star_algo.py <input_file_name> <output_file_name> <k_value>
def main():
    filename = sys.argv[1]
    output_filename = sys.argv[2]
    k_value = int(sys.argv[3])
    grid, src, dest = load_grid_from_file(filename)
    if src and dest:
        a_star_search(grid, src, dest, k_value)
        output_file(output_filename, grid)

if __name__ == "__main__":
    main()


