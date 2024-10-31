import math
import heapq
import sys

# Define global variables for the grid and direction
ROW, COL = 50, 30
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
        self.parent_i, self.parent_j = 0, 0
        self.f, self.g, self.h = float('inf'), float('inf'), 0

def is_valid(row, col):
    return (row >= 0) and (row < ROW) and (col >= 0) and (col < COL)

def is_unblocked(grid, row, col):
    return grid[row][col] != 1

def is_destination(row, col, dest):
    return row == dest[0] and col == dest[1]

def calculate_h_value(row, col, dest):
    return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

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

def trace_path(cell_details, dest):
    global depth, path, path_directions, path_f_values
    row, col = dest[0], dest[1]
    while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        path.append((row, col))
        parent_row, parent_col = cell_details[row][col].parent_i, cell_details[row][col].parent_j
        path_f_values.append(cell_details[row][col].f)

        # Calculate the direction from parent to current cell
        direction = (row - parent_row, col - parent_col)
        _, type = DIRECTIONS[direction]
        path_directions.append(type)

        row, col = parent_row, parent_col
    path.append((row, col))
    path_f_values.append(cell_details[row][col].f)  # Add f-value for the start
    path.reverse()
    path_directions.reverse()
    path_f_values.reverse()
    depth = len(path) - 1

def a_star_search(grid, src, dest, k):
    global path_f_values, path_directions, node_count
    if not is_valid(src[0], src[1]) or not is_valid(dest[0], dest[1]):
        return "Invalid source or destination"
    if not is_unblocked(grid, src[0], src[1]) or not is_unblocked(grid, dest[0], dest[1]):
        return "Source or destination is blocked"
    if is_destination(src[0], src[1], dest):
        return "Already at destination"
    
    closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
    cell_details = [[Cell() for _ in range(COL)] for _ in range(ROW)]
    i, j = src
    cell_details[i][j].f, cell_details[i][j].g = 0, 0
    cell_details[i][j].parent_i, cell_details[i][j].parent_j = i, j
    
    open_list = []
    heapq.heappush(open_list, (0.0, i, j, None))
    node_count = 0 

    while open_list:
        p = heapq.heappop(open_list)
        i, j, last_direction = p[1], p[2], p[3]
        closed_list[i][j] = True
        node_count += 1

        for dir in DIRECTIONS:
            new_i, new_j = i + dir[0], j + dir[1]
            if is_valid(new_i, new_j) and is_unblocked(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                if is_destination(new_i, new_j, dest):
                    cell_details[new_i][new_j].parent_i, cell_details[new_i][new_j].parent_j = i, j
                    trace_path(cell_details, dest)
                    return
                step_cost, new_direction = calculate_step_cost(i, j, new_i, new_j, last_direction, k)
                g_new = cell_details[i][j].g + step_cost
                h_new = calculate_h_value(new_i, new_j, dest)
                f_new = g_new + h_new

                if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                    heapq.heappush(open_list, (f_new, new_i, new_j, new_direction))
                    cell_details[new_i][new_j].f, cell_details[new_i][new_j].g, cell_details[new_i][new_j].h = f_new, g_new, h_new
                    cell_details[new_i][new_j].parent_i, cell_details[new_i][new_j].parent_j = i, j

    return "Failed to find the destination"


def load_grid_from_file(filename):
    grid = []
    src = None
    dest = None

    with open(filename, 'r') as file:
        first_line = file.readline().strip().split()
        src = (int(first_line[0]), int(first_line[1]))
        dest = (int(first_line[2]), int(first_line[3]))
        print("Source:", src)
        print("Destination:", dest)

        # Read grid from second line
        for line in file:
            row = list(map(int, line.strip().split()))
            grid.append(row)

    grid.reverse()
    grid = list(map(list, zip(*grid)))
    return grid, src, dest

def output_file(filename, grid):
    for row, col in path[1:-1]:
        grid[row][col] = 4
    grid[path[0][0]][path[0][1]] = 2
    grid[path[-1][0]][path[-1][1]] = 5
    grid = list(map(list, zip(*grid)))
    
    grid.reverse()
    
    with open(filename, 'w') as file:
        file.write(f"{depth}\n{node_count}\n{' '.join(map(str, path_directions))}\n{' '.join(map(str, path_f_values))}\n")
        for row in grid:
            file.write(" ".join(map(str, row)) + "\n")

def main():
    filename, output_filename, k_value = sys.argv[1], sys.argv[2], int(sys.argv[3])
    grid, src, dest = load_grid_from_file(filename)
    if src and dest:
        a_star_search(grid, src, dest, k_value)
        output_file(output_filename, grid)

if __name__ == "__main__":
    main()


