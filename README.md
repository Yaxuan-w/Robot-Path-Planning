# A* Pathfinding Algorithm for Robot Path Planning

The A* Search Algorithm with graph search (no repeated states) is implemented for solving a robot path planning problem. The objective is to determine the lowest-cost path for a point robot to navigate from a start position to a goal position in a 2D workspace, while avoiding obstacles and minimizing the movement cost.

## Description

- **A-star Search Algorithm:** Finds the optimal path between source and destination based on `f(x) = g(x) + h(x)`.
- **Customizable Turn Cost:** Adds an angle-based penalty to turning, controlled by a user-defined parameter `k`.
- **Grid Representation:** Supports blocked and unblocked cells with a visualizable grid output.
- **Heuristic:** Uses Euclidean distance as the heuristic to estimate the cost to the destination.

## Input and Output

### Input File Format

The input file should be a text file with the following format:

1. First line: Source and destination coordinates as integers.

```txt
<src_row> <src_col> <dest_row> <dest_col>
```

2. Subsequent lines: Grid representation with 0 for unblocked cells and 1 for blocked cells.

```txt
0 0 0 1 ...
0 1 0 0 ...
...
```

### Output File Format

The output file contains the following information:

- Depth: Length of the solution path.
- Node Count: Total nodes explored.
- Directions: Sequence of directions taken along the path.
- f-values: f-values of the nodes on the path.
- Grid: Updated grid with:
    - 2 for the source.
    - 5 for the destination.
    - 4 for the path.

## Usage

### Requirements

Python 3.x

### Command

```sh
python3 a_star_algo.py <input_file_name> <output_file_name> <k_value>
```

- `<input_file_name>`: Path to the input file containing the grid and source/destination coordinates.
- `<output_file_name>`: Path to save the output file with results and updated grid.
- `<k_value>`: Coefficient for the angle-based turn cost.

### Visualizing the Grid

`vis.py` can be used to visualize the workspace grid from either the input or output files.

#### Steps for Visualization:

##### Prepare the File:

- For input files, remove the first line containing start and goal positions.
- For output files, remove the first four lines containing depth, node count, directions, and f-values.
- Ensure the file contains only the workspace grid (30 x 50).

##### Run the Visualization Script:

```sh
python vis.py <file_name>
```

- `<file_name>`: Path to the modified file for visualization.

This script will display the workspace grid in a graphical format.
