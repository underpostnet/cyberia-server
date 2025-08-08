# **Go Pathfinding CLI**

This is a command-line tool that demonstrates the **A**\* pathfinding algorithm. It generates a grid with random obstacles, finds the optimal path from a start to an end point, and can animate the path in the terminal.

## **Installation**

To run this application, you must have **Go** installed. Navigate to the project directory and run:

```bash
# You can run the application directly without building an executable
go run . main.go <arguments>
```

The program's dependencies (such as the **cobra** library) will be fetched automatically.

## **Usage**

You can run the program using either **positional arguments** or more descriptive **flags**.

### **Positional Arguments**

The simplest way to run the program is by providing all the arguments in a specific order:

```bash
go run . <start_x> <start_y> <end_x> <end_y> <obj_w> <obj_h> <grid_width> <grid_height>
```

**Example:**

```bash
go run . 10 10 40 40 2 2 50 50
```

### **Flags**

Using flags is more flexible, as you can specify only the arguments you need. All flags have default values.

* `--start-x`, `--start-y`: The starting coordinates.
* `--end-x`, `--end-y`: The ending coordinates.
* `--obj-w`, `--obj-h`: The width and height of the moving object.
* `--grid-w`, `--grid-h`: The dimensions of the grid.
* `--show`: A boolean flag to display the animated path in the console.
* `--save <file_path>`: Saves the generated obstacle map to a file.
* `--load <file_path>`: Loads a pre-existing obstacle map from a file.

**Examples:**

```bash
# Run with flags and animation
go run . --start-x 10 --start-y 10 --end-x 40 --end-y 40 --obj-w 2 --obj-h 2 --grid-w 50 --grid-h 50 --show

# Generate a map and save it to a file
go run . --save obstacle.json --grid-w 50 --grid-h 50

# Load a pre-saved map and find a path
go run . --load obstacle.json --start-x 15 --start-y 15 --end-x 35 --end-y 35 --show
```
