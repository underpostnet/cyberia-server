

### Flags

* `--start-x`, `--start-y`: The starting coordinates.
* `--end-x`, `--end-y`: The ending coordinates.
* `--obj-w`, `--obj-h`: The width and height of the moving object.
* `--grid-w`, `--grid-h`: The dimensions of the grid.
* `--show`: A boolean flag to display the animated path in the console.
* `--save <file_path>`: Saves the generated obstacle map to a file.
* `--load <file_path>`: Loads a pre-existing obstacle map from a file.

### Examples

```bash
# Run with flags and animation
go run . --start-x 10 --start-y 10 --end-x 40 --end-y 40 --obj-w 2 --obj-h 2 --grid-w 50 --grid-h 50 --show

# Generate a map and save it to a file
go run . --save obstacle.json --grid-w 50 --grid-h 50

# Load a pre-saved map and find a path
go run . --load obstacle.json --start-x 15 --start-y 15 --end-x 35 --end-y 35 --show
```
