package main

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"strconv"
	"time"
)

// main is the CLI runner for the pathfinding algorithm.
// It handles argument parsing, map generation, and animation.
func main() {
	// 1. Argument Parsing
	if len(os.Args) != 9 {
		fmt.Println("Usage: go run main.go <start_x> <start_y> <end_x> <end_y> <obj_w> <obj_h> <grid_width> <grid_height>")
		os.Exit(1)
	}

	parse := func(s string) float64 {
		v, err := strconv.ParseFloat(s, 64)
		if err != nil {
			fmt.Printf("Invalid arg: %s\n", s)
			os.Exit(1)
		}
		return v
	}

	startXf := parse(os.Args[1])
	startYf := parse(os.Args[2])
	endXf := parse(os.Args[3])
	endYf := parse(os.Args[4])
	objW := parse(os.Args[5])
	objH := parse(os.Args[6])
	gridWf := parse(os.Args[7])
	gridHf := parse(os.Args[8])

	gridW := int(gridWf)
	gridH := int(gridHf)
	start := PointI{int(math.Round(startXf)), int(math.Round(startYf))}
	end := PointI{int(math.Round(endXf)), int(math.Round(endYf))}

	// 2. Input Validation
	if gridW <= 1 || gridH <= 1 {
		fmt.Println("grid must be >1")
		os.Exit(1)
	}
	if start.X < 0 || start.X >= gridW || start.Y < 0 || start.Y >= gridH {
		fmt.Println("start out of bounds")
		os.Exit(1)
	}
	if end.X < 0 || end.X >= gridW || end.Y < 0 || end.Y >= gridH {
		fmt.Println("end out of bounds")
		os.Exit(1)
	}

	rand.Seed(time.Now().UnixNano())

	// 3. Pathfinding Attempts and Animation
	const maxAttempts = 10
	for attempt := 1; attempt <= maxAttempts; attempt++ {
		pf := NewPathfinder(gridW, gridH)
		// Decreased obstacle density for a higher chance of finding a path
		numObs := int((gridW * gridH) / 80)
		pf.GenerateMap(numObs, start, end, objW, objH)

		fmt.Println("--------------------------------------------------")
		fmt.Printf("Attempt %d of %d:\n", attempt, maxAttempts)
		fmt.Printf("  Grid size: %dx%d\n", gridW, gridH)
		fmt.Printf("  Object size: %.2fx%.2f\n", objW, objH)
		fmt.Printf("  Start: (%d, %d)  End: (%d, %d)\n", start.X, start.Y, end.X, end.Y)
		fmt.Println("--------------------------------------------------")

		path, err := pf.findPath(start, end)
		if err == nil {
			fmt.Println("Path found! Beginning animation.")
			// Animate the path
			for i, p := range path {
				printGrid(pf, p, path, i)
			}
			fmt.Println("--------------------------------------------------")
			fmt.Println("Animation complete. Total path length:", len(path), "steps.")
			os.Exit(0)
		}

		fmt.Printf("Error in attempt %d: %v. Retrying...\n", attempt, err)
	}

	fmt.Println("--------------------------------------------------")
	fmt.Printf("Failed to find a path after %d attempts. Exiting.\n", maxAttempts)
	os.Exit(1)
}

// printGrid animates the path by printing the grid at each step.
func printGrid(pf *Pathfinder, current PointI, path []PointI, pathIndex int) {
	// ANSI escape codes to clear the screen and move the cursor to the home position
	fmt.Print("\033[2J\033[H")

	fmt.Println("--------------------------------------------------")
	fmt.Printf("Animating pathfinding... Step %d\n", pathIndex+1)
	fmt.Printf("Current Position: (%d, %d)\n", current.X, current.Y)
	fmt.Println("--------------------------------------------------")

	// Calculate the rendering size of the object
	renderW := int(math.Floor(pf.objectW))
	renderH := int(math.Floor(pf.objectH))
	if renderW < 1 {
		renderW = 1
	}
	if renderH < 1 {
		renderH = 1
	}

	// Calculate the top-left corner of the object for rendering
	offsetX := int(math.Floor(float64(current.X) - float64(renderW)/2.0 + 0.5))
	offsetY := int(math.Floor(float64(current.Y) - float64(renderH)/2.0 + 0.5))

	for y := 0; y < pf.gridH; y++ {
		for x := 0; x < pf.gridW; x++ {
			isPath := false
			for i := 0; i <= pathIndex; i++ {
				if path[i].X == x && path[i].Y == y {
					isPath = true
					break
				}
			}

			// Check if the current grid cell is part of the object's rendering box
			isObject := x >= offsetX && x < offsetX+renderW && y >= offsetY && y < offsetY+renderH

			if isObject {
				fmt.Print("X ") // Moving object (simplified visual)
			} else if x == pf.start.X && y == pf.start.Y {
				fmt.Print("S ")
			} else if x == pf.end.X && y == pf.end.Y {
				fmt.Print("E ")
			} else if pf.grid[y][x] == 1 {
				fmt.Print("● ") // Obstacle
			} else if isPath {
				fmt.Print("+ ") // Path trail
			} else {
				fmt.Print(". ") // Empty space
			}
		}
		fmt.Println()
	}
	time.Sleep(50 * time.Millisecond) // Adjust sleep time to change animation speed
}
