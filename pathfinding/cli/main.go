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

	// Store the original, unclamped end point from user input for display purposes.
	originalEnd := PointI{int(math.Round(endXf)), int(math.Round(endYf))}

	// Clamp start and end coordinates to be within the grid bounds.
	// This prevents the program from trying to path to or from an impossible location.
	clamp := func(val, min, max float64) int {
		return int(math.Max(float64(min), math.Min(float64(max-1), val)))
	}

	start := PointI{clamp(startXf, 0, float64(gridW)), clamp(startYf, 0, float64(gridH))}
	end := PointI{clamp(endXf, 0, float64(gridW)), clamp(endYf, 0, float64(gridH))}

	// 2. Input Validation
	if gridW <= 1 || gridH <= 1 {
		fmt.Println("grid must be >1")
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
		fmt.Printf("  Start: (%d, %d)  Original End: (%d, %d)\n", start.X, start.Y, originalEnd.X, originalEnd.Y)
		fmt.Println("--------------------------------------------------")

		path, newEnd, err := pf.findPath(start, end)
		if err == nil {
			if newEnd.X != end.X || newEnd.Y != end.Y {
				fmt.Printf("Original end point not walkable. Finding path to closest walkable point at (%d, %d).\n", newEnd.X, newEnd.Y)
			}
			fmt.Println("Path found! Beginning animation.")
			// Animate the path
			for i, p := range path {
				printGrid(pf, p, path, i, newEnd)
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
func printGrid(pf *Pathfinder, current PointI, path []PointI, pathIndex int, newEnd PointI) {
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
			} else if x == newEnd.X && y == newEnd.Y {
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
