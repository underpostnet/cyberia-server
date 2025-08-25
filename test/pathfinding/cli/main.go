package main

import (
	"fmt"
	"math/rand"
	"os"
	"strconv"
	"time"

	"github.com/spf13/cobra"
)

// Global variables for command-line flags.
var (
	startX, startY, endX, endY, gridW, gridH int
	objW, objH                               float64
	savePath                                 string
	loadPath                                 string
	show                                     bool
	aoiRadius                                float64 // New flag for Area of Interest radius
)

// newPathfinderFromArgs creates and configures a Pathfinder instance based on command-line arguments.
func newPathfinderFromArgs() *Pathfinder {
	// Pathfinder now handles object dimensions.
	pf := NewPathfinder(gridW, gridH, objW, objH)

	// Load or generate obstacles
	if loadPath != "" {
		fmt.Printf("Loading map from file: %s\n", loadPath)
		if err := pf.LoadObstacles(loadPath); err != nil {
			fmt.Println("Error loading obstacles:", err)
			os.Exit(1)
		}
		fmt.Println("Map loaded successfully!")
	} else {
		fmt.Println("Generating new map...")
		numObs := int((gridW * gridH) / 80)
		start := PointI{X: startX, Y: startY}
		end := PointI{X: endX, Y: endY}
		pf.GenerateMap(numObs, start, end)

		if savePath != "" {
			fmt.Printf("Saving map to file: %s\n", savePath)
			if err := pf.SaveObstacles(savePath); err != nil {
				fmt.Println("Error saving obstacles:", err)
				os.Exit(1)
			}
			fmt.Println("Map saved successfully!")
		}
	}

	return pf
}

// runPathfindingAndAnimation executes the A* algorithm and animates the result if requested.
func runPathfindingAndAnimation(pf *Pathfinder) {
	start := pf.start
	end := pf.end

	fmt.Println("--------------------------------------------------")
	fmt.Printf("  Grid size: %dx%d\n", gridW, gridH)
	fmt.Printf("  Object size: %.2fx%.2f\n", objW, objH)
	fmt.Printf("  Start: (%d, %d)  End: (%d, %d)\n", start.X, start.Y, end.X, end.Y)
	if aoiRadius > 0 {
		fmt.Printf("  AOI Radius: %.2f\n", aoiRadius)
	}
	fmt.Println("--------------------------------------------------")

	path, closestPoint, err := pf.findPath(start, end)
	if err != nil {
		fmt.Println("Error finding path:", err)
		os.Exit(1)
	}

	fmt.Println("Path found!")
	if show {
		fmt.Printf("Animating path to closest point (%d, %d)\n", closestPoint.X, closestPoint.Y)
		for i, p := range path {
			fmt.Printf("\033[H\033[2J") // Clear screen for animation
			fmt.Println(pf.AnimateGrid(p, path, aoiRadius))
			fmt.Printf("Step %d of %d: (%d, %d)\n", i+1, len(path), p.X, p.Y)
			time.Sleep(100 * time.Millisecond) // Simulate animation delay
		}
	}
}

var rootCmd = &cobra.Command{
	Use:   "cli",
	Short: "A command-line pathfinding animator",
	Long:  `A command-line tool that uses the A* algorithm to find and animate a path from a start to an end point on a grid.`,
	Run: func(cmd *cobra.Command, args []string) {
		// Read arguments if they are provided, otherwise use flags.
		if len(args) == 8 {
			var err error
			if startX, err = strconv.Atoi(args[0]); err != nil {
				fmt.Println("Invalid start-x argument.")
				os.Exit(1)
			}
			if startY, err = strconv.Atoi(args[1]); err != nil {
				fmt.Println("Invalid start-y argument.")
				os.Exit(1)
			}
			if endX, err = strconv.Atoi(args[2]); err != nil {
				fmt.Println("Invalid end-x argument.")
				os.Exit(1)
			}
			if endY, err = strconv.Atoi(args[3]); err != nil {
				fmt.Println("Invalid end-y argument.")
				os.Exit(1)
			}
			if objW, err = strconv.ParseFloat(args[4], 64); err != nil {
				fmt.Println("Invalid obj-w argument.")
				os.Exit(1)
			}
			if objH, err = strconv.ParseFloat(args[5], 64); err != nil {
				fmt.Println("Invalid obj-h argument.")
				os.Exit(1)
			}
			if gridW, err = strconv.Atoi(args[6]); err != nil {
				fmt.Println("Invalid grid-w argument.")
				os.Exit(1)
			}
			if gridH, err = strconv.Atoi(args[7]); err != nil {
				fmt.Println("Invalid grid-h argument.")
				os.Exit(1)
			}
		}

		rand.Seed(time.Now().UnixNano())

		// Create a new pathfinder instance
		pf := newPathfinderFromArgs()

		// Validate and set start and end points
		start := PointI{X: startX, Y: startY}
		end := PointI{X: endX, Y: endY}
		validateStartPoint(pf, &start)
		validateEndPoint(pf, &end)
		pf.SetStartAndEnd(start, end)

		// Run pathfinding and animation
		runPathfindingAndAnimation(pf)
	},
}

func init() {
	rootCmd.PersistentFlags().IntVar(&startX, "start-x", 10, "The starting X coordinate.")
	rootCmd.PersistentFlags().IntVar(&startY, "start-y", 10, "The starting Y coordinate.")
	rootCmd.PersistentFlags().IntVar(&endX, "end-x", 40, "The ending X coordinate.")
	rootCmd.PersistentFlags().IntVar(&endY, "end-y", 40, "The ending Y coordinate.")
	rootCmd.PersistentFlags().Float64Var(&objW, "obj-w", 1.0, "The width of the moving object.")
	rootCmd.PersistentFlags().Float64Var(&objH, "obj-h", 1.0, "The height of the moving object.")
	rootCmd.PersistentFlags().IntVar(&gridW, "grid-w", 50, "The width of the grid.")
	rootCmd.PersistentFlags().IntVar(&gridH, "grid-h", 50, "The height of the grid.")
	rootCmd.PersistentFlags().StringVar(&savePath, "save", "", "Path to save the generated obstacle map.")
	rootCmd.PersistentFlags().StringVar(&loadPath, "load", "", "Path to load a pre-existing obstacle map.")
	rootCmd.PersistentFlags().BoolVar(&show, "show", false, "Show the animated path.")
	rootCmd.PersistentFlags().Float64Var(&aoiRadius, "aoi", 0.0, "The radius for the Area of Interest (AOI).")
}

func main() {
	if err := rootCmd.Execute(); err != nil {
		fmt.Println(err)
		os.Exit(1)
	}
}
