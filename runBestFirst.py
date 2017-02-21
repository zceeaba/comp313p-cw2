#! /usr/bin/env python

from OccupancyGrid import OccupancyGrid
from BestFirstPlanner import BestFirstPlanner

# Create the occupancy grid
occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(1, 19):
    occupancyGrid.setCell(11, y, 1)

    
# Start and goal cells
start = (3, 18)
goal = (20, 0)

# Create the planner
planner = BestFirstPlanner(occupancyGrid);
planner.setPauseTime(0)

# Run it
planner.plan(start, goal)

# Pause
planner.gridDrawer.waitForKeyPress()

# Show the path
planner.extractPathToGoal()
planner.gridDrawer.waitForKeyPress()

