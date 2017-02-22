#! /usr/bin/env python

from OccupancyGrid import OccupancyGrid
from AStarPlanner import AStarPlanner

# Create the occupancy grid
occupancyGrid = OccupancyGrid(21, 21, 0.5)

for y in xrange(0, 19):
    occupancyGrid.setCell(11, y, 1)

    
# Start and goal cells
start = (3, 18)
goal = (20, 0)

heuristics = ["zero","euclidean","manhattan"]

for h in heuristics:

    print "\n\nHeuristic: %s" % h

    # Create the planner
    planner = AStarPlanner(occupancyGrid, h);
    planner.setPauseTime(0)

    # Run it
    planner.plan(start, goal)

    # Pause
    planner.gridDrawer.waitForKeyPress()

    # Show the path
    path = planner.extractPathToGoal()
    planner.gridDrawer.waitForKeyPress()


    print "Number of cells visited: %d" % planner.numberOfCellsVisited
    print "Number of Waypoints: %d" % path.getNumberOfWaypoints()
    print "Travel length of planned path: %f" % path.getTotalLength()
    print "Total angle turned on planned path: %d" % path.getTotalAngle()
