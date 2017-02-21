from OccupancyGrid import OccupancyGrid
from SearchGrid import SearchGrid
from GridDrawer import GridDrawer
from Cell import CellLabel
from PlannedPath import PlannedPath
import time
import math
from collections import deque

# This class implements the basic components of the forward search
# planning algorithm in LaValle's book and the lecture slides. The
# same general framework can be used to implement a wide array of
# algorithms, including depth first search, breadth first search,
# greedy (shortest distance first) search, Dijkstra and A*. The code
# here is written to be easy to uncerstand and is not optimised in any
# way.

# The code includes a number of hooks which do not appear in LaValle's
# description, but are useful when implementing some techniques. In
# addition, the code can optionally use a graphics library to draw the
# grid cells.

# The planner itself takes an occupancy map as an input. This
# specifies the structure of the environment - basically how big is
# it, which cells are blocked and which cells are open. The planner
# internally constructs a SearchGrid. This contains the nodes and
# edges from the planner and the labels associated with them.

class PlannerBase(object):

    # Construct a new planner object and set defaults.
    def __init__(this, occupancyGrid):
        this.occupancyGrid = occupancyGrid;
        this.searchGrid = None
        this.pauseTimeInSeconds = 0.05
        this.showGraphics = True
        this.showGraphicsEachIteration = False
        this.goalReached = None
        this.gridDrawer = None

    # This method pushes a cell onto the queue Q. Its implementation
    # depends upon the type of search algorithm used. If necessary,
    # this could also do things like update path costs as well.
    def pushCellOntoQueue(this, cell):
        raise NotImplementedError()

    # This method returns a boolean - true if the queue is empty,
    # false if it still has some cells on it. Its implementation
    # depends upon the the type of search algorithm used.
    def isQueueEmpty(this):
        raise NotImplementedError()

    # Handle the case that a cell has been visited already. This is
    # used by some algorithms to rewrite paths to identify the
    # shortest path.
    def resolveDuplicate(this, cell, parentCell):
        raise NotImplementedError()

    # This method finds the first cell (at the head of the queue),
    # removes it from the queue, and returns it. Its implementation
    # depends upon the the type of search algorithm used.
    def popCellFromQueue(this):
        raise NotImplementedError()

    # This method determines if the goal has been reached.
    def hasGoalBeenReached(this, cell):
        return cell == this.goal

    # This method gets the list of cells which potentially could be
    # visited next. Each candidate position has to be tested
    # separately.
    def getNextCellsToBeVisited(this, cell):

        # This stores the set of valid actions / cells
        cells = list();

        # Go through all the neighbours and add the cells if they
        # don't fall outside the grid and they aren't the cell we
        # started with. The order has been manually written down to
        # create a spiral.
        this.pushBackCandidateCellIfValid(cell, cells, 0, -1)
        this.pushBackCandidateCellIfValid(cell, cells, 1, -1)
        this.pushBackCandidateCellIfValid(cell, cells, 1, 0)
        this.pushBackCandidateCellIfValid(cell, cells, 1, 1)
        this.pushBackCandidateCellIfValid(cell, cells, 0, 1)
        this.pushBackCandidateCellIfValid(cell, cells, -1, 1)
        this.pushBackCandidateCellIfValid(cell, cells, -1, 0)
        this.pushBackCandidateCellIfValid(cell, cells, -1, -1)

        return cells

    # This helper method checks if the robot, at cell.coords, can move
    # to cell.coords+(offsetX, offsetY). Reasons why it can't do this
    # include falling off the edge of the map or running into an
    # obstacle.
    def pushBackCandidateCellIfValid(this, cell, cells, offsetX, offsetY):
        newX = cell.coords[0] + offsetX
        newY = cell.coords[1] + offsetY
        if ((newX >= 0) & (newX < this.occupancyGrid.getWidth()) \
            & (newY >= 0) & (newY < this.occupancyGrid.getHeight())):
            newCoords = (newX, newY)
            newCell = this.searchGrid.getCellFromCoords(newCoords)
            if (newCell.label != CellLabel.OBSTRUCTED):
                cells.append(newCell)

    # This method determines whether a cell has been visited already.
    def hasCellBeenVisitedAlready(this, cell):
        return (cell.label == CellLabel.OBSTRUCTED) | (cell.label == CellLabel.DEAD) \
            | (cell.label == CellLabel.ALIVE)

    # Mark that the cell has been visited. Also note the parent, which
    # is used to extract the path later on.
    def markCellAsVisitedAndRecordParent(this, cell, parentCell):
        cell.label = CellLabel.ALIVE
        cell.parent = parentCell

    # Mark that a cell is dead. A dead cell is one in which all of its
    # immediate neighbours have been visited.
    def markCellAsDead(this, cell):
        cell.label = CellLabel.DEAD
    
    # Draw the output and sleep for the pause time.
    def drawCurrentState(this):
        if (this.showGraphics == True):
            this.gridDrawer.update()
            time.sleep(this.pauseTimeInSeconds)

    # Set the pause time
    def setPauseTime(this, pauseTimeInSeconds):
        this.pauseTimeInSeconds = pauseTimeInSeconds
        
    # The main search routine. Given the input startCoords (x,y) and
    # goalCoords (x,y), compute a plan. Note that the coordinates
    # index from 0 and refer to the cell number.
    def plan(this, startCoords, goalCoords):

        # Empty the queue. This is needed to make sure everything is reset
        while (this.isQueueEmpty() == False):
            this.popCellFromQueue()
        
        # Create the search grid from the occupancy grid and seed
        # unvisited and occupied cells.
        if (this.searchGrid is None):
            this.searchGrid = SearchGrid.fromOccupancyGrid(this.occupancyGrid)
        else:
            this.searchGrid.setFromOccupancyGrid(this.occupancyGrid)

        # Get the start cell object and label it as such. Also set its
        # path cost to 0.
        this.start = this.searchGrid.getCellFromCoords(startCoords)
        this.start.label = CellLabel.START
        this.start.pathCost = 0

        # Get the goal cell object and label it.
        this.goal = this.searchGrid.getCellFromCoords(goalCoords)
        this.goal.label = CellLabel.GOAL

        # If required, set up the grid drawer and show the initial state
        if (this.showGraphics == True):
            if (this.gridDrawer is None):
                this.gridDrawer = GridDrawer(this.searchGrid);
            this.drawCurrentState()

        # Insert the start on the queue to start the process going.
        this.markCellAsVisitedAndRecordParent(this.start, None)
        this.pushCellOntoQueue(this.start)

        # Reset the count
        this.numberOfCellsVisited = 0

        # Indicates if we reached the goal or not
        this.goalReached = False
        
        # Iterate until we have run out of live cells to try or we reached the goal
        while (this.isQueueEmpty() == False):
            cell = this.popCellFromQueue()
            if (this.hasGoalBeenReached(cell) == True):
                this.goalReached = True
                break
            cells = this.getNextCellsToBeVisited(cell)
            for nextCell in cells:
                if (this.hasCellBeenVisitedAlready(nextCell) == False):
                    this.markCellAsVisitedAndRecordParent(nextCell, cell)
                    this.pushCellOntoQueue(nextCell)
                    this.numberOfCellsVisited = this.numberOfCellsVisited + 1
                else:
                    this.resolveDuplicate(nextCell, cell)

            # Now that we've checked all the actions for this cell,
            # mark it as dead
            this.markCellAsDead(cell)

            # Draw the update if required
            if (this.showGraphicsEachIteration == True):
                this.drawCurrentState()

        # Draw the final results if required
        this.drawCurrentState()

        if (this.goalReached == True):
            print "Reached the goal after visiting " + str(this.numberOfCellsVisited) + " cells"
        else:
            print "Could not reach the goal after visiting " + str(this.numberOfCellsVisited) + " cells"
            
        return this.goalReached


    # This method extracts a path from the pathEndCell to the start
    # cell. The path is a list actually sorted in the order:
    # cell(x_I), cell(x_1), ... , cell(x_K), cell(x_G). You can use
    # this method to try to find the path from any end cell. However,
    # depending upon the planner used, the results might not be
    # valid. In this case, the path will probably not terminate at the
    # start cell.
    def extractPath(this, pathEndCell):

        # Construct the path object and mark if the goal was reached
        path = PlannedPath()
        path.goalReached = this.goalReached
        
        # Initial condition - the goal cell
        path.waypoints.append(pathEndCell)
               
        # Start at the goal and find the parent
        cell = pathEndCell.parent
        
        # Iterate back through and extract each parent in turn and add
        # it to the path. To work out the travel length along the
        # path, you'll also have to add this at this stage.
        while (cell is not None):
            path.waypoints.appendleft(cell)
            cell = cell.parent

        # Now draw the path
        if (this.showGraphics == True):
            this.gridDrawer.update()
            this.gridDrawer.drawPath(path)

        # Return the path
        return path

    # Extract the path between the start and goal.
    def extractPathToGoal(this):
        path = this.extractPath(this.goal)
        return path

            
