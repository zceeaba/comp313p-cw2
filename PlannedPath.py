from collections import deque

# This class stores the planned path

class PlannedPath(object):

    # Construct a new planner object and set defaults.
    def __init__(this):

        # Does the path actually reach the goal or not?
        this.goalReached = False
        
        # The list of waypoint cells, from start to finish, which make
        # up the path.
        this.waypoints = deque()

        # Performance information - number of waypoints, and the
        # travel cost of the path.
        this.numberOfWaypoints = 0
        this.pathTravelCost = 0

    def getNumberOfWaypoints(this):
        this.numberOfWaypoints = len(this.waypoints)
        return this.numberOfWaypoints

    # Sum distances between each cell
    def getTotalLength(this):
        
        total = 0
        prev = this.waypoints[0]
        for cell in this.waypoints:
            total += cell.distanceToCell(prev)
            prev = cell
        
        this.pathTravelCost = total

        return total
