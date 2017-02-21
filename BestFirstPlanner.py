from PlannerBase import PlannerBase

import math

class BestFirstPlanner(PlannerBase):

    def __init__(this, occupancyGrid):
        PlannerBase.__init__(this, occupancyGrid)
        this.Queue = dict()

    # Score is Euclidean Distance to goal. Smaller score = better.
    # Add to dictionary with score as value
    def pushCellOntoQueue(this, cell):
        score = cell.distanceToCell(this.goal)
        this.Queue[cell] = score
    
    # Check the queue size is zero
    def isQueueEmpty(this):
        return not this.Queue

    # Get cell with smallest score
    def popCellFromQueue(this):
        cell = min(this.Queue, key=this.Queue.get)
        del this.Queue[cell]
        return cell

    def resolveDuplicate(this, cell, parentCell):
        # Nothing to do in this case
        pass
