from PlannerBase import PlannerBase
from Cell import CellLabel

import math

class DijkstraPlanner(PlannerBase):

    def __init__(this, occupancyGrid):
        PlannerBase.__init__(this, occupancyGrid)
        this.Queue = dict()

    # Path Cost is Euclidean Distance. Smaller score = better.
    # Add to dictionary with score as value
    def pushCellOntoQueue(this, cell):
        if cell.parent:
            cell.pathCost = cell.parent.pathCost + cell.distanceToCell(cell.parent)
        this.Queue[cell] = cell.pathCost
    
    # Check the queue size is zero
    def isQueueEmpty(this):
        return not this.Queue

    # Get cell with smallest score
    def popCellFromQueue(this):
        cell = min(this.Queue, key=this.Queue.get)
        del this.Queue[cell]
        return cell


    # If cell is alive, check for more efficient path
    def resolveDuplicate(this, cell, parentCell):
        if cell.label != CellLabel.DEAD:
            distance = parentCell.pathCost + cell.distanceToCell(parentCell)
            if distance < cell.pathCost:
                cell.pathCost = distance
                cell.parent = parentCell
                this.Queue[cell] = distance
            
