from PlannerBase import PlannerBase
from Cell import CellLabel

import math

class AStarPlanner(PlannerBase):

    def __init__(this, occupancyGrid, heuristic, hscale=0):
        PlannerBase.__init__(this, occupancyGrid)
        this.Queue = dict()
        this.Heuristic = heuristic
        this.Hscale = hscale

    # Add to dictionary with score as value
    def pushCellOntoQueue(this, cell):
        if cell.parent:
            cell.pathCost = cell.parent.pathCost + cell.distanceToCell(cell.parent)
        this.Queue[cell] = cell.pathCost + this.heuristic(cell)
    
    def heuristic(this, cell):
        if this.Heuristic == "zero":
            h = 0
        
        if this.Heuristic == "euclidean":
            h = cell.distanceToCell(this.goal)

        if this.Heuristic == "manhattan":
            x = abs(cell.coords[0]-this.goal.coords[0])
            y = abs(cell.coords[1]-this.goal.coords[1])
            h = x+y


        if this.Heuristic == "diagonal":
            x = abs(cell.coords[0]-this.goal.coords[0])
            y = abs(cell.coords[1]-this.goal.coords[1])
            h = (x+y) + (math.sqrt(2) - 2)*min(x,y)

        return h * (1.0 + this.Hscale)

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
                this.Queue[cell] = cell.pathCost + this.heuristic(cell)
            
