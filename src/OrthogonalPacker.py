from ortools.sat.python import cp_model
from ortools.sat.python.cp_model import Domain
import collections

import json

from gurobipy import GRB
import DataManager
from PlacementPoints import *
from Enums import *

class BinPacking2D:

    def __init__(self):
        self.xStarts = {}
        self.Positions = {}

        self.Intervals = collections.defaultdict(list)
        self.PartWidths = {}
        self.PartLengths = {}

        self.VariantUsed = {}
        self.BuildUsed = {}

        self.model = cp_model.CpModel()
        self.solver = cp_model.CpSolver()

        self.PlacementGenerator = PlacementPointGenerator()
    
    def AddParts(self, parts):
        self.Parts = parts
    
    def AddBin(self, machine):
        self.Bin = machine
    
    def CreateVariables(self, placementStrategy):
        position = collections.namedtuple('position', 'xStart xEnd yStart yEnd')

        if placementStrategy == PlacementPointStrategy.MinimalMeetInTheMiddle:
            allPlacementPointsX, allPlacementPointsY = self.PlacementGenerator.DetermineMinimalMeetInTheMiddlePatterns(self.Parts, self.Bin)
        
        for i, part in enumerate(self.Parts):
 
            for variant in part.Variants:
                tmpIntervalName = '(' + str(i) + ',' + str(variant.VariantId) + ')'
                self.VariantUsed[(i, variant.VariantId)] = self.model.NewBoolVar('VariantUsed' + tmpIntervalName)
        #solutions = {}
        for i, part in enumerate(self.Parts):
            sol = {}
            tmpIntervalName = str(i)

            partsReduced = [other for other in self.Parts if other != part]

            if placementStrategy == PlacementPointStrategy.NoStrategy:
                placementPointsX = range(0, self.Bin.Width - min([variant.Width for variant in part.Variants]) + 1)
                placementPointsY = range(0, self.Bin.Length - min([variant.Length for variant in part.Variants]) + 1)
            elif placementStrategy == PlacementPointStrategy.NormalPatterns:
                placementPointsX, placementPointsY = self.PlacementGenerator.DetermineNormalPatterns(partsReduced, self.Bin.Width - min([variant.Width for variant in part.Variants]), self.Bin.Length - min([variant.Length for variant in part.Variants]))
            elif placementStrategy == PlacementPointStrategy.MeetInTheMiddle:
                placementPointsX, placementPointsY = self.PlacementGenerator.DetermineMeetInTheMiddlePatterns(partsReduced, part, self.Bin.Width, self.Bin.Length)
            elif placementStrategy == PlacementPointStrategy.MinimalMeetInTheMiddle:
                placementPointsX, placementPointsY = allPlacementPointsX[i], allPlacementPointsY[i]
            
            
            placementPointStartX = sorted([point for point in placementPointsX if point + min([variant.Width for variant in part.Variants]) <= self.Bin.Width])
            placementPointEndX = sorted([point + variant.Width for point in placementPointStartX for variant in part.Variants if point + variant.Width <= self.Bin.Width])

            placementPointStartY = sorted([point for point in placementPointsY if point + min([variant.Length for variant in part.Variants]) <= self.Bin.Length])
            placementPointEndY = sorted([point + variant.Length for point in placementPointStartY for variant in part.Variants if point + variant.Length <= self.Bin.Length])

            xStart = self.model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointStartX), 'xStart(' + tmpIntervalName + ')')
            xEnd = self.model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointEndX), 'xnEnd(' + tmpIntervalName + ')')

            yStart = self.model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointStartY), 'y_start(' + tmpIntervalName + ')')
            yEnd = self.model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointEndY), 'y_end(' + tmpIntervalName + ')')
                
            partWidth = self.model.NewIntVarFromDomain(cp_model.Domain.FromIntervals([[min([variant.Width for variant in part.Variants])], [max([variant.Width for variant in part.Variants])]]), 'Width(' + tmpIntervalName + ')')
            partLength = self.model.NewIntVarFromDomain(cp_model.Domain.FromIntervals([[min([variant.Length for variant in part.Variants])], [max([variant.Length for variant in part.Variants])]]), 'Length(' + tmpIntervalName + ')')

            xInterval = self.model.NewIntervalVar(xStart, partWidth, xEnd, 'xInterval(' + tmpIntervalName + ')')
            yInterval = self.model.NewIntervalVar(yStart, partLength, yEnd, 'yInterval(' + tmpIntervalName + ')')

            self.Intervals['x'].append(xInterval)
            self.Intervals['y'].append(yInterval)

            self.xStarts[i] = xStart
            self.Positions[i] = position(xStart=xStart, xEnd=xEnd, yStart=yStart, yEnd=yEnd)
            self.PartLengths[i] = partLength
            self.PartWidths[i] = partWidth
            
    def CreateConstraints(self):

        for i, part in enumerate(self.Parts):
            for variant in part.Variants:
                self.model.Add(self.PartWidths[i] == variant.Width).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)])
                self.model.Add(self.PartLengths[i]== variant.Length).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)])
                self.model.Add(self.PartWidths[i] == variant.Length).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)].Not())
                self.model.Add(self.PartLengths[i] == variant.Width).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)].Not())

        self.model.AddNoOverlap2D(self.Intervals['x'],self.Intervals['y'])

        # self.model.AddCumulative(self.Intervals['y'], self.PartWidths, self.Bin.Length)
        # self.model.AddCumulative(self.Intervals['x'], self.PartLengths, self.Bin.Width)

        for i, part in enumerate(self.Parts):
            self.model.Add(sum(self.VariantUsed[(i, variant.VariantId)] for variant in part.Variants) == 1)

    def ExtractSolution(self):
        self.StartPositionsX = [self.solver.Value(position.xStart) for position in self.Positions.values()]
        self.StartPositionsY = [self.solver.Value(position.yStart) for position in self.Positions.values()]
        self.FinalLengths = [self.solver.Value(length) for length in self.PartLengths.values()]
        self.FinalWidths = [self.solver.Value(width) for width in self.PartWidths.values()]

        return self.StartPositionsX, self.StartPositionsY, self.FinalLengths, self.FinalWidths

    def Solve(self, model, threads, timeLimit = 3600):
        if len(self.Parts) == 1:
            if (self.Parts[0].Variants[0].Width <= self.Bin.Width and self.Parts[0].Variants[0].Length <= self.Bin.Length) or (self.Parts[0].Variants[0].Length <= self.Bin.Width and self.Parts[0].Variants[0].Width <= self.Bin.Length):
                return True
            else:
                return False
        
        solution_callback = cp_model.ObjectiveSolutionPrinter()
        self.solver.parameters.num_search_workers = threads
        self.solver.parameters.max_time_in_seconds = timeLimit
        self.solver.parameters.stop_after_first_solution = True
        self.solver.parameters.random_seed = model._Seed
        self.solver.parameters.cp_model_presolve = False
        
        status = self.solver.Solve(self.model)

        if status == cp_model.FEASIBLE or status == cp_model.OPTIMAL:
            return True
        elif status == cp_model.INFEASIBLE:
            return False
        elif status == cp_model.MODEL_INVALID:
            raise ValueError("Invalid Model!")
        elif status == cp_model.UNKNOWN:
            model._BPTimeOuts += 1
            model._TOAssignments.append({"Objective Value": model.cbGet(GRB.Callback.MIPSOL_OBJ), "Machine ID": self.Bin.MachineId, "Part IDs": [part.PartId for part in self.Parts]})
            # print("Unknown after 3600 Seconds, adding Cut!")
            return False
        else:
            raise ValueError("Something went wrong!")

    def HeuristicSolve(self, model, threads):
        if len(self.Parts) == 1:
            if (self.Parts[0].Variants[0].Width <= self.Bin.Width and self.Parts[0].Variants[0].Length <= self.Bin.Length) or (self.Parts[0].Variants[0].Length <= self.Bin.Width and self.Parts[0].Variants[0].Width <= self.Bin.Length):
                return True
            else:
                return False
        
        #solution_callback = cp_model.ObjectiveSolutionPrinter()
        self.solver.parameters.num_search_workers = threads
        self.solver.parameters.max_time_in_seconds = 1
        self.solver.parameters.stop_after_first_solution = True
        self.solver.parameters.random_seed = model._Seed


        status = self.solver.Solve(self.model)
        
        if status == cp_model.FEASIBLE or status == cp_model.OPTIMAL:
            #print(self.solver.StatusName())
            return True
        elif status == cp_model.INFEASIBLE:
            #print(self.solver.StatusName())
            return False
        elif status == cp_model.UNKNOWN:
            return None
        elif status == cp_model.MODEL_INVALID:
            raise ValueError("Invalid Model!")
        else:
            raise ValueError("Something went wrong!")

class BinPacking2DStrengthened:

    def __init__(self):
        self.xStarts = {}
        self.Positions = {}

        self.Intervals = collections.defaultdict(list)
        self.PartWidths = {}
        self.PartLengths = {}

        self.VariantUsed = {}
        self.BuildUsed = {}

        self.model = cp_model.CpModel()
        self.solver = cp_model.CpSolver()

        self.PlacementGenerator = PlacementPointGenerator()
    
    def AddParts(self, parts):
        self.Parts = parts
    
    def AddBin(self, machine):
        self.Bin = machine
    
    def CreateVariables(self, placementStrategy):
        position = collections.namedtuple('position', 'xStart xEnd yStart yEnd')

        if placementStrategy == PlacementPointStrategy.MinimalMeetInTheMiddle:
            allPlacementPointsX, allPlacementPointsY = self.PlacementGenerator.DetermineMinimalMeetInTheMiddlePatterns(self.Parts, self.Bin)
        
        for i, part in enumerate(self.Parts):
 
            for variant in part.Variants:
                tmpIntervalName = '(' + str(i) + ',' + str(variant.VariantId) + ')'
                self.VariantUsed[(i, variant.VariantId)] = self.model.NewBoolVar('VariantUsed' + tmpIntervalName)
    
        for i, part in enumerate(self.Parts):
            tmpIntervalName = str(i)

            partsReduced = [other for other in self.Parts if other != part]

            if placementStrategy == PlacementPointStrategy.NoStrategy:
                placementPointsX = range(0, self.Bin.Width - min([variant.Width for variant in part.Variants]) + 1)
                placementPointsY = range(0, self.Bin.Length - min([variant.Length for variant in part.Variants]) + 1)
            elif placementStrategy == PlacementPointStrategy.NormalPatterns:
                placementPointsX, placementPointsY = self.PlacementGenerator.DetermineNormalPatterns(partsReduced, self.Bin.Width - min([variant.Width for variant in part.Variants]), self.Bin.Length - min([variant.Length for variant in part.Variants]))
            elif placementStrategy == PlacementPointStrategy.MeetInTheMiddle:
                placementPointsX, placementPointsY = self.PlacementGenerator.DetermineMeetInTheMiddlePatterns(partsReduced, part, self.Bin.Width, self.Bin.Length)
            elif placementStrategy == PlacementPointStrategy.MinimalMeetInTheMiddle:
                placementPointsX, placementPointsY = allPlacementPointsX[i], allPlacementPointsY[i]     


            placementPointStartX = sorted([point for point in placementPointsX if point + min([variant.Width for variant in part.Variants]) <= self.Bin.Width])
            placementPointEndX = sorted([point + variant.Width for point in placementPointStartX for variant in part.Variants if point + variant.Width <= self.Bin.Width])

            placementPointStartY = sorted([point for point in placementPointsY if point + min([variant.Length for variant in part.Variants]) <= self.Bin.Length])
            placementPointEndY = sorted([point + variant.Length for point in placementPointStartY for variant in part.Variants if point + variant.Length <= self.Bin.Length])

            xStart = self.model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointStartX), 'xStart(' + tmpIntervalName + ')')
            xEnd = self.model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointEndX), 'xnEnd(' + tmpIntervalName + ')')

            yStart = self.model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointStartY), 'y_start(' + tmpIntervalName + ')')
            yEnd = self.model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointEndY), 'y_end(' + tmpIntervalName + ')')
                
            partWidth = self.model.NewIntVarFromDomain(cp_model.Domain.FromIntervals([[min([variant.Width for variant in part.Variants])], [max([variant.Width for variant in part.Variants])]]), 'Width(' + tmpIntervalName + ')')
            partLength = self.model.NewIntVarFromDomain(cp_model.Domain.FromIntervals([[min([variant.Length for variant in part.Variants])], [max([variant.Length for variant in part.Variants])]]), 'Length(' + tmpIntervalName + ')')

            xInterval = self.model.NewIntervalVar(xStart, partWidth, xEnd, 'xInterval(' + tmpIntervalName + ')')
            yInterval = self.model.NewIntervalVar(yStart, partLength, yEnd, 'yInterval(' + tmpIntervalName + ')')

            self.Intervals['x'].append(xInterval)
            self.Intervals['y'].append(yInterval)

            self.xStarts[i] = xStart
            self.Positions[i] = position(xStart=xStart, xEnd=xEnd, yStart=yStart, yEnd=yEnd)
            self.PartLengths[i] = partLength
            self.PartWidths[i] = partWidth
    
    def CreateConstraints(self):

        for i, part in enumerate(self.Parts):
            for variant in part.Variants:
                self.model.Add(self.PartWidths[i] == variant.Width).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)])
                self.model.Add(self.PartLengths[i]== variant.Length).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)])
                self.model.Add(self.PartWidths[i] == variant.Length).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)].Not())
                self.model.Add(self.PartLengths[i] == variant.Width).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)].Not())

        self.model.AddNoOverlap2D(self.Intervals['x'],self.Intervals['y'])
        
        for i, part in enumerate(self.Parts):
            self.model.Add(sum(self.VariantUsed[(i, variant.VariantId)] for variant in part.Variants) == 1)
    
    def ExtractSolution(self):
        self.StartPositionsX = [self.solver.Value(position.xStart) for position in self.Positions.values()]
        self.StartPositionsY = [self.solver.Value(position.yStart) for position in self.Positions.values()]
        self.FinalLengths = [self.solver.Value(length) for length in self.PartLengths.values()]
        self.FinalWidths = [self.solver.Value(width) for width in self.PartWidths.values()]

        return self.StartPositionsX, self.StartPositionsY, self.FinalLengths, self.FinalWidths
    
    def Solve(self, model, threads):
        if len(self.Parts) == 1:
            if (self.Parts[0].Variants[0].Width <= self.Bin.Width and self.Parts[0].Variants[0].Length <= self.Bin.Length) or (self.Parts[0].Variants[0].Length <= self.Bin.Width and self.Parts[0].Variants[0].Width <= self.Bin.Length):
                return True
            else:
                return False
        
        #solution_callback = cp_model.ObjectiveSolutionPrinter()
        self.solver.parameters.num_search_workers = threads
        self.solver.parameters.max_time_in_seconds = 2
        self.solver.parameters.stop_after_first_solution = True
        self.solver.parameters.random_seed = model._Seed


        status = self.solver.Solve(self.model)
        
        if status == cp_model.FEASIBLE or status == cp_model.OPTIMAL:
            #print(self.solver.StatusName())
            return True
        elif status == cp_model.INFEASIBLE or status == cp_model.UNKNOWN:
            #print(self.solver.StatusName())
            return False
        elif status == cp_model.MODEL_INVALID:
            raise ValueError("Invalid Model!")
        else:
            raise ValueError("Something went wrong!")

# this implementation of the knapsack problem is based on https://github.com/ktnr/BinPacking2D/blob/master/packingSolver/OrthogonalPacking.py
class Knapsack2D:
    def __init__(self):
        self.Model = cp_model.CpModel()
        self.Solver = None

    def Solve(self, items, stripHeight, stripWidth, timeLimit = 1.0):
        n = len(items)
        objectiveCoefficients = [1 for i in range(n)]
        H = stripHeight
        W = stripWidth

        # variables
        self.b = []
        self.xb1 = []
        self.xb2 = []
        self.y1 = []
        self.y2 = []

        for i, item in enumerate(items):

            f = self.Model.NewBoolVar(f'b{i}')
            self.b.append(f)

            yStart = self.Model.NewIntVar(0, H - item.Variants[0].Width,f'y1.{i}')
            yEnd = self.Model.NewIntVar(item.Variants[0].Width, H,f'y2.{i}')

            self.y1.append(yStart)
            self.y2.append(yEnd)

            
            # TODO: apply bin domains to these variables
            binStart = (i + 1) * stripWidth
            d = self.Model.NewIntVarFromDomain(Domain.FromIntervals([[0, stripWidth - item.Variants[0].Width], [binStart, binStart]]), f'xb1.{i}')
            e = self.Model.NewIntVarFromDomain(Domain.FromIntervals([[item.Variants[0].Width, stripWidth], [binStart + item.Variants[0].Width, binStart + item.Variants[0].Width]]), f'xb2.{i}')
            
            self.xb1.append(d)
            self.xb2.append(e)

        # interval variables
        self.xival = [self.Model.NewIntervalVar(self.xb1[i], items[i].Variants[0].Width, self.xb2[i],f'xival{i}') for i in range(n)]
        self.yival = [self.Model.NewIntervalVar(self.y1[i], items[i].Variants[0].Width, self.y2[i],f'yival{i}') for i in range(n)]

        # constraints
        self.Model.AddNoOverlap2D(self.xival, self.yival)
        self.Model.Add(sum(self.b[i]*item.Variants[0].Width**2 for i, item in enumerate(items)) == W*H)

        for i, item in enumerate(items):
            
            self.Model.Add(self.xb2[i] <= stripWidth).OnlyEnforceIf(self.b[i])
            self.Model.Add(self.xb2[i] >= stripWidth + item.Variants[0].Width).OnlyEnforceIf(self.b[i].Not())

        # objective
        self.Model.Maximize(sum(self.b[i] * int(objectiveCoefficients[i]) for i in range(n)))

        # solve model
        self.Solver = cp_model.CpSolver()
        self.Solver.parameters.log_search_progress = False
        self.Solver.parameters.max_time_in_seconds = timeLimit
        self.Solver.parameters.num_search_workers = 1
        

        self.Solver.Solve(self.Model)

        return self.Solver.StatusName(), self.Solver.BestObjectiveBound(), self.Solver.ObjectiveValue()
    
    def ExtractSolution(self):
        values = [self.Solver.Value(variable) for variable in self.b]
        return values


class SlaveStripPacking:
    def __init__(self):
        self.Model = cp_model.CpModel()
        self.Solver = None

    def CreateVariables(self, activeItems, heights, stripHeight, stripWidth):

        self.H = stripHeight
        self.W = stripWidth
        self.ActiveItems = activeItems
        self.PlacementPoints = {}

        MMIM = PlacementPointGenerator.DetermineMinimalMeetInTheMiddlePatternsWoR([heights[i] for i in activeItems.keys()], stripHeight)
        for idx, i in enumerate(activeItems.keys()):
            self.PlacementPoints[i] = MMIM[idx]
        # variables
        self.yStarts = {}
        self.yEnds = {}
        self.yival = {}

        for i in self.ActiveItems.keys():
            height = heights[i]

            placementPointStartY = sorted([point for point in self.PlacementPoints[i] if point + height <= self.H])
            placementPointEndY = sorted([point + height for point in placementPointStartY if point + height <= self.H])


            yStart = self.Model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointStartY), f'y1.{i}')#self.Model.NewIntVar(0, self.H - height,f'y1.{i}')
            yEnd = self.Model.NewIntVarFromDomain(cp_model.Domain.FromValues(placementPointEndY), f'y2.{i}')#NewIntVar(height, self.H,f'y2.{i}')

            self.yStarts[i] = yStart
            self.yEnds[i] = yEnd

        
            self.yival[i] = self.Model.NewIntervalVar(yStart, height, yEnd,f'yival{i}')

    def CreateConstraints(self):
        # constraints
        for q in range(self.W):
            self.Model.AddNoOverlap([self.yival[i] for i, arc in self.ActiveItems.items() if arc[0] <= q < arc[1]])
        
        for i in self.ActiveItems.keys():
            self.Model.Add(self.yEnds[i] <= self.H)
        
    def Solve(self, model, timeLimit):

        self.Solver = cp_model.CpSolver()
        self.Solver.parameters.log_search_progress = False
        self.Solver.parameters.max_time_in_seconds = timeLimit
        self.Solver.parameters.num_search_workers = 8
        

        status = self.Solver.Solve(self.Model)
      
        if status == cp_model.FEASIBLE or status == cp_model.OPTIMAL:
            return True
        elif status == cp_model.INFEASIBLE:
            return False
        elif status == cp_model.MODEL_INVALID:
            raise ValueError("Invalid Model!")
        elif status == cp_model.UNKNOWN:
            model._ODBPTimeOuts += 1
            return False
        else:
            raise ValueError("Something went wrong!")
    
    def HeuristicSolve(self, timeLimit = 1.0):
        self.Solver = cp_model.CpSolver()
        self.Solver.parameters.log_search_progress = False
        self.Solver.parameters.max_time_in_seconds = timeLimit
        self.Solver.parameters.num_search_workers = 8
        
        status = self.Solver.Solve(self.Model)
        
        if status == cp_model.FEASIBLE or status == cp_model.OPTIMAL:
            return True
        elif status == cp_model.INFEASIBLE:
            return False
        elif status == cp_model.UNKNOWN:
            return None
        elif status == cp_model.MODEL_INVALID:
            raise ValueError("Invalid Model!")
        else:
            raise ValueError("Something went wrong!")






if __name__ == "__main__":
    from DataManager import *
    from Scheduler import *
    import time
    import math
    import os
    import pandas as pd

    path = "C:/Users/Administrator/Documents/BNC_Repo/BNC_Tests/02_TestData/PackingData_Test2"

    BINS = {
        "ht01": (20, 20),
        "ht02": (20, 20),
        "ht03": (20, 20),
        "ht04": (40, 15),
        "ht05": (40, 15),
        "ht06": (40, 15),
        "ht07": (60, 30),
        "ht08": (60, 30),
        "ht09": (60, 30),
        "BENG01": (25, 30),
        "BENG02": (25, 57),
        "BENG03": (25, 84),
        "BENG04": (25, 107),
        "BENG05": (25, 134),
        "BENG06": (40, 36),
        "BENG07": (40, 67),
        "BENG08": (40, 101),
        "BENG09": (40, 126),
        "BENG10": (40, 156),
        "CGCUT1": (10, 23),
        "CGCUT2": (70, 63),
        "NGCUT1": (10, 20),
        "NGCUT2": (10, 28),
        "NGCUT3": (10, 28),
        "NGCUT4": (10, 18),
        "NGCUT5": (10, 36),
        "NGCUT6": (10, 29),
        "NGCUT7": (20, 10),
        "NGCUT8": (20, 33),
        "NGCUT9": (20, 49),
        "NGCUT10": (30, 59),
        "NGCUT12": (30, 77)
    }

    results = []
    for file in os.listdir(path):
        if file.endswith(".ins2D"):
            if file.startswith("CGCUT3") or file.startswith("NGCUT11"):
                continue
            print(f"File: {file}")
            print("")
            with open(os.path.join(path, file), "r") as inFile:
                data = inFile.readlines()
            data = [i.strip() for i in data]
            data = [i.split(" ") for i in data]

            l = []
            for line in data[2:]:
                for rep in range(int(line[-2])):
                    l.append((int(line[1]), int(line[2])))

            print(f"Items: {len(l)}")

            b = BINS[file[:-6]]

            items = [Part(i, i, [Variant(0, l[i][1], l[i][0], 0, 0)], 0, 'black') for i in range(len(l))]
            for part in items:
                part.createNewVariant()
            machine = Machine(0, b[0], b[1], 0, 0, 0, 0)
        


            print("Start with Orthogonal Packer")
            print("________________________________________________________")
            
            print([(part.Variants[0].Width, part.Variants[0].Length) for part in items])
            print(sum([part.Area for part in items]))

            print(machine.Width, machine.Length)
            print(machine.Area)
            binPacking2D = BinPacking2D()
            binPacking2D.AddParts(items)
            binPacking2D.AddBin(machine)

            binPacking2D.CreateVariables(PlacementPointStrategy.MinimalMeetInTheMiddle)
            binPacking2D.CreateConstraints()

            tmpModel = MachineScheduling("sjasdh", 23)
            tmpModel._BPTimeOuts = 0
            tmpModel._TOAssignments = []

            tmpModel._Seed = 23
            start = time.time()

            isFeasible = binPacking2D.Solve(tmpModel, 8, 3600)

            end = time.time()

            print(f"Consumed Time: {end - start}")
            print(f"Result: {isFeasible}")
            print("________________________________________________________")
            results.append({"File": file, "Items": len(l), "Container": b, "Result": isFeasible, "Time": end - start, "Nodes": '-', "StairRestriction": "-"})

            del binPacking2D

    # pd.DataFrame(results).to_csv(os.path.join(path, "resultsCP.csv"), index = False)
    # newData = DataModel(0, True)
    # newData.BuildData(f"C:/Users/Administrator/Documents/BNC_Repo/BNC_Tests/02_TestData/n80m2/Instance_Class1_Parts80_Machines2_Instance4.json")

    # tmpParts = [part for p, part in enumerate(newData.Parts) if p in [
    #                 1,
    #                 4,
    #                 6,
    #                 7,
    #                 10,
    #                 11,
    #                 12,
    #                 13,
    #                 14,
    #                 15,
    #                 20,
    #                 21,
    #                 27,
    #                 32,
    #                 33,
    #                 36,
    #                 37,
    #                 38,
    #                 39,
    #                 40,
    #                 43,
    #                 44,
    #                 46,
    #                 48,
    #                 49,
    #                 50,
    #                 51,
    #                 52,
    #                 57,
    #                 59,
    #                 66,
    #                 67,
    #                 68,
    #                 69,
    #                 70,
    #                 71,
    #                 78
    #             ]]
    # print([(part.Variants[0].Width, part.Variants[0].Length) for part in tmpParts])
    # print(sum([part.Area for part in tmpParts]))
    # machine = newData.Machines[1]
    # print(machine.Width, machine.Length)
    # print(machine.Area)
    # binPacking2D = BinPacking2D()
    # binPacking2D.AddParts(tmpParts)
    # binPacking2D.AddBin(machine)

    # binPacking2D.CreateVariables(PlacementPointStrategy.MinimalMeetInTheMiddle)
    # binPacking2D.CreateConstraints()

    # tmpModel = MachineScheduling("sjasdh", 23)
    # tmpModel._Seed = 23
    # isFeasible = binPacking2D.Solve(tmpModel, 8, max(0, 30))
    # print(isFeasible)

    # xs, ys, ls, ws = binPacking2D.ExtractSolution()

    # import random
    # import matplotlib.pyplot as plt
    # from matplotlib.patches import Rectangle
    # random.seed(23)
    
    
    # colors = ["#"+''.join([random.choice('0123456789ABCDEF') for j in range(6)])
    #         for i in range(len(tmpParts))]



    # plt.figure()
    # axes = plt.gca()
    # axes.set_xlim([0,machine.Width])
    # axes.set_ylim([0,machine.Length])
    #                 # axes.set_aspect('equal')
    # for i in range(len(xs)):
    #     axes.add_patch(Rectangle((xs[i], ys[i]), ws[i], ls[i], facecolor= colors[i], alpha=0.5))
    # plt.show()
