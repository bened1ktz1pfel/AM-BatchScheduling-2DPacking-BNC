import gurobipy as gp
from gurobipy import GRB

import time
from math import gcd
import os

from StrippackingCallback import *
from PlacementPoints import *
from Preprocess import *
from DataManager import *

class OneDimContBinPacking:
    def __init__(self, seed):
        self.Seed = seed
        self.model = gp.Model()

        self.x = []
             
    def AddParts(self, parts):
        self.ItemNumber = len(parts)
        self.Parts = []
        
        for i, part in enumerate(parts):
            self.Parts.append(Item(i, part.Variants[0].Width, part.Variants[0].Length))
        
        for i, part in enumerate(parts):
            self.Parts.append(Item(self.ItemNumber + i, part.Variants[-1].Width, part.Variants[-1].Length))
    
    def AddBoundaries(self, containerWidth, containerLength):
        self.Container = Container(containerWidth, containerLength)
        # tmpList = [part.Width for part in self.Parts] + [part.Length for part in self.Parts] + [containerWidth, containerLength]

        # self.GCD = gcd(*tmpList)
        # print(self.GCD)
        # print()
    
    def Preprocess(self):
        preprocess = StrippackingPreprocess(self.Parts, self.Container, True)
        preprocess.Run()

        MMIM = PlacementPointGenerator.DetermineMinimalMeetInTheMiddlePatternsWoR([item.Width for item in self.Parts], self.Container.Width)
        self.PlacementPoints = {}
        for i, part in enumerate(self.Parts):
            self.PlacementPoints[i] = MMIM[i]

    def DetermineArcs(self):
        self.Arcs = {}
        self.Lengths = {}
        self.Widths = {}
        for j in range(len(self.Parts) + 1):
            self.Arcs[j] = []
            if j == 0:
                self.Lengths[j] = 1
                self.Widths[j] = 1
                for d in range(self.Container.Width):
                    self.Arcs[j].append((d, d + 1))
                continue
            self.Lengths[j] = self.Parts[j - 1].Length
            self.Widths[j] = self.Parts[j - 1].Width
            for d in self.PlacementPoints[j - 1]:
                for e in range(self.Container.Width + 1):
                    if e - d == self.Parts[j - 1].Width:
                        self.Arcs[j].append((d, e))
    
    def CreateVariables(self):
        for j in self.Arcs.keys():
            self.x.append({})
            for arc in self.Arcs[j]:
                d, e = arc
                self.x[j][arc] = self.model.addVar(vtype = GRB.INTEGER, name = "x[%d,%d,%d]" % (j, d, e))
        
    def CreateConstraints(self):

        for e in range(self.Container.Width + 1):
            if e == 0:
                self.model.addConstr(sum((-sum(self.x[j][arc] for arc in self.Arcs[j] if arc[1] == e)*self.Lengths[j] + sum(self.x[j][arc] for arc in self.Arcs[j] if arc[0] == e)*self.Lengths[j]) for j in self.Arcs.keys()) == self.Container.Length, name = "WidthConstraint[%d]" % e)
            elif e == self.Container.Width:
                self.model.addConstr(sum((-sum(self.x[j][arc] for arc in self.Arcs[j] if arc[1] == e)*self.Lengths[j] + sum(self.x[j][arc] for arc in self.Arcs[j] if arc[0] == e)*self.Lengths[j]) for j in self.Arcs.keys()) == -self.Container.Length, name = "WidthConstraint[%d]" % e)
            else:
                self.model.addConstr(sum((-sum(self.x[j][arc] for arc in self.Arcs[j] if arc[1] == e)*self.Lengths[j] + sum(self.x[j][arc] for arc in self.Arcs[j] if arc[0] == e)*self.Lengths[j]) for j in self.Arcs.keys()) == 0, name = "WidthConstraint[%d]" % e)

        for j in range(1, self.ItemNumber + 1):
            self.model.addConstr(sum(self.x[j][arc] for arc in self.Arcs[j]) + sum(self.x[self.ItemNumber + j][arc] for arc in self.Arcs[self.ItemNumber + j]) == 1)

        # for j in self.Arcs.keys():
        #     if j > 0:
        #         for arc in self.Arcs[j]:
        #             d, e = arc
        #             self.model.addConstr(self.x[j][arc] <= 1)

    def SetCallbackData(self):
        self.model._Parts = self.Parts
        self.model._Arcs = self.Arcs
        self.model._Container = self.Container
        self.model._Lengths = self.Lengths
        self.model._Widths = self.Widths

        self.model._XAssignments = self.x
        self.model._Seed = self.Seed

        self.model._EnableCutLifting = True
        self.model._EnableCutStrengthening = True
        
        self.model._FeasibleAssignments = []
        self.model._InfeasibleAssignments = []

        self.model._ODBPTimeOuts = 0


    def Solve(self, model, threads, timeLimit):
        # self.model.setParam("LogFile", os.path.join(self.Path, str(modelId)) + "_logfileBnC" + str(self.Seed) + ".txt")
        self.model.Params.OutputFlag = 0
        self.model.Params.Seed = self.Seed
        self.model.Params.Threads = threads
        self.model.Params.TimeLimit = timeLimit
        self.model.Params.lazyConstraints = 1
        self.model.Params.Heuristics = 0.5
        self.model.Params.MIPFocus = 1

        self.model._TimeLimit = timeLimit
        self.model.optimize(StripCallback.Callback)
        

        if self.model.status == GRB.INFEASIBLE:
            return False
        elif self.model.status == GRB.OPTIMAL:
            return True
        elif self.model.status == GRB.TIME_LIMIT:
            model._BPTimeOuts += 1
            model._TOAssignments.append({"Objective Value": model.cbGet(GRB.Callback.MIPSOL_OBJ), "Machine ID": (self.Container.Width, self.Container.Length), "Part IDs": [part.Id for part in self.Parts]})
            return False
        else:
            raise ValueError("Something went wrong!")
    
    def HeuristicSolve(self, model, threads):
        
        self.model.Params.OutputFlag = 0
        self.model.Params.Seed = self.Seed
        self.model.Params.Threads = threads
        self.model.Params.TimeLimit = 1
        self.model.Params.lazyConstraints = 1
        self.model.Params.Heuristics = 0.5
        self.model.Params.MIPFocus = 1

        self.model._TimeLimit = 1
        self.model.optimize(StripCallback.Callback)
        
        if self.model.status == GRB.INFEASIBLE:
            return False
        elif self.model.status == GRB.OPTIMAL:
            return True
        else:
            return None

    def ExtractSolution(self):
        StartPositionsX = []
        StartPositionsY = []
        FinalLengths = []
        FinalWidths = []

        for j in self.Arcs.keys():
            for arc in self.Arcs[j]:
                if self.x[j][arc].X > 0.5:
                    StartPositionsX.append(arc[0])
                    StartPositionsY.append("TODO")
                    FinalLengths.append(self.Lengths[j])
                    FinalWidths.append(self.Widths[j])
        
        return StartPositionsX, StartPositionsY, FinalLengths, FinalWidths
    



