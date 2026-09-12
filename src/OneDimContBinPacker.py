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
    

if __name__ == "__main__":
    from DataManager import *
    from Scheduler import *
    import time
    import math
    import os
    import pandas as pd

    path = "C:/Users/Administrator/Documents/BNC_Repo/BNC_Tests/02_TestData/PackingData"#Che_ht2_MHU"

    BINS = {
        'Test1': (20, 20),
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
        "NGCUT11": (30, 51),
        "NGCUT12": (30, 77)
    }

    results = []
    for file in os.listdir(path):
        # if file.startswith("ht2_3"):
        #     data = DataModel(file)
        #     data.BuildData(os.path.join(path, file))

        #     items = [part for i, part in enumerate(data.Parts) if i in [4,
        #             5,
        #             6,
        #             8,
        #             9,
        #             10,
        #             15,
        #             18,
        #             19,
        #             21,
        #             24,
        #             26,
        #             27,
        #             29,
        #             30,
        #             31,
        #             32,
        #             38,
        #             47,
        #             48]]
        #     machine = data.Machines[1]

        #     print("Start with Orthogonal Packer")
        #     print("________________________________________________________")
            
        #     print([(part.Variants[0].Width, part.Variants[0].Length) for part in items])
        #     print(sum([part.Area for part in items]))

        #     print(machine.Width, machine.Length)
        #     print(machine.Area)
        #     oneDim = OneDimContBinPacking(0)
        #     oneDim.AddParts(items)
        #     oneDim.AddBoundaries(machine.Width, machine.Length)
        #     oneDim.Preprocess()
        #     oneDim.DetermineArcs()
        #     oneDim.CreateVariables()
        #     oneDim.CreateConstraints()
        #     oneDim.SetCallbackData()
        #     oneDim.Solve(8, 3600, 0.00001, file[:-6])

        #     print('')

        if file.endswith(".ins2D"):
            if not file.startswith("NGCUT1."):
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
            machine = Machine(0, b[1], b[0], 0, 0, 0, 0)

            print("Start with Orthogonal Packer")
            print("________________________________________________________")
            
            print([(part.Variants[0].Width, part.Variants[0].Length) for part in items])
            print(sum([part.Area for part in items]))

            print(machine.Width, machine.Length)
            print(machine.Area)
            oneDim = OneDimContBinPacking(0)
            oneDim.AddParts(items)
            oneDim.AddBoundaries(machine.Width, machine.Length)
            oneDim.Preprocess()
            oneDim.DetermineArcs()
            oneDim.CreateVariables()
            oneDim.CreateConstraints()
            oneDim.SetCallbackData()
            class Model:
                _BPTimeOuts = 0
                _TOAssignments = []
            oneDim.Solve(Model(), 8, 3600)

            print('')

    
    # def CreateVariables(self):
    #     for i, part in enumerate(self.Parts):
    #         self.x.append([])
    #         for b in range(self.Builds):
    #             self.x[i].append([])
    #             for k, machine in enumerate(self.Machines):
    #                 self.x[i][b].append(self.model.addVar(vtype = GRB.BINARY, name = "x[%d,%d,%d]" % (i, b, k)))

    #     for b in range(self.Builds):
    #         self.y.append([])
    #         for k, machine in enumerate(self.Machines):
    #             self.y[b].append(self.model.addVar(vtype = GRB.BINARY, name = "y[%d,%d]" % (b, k)))

    #     for b in range(self.Builds):
    #         self.c.append([])
    #         self.p.append([])
    #         for k, machine in enumerate(self.Machines):
    #             self.c[b].append(self.model.addVar(vtype = GRB.CONTINUOUS, name = "c[%d,%d]" % (b, k)))
    #             self.p[b].append(self.model.addVar(vtype = GRB.CONTINUOUS, name = "p[%d,%d]" % (b, k)))
        
    #     for b in range(self.Builds):
    #         self.h.append([])
    #         for k, machine in enumerate(self.Machines):
    #             self.h[b].append(self.model.addVar(vtype = GRB.INTEGER, name = "hb[%d,%d]" % (b, k)))

    #     self.MakeSpan = self.model.addVar(vtype = GRB.CONTINUOUS, name = "MakeSpan")

    # def CreateConstraints(self):
    #     for b in range(self.Builds):
    #         for k, machine in enumerate(self.Machines):
    #             self.model.addConstr(self.MakeSpan >= self.c[b][k], name = "Makespan[%d,%d]" % (b, k))

    #     for i, part in enumerate(self.Parts):
    #         self.model.addConstr(sum(self.x[i][b][machine.MachineId] for b in range(self.Builds) for machine in self.Machines) == 1, name = "AllPartsProcessed[%d]" % i)
        
    #     for i, part in enumerate(self.Parts):
    #         for b in range(self.Builds):
    #             for k, machine in enumerate(self.Machines):
    #                 self.model.addConstr(self.x[i][b][k] <= self.y[b][k], name = "NoAssignmentToUnusedBuilds[%d,%d,%d]" % (i, b, k))
        
    #     for i, part in enumerate(self.Parts):
    #         for b in range(self.Builds):
    #             for k, machine in enumerate(self.Machines):
    #                 self.model.addConstr(self.h[b][k] >= part.Height * self.x[i][b][k], name = "BuildHeight[%d,%d,%d]" % (i, b, k))
        
    #     for b in range(self.Builds):
    #         for k, machine in enumerate(self.Machines):
    #             if b == 0:
    #                 self.model.addConstr(self.c[b][k] >= self.p[b][k], name = "CompletionTime1[%d,%d]" % (b, k))
    #             else:
    #                 self.model.addConstr(self.c[b][k] >= self.c[b - 1][k] + self.p[b][k], name = "CompletionTime2[%d,%d]" % (b, k))

    #     for b in range(self.Builds):
    #         for k, machine in enumerate(self.Machines):
    #             self.model.addConstr(self.p[b][k] == machine.Setup * self.y[b][k] + machine.ScanTime * sum(part.Volume * self.x[part.PartId][b][k] for part in self.Parts) + machine.RecoatTime * self.h[b][k], name = "ProcessTime[%d,%d]" % (b,k))
       
    #     self.model.setObjective(self.MakeSpan, GRB.MINIMIZE)

    #     ### Symmetry Breaking Constraints
    #     ### Valid Inequalities
    #     """ for b in range(self.Builds):
    #         if b > 0:
    #             for k, machine in enumerate(self.Machines):
    #                 self.model.addConstr(sum(self.x[part.PartId][b][k] for part in self.Parts) <= sum(self.x[part.PartId][b-1][k] for part in self.Parts), name = "IncrementallyPartFilling[%d,%d]" %(b, k))
    #      """
    #     """ for b in range(self.Builds):
    #         for k, machine in enumerate(self.Machines):
    #             self.model.addConstr(self.y[b][k] * min(part.Height for part in self.Parts if part.Height <= machine.Height) <=  self.h[b][k], name = "HeightRestriction3[%d,%d]" % (b, k))
    #      """

    # def InitializePreprocessResults(self, incompatibleParts, infeasibleAssignments, partAssignment):

    #     self.IncompatibleParts = incompatibleParts
    #     self.InfeasibleAssignments = infeasibleAssignments
    #     self.PartAssignment = partAssignment 
        
    #     self.AddInfeasibleAssignmentCuts(infeasibleAssignments)
    #     self.SetCallbackData()
        
    # def AddSymmetryBreakingConstraints(self):
    #     for b in range(self.Builds):
    #         if b > 0:
    #             for k, machine in enumerate(self.Machines):
    #                 self.model.addConstr(self.y[b - 1][k] >= self.y[b][k], name="SuccessiveBuildUsage[%d,%d]" % (b, k))
        
    #     for i, part in enumerate(self.Parts):
    #         for b in range(self.Builds):
    #             if b > i:
    #                 for k, machine in enumerate(self.Machines):
    #                     self.model.addConstr(self.x[i][b][k] == 0, name="AssignmentBreaking[%d,%d,%d]" % (i, b, k))

    #     for b in range(self.Builds):
    #         for k, machine in enumerate(self.Machines):
    #             self.model.addConstr(self.h[b][k] <= max(part.Height for part in self.Parts if part.Height <= machine.Height) * self.y[b][k], name = "AreaConsumption[%d,%d]" % (b, k))
        
    #     self.AddIncompatibilityCuts()
    #     #self.LiftIncompatibleParts()

    # def AddAreaRestriction(self):
    #     for b in range(self.Builds):
    #        for k, machine in enumerate(self.Machines):
    #            self.model.addConstr(sum(self.x[part.PartId][b][k]*part.Area for part in self.Parts) <= machine.Area, name = "AreaConsumptionRestricted[%d,%d]" % (b, k))

        
    # def AddEnforcedAreaRestriction(self):
    #     for b in range(self.Builds):
    #        for k, machine in enumerate(self.Machines):
    #            self.model.addConstr(sum(self.x[part.PartId][b][k]*part.Area for part in self.Parts) <= 0.9*machine.Area, name = "AreaConsumptionRestricted[%d,%d]" % (b, k))

    # def AddMIPStartSolution(self):
    #     if self.PartAssignment == None:
    #         return
            
    #     for i, part in enumerate(self.Parts):
    #        for b in range(self.Builds):
    #            for k, machine in enumerate(self.Machines):
    #                self.x[i][b][k].Start = self.PartAssignment[i][b][k]
    #     return

    # def AddIncompatibilityCuts(self):
    #     for k, incompatibleSets in self.IncompatibleParts.items():
    #         for b in range(self.Builds):
    #             for pair in incompatibleSets:
    #                 self.model.addConstr(sum(self.x[i][b][k] for i in pair) <= 1)

    # def AddInfeasibleAssignmentCuts(self, infeasibleAssignments):
    #     self.InfeasibleAssignments = infeasibleAssignments
    #     for assignment in infeasibleAssignments:
    #         i, k = assignment
    #         for b in range(self.Builds):
    #             self.model.addConstr(self.x[i][b][k] == 0, name="InfeasibleMachineAssignment[%d,%d,%d]" % (i, b, k))
    
    # def SetCallbackData(self):
    #     self.model._Parts = self.Parts
    #     self.model._Builds = self.Builds
    #     self.model._Machines = self.Machines
    #     self.model._XAssignments = self.x
    #     self.model._YBatches = self.y
    #     self.model._Seed = self.Seed
    #     self.model._PlacementPoints = {}

    #     self.model._EnablePreprocessing = False
    #     self.model._EnableCutLifting = False
    #     self.model._EnableCutLiftingAtAreaCut = False
    #     self.model._EnableCutStrengthening = True

    #     self.model._FeasibleAssignments = {}

    #     self.model._InfeasibleAssignments = self.InfeasibleAssignments
    #     self.model._IncompatibleParts = self.IncompatibleParts
    #     self.model._ProvenInfeasibleSets = {machine.MachineId: [] for machine in self.Machines}

    #     self.model._PlacementStrategy = PlacementPointStrategy.MinimalMeetInTheMiddle
    #     self.model._AreaConstraints = 0
    #     self.model._LBConstraints = 0
    #     self.model._ORConstraints = 0
    #     self.model._BRConstraints = 0
    #     self.model._BPConstraints = 0
    #     self.model._TotalCuts = 0
    #     self.model._BPTimeOuts = 0
    #     self.model._TOAssignments = []

    #     self.model._TimeConPP = 0
    #     self.model._TimeConLB = 0
    #     self.model._TimeConOR = 0
    #     self.model._TimeConBR = 0
    #     self.model._TimeConBP = 0
    #     self.model._TimeConSC = 0
    #     self.model._Checks = []

    #     self.model._BatchChecks = 0
    #     self.model._SolutionChecks = 0
    
    # def LiftIncompatibleParts(self):
    #     for k, incompatibleSets in self.IncompatibleParts.items():
    #         for pair in incompatibleSets:
    #             tmpMachine = self.Machines[k]
    #             tmpParts = [part for part in self.Parts if part.PartId in pair]
    #             MachineScheduling.AddLiftingCut(self.model, tmpParts, tmpMachine)

    # @staticmethod
    # def AddLiftingCut(model, assignedParts, machine):
    #     profits = [1 if model._Parts[i] in assignedParts else 0 for i in range(len(model._Parts))]
    #     liftingCoefficients = list(profits)

    #     liftedAssignment = [part for part in assignedParts]
    #     additionalParts = []
    #     for i, part in enumerate(model._Parts):
    #         if part in assignedParts or (part.PartId, machine.MachineId) in model._InfeasibleAssignments:
    #             continue

    #         noValidAssignment = False
    #         for partJ in assignedParts:
    #             if frozenset((part.PartId, partJ.PartId)) in model._IncompatibleParts[machine.MachineId] or frozenset((part, partJ)) in model._ProvenInfeasibleSets[machine.MachineId]:
    #                 noValidAssignment = True
    #                 break
            
    #         if noValidAssignment:
    #             continue

    #         upperBound = ValidationCallback.UKnapSack2DSolve(liftedAssignment, part, machine, profits, model)

    #         liftingCoefficient = max(0, len(assignedParts) - 1 - upperBound)

    #         if liftingCoefficient > 0:
    #             liftedAssignment.append(part)
    #             additionalParts.append(part)

    #             liftingCoefficients[part.PartId] = int(liftingCoefficient)
    #             profits[part.PartId] = int(liftingCoefficient)

    #     MachineScheduling.AddLiftedCoverInequality(model, assignedParts, machine, additionalParts, liftingCoefficients)

    # @staticmethod
    # def AddLiftedCoverInequality(model, assignedParts, machine, additionalParts, liftingCoefficients):
    #     for b in range(model._Builds):
    #         expr = gp.LinExpr()
    #         for part in assignedParts:
    #             expr += model._XAssignments[part.PartId][b][machine.MachineId]
            
    #         for part in additionalParts:
    #             expr += liftingCoefficients[part.PartId] * model._XAssignments[part.PartId][b][machine.MachineId]
            
    #         model.addConstr(expr <= len(assignedParts) - 1)
         
    # def Solve(self, threads, timeLimit, mipGap, enablePreprocessOP, enablePackingRelaxations, orthogonalPackingMethod, modelId):
    #     self.model.write(os.path.join(self.Path, str(modelId)) + "_ModelFile" + str(self.Seed) + ".LP")
    #     self.model.setParam("LogFile", os.path.join(self.Path, str(modelId)) + "_logfileBnC" + str(self.Seed) + ".txt")
        
    #     self.model.Params.Seed = self.Seed
    #     self.model.Params.Threads = threads
    #     self.model.Params.TimeLimit = timeLimit
    #     self.model.Params.MIPFocus = 2
    #     # self.model.Params.NoRelHeurWork = 5
    #     self.model.Params.MIPGap = mipGap
    #     self.model.Params.lazyConstraints = 1

    #     self.model._TimeLimit = timeLimit
    #     self.model._EnablePreprocessing = enablePreprocessOP
    #     self.model._EnablePackingRelaxations = enablePackingRelaxations
    #     self.model._OrthogonalPackingMethod = orthogonalPackingMethod

    #     self.model.optimize(ValidationCallback.Callback)
        
    #     #self.model.Params.Cuts = 0

    #     # p = self.model.presolve()
    #     # if self.model.status == GRB.INFEASIBLE:
    #     #     self.model.computeIIS()
    #     #     self.model.write(os.path.join(self.Path, str(modelId)) + "_InfeasibleModelFile" + str(self.Seed) + ".ilp")
    #     # # p.computeIIS()
    #     # p.write("Test.ilp")
    #     #self.model.write(os.path.join(self.Path, str(modelId)) + "BnC" + str(seed) + ".json")
    #     #self.model.printAttr("X")
    
    # def PreSolve(self, threads, timeLimit, enablePackingRelaxations, orthogonalPackingMethod, modelId):
    #     self.model.setParam("LogFile", os.path.join(self.Path, str(modelId)) + "_logfilePreSolve" + str(self.Seed) + ".txt")
    #     self.model.Params.Seed = self.Seed
    #     self.model.Params.Threads = threads
    #     self.model.Params.TimeLimit = timeLimit
    #     self.model.Params.MIPFocus = 1
    #     self.model.Params.lazyConstraints = 1

    #     self.model._TimeLimit = timeLimit       
    #     self.model._EnablePreprocessing = False
    #     self.model._EnablePackingRelaxations = enablePackingRelaxations
    #     self.model._OrthogonalPackingMethod = orthogonalPackingMethod
        
    #     self.model.optimize(ValidationCallback.Callback)

    #     return (self.model.objval != np.inf)
    
    # def ExtractSolution(self):
    #     partAssignment = [[[0 for i in range(len(self.Machines))] for b in range(len(self.Parts))] for k in range(len(self.Parts))]
    #     for i, item in enumerate(self.Parts):
    #         for b in range(self.Builds):
    #            for k, machine in enumerate(self.Machines):
    #                if self.x[i][b][k].X > 0.5:
    #                    partAssignment[i][b][k] = 1
        
    #     return partAssignment
