import gurobipy as gp
from gurobipy import GRB

import time
import os

from SchedulingCallback import *
from OrthogonalPacker import PlacementPointStrategy
from ConstructiveAlgorithms import BuildApproximater

class MachineScheduling:
    def __init__(self, path, seed):
        self.Path = path
        self.Seed = seed
        self.model = gp.Model()

        self.x = []
        self.y = []
        self.c = []
        self.p = []
        self.h = []
        
    def AddParts(self, parts):
        self.Parts = parts
    
    def AddMachines(self, machines):
        self.Machines = machines
    
    def DetermineMaximumBuilds(self, assignmentStrategy):
        self.Builds = BuildApproximater(copy.deepcopy(self.Machines), copy.deepcopy(self.Parts)).Extract(assignmentStrategy) + 1
    
    def CreateVariables(self):
        for i, part in enumerate(self.Parts):
            self.x.append([])
            for b in range(self.Builds):
                self.x[i].append([])
                for k, machine in enumerate(self.Machines):
                    self.x[i][b].append(self.model.addVar(vtype = GRB.BINARY, name = "x[%d,%d,%d]" % (i, b, k)))

        for b in range(self.Builds):
            self.y.append([])
            for k, machine in enumerate(self.Machines):
                self.y[b].append(self.model.addVar(vtype = GRB.BINARY, name = "y[%d,%d]" % (b, k)))

        for b in range(self.Builds):
            self.c.append([])
            self.p.append([])
            for k, machine in enumerate(self.Machines):
                self.c[b].append(self.model.addVar(vtype = GRB.CONTINUOUS, name = "c[%d,%d]" % (b, k)))
                self.p[b].append(self.model.addVar(vtype = GRB.CONTINUOUS, name = "p[%d,%d]" % (b, k)))
        
        for b in range(self.Builds):
            self.h.append([])
            for k, machine in enumerate(self.Machines):
                self.h[b].append(self.model.addVar(vtype = GRB.INTEGER, name = "hb[%d,%d]" % (b, k)))

        self.MakeSpan = self.model.addVar(vtype = GRB.CONTINUOUS, name = "MakeSpan")

    def CreateConstraints(self):
        for b in range(self.Builds):
            for k, machine in enumerate(self.Machines):
                self.model.addConstr(self.MakeSpan >= self.c[b][k], name = "Makespan[%d,%d]" % (b, k))

        for i, part in enumerate(self.Parts):
            self.model.addConstr(sum(self.x[i][b][machine.MachineId] for b in range(self.Builds) for machine in self.Machines) == 1, name = "AllPartsProcessed[%d]" % i)
        
        for i, part in enumerate(self.Parts):
            for b in range(self.Builds):
                for k, machine in enumerate(self.Machines):
                    self.model.addConstr(self.x[i][b][k] <= self.y[b][k], name = "NoAssignmentToUnusedBuilds[%d,%d,%d]" % (i, b, k))
        
        for i, part in enumerate(self.Parts):
            for b in range(self.Builds):
                for k, machine in enumerate(self.Machines):
                    self.model.addConstr(self.h[b][k] >= part.Height * self.x[i][b][k], name = "BuildHeight[%d,%d,%d]" % (i, b, k))
        
        for b in range(self.Builds):
            for k, machine in enumerate(self.Machines):
                if b == 0:
                    self.model.addConstr(self.c[b][k] >= self.p[b][k], name = "CompletionTime1[%d,%d]" % (b, k))
                else:
                    self.model.addConstr(self.c[b][k] >= self.c[b - 1][k] + self.p[b][k], name = "CompletionTime2[%d,%d]" % (b, k))

        for b in range(self.Builds):
            for k, machine in enumerate(self.Machines):
                self.model.addConstr(self.p[b][k] == machine.Setup * self.y[b][k] + machine.ScanTime * sum(part.Volume * self.x[part.PartId][b][k] for part in self.Parts) + machine.RecoatTime * self.h[b][k], name = "ProcessTime[%d,%d]" % (b,k))
       
        self.model.setObjective(self.MakeSpan, GRB.MINIMIZE)

        ### Symmetry Breaking Constraints
        ### Valid Inequalities
        """ for b in range(self.Builds):
            if b > 0:
                for k, machine in enumerate(self.Machines):
                    self.model.addConstr(sum(self.x[part.PartId][b][k] for part in self.Parts) <= sum(self.x[part.PartId][b-1][k] for part in self.Parts), name = "IncrementallyPartFilling[%d,%d]" %(b, k))
         """
        """ for b in range(self.Builds):
            for k, machine in enumerate(self.Machines):
                self.model.addConstr(self.y[b][k] * min(part.Height for part in self.Parts if part.Height <= machine.Height) <=  self.h[b][k], name = "HeightRestriction3[%d,%d]" % (b, k))
         """

    def InitializePreprocessResults(self, incompatibleParts, infeasibleAssignments, partAssignment, 
                                    enablePOP, enablePR, enablePH, enableCL, enableCS, orthogonalPackingMethod, 
                                    heuristicTimeLimit, StairCaseRestriction):

        self.IncompatibleParts = incompatibleParts
        self.InfeasibleAssignments = infeasibleAssignments
        self.PartAssignment = partAssignment 
        
        self.AddInfeasibleAssignmentCuts(infeasibleAssignments)
        self.SetCallbackData(enablePOP, enablePR, enablePH, enableCL, enableCS, orthogonalPackingMethod, heuristicTimeLimit, StairCaseRestriction)
        
    def AddSymmetryBreakingConstraints(self):
        for b in range(self.Builds):
            if b > 0:
                for k, machine in enumerate(self.Machines):
                    self.model.addConstr(self.y[b - 1][k] >= self.y[b][k], name="SuccessiveBuildUsage[%d,%d]" % (b, k))
        
        for i, part in enumerate(self.Parts):
            for b in range(self.Builds):
                if b > i:
                    for k, machine in enumerate(self.Machines):
                        self.model.addConstr(self.x[i][b][k] == 0, name="AssignmentBreaking[%d,%d,%d]" % (i, b, k))

        for b in range(self.Builds):
            for k, machine in enumerate(self.Machines):
                self.model.addConstr(self.h[b][k] <= max(part.Height for part in self.Parts if part.Height <= machine.Height) * self.y[b][k], name = "AreaConsumption[%d,%d]" % (b, k))
        
        self.AddIncompatibilityCuts()
        #self.LiftIncompatibleParts()

    def AddAreaRestriction(self):
        for b in range(self.Builds):
           for k, machine in enumerate(self.Machines):
               self.model.addConstr(sum(self.x[part.PartId][b][k]*part.Area for part in self.Parts) <= machine.Area, name = "AreaConsumptionRestricted[%d,%d]" % (b, k))

        
    def AddEnforcedAreaRestriction(self):
        for b in range(self.Builds):
           for k, machine in enumerate(self.Machines):
               self.model.addConstr(sum(self.x[part.PartId][b][k]*part.Area for part in self.Parts) <= 0.9*machine.Area, name = "AreaConsumptionRestricted[%d,%d]" % (b, k))

    def AddMIPStartSolution(self):
        if self.PartAssignment == None:
            return
            
        for i, part in enumerate(self.Parts):
           for b in range(self.Builds):
               for k, machine in enumerate(self.Machines):
                   self.x[i][b][k].Start = self.PartAssignment[i][b][k]
        return

    def AddIncompatibilityCuts(self):
        for k, incompatibleSets in self.IncompatibleParts.items():
            for b in range(self.Builds):
                for pair in incompatibleSets:
                    self.model.addConstr(sum(self.x[i][b][k] for i in pair) <= 1)

    def AddInfeasibleAssignmentCuts(self, infeasibleAssignments):
        self.InfeasibleAssignments = infeasibleAssignments
        for assignment in infeasibleAssignments:
            i, k = assignment
            for b in range(self.Builds):
                self.model.addConstr(self.x[i][b][k] == 0, name="InfeasibleMachineAssignment[%d,%d,%d]" % (i, b, k))
    
    def SetCallbackData(self, enablePOP, enablePR, enablePH, enableCL, enableCS, orthogonalPackingMethod, heuristicTimeLimit, StairCaseRestriction):
        self.model._Parts = self.Parts
        self.model._Builds = self.Builds
        self.model._Machines = self.Machines
        self.model._XAssignments = self.x
        self.model._YBatches = self.y
        self.model._Seed = self.Seed
        self.model._PlacementPoints = {}

        self.model._EnablePreprocessing = enablePOP
        self.model._EnablePackingRelaxations = enablePR
        self.model._EnablePackingHeuristics = enablePH
        self.model._EnableCutLifting = enableCL
        self.model._EnableCutStrengthening = enableCS

        self.model._OrthogonalPackingMethod = orthogonalPackingMethod
        self.model._HeuristicTimeLimit = heuristicTimeLimit
        self.model._StairCaseRestriction = StairCaseRestriction

        self.model._FeasibleAssignments = {}

        self.model._InfeasibleAssignments = self.InfeasibleAssignments
        self.model._IncompatibleParts = self.IncompatibleParts
        self.model._ProvenInfeasibleSets = {machine.MachineId: [] for machine in self.Machines}

        self.model._PlacementStrategy = PlacementPointStrategy.MinimalMeetInTheMiddle
        self.model._AreaConstraints = 0
        self.model._LBConstraints = 0
        self.model._ORConstraints = 0
        self.model._BRConstraints = 0
        self.model._BPConstraints = 0
        self.model._TotalCuts = 0
        self.model._BPTimeOuts = 0
        self.model._TOAssignments = []

        self.model._TimeConPP = 0
        self.model._TimeConLB = 0
        self.model._TimeConOR = 0
        self.model._TimeConBR = 0
        self.model._TimeConBP = 0
        self.model._TimeConSC = 0
        self.model._Checks = []

        self.model._BatchChecks = 0
        self.model._SolutionChecks = 0
    
    def LiftIncompatibleParts(self):
        for k, incompatibleSets in self.IncompatibleParts.items():
            for pair in incompatibleSets:
                tmpMachine = self.Machines[k]
                tmpParts = [part for part in self.Parts if part.PartId in pair]
                MachineScheduling.AddLiftingCut(self.model, tmpParts, tmpMachine)

    @staticmethod
    def AddLiftingCut(model, assignedParts, machine):
        profits = [1 if model._Parts[i] in assignedParts else 0 for i in range(len(model._Parts))]
        liftingCoefficients = list(profits)

        liftedAssignment = [part for part in assignedParts]
        additionalParts = []
        for i, part in enumerate(model._Parts):
            if part in assignedParts or (part.PartId, machine.MachineId) in model._InfeasibleAssignments:
                continue

            noValidAssignment = False
            for partJ in assignedParts:
                if frozenset((part.PartId, partJ.PartId)) in model._IncompatibleParts[machine.MachineId] or frozenset((part, partJ)) in model._ProvenInfeasibleSets[machine.MachineId]:
                    noValidAssignment = True
                    break
            
            if noValidAssignment:
                continue

            upperBound = ValidationCallback.UKnapSack2DSolve(liftedAssignment, part, machine, profits, model)

            liftingCoefficient = max(0, len(assignedParts) - 1 - upperBound)

            if liftingCoefficient > 0:
                liftedAssignment.append(part)
                additionalParts.append(part)

                liftingCoefficients[part.PartId] = int(liftingCoefficient)
                profits[part.PartId] = int(liftingCoefficient)

        MachineScheduling.AddLiftedCoverInequality(model, assignedParts, machine, additionalParts, liftingCoefficients)

    @staticmethod
    def AddLiftedCoverInequality(model, assignedParts, machine, additionalParts, liftingCoefficients):
        for b in range(model._Builds):
            expr = gp.LinExpr()
            for part in assignedParts:
                expr += model._XAssignments[part.PartId][b][machine.MachineId]
            
            for part in additionalParts:
                expr += liftingCoefficients[part.PartId] * model._XAssignments[part.PartId][b][machine.MachineId]
            
            model.addConstr(expr <= len(assignedParts) - 1)
    
    def Solve(self, threads, timeLimit, mipGap, modelId):
        # self.model.write(os.path.join(self.Path, str(modelId)) + "_ModelFile" + str(self.Seed) + ".LP")
        self.model.setParam("LogFile", os.path.join(self.Path, str(modelId)) + "_logfileBnC" + str(self.Seed) + ".txt")
        
        self.model.Params.Seed = self.Seed
        self.model.Params.Threads = threads
        self.model.Params.TimeLimit = timeLimit
        self.model.Params.MIPFocus = 2
        self.model.Params.MIPGap = mipGap
        self.model.Params.lazyConstraints = 1

        self.model._TimeLimit = timeLimit
        self.model._IsPreSolve = False

        self.model.optimize(ValidationCallback.Callback)
            
    def PreSolve(self, threads, timeLimit, modelId):
        self.model.setParam("LogFile", os.path.join(self.Path, str(modelId)) + "_logfilePreSolve" + str(self.Seed) + ".txt")
        self.model.Params.Seed = self.Seed
        self.model.Params.Threads = threads
        self.model.Params.TimeLimit = timeLimit
        self.model.Params.MIPFocus = 1
        self.model.Params.lazyConstraints = 1

        self.model._TimeLimit = timeLimit   

        self.model._EnablePreprocessing = False

        self.model._IsPreSolve = True
        self.model._EarlyStop = 20
        self.model._CurrentObj = np.inf
        self.model._CurrentTime = time.time()

        self.model.optimize(ValidationCallback.Callback)

        return (self.model.objval != np.inf)
    
    def ExtractSolution(self):
        partAssignment = [[[0 for i in range(len(self.Machines))] for b in range(len(self.Parts))] for k in range(len(self.Parts))]
        for i, item in enumerate(self.Parts):
            for b in range(self.Builds):
               for k, machine in enumerate(self.Machines):
                   if self.x[i][b][k].X > 0.5:
                       partAssignment[i][b][k] = 1
        
        return partAssignment
