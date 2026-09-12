import math
import time
import copy

from OrthogonalPacker import *
from Preprocess import *
from LiftingLP import *



class StripCallback:
    def __init__(self):
        pass

    @staticmethod
    def AddCut(model, activeItems, label = ""):
        
        model.cbLazy(sum(model._XAssignments[j][activeItems[j]] for j in activeItems) <= len(activeItems) - 1)

    @staticmethod
    def AddLiftedCut(model, activeItems):

        model.cbLazy(sum(sum(model._XAssignments[j][arc] for arc in model._Arcs[j] if activeItems[j]['Left'] <= arc[0] <= activeItems[j]['Right']) for j in activeItems) <= len(activeItems) - 1)
    
    @staticmethod
    def CheckMemory(model, activeItems):
        if activeItems in model._FeasibleAssignments:
            return True
        elif activeItems in model._InfeasibleAssignments:
            return False
        return None

    @staticmethod
    def CheckAssignment(model, activeItems):
        isFeasible = StripCallback.CheckMemory(model, activeItems)
        
        if isFeasible == None:
            slaveStripPacking = SlaveStripPacking()
            slaveStripPacking.CreateVariables(activeItems, model._Lengths, model._Container.Length, model._Container.Width)
            slaveStripPacking.CreateConstraints()
            isFeasible = slaveStripPacking.Solve(model, max(0, model._TimeLimit - model.cbGet(GRB.Callback.RUNTIME)))

            if isFeasible:
                model._FeasibleAssignments.append(activeItems)
            else:
                model._InfeasibleAssignments.append(activeItems)

        return isFeasible
    
    @staticmethod
    def CheckAssignmentHeuristically(model, activeItems):
        isFeasible = StripCallback.CheckMemory(model, activeItems)
        
        if isFeasible == None:
            slaveStripPacking = SlaveStripPacking()
            slaveStripPacking.CreateVariables(activeItems, model._Lengths, model._Container.Length, model._Container.Width)
            slaveStripPacking.CreateConstraints()
            isFeasible = slaveStripPacking.HeuristicSolve()

            if isFeasible:
                model._FeasibleAssignments.append(activeItems)
        
        return isFeasible
    
    @staticmethod
    def AddLiftedCuts(model, minimalSubSets):
        for subSet in minimalSubSets:
            liftedCut = LiftingAreaPlanner()
            liftedCut.AddItems(subSet)
            liftedCut.AddModel(model)

            liftedCut.DetermineOverlappingItems()
            liftedCut.CreateVariables()
            liftedCut.CreateConstraints()

            liftedCut.Solve()
            
            if liftedCut.model.status == 2:
                activeItems = {j: {'Left': liftedCut.l[j].X, 'Right': liftedCut.r[j].X} for j in subSet.keys()}
                StripCallback.AddLiftedCut(model, activeItems)
                

    @staticmethod
    def GetCutPoints(activeItems):
        maxEnd = max([val[1] for val in activeItems.values()])
        cutPoints = []
        for p in range(maxEnd + 1):
            N_1 = {j for j in activeItems if activeItems[j][1] <= p}
            N_2 = {j for j in activeItems if activeItems[j][0] >= p}
            if len(N_1) > 0 and len(N_2) > 0 and len(N_1) + len(N_2) == len(activeItems):
                cutPoints.append(p)
        return cutPoints
    
    @staticmethod
    def GetInfeasibleSubsets(model, activeItems, cutPoints):
        infeasibleSubSets = []
        for p in cutPoints:
            tmpDictBefore = {i: val for i, val in activeItems.items() if val[1] <= p}
            tmpDictAfter = {i: val for i, val in activeItems.items() if val[0] >= p}
            
            if tmpDictBefore not in infeasibleSubSets:
                isFeasible = StripCallback.CheckAssignmentHeuristically(model, tmpDictBefore)
                if isFeasible != True and isFeasible != None:
                    infeasibleSubSets.append(tmpDictBefore)
            if tmpDictAfter not in infeasibleSubSets:
                isFeasible = StripCallback.CheckAssignmentHeuristically(model, tmpDictAfter)
                if isFeasible != True and isFeasible != None:
                    infeasibleSubSets.append(tmpDictAfter)
        
        if infeasibleSubSets == []:
            infeasibleSubSets = [activeItems]

        return infeasibleSubSets
    
    @staticmethod
    def ReduceSubSets(model, infeasibleSubSets):
        partialReducedSubSets = []
        alreadyVisited = []

        while infeasibleSubSets != []:
            subSet = infeasibleSubSets.pop()
            startColumn = min([val[0] for val in subSet.values()])
            for p in range(startColumn, model._Container.Width):
                tmpSet = {i: val for i, val in subSet.items() if val[0] != p}
                if tmpSet not in partialReducedSubSets and tmpSet not in alreadyVisited:
                    alreadyVisited.append(tmpSet)
                    
                    isFeasible = StripCallback.CheckAssignmentHeuristically(model, tmpSet)
                    
                    if isFeasible != True and isFeasible != None:
                        subSet = {i: val for i, val in tmpSet.items()}
                    else:
                        partialReducedSubSets.append(subSet)
                        break
        
        reducedSubSets = []
        alreadyVisited = []

        while partialReducedSubSets != []:
            subSet = partialReducedSubSets.pop()
            startColumn = max([val[1] - 1 for val in subSet.values()] + [model._Container.Width - 1])
            for p in range(startColumn, -1, -1):
                tmpSet = {i: val for i, val in subSet.items() if val[1] != p + 1}
                if tmpSet not in reducedSubSets and tmpSet not in alreadyVisited:
                    alreadyVisited.append(tmpSet)
                    
                    isFeasible = StripCallback.CheckAssignmentHeuristically(model, tmpSet)
                    
                    if isFeasible != True and isFeasible != None:
                        subSet = {i: val for i, val in tmpSet.items()}
                    else:
                        reducedSubSets.append(subSet)
                        break
        return reducedSubSets

    @staticmethod
    def FindMinimalSubSets(model, reducedSubSets):
        minimalSubSets = []

        for subSet in reducedSubSets:
            subSet = dict(sorted(subSet.items(), key = lambda item: model._Widths[item[0]] * model._Lengths[item[0]]))

            reduced = True
            while reduced:
                reduced = False
                for j in subSet:
                    tmpSet = {i: val for i, val in subSet.items() if i != j}
                    
                    isFeasible = StripCallback.CheckAssignmentHeuristically(model, tmpSet)

                    if isFeasible != True and isFeasible != None:
                        subSet = {i: val for i, val in tmpSet.items()}
                        reduced = True
                        break
                    else:
                        minimalSubSets.append(tmpSet)
                        break

        return minimalSubSets
    
    @staticmethod
    def AddStrengthendCut(model, activeItems):
        cutPoints = StripCallback.GetCutPoints(activeItems)

        if cutPoints == []:
            infeasibleSubSets = [activeItems]
        else:
            infeasibleSubSets = StripCallback.GetInfeasibleSubsets(model, activeItems, cutPoints)

        reducedSubSets = StripCallback.ReduceSubSets(model, infeasibleSubSets)

        minimalSubSets = StripCallback.FindMinimalSubSets(model, reducedSubSets)

        if model._EnableCutLifting:
            StripCallback.AddLiftedCuts(model, minimalSubSets)
        else:
            for subSet in minimalSubSets:
                StripCallback.AddCut(model, subSet, "CS")
        
    @staticmethod
    def AddCuts(model, activeItems, timelimitReached):
        
        if timelimitReached:
            StripCallback.AddCut(model, activeItems, "TO")
            return
        
        isFeasible = StripCallback.CheckAssignment(model, activeItems)

        if isFeasible:
            return
        
        if model._EnableCutStrengthening:
            StripCallback.AddStrengthendCut(model, activeItems)
        else:
            StripCallback.AddCut(model, activeItems, "OP")

    @staticmethod
    def FindActiveItems(model):
        activeItems = {}
        for q in range(model._Container.Width):
            for j in model._Arcs.keys():
                if j > 0:
                    W_hat = [i for i in range(q - model._Widths[j] + 1, q + 1)]
                    if sum([model.cbGetSolution(model._XAssignments[j][(p, p + model._Widths[j])]) for p in W_hat if (p, p + model._Widths[j]) in model._Arcs[j]]) == 1:
                        if j not in activeItems.keys():
                            activeItems[j] = [arc for arc in model._Arcs[j] if model.cbGetSolution(model._XAssignments[j][arc]) > 0.5][0]

        return activeItems

    @staticmethod
    def Callback(model, where):
        if where == GRB.Callback.MIPSOL:
            timelimitReached = False
            if model.cbGet(GRB.Callback.RUNTIME) > model._TimeLimit:
                timelimitReached = True
                model.terminate()

            activeItems = StripCallback.FindActiveItems(model)
            StripCallback.AddCuts(model, activeItems, timelimitReached)
