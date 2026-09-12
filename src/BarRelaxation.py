import gurobipy as gp
from gurobipy import GRB

import copy

class BarRelaxationVariant:
    def __init__(self):
        self.mastermodel = gp.Model()
        self.pricingmodel = gp.Model()

        self.z = {}
        self.x = {}
        self.items = {}
        self.ChoosingConstraints = {}
        self.RestrictingConstraints = {}
    
    def AddParts(self, parts):
        self.Parts = parts
        self.NumParts = len(parts)
        self.NumRealParts = int(len(parts)/2)
    
    def AddPattern(self, pattern):
        self.Patterns = copy.deepcopy(pattern)
    
    def AddMachine(self, machine):
        self.Machine = machine

    def Solve(self, seed):
        self.mastermodel.setParam("Outputflag", 0)
                
        self.T = len(self.Patterns)

        for t in range(self.T):
            self.z[t] = self.mastermodel.addVar(vtype = GRB.INTEGER, name = "z[%d]" % t)
        
        for j in range(self.NumRealParts):
            self.items[j] = self.mastermodel.addConstr(sum(self.Patterns[t][j] * self.z[t] for t in range(self.T)) >= self.Parts[j]["width"], name = "c(%d)" % j)

        self.mastermodel.setObjective(sum(self.z[t] for t in range(self.T)), GRB.MINIMIZE)
        self.mastermodel.update()

        while True:
            relax = self.mastermodel.relax()
            relax.setParam("Outputflag", 0)
            relax.optimize()
            _lam = [relax.getConstrByName("c(" + str(j) + ")").Pi for j in range(self.NumRealParts)] + [relax.getConstrByName("c(" + str(j) + ")").Pi for j in range(self.NumRealParts)]
            pricing = gp.Model()
            y = {}
            for j in range(self.NumParts):
                y[j] = pricing.addVar(vtype = GRB.BINARY, name = "y[%d]" % j)
            pricing.update()
            pricing.addConstr(sum(self.Parts[j]["length"] * y[j] for j in range(self.NumParts)) <= self.Machine.Length)
            for j in range(self.NumRealParts):
                pricing.addConstr(y[j] + y[j + self.NumRealParts] <= 1)
            pricing.setObjective(1 - (sum((_lam[j])* y[j] for j in range(self.NumRealParts)) + sum((_lam[j])*(self.Parts[j]["length"]/self.Parts[j]["width"]) * y[j] for j in range(self.NumRealParts, self.NumParts))), GRB.MINIMIZE)
            
            pricing.setParam("Outputflag", 0)
            #pricing.Params.Seed = seed
            pricing.Params.PoolSearchMode = 0
            pricing.Params.Threads = 8
            pricing.Params.TimeLimit = 2
            pricing.Params.BestObjStop = -0.00001
            pricing.optimize()
            
            if pricing.ObjVal >= -0.00001:
                break
            
            ### Add All Solutions found to the rMP ###
            NumSolutions = pricing.SolCount
            for sol in range(NumSolutions):
                pricing.Params.SolutionNumber = sol
                allPatterns = pricing.Xn
                pat = []
                for i in range(self.NumRealParts):
                    if allPatterns[i] > 0.5 and allPatterns[i + self.NumRealParts] < 0.5:
                        pat.append(1)
                    elif allPatterns[i] < 0.5 and allPatterns[i + self.NumRealParts] > 0.5:
                        pat.append(self.Parts[i]["width"]/self.Parts[i]["length"])
                    else:
                        pat.append(0)
                self.Patterns.append(pat)
                newColumn = gp.Column()
                for j in range(self.NumRealParts):
                    if self.Patterns[self.T][j] > 0:
                        newColumn.addTerms(self.Patterns[self.T][j], self.items[j])
                        self.mastermodel.update()
                self.z[self.T] =self.mastermodel.addVar(obj=1, vtype=GRB.INTEGER, name = "z[%d]" % self.T, column = newColumn)
                self.T += 1
                self.mastermodel.update()

            self.mastermodel.update()
            del relax
        
        finalRelaxation = self.mastermodel.relax()
        finalRelaxation.setParam("Outputflag", 0)
        #finalRelaxation.Params.Seed = seed
        finalRelaxation.Params.Threads = 8
        finalRelaxation.Params.TimeLimit = 3600
        finalRelaxation.optimize()

        return finalRelaxation.ObjVal