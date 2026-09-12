import gurobipy as gp
from gurobipy import GRB

import copy

class LiftingAreaPlanner:
    def __init__(self):
        self.model = gp.Model()

        self.r = {}
        self.l = {}
    
    def AddItems(self, activeItems):
        self.ActiveItems = activeItems

    def AddModel(self, model):
        self.ModelData = model
    
    def DetermineOverlappingItems(self):
        self.OverlappingItems = {}
        for j, arc in self.ActiveItems.items():
            self.OverlappingItems[j] = []
            for i, arc2 in self.ActiveItems.items():
                if j != i:
                    if arc2[0] <= arc[0] and arc2[1] > arc[0]:
                        self.OverlappingItems[j].append(i)
                    elif arc2[0] < arc[1] and arc2[1] >= arc[1]:
                        self.OverlappingItems[j].append(i)

    def CreateVariables(self):
        for j in self.ActiveItems:
            self.r[j] = self.model.addVar(vtype = GRB.CONTINUOUS, name = "r[%d]" % j)
            self.l[j] = self.model.addVar(vtype = GRB.CONTINUOUS, name = "l[%d]" % j)
        self.model.update()

    def CreateConstraints(self):
        for j in self.ActiveItems.keys():
            for i in self.OverlappingItems[j]:
                self.model.addConstr(self.l[j] + self.ModelData._Widths[j] >= self.r[i] + 1, name = "(%d,%d)" % (j, i))
        
        for j in self.ActiveItems.keys():
            self.model.addConstr(self.l[j] >= 0, name = "l1(%d)" % j)
            self.model.addConstr(self.l[j] <= self.ActiveItems[j][0], name = "l2(%d)" % j)
            self.model.addConstr(self.r[j] >= self.ActiveItems[j][0], name = "r1(%d)" % j)
            self.model.addConstr(self.r[j] <= self.ModelData._Container.Width - self.ModelData._Widths[j], name = "r2(%d)" % j)

        self.model.setObjective(sum(self.r[j] - self.l[j] for j in self.ActiveItems.keys()), GRB.MAXIMIZE)
    def Solve(self):

        self.model.Params.OutputFlag = 0
        self.model.Params.Seed = self.ModelData._Seed
        self.model.Params.Threads = 8
        self.model.Params.TimeLimit = max(1, self.ModelData._TimeLimit - self.ModelData.cbGet(GRB.Callback.RUNTIME))
        # 

        self.model.optimize()
