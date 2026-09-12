from collections import namedtuple
import numpy as np
import DataManager
import math

import gurobipy as gp
from gurobipy import GRB

class RelaxedPacker:
    def __init__(self):

        self.x = []
                
        #self.model = gp.Model()
        self.ConservativeScales = {}
    
    def AddItems(self, items):# items = [(width, length)....]
        ReducedItem = namedtuple("ReducedItem", "Width Length")

        self.OriginalNumItems = len(items)
        self.Items = [ReducedItem(item[0], item[1]) for item in items]
        self.Items += [ReducedItem(item[1], item[0]) for item in items]
            
    def AddBin(self, binWidth, binLength):
        self.BinWidth = binWidth
        self.BinLength = binLength
    
    ## According to Fekete and Schepers (2004) L_2d
    def CreateConservativeScales(self, pValues, qValues):
        scaledLengths = [item.Length/self.BinLength for item in self.Items]
        scaledWidths = [item.Width/self.BinWidth for item in self.Items]

        def ApplyDFF_u(scaledXArray, k):  
    
            newArray = []

            for x in scaledXArray:
                if (round(x*(k + 1), 5)).is_integer():
                    newArray.append(x)
                else:
                    newArray.append(math.floor(round(x*(k + 1), 5))*(1/k))
            
            return newArray

        def ApplyDFF_U(scaledXArray, eps):

            newArray = []

            for x in scaledXArray:
                if x > 1 - eps:
                    newArray.append(1)
                elif eps <= x <= 1 - eps:
                    newArray.append(x)
                else:
                    newArray.append(0)
            
            return newArray

        def ApplyDFF_Phi(scaledXArray, eps):

            newArray = []

            for x in scaledXArray:
                if x > 0.5:
                    newArray.append(1 - (math.floor(round((1- x)*eps**(-1), 5)))/math.floor(round(eps**(-1), 5)))
                elif eps <= x <= 0.5:
                    newArray.append(1/math.floor(eps**(-1)))
                else:
                    newArray.append(0)
            
            return newArray

        idxCounter = 0

        for p in pValues:
            self.ConservativeScales[idxCounter] = {"Widths": ApplyDFF_u(scaledWidths, 1), "Lengths": ApplyDFF_U(scaledLengths, p)}
            idxCounter += 1
        
        for p in pValues:
            self.ConservativeScales[idxCounter] = {"Widths": ApplyDFF_U(scaledWidths, p), "Lengths": ApplyDFF_u(scaledLengths, 1)}
            idxCounter += 1
        
        for p in pValues:
            self.ConservativeScales[idxCounter] = {"Widths": ApplyDFF_u(scaledWidths, 1), "Lengths": ApplyDFF_Phi(scaledLengths, p)}
            idxCounter += 1
        
        for p in pValues:
            self.ConservativeScales[idxCounter] = {"Widths": ApplyDFF_Phi(scaledWidths, p), "Lengths": ApplyDFF_u(scaledLengths, 1)}
            idxCounter += 1
        
        for p in pValues:
            self.ConservativeScales[idxCounter] = {"Widths": [w for w in scaledWidths], "Lengths": ApplyDFF_U(scaledLengths, p)}
            idxCounter += 1
        
        for p in pValues:
            self.ConservativeScales[idxCounter] = {"Widths": ApplyDFF_U(scaledWidths, p), "Lengths": [l for l in scaledLengths]}
            idxCounter += 1
        
        for p in pValues:
            for q in qValues:
                self.ConservativeScales[idxCounter] = {"Widths": ApplyDFF_Phi(scaledWidths, p), "Lengths":  ApplyDFF_Phi(scaledLengths, q)}
                idxCounter += 1

    ## According to Fekete and Schepers (2007)
    def CreateConservativeScales2(self):
        scaledLengths = [item.Length/self.BinLength for item in self.Items]
        scaledWidths = [item.Width/self.BinWidth for item in self.Items]

        def ApplyDFF_u(scaledXArray, k):  
    
            newArray = []

            for x in scaledXArray:
                if (round(x*(k + 1), 5)).is_integer():
                    newArray.append(x)
                else:
                    newArray.append(math.floor(round(x*(k + 1), 5))*(1/k))
            
            return newArray

        idxCounter = len(self.ConservativeScales)

        for k in range(1, int(self.BinWidth/2) + 1):
            self.ConservativeScales[idxCounter] = {"Widths": ApplyDFF_u(scaledWidths, k), "Lengths": [l for l in scaledLengths]}
            idxCounter += 1
        
        for k in range(1, int(self.BinLength/2) + 1):
            self.ConservativeScales[idxCounter] = {"Widths": [w for w in scaledWidths], "Lengths": ApplyDFF_u(scaledLengths, k)}
            idxCounter += 1
        
    def DetermineConservativeScaledAreas(self):

        self.ConservativeScaledAreas = {}

        for key, scale in self.ConservativeScales.items():
            if any([w > 1 for w in scale["Widths"]]) or any([w > 1 for w in scale["Lengths"]]):
                continue
            self.ConservativeScaledAreas[key] = [a*b for a,b in zip(scale['Widths'],scale["Lengths"])]
            #print(f"{key}:  {round(sum(self.ConservativeScaledAreas[key][:self.OriginalNumItems]), 4)} {round(sum(self.ConservativeScaledAreas[key][:]), 4)}")

    def GetLowerBound(self):
        tmpList = []

        for c, scaledAreas in self.ConservativeScaledAreas.items():
            tmpList.append(sum([min(scaledAreas[i], scaledAreas[i + self.OriginalNumItems]) for i in range(self.OriginalNumItems)]))
        
        return max(tmpList)
    
    def CreateVariables(self):
        for i, item in enumerate(self.Items):
            self.x.append(self.model.addVar(vtype=GRB.BINARY, name="x[%i]" % i))
                
    def CreateConstraints(self):
        for i in range(self.OriginalNumItems):
            self.model.addConstr(self.x[i] + self.x[i + self.OriginalNumItems] == 1, name="RotationDecision[%d]" % i)
        
        for i, item in enumerate(self.Items):
            if item.Width > self.BinWidth or item.Length > self.BinLength:
                self.model.addConstr(self.x[i] == 0, name="InfeasibleRotation[%d]" % i)
            
        for c, scaledAreas in self.ConservativeScaledAreas.items():
            self.model.addConstr(sum(scaledAreas[i] * self.x[i] +  scaledAreas[i + self.OriginalNumItems] * self.x[i + self.OriginalNumItems] for i in range(self.OriginalNumItems)) <= 1, name="RelaxationDFF[%d]" % (c))
        
                    
    def Solve(self, threads, timeLimit):
        self.model.setParam("Outputflag", 0)
        #self.model.write("_Model" + ".lp")
        #self.model.setParam("LogFile", self.Path[:-5] + "_logfileMIP" + str(self.Seed) + ".txt")
        self.model.Params.Seed = self.Seed
        

        self.model.Params.Threads = threads
        self.model.Params.TimeLimit = timeLimit
        #self.model.Params.MIPGap = 0.000006
        # self.model.computeIIS()
        # self.model.write("Model" + ".ilp")
        self.model.optimize()
        # if self.model.status == GRB.INFEASIBLE:
        #     self.model.computeIIS()
        #     self.model.write("Model" + ".ilp")
    
        # print(f"Status: {self.model.status}")
        # print(f"Runtime: {self.model.Runtime}")
        return self.model.status == GRB.INFEASIBLE

if __name__ == "__main__":
    # data = DataManager.DataModel(0, True)

    # data.BuildData("U:/Zipfel/01 - Additive_Manufacturing/02 - AdditiveManufacturing Tests/02 - Testdata revised/09 - Finaltest_BnCvsMIP/ht2_2.json")
        
    # machineId = 1
    # machine = data.Machines[machineId]
    # partIds = [ 11,
    #                     12,
    #                     15,
    #                     16,
    #                     19,
    #                     20,
    #                     21,
    #                     23,
    #                     25,
    #                     26,
    #                     27,
    #                     28,
    #                     34,
    #                     35,
    #                     38,
    #                     39,
    #                     46]

    # tmpItems = [(item.Variants[0].Width, item.Variants[0].Length) for i, item in enumerate(data.Parts) if item.PartId in partIds]

    newRelaxedModel = RelaxedPacker(11)
    newRelaxedModel.AddItems([(28, 22), (28, 28)])
    newRelaxedModel.AddBin(50, 28)
    newRelaxedModel.CreateConservativeScales([0.01, 0.1, 0.2, 0.3, 0.4, 0.49], [0.01, 0.1, 0.2, 0.3, 0.4, 0.49])
    newRelaxedModel.CreateConservativeScales2()
    newRelaxedModel.DetermineConservativeScaledAreas()
    newRelaxedModel.CreateVariables()
    newRelaxedModel.CreateConstraints()
    print(newRelaxedModel.Solve(6, 10))

