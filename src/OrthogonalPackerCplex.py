from docplex.cp.model import *
from gurobipy import GRB

from PlacementPoints import *
from Enums import *

import collections

 
class CplexOrthogonalPacking2D:

    def __init__(self):
        self.xStarts = []
        self.xEnds = []
        self.yStarts = []
        self.yEnds = []
        self.Rotation = []

        self.Positions = {}

        self.Intervals = collections.defaultdict(list)
        self.PartWidths = {}
        self.PartLengths = {}

        self.VariantUsed = {}

        self.model = CpoModel('OrthogonalPacking2D')

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

            # vx = [interval_var(size=SIZE_SUBSQUARE[i], name='X{}'.format(i), end=(0, SIZE_SQUARE)) for i in range(NB_SUBSQUARE)]
            placementPointStartX = sorted([point for point in placementPointsX if point + min([variant.Width for variant in part.Variants]) <= self.Bin.Width])
            placementPointEndX = sorted([point + variant.Width for point in placementPointStartX for variant in part.Variants if point + variant.Width <= self.Bin.Width])

            placementPointStartY = sorted([point for point in placementPointsY if point + min([variant.Length for variant in part.Variants]) <= self.Bin.Length])
            placementPointEndY = sorted([point + variant.Length for point in placementPointStartY for variant in part.Variants if point + variant.Length <= self.Bin.Length])

            xStart = self.model.integer_var(0, self.Bin.Width, name='x')
            yStart = self.model.integer_var(0, self.Bin.Length, name='y')

            xStart.set_domain(placementPointStartX)
            yStart.set_domain(placementPointStartY)

            xInterval = self.model.interval_var(size = part.Variants[0].Width, end = (0, self.Bin.Width), optional = True, name = 'xInterval(' + tmpIntervalName + ')') 
            yInterval = self.model.interval_var(size = part.Variants[0].Length, end = (0, self.Bin.Length), optional = True, name = 'yInterval(' + tmpIntervalName + ')')

            xIntervalRot = self.model.interval_var(size = part.Variants[-1].Width, end = (0, self.Bin.Width), optional = True, name = 'xInterval(' + tmpIntervalName + ')') 
            yIntervalRot = self.model.interval_var(size = part.Variants[-1].Length, end = (0, self.Bin.Length), optional = True, name = 'yInterval(' + tmpIntervalName + ')')

            self.Intervals['x'].append(xInterval)
            self.Intervals['y'].append(yInterval)
            self.Intervals['xRot'].append(xIntervalRot)
            self.Intervals['yRot'].append(yIntervalRot)

            self.xStarts.append(xStart)
            self.yStarts.append(yStart)
            
            
    def CreateConstraints(self):

        # # Create dependencies between variables
        for i in range(len(self.Parts)):
            for j in range(i):
                self.model.add(  
                    (end_of(self.Intervals['x'][i]) + end_of(self.Intervals['xRot'][i]) <= start_of(self.Intervals['x'][j]) + start_of(self.Intervals['xRot'][j])) | 
                    (end_of(self.Intervals['x'][j]) + end_of(self.Intervals['xRot'][j]) <= start_of(self.Intervals['x'][i]) + start_of(self.Intervals['xRot'][i])) | 
                    (end_of(self.Intervals['y'][i]) + end_of(self.Intervals['yRot'][i]) <= start_of(self.Intervals['y'][j]) + start_of(self.Intervals['yRot'][j])) | 
                    (end_of(self.Intervals['y'][j]) + end_of(self.Intervals['yRot'][j]) <= start_of(self.Intervals['y'][i]) + start_of(self.Intervals['yRot'][i])))

        for i in range(len(self.Parts)):
            self.model.add(presence_of(self.Intervals['x'][i]) + presence_of(self.Intervals['xRot'][i]) == 1)
            # self.model.add(presence_of(self.Intervals['y'][i]) + presence_of(self.Intervals['yRot'][i]) == 1)
            self.model.add(presence_of(self.Intervals['x'][i]) == presence_of(self.Intervals['y'][i]))
            self.model.add(presence_of(self.Intervals['xRot'][i]) == presence_of(self.Intervals['yRot'][i]))
        
        for i in range(len(self.Parts)):
            self.model.add(if_then(presence_of(self.Intervals['x'][i]), start_of(self.Intervals['x'][i]) == self.xStarts[i]))
            self.model.add(if_then(presence_of(self.Intervals['y'][i]), start_of(self.Intervals['y'][i]) == self.yStarts[i]))
            self.model.add(if_then(presence_of(self.Intervals['xRot'][i]), start_of(self.Intervals['xRot'][i]) == self.xStarts[i]))
            self.model.add(if_then(presence_of(self.Intervals['yRot'][i]), start_of(self.Intervals['yRot'][i]) == self.yStarts[i]))

    def Solve(self, model, threads, timeLimit = 3600):
        if len(self.Parts) == 1:
            if (self.Parts[0].Variants[0].Width <= self.Bin.Width and self.Parts[0].Variants[0].Length <= self.Bin.Length) or (self.Parts[0].Variants[0].Length <= self.Bin.Width and self.Parts[0].Variants[0].Width <= self.Bin.Length):
                return True
            else:
                return False
        
        self.result = self.model.solve(Workers=threads, TimeLimit=timeLimit, LogVerbosity='Quiet')        
        status = self.result.get_solve_status()

        if status == "Feasible" or status == "Optimal":
            #print(self.solver.StatusName())
            return True
        elif status == "Infeasible":
            #print(self.solver.StatusName())
            return False
        elif status == "Unknown":
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
        
        self.result = self.model.solve(Workers=threads, TimeLimit=1, RandomSeed = model._Seed, LogVerbosity='Quiet')        
        status = self.result.get_solve_status()

        if status == "Feasible" or status == "Optimal":
            #print(self.solver.StatusName())
            return True
        elif status == "Infeasible":
            #print(self.solver.StatusName())
            return False
        elif status == "Unknown":
            return None
        else:
            raise ValueError("Something went wrong!")
                
    def ExtractSolution(self):
        self.StartPositionsX = [self.result.get_var_solution(self.Intervals['x'][i]).get_start() if self.result.get_var_solution(self.Intervals['x'][i]).is_present() else self.result.get_var_solution(self.Intervals['xRot'][i]).get_start() for i in range(len(self.Parts))]
        self.StartPositionsY = [self.result.get_var_solution(self.Intervals['y'][i]).get_start() if self.result.get_var_solution(self.Intervals['y'][i]).is_present() else self.result.get_var_solution(self.Intervals['yRot'][i]).get_start() for i in range(len(self.Parts))]
        self.FinalWidths = [self.Parts[i].Variants[0].Width if self.result.get_var_solution(self.Intervals['y'][i]).is_present() else self.Parts[i].Variants[0].Length for i in range(len(self.Parts))]
        self.FinalLengths = [self.Parts[i].Variants[0].Length if self.result.get_var_solution(self.Intervals['y'][i]).is_present() else self.Parts[i].Variants[0].Width for i in range(len(self.Parts))]

        return self.StartPositionsX, self.StartPositionsY, self.FinalLengths, self.FinalWidths
    
    def ShowResult(self):
        import docplex.cp.utils_visu as visu
        if self.result and visu.is_visu_enabled():
            import matplotlib.pyplot as plt
            import matplotlib.cm as cm
            from matplotlib.patches import Polygon

            # Plot external square
            print('Plotting squares...')
            fig, ax = plt.subplots()
            plt.plot((0, 0), (0, self.Bin.Length), (self.Bin.Width, self.Bin.Length), (self.Bin.Width, 0))
            for i in range(len(self.Parts)):
                # Display square i
                if not self.result.get_var_solution(self.Intervals['x'][i]).is_present():
                    sx, sy = self.result.get_var_solution(self.Intervals['xRot'][i]), self.result.get_var_solution(self.Intervals['yRot'][i])
                else:
                    sx, sy = self.result.get_var_solution(self.Intervals['x'][i]), self.result.get_var_solution(self.Intervals['y'][i])
                
                (sx1, sx2, sy1, sy2) = (sx.get_start(), sx.get_end(), sy.get_start(), sy.get_end())
                poly = Polygon([(sx1, sy1), (sx1, sy2), (sx2, sy2), (sx2, sy1)], fc=cm.Set2(float(i) / len(self.Parts)))
                ax.add_patch(poly)
                # Display identifier of square i at its center
                ax.text(float(sx1 + sx2) / 2, float(sy1 + sy2) / 2, str(self.Parts[i].PartId), ha='center', va='center')
            plt.margins(0)
            plt.show()

if __name__ == "__main__":
    from DataManager import *
    from Scheduler import *
    newData = DataModel(0, True)
    newData.BuildData(f"C:/Users/Administrator/Documents/BNC_Repo/BNC_Tests/02_TestData/Che_ht2_MHU/ht2_11.json")

    tmpParts = [part for p, part in enumerate(newData.Parts)]
    print([(part.Variants[0].Width, part.Variants[0].Length) for part in tmpParts])
    binPacking2D = CplexOrthogonalPacking2D()
    # newData.Machines[1].Length = 130
    binPacking2D.AddParts(tmpParts)
    binPacking2D.AddBin(newData.Machines[1])


    binPacking2D.CreateVariables(PlacementPointStrategy.MinimalMeetInTheMiddle)
    binPacking2D.CreateConstraints()

    tmpModel = MachineScheduling("sjasdh", 23)
    tmpModel._Seed = 23
    isFeasible = binPacking2D.Solve(tmpModel, 8, 3600)
    print(isFeasible)