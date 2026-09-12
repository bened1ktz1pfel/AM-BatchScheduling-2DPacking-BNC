import random
import copy
import numpy as np
from enum import IntEnum
import math

### Preliminary Classes ###
class SortingStrategy(IntEnum):
    NoStrategy = 0
    Height = 1
    Area = 2

class AllocationStrategy(IntEnum):
    NoStrategy = 0
    AreaShare = 1
    HeightShare = 2
    LocalSearch = 3

class Solution:
    def __init__(self, partList, machineList, sortedPartIds):
        self.Parts = partList
        self.Machines = machineList
        self.SortedPartIds = sortedPartIds
        self.AssignmentArray = []
        self.Makespan = -1

        self.MachineAllocation = {part.PartId: set() for part in self.Parts}
        self.PossibleMachines = {part.PartId: set() for part in self.Parts}
        self.PartPermutation = []
    
    def setPermutation(self, newPermutation):
        self.BinPermutation = newPermutation
    
    def setMakespan(self, newMakespan):
        self.Makespan = newMakespan
    
    def setPartAssignment(self):
        self.PartAssignment = [[[0 for i in range(len(self.Machines))] for b in range(len(self.Parts))] for k in range(len(self.Parts))]
        for k, machineAssignments in enumerate(self.AssignmentArray):
            for b, buildAssignment in enumerate(machineAssignments):
                for i, part in enumerate(buildAssignment):
                    self.PartAssignment[part.PartId][b][k] = 1    

class SolutionPool:
    def __init__(self):
        self.Solutions = []
    
    def AddSolution(self, newSolution):
        self.Solutions.append(newSolution)
    
    def __len__(self):
        return len(self.Solutions)

    def GetBestSolution(self):
        self.Solutions.sort(key = lambda solution: solution.Makespan)

        return self.Solutions[0]

class EvaluateObjective:
    def __init__(self):
        pass
    
    def DetermineMakespan(self, solution):
        processingTimes = []

        for k in range(len(solution.AssignmentArray)):
            processingTimes.append([])
            if solution.AssignmentArray[k] == []:
                continue
            for build in solution.AssignmentArray[k]:
                tmpProcessTime = solution.Machines[k].Setup + solution.Machines[k].ScanTime * sum(part.Volume for part in build) + max(part.Height for part in build) * solution.Machines[k].RecoatTime
                processingTimes[k].append(tmpProcessTime)
        solution.setMakespan(max(sum(times) for times in processingTimes))


### Algorithms ###
class BinPacker:
    def __init__(self, machine):
        self.binWidth = machine.Width
        self.binLength = machine.Length
        self.binHeight = machine.Height
    
    def IsFeasible(self, part):
        tmpCheckList = []
        for variant in part.Variants:
            tmpCheckList.append(variant.Width <= self.binWidth and variant.Length <= self.binLength)
        
        return any(tmpCheckList) and (part.Height <= self.binHeight)

    def FirstFit(self, parts):
        
        self.Bins = []
        partsplaced = 0

        partDims = [(max(item.Variants[0].Width, item.Variants[0].Length), min(item.Variants[0].Width, item.Variants[0].Length)) for item in [part for part in parts if self.IsFeasible(part) and part.Variants[0].Width <= self.binWidth]]
        partDims = sorted(partDims, key = lambda t: t[0], reverse=True)

        for i in range(len(partDims)):
            self.Bins.append([self.binWidth, 0])

        for part in partDims:
            for b in self.Bins:
                if b[1] + part[1] <= self.binLength:
                     b[1] = b[1] + part[1]
                     partsplaced += 1
                     break

        if partsplaced < len(partDims):
            print("Not all parts are assigned...")
        return len([b for b in self.Bins if b[1] > 0])
    
    def NextFit(self, parts):
        
        self.Bins = []
        partsplaced = 0

        partDims = [(max(item.Variants[0].Width, item.Variants[0].Length), min(item.Variants[0].Width, item.Variants[0].Length)) for item in [part for part in parts if self.IsFeasible(part)]]
        partDims = sorted(partDims, key = lambda t: t[0], reverse=True)

        tmpBin = [self.binWidth, 0]

        for part in partDims:
            if tmpBin[1] + part[1] <= self.binLength:
                tmpBin[1] = tmpBin[1] + part[1]
                partsplaced += 1
                continue
            else:
                self.Bins.append([i for i in tmpBin])
                tmpBin = [self.binWidth, part[1]]
                partsplaced += 1
                continue
        self.Bins.append([i for i in tmpBin])
        
        if partsplaced < len(partDims):
            print("Not all parts are assigned...")

        return len([b for b in self.Bins if b[1] > 0])

class BuildApproximater:

    def __init__(self, machines, parts):
        self.Machines = machines
        self.Parts = parts

        self.Builds = []
    
    def Extract(self, assignmentStrategy):
        if assignmentStrategy == "NoStrategy":
            return len(self.Parts)

        for machine in self.Machines:
            bp = BinPacker(machine)
            if assignmentStrategy == "FirstFit":
                self.Builds.append(bp.FirstFit(self.Parts))
            elif assignmentStrategy == "NextFit":
                self.Builds.append(bp.NextFit(self.Parts))
            else:
                print("Wrong Assignment Strategy!")
        
        return max(self.Builds)

### Construction of an initial solution ###
class ConstructiveHeuristic:
    def __init__(self, rng):
        self.RNG = rng

        self.EvaluationLogic = EvaluateObjective()
        self.SolutionPool = SolutionPool()

        self.Moves = []
        self.AreaShare = 1
    
    def InitializeAreaRestriction(self, areaShare):
        self.AreaShare = areaShare

    def CheckForFeasiblePartPair(self, partPair, machine):

        firstPart = partPair[0]
        secondPart = partPair[1]
        comparison = []
        for firstVariant in firstPart.Variants:
            for secondVariant in secondPart.Variants:

                comparison.append((firstVariant.Width + secondVariant.Width <= machine.Width and max(firstVariant.Length, secondVariant.Length) <= machine.Length) or 
                                (max(firstVariant.Width, secondVariant.Width) <= machine.Width and firstVariant.Length + secondVariant.Length <= machine.Length))
        
        return any(comparison)

    def InitialSortingParts(self, parts, criterion):
        if criterion == SortingStrategy.NoStrategy:
            self.RNG.shuffle(parts)

        elif criterion == SortingStrategy.Height:
            parts = sorted(parts, key = lambda part: -part.Height)
        
        elif criterion == SortingStrategy.Area:
            parts = sorted(parts, key = lambda part: (-part.Area, -max(variant.Width for variant in part.Variants)))

        return [part.PartId for part in parts]

    def CreateInitialSolution(self, partList, machineList, sortingStrategy, allocationStrategy, localSearchIterations):
        
        sortedPartIds = self.InitialSortingParts(copy.deepcopy(partList), sortingStrategy) # sorting Parts

        tmpSolution = Solution(partList, machineList, sortedPartIds) # creating a solution

        self.RetrievePossibleMachineAllocations(tmpSolution) # retrieve possible allocations

        self.AllocateParts(tmpSolution, allocationStrategy, localSearchIterations) # choose Allocations

        self.FirstFitBuildAssignment(tmpSolution)

        return tmpSolution

    def RetrievePossibleMachineAllocations(self, solution):

        for i, part in enumerate(solution.Parts):
            for machine in solution.Machines:

                if any([(variant.Width <= machine.Width and variant.Length <= machine.Length and part.Height <= machine.Height) for variant in part.Variants]):
                    solution.PossibleMachines[i].add(machine.MachineId)
    
    def TotalMachineRunTime(self, solution):
        allocations = [[part for i, part in enumerate(solution.Parts) if solution.MachineAllocation[i] == m] for m in range(len(solution.Machines))]

        runTimes = []

        for r in range(len(allocations)):
            machine = solution.Machines[r]
            runTimes.append(sum(part.Volume for part in allocations[r])*machine.ScanTime + max([part.Height for part in allocations[r]] + [0])*machine.RecoatTime)
            
        return max(runTimes)

    def DiscoverMoves(self, tmpSolution):
        self.Moves = []
        for p in tmpSolution.MachineAllocation.keys():
            currentAllocation = tmpSolution.MachineAllocation[p]
            for q in tmpSolution.PossibleMachines[p]:
                if q != currentAllocation:
                    newSolution = copy.deepcopy(tmpSolution)
                    newSolution.MachineAllocation[p] = q
                    
                    self.Moves.append(newSolution)

    def EvaluateMoves(self):
        self.EvaluatedMoves = []
        for move in self.Moves:
            self.EvaluatedMoves.append((move, self.TotalMachineRunTime(move)))
        
        self.EvaluatedMoves.sort(key= lambda t: t[1])

    def AllocationLocalSearch(self, tmpSolution, localSearchIterations):

        for i, part in enumerate(tmpSolution.Parts):

            tmpSolution.MachineAllocation[i] = self.RNG.choice(list(tmpSolution.PossibleMachines[i]), 1)[0]
        
        i = 0
        improvedSolution = True

        while improvedSolution and i < localSearchIterations:
            self.DiscoverMoves(tmpSolution)
            self.EvaluateMoves()
            if self.EvaluatedMoves != []:
                bestNeighbor = self.EvaluatedMoves[0]
            else:
                bestNeighbor = (-1, np.inf)

            if bestNeighbor[1] < self.TotalMachineRunTime(tmpSolution):
                tmpSolution = bestNeighbor[0]

            else:
                improvedSolution = False
            i += 1

        return tmpSolution           

    def AllocateParts(self, solution, criterion, localSearchIterations):

        if criterion == AllocationStrategy.NoStrategy:
            for i, part in enumerate(solution.Parts):

                solution.MachineAllocation[i] = self.RNG.choice(list(solution.PossibleMachines[i]), 1)[0]
        
        elif criterion == AllocationStrategy.AreaShare:
            for i, part in enumerate(solution.Parts):

                solution.MachineAllocation[i] = max([(m, part.Area/solution.Machines[m].Area) for m in solution.PossibleMachines[i]], key = lambda t: t[1])[0]
        
        elif criterion == AllocationStrategy.HeightShare:
            for i, part in enumerate(solution.Parts):

                solution.MachineAllocation[i]  = max([(m, part.Height/solution.Machines[m].Height) for m in solution.PossibleMachines[i]], key = lambda t: t[1])[0]

        elif criterion == AllocationStrategy.LocalSearch:

            solution = self.AllocationLocalSearch(solution, localSearchIterations)

    def FirstFitBuildAssignment(self, solution):
        solution.AssignmentArray = [[[] for i in range(len(solution.Parts))] for k in range(len(solution.Machines))]
        buildCounter = [0 for k in range(len(solution.Machines))]

        for i, idx in enumerate(solution.SortedPartIds):
            part = solution.Parts[idx]
            binId = solution.MachineAllocation[idx]

            if i == 0:
                solution.AssignmentArray[binId][buildCounter[binId]].append(part)
                continue
            
            assigned = False
            for build in solution.AssignmentArray[binId]:
                if not build:
                    continue
                tmpBuild = [item for item in build]
                tmpBuild.append(part)

                if len(tmpBuild) == 2:
                    if sum(part.Area for part in tmpBuild) > self.AreaShare*solution.Machines[binId].Area:
                        continue
                    isFeasible = self.CheckForFeasiblePartPair(tmpBuild, solution.Machines[binId])
                    if isFeasible:
                        build.append(part)
                        assigned = True
                        break
                
                if len(tmpBuild) > 2:
                    if sum(part.Area for part in tmpBuild) > self.AreaShare*solution.Machines[binId].Area:
                        continue

                    isFeasible = SFF(tmpBuild, solution.Machines[binId])
                    if isFeasible:
                        build.append(part)
                        assigned = True
                        break
            
            if not assigned:
                buildCounter[binId] += 1
                solution.AssignmentArray[binId][buildCounter[binId]].append(part)
                continue
        
        solution.AssignmentArray = [[sorted(build, key = lambda part: part.PartId) for build in assignments if build != []] for assignments in solution.AssignmentArray]

        solution.AssignmentArray = [sorted(assignments, key = lambda listOfParts: listOfParts[0].PartId) for assignments in solution.AssignmentArray]

        self.EvaluationLogic.DetermineMakespan(solution)

    def Run(self, data, sortingStrategies = [SortingStrategy.Area], allocationStrategies = [AllocationStrategy.LocalSearch], localSearchIterations = 5):
        
        for allocation in allocationStrategies:
            for sorting in sortingStrategies:
                initialSolution = self.CreateInitialSolution(data.Parts, data.Machines, sorting, allocation, localSearchIterations)
                
                for part in initialSolution.Parts:
                    initialSolution.PartPermutation.append(part.PartId)

                initialSolution.setPartAssignment()
                
                self.SolutionPool.AddSolution(initialSolution)

        return self.SolutionPool.GetBestSolution()

def SFF (parts, machine):

    machineWidth, machineLength = machine.Width, machine.Length
    if machineWidth < machineLength:
        machineWidth, machineLength = machineLength, machineWidth

    shelves = [[0,0,0],]  # ( lastx, floor, ceil )  For each shelf

    for part in parts:
        partWidth, partLength = part.Variants[0].Width, part.Variants[0].Length

        if partWidth < partLength:
            partWidth, partLength = partLength, partWidth

        if shelves[0][2] == 0:
            if partLength > machineLength:
                return False
            shelves[0][2] = partLength
            shelves[0][0] += partWidth
            continue
        
        for shelf in shelves:
            lastx, floor, ceil = shelf
            if lastx + partLength <= machineWidth and floor + partWidth <= ceil:
                shelf[0] += partLength
                break
            if lastx + partWidth <= machineWidth and floor + partLength <= ceil:
                shelf[0] += partWidth
                break
        
        else:
            ceil = shelves[-1][2]
            if ceil + partLength > machineLength:
                if ceil + partWidth > machineLength:
                    return False
                shelves.append([partLength, ceil, ceil + partWidth])
            shelves.append([partWidth, ceil, ceil + partLength])
    
    return True

