import copy
from collections import namedtuple

from OrthogonalPacker import *
from Helper import *

class CallbackPreprocess:
    def __init__(self, parts, machine):
        self.Parts = parts
        self.Machine = machine

        self.PreprocessedParts = parts
        self.PreprocessedMachine = machine

        self.IsMachineShrinked = False
        self.ItemsIncreased = False
        self.IsInFeasible = False

        self.MachineWidthShrinkRate = 0
        self.MachineLengthShrinkRate = 0
        self.IncreasedItems = {}
    
    def ShrinkMachineSizes(self, parts, machine):

        items = TransformItems(parts, machine.Width)
        if items is None:
            self.IsInFeasible = True
            return 

        subsetsum = SubsetSumWithRotation(items, machine.Width)
        machine.Width = subsetsum.solve()

        items = TransformItems(parts, machine.Length)
        if items is None:
            self.IsInFeasible = True
            return

        subsetsum = SubsetSumWithRotation(items, machine.Length)
        machine.Length = subsetsum.solve()
    
    def IncreaseItemSizes(self, partList, machine):
        for i, fullItem in enumerate(partList):
            redItem = (fullItem.Variants[0].Width, fullItem.Variants[0].Length)
            tmpItems = [(part.Variants[0].Width, part.Variants[0].Length) for j, part in enumerate(partList) if j != i]

            if redItem[0] < machine.Length and redItem[0] < machine.Width:
                ssp = SubsetSumWithRotation(tmpItems, machine.Length - redItem[0])
                newLength = ssp.solve()

                ssp = SubsetSumWithRotation(tmpItems, machine.Width - redItem[0])
                newWidth = ssp.solve()

                bestIncrease = min(machine.Length - newLength, machine.Width - newWidth)

                if redItem[0] < bestIncrease:
                    self.ItemsIncreased = True
                    fullItem.Variants[0].Width = bestIncrease
                    if len(fullItem.Variants) > 1:
                        fullItem.Variants[1].Length = bestIncrease
                    # print(f"Increase {fullItem.PartId}")
            
            elif redItem[0] < machine.Length:
                ssp = SubsetSumWithRotation(tmpItems, machine.Length - redItem[0])
                newLength = ssp.solve()

                bestIncrease = min(machine.Length - newLength, machine.Width)

                if redItem[0] < bestIncrease:
                    self.ItemsIncreased = True
                    fullItem.Variants[0].Width = bestIncrease
                    if len(fullItem.Variants) > 1:
                        fullItem.Variants[1].Length = bestIncrease
                    # print(f"Increase {fullItem.PartId}")
            
            elif redItem[0] < machine.Width:
                ssp = SubsetSumWithRotation(tmpItems, machine.Width - redItem[0])
                newWidth = ssp.solve()

                bestIncrease = min(machine.Width - newWidth, machine.Length)

                if redItem[0] < bestIncrease:
                    self.ItemsIncreased = True
                    fullItem.Variants[0].Width = bestIncrease
                    if len(fullItem.Variants) > 1:
                        fullItem.Variants[1].Length = bestIncrease
                    # print(f"Increase {fullItem.PartId}")
            
            if redItem[1] < machine.Length and redItem[1] < machine.Width:
                ssp = SubsetSumWithRotation(tmpItems, machine.Length - redItem[1])
                newLength = ssp.solve()

                ssp = SubsetSumWithRotation(tmpItems, machine.Width - redItem[1])
                newWidth = ssp.solve()

                bestIncrease = min(machine.Length - newLength, machine.Width - newWidth)

                if redItem[1] < bestIncrease:
                    self.ItemsIncreased = True
                    if len(fullItem.Variants) > 1:
                        fullItem.Variants[1].Width = bestIncrease
                    fullItem.Variants[0].Length = bestIncrease
                    # print(f"Increase {fullItem.PartId}")
            
            elif redItem[1] < machine.Length:
                ssp = SubsetSumWithRotation(tmpItems, machine.Length - redItem[1])
                newLength = ssp.solve()

                bestIncrease = min(machine.Length - newLength, machine.Width)

                if redItem[1] < bestIncrease:
                    self.ItemsIncreased = True
                    if len(fullItem.Variants) > 1:
                        fullItem.Variants[1].Width = bestIncrease
                    fullItem.Variants[0].Length = bestIncrease
                    # print(f"Increase {fullItem.PartId}")
            
            elif redItem[1] < machine.Width:
                ssp = SubsetSumWithRotation(tmpItems, machine.Width - redItem[1])
                newWidth = ssp.solve()

                bestIncrease = min(machine.Width - newWidth, machine.Length)

                if redItem[1] < bestIncrease:
                    self.ItemsIncreased = True
                    if len(fullItem.Variants) > 1:
                        fullItem.Variants[1].Width = bestIncrease
                    fullItem.Variants[0].Length = bestIncrease
                    # print(f"Increase {fullItem.PartId}")

    def RemoveLargeItems(self, parts, machine):
        itemRemoved = True
        referenceWidth = machine.Width
        while itemRemoved:
            itemRemoved = False
            itemsToRemove = []
            for i, item in enumerate(parts):
                for variant in item.Variants:
                    if variant.Width == referenceWidth:
                        machine.Length -= variant.Length

                        itemsToRemove.append(item)
                        itemRemoved = True

                        break

                    if variant.Length == referenceWidth:
                        machine.Length -= variant.Width

                        itemsToRemove.append(item)
                        itemRemoved = True

                        break
                
                if itemRemoved:
                    break

            for item in itemsToRemove:
                parts.remove(item)
    
    def CombineAndRemoveSquares(self, parts, machine):
        squares = [item for item in parts if len(item.Variants) == 1]
        referenceWidth = machine.Width      
        
        itemsCombined = True
        while itemsCombined:
            itemsCombined = False
            itemsToRemove = []

            for stripHeight in reversed(list(set([item.Variants[0].Width for item in squares]))):
                newKnapSack = Knapsack2D()
                status, upperBound, objective = newKnapSack.Solve(squares, stripHeight, referenceWidth)
                
                if status == 'OPTIMAL':
                    isItemUsed = newKnapSack.ExtractSolution()
                    itemsToRemove = [item for i, item in enumerate(squares) if isItemUsed[i]]
                    itemsCombined = True
                    break
            
            for item in itemsToRemove:
                parts.remove(item)
                squares.remove(item)
            
            if itemsCombined:
                machine.Length -= stripHeight

    def Run(self):
        newParts = copy.deepcopy(self.Parts)
        newMachine = copy.deepcopy(self.Machine)

        if newMachine.Width == newMachine.Length:
            #self.CombineAndRemoveSquares(newParts, newMachine)
            self.RemoveLargeItems(newParts, newMachine)

        self.ShrinkMachineSizes(newParts, newMachine)
        self.IncreaseItemSizes(newParts, newMachine)

        if not self.IsInFeasible:
            self.PreprocessedParts = newParts
            self.PreprocessedMachine = newMachine

            self.IsMachineShrinked = self.Machine.Width != self.PreprocessedMachine.Width or self.Machine.Length != self.PreprocessedMachine.Length

            if self.IsMachineShrinked:
                self.MachineLengthShrinkRate = (self.Machine.Length - self.PreprocessedMachine.Length)/ self.Machine.Length
                self.MachineWidthShrinkRate = (self.Machine.Width - self.PreprocessedMachine.Width)/ self.Machine.Width
            
            if self.ItemsIncreased:
                self.IncreasedItems = {part.PartId: ((part.Variants[0].Width - self.Parts[p].Variants[0].Width)/ self.Parts[p].Variants[0].Width, 
                (part.Variants[0].Length - self.Parts[p].Variants[0].Length)/ self.Parts[p].Variants[0].Length) for p, part in enumerate(self.PreprocessedParts)}
            


class SchedulingPreprocess:
    def __init__(self, data, enableSizeTranformations = False):
        self.Data = data
        self.EnableSizeTransformations = enableSizeTranformations

        self.IsCompleted = False

        self.InfeasibleAssignments = []
        self.IncompatibleParts = {}

        self.MachinesShrinked = False
        self.MachineLengthShrinkRates = {}
        self.MachineWidthShrinkRates = {}

    def DetermineIncompatibleParts(self):

        for k, machine in enumerate(self.Data.Machines):
            self.IncompatibleParts[k] = set()

            for i, firstPart in enumerate(self.Data.Parts):
                if (i, k) in self.InfeasibleAssignments:
                    continue
                for j in range(i + 1, len(self.Data.Parts)):
                    secondPart = self.Data.Parts[j]
                    if (j, k) in self.InfeasibleAssignments:
                        continue
                    comparison = []
                    for firstVariant in firstPart.Variants:
                        for secondVariant in secondPart.Variants:

                            comparison.append((firstVariant.Width + secondVariant.Width <= machine.Width and max(firstVariant.Length, secondVariant.Length) <= machine.Length) 
                            or (max(firstVariant.Width, secondVariant.Width) <= machine.Width and firstVariant.Length + secondVariant.Length <= machine.Length))
                    
                    isFeasible = any(comparison)
                    
                    if not isFeasible:
                        if frozenset((i, j)) not in self.IncompatibleParts[k]:
                            self.IncompatibleParts[k].add(frozenset((i, j)))
                    
    def IsFeasible(self, part, machine):
        tmpCheckList = []
        for variant in part.Variants:
            tmpCheckList.append(variant.Width <= machine.Width and variant.Length <= machine.Length)
        
        return any(tmpCheckList) and (part.Height <= machine.Height)

    def DetermineInfeasibleAssignments(self):
        for i, part in enumerate(self.Data.Parts):
            for k, machine in enumerate(self.Data.Machines):
                if not self.IsFeasible(part, machine):
                    self.InfeasibleAssignments.append((i, k))

    def IncreaseItemSizes(self, partList, machine):
        # TODO: Implement from CallbackPreprocess and Check if it works
        # New Data Structure for Items required
        pass

    def TransformSizes(self):
        if not self.EnableSizeTransformations:
            return
        
        for m, machine in enumerate(self.Data.Machines):
            
            tmpItems = [part for i, part in enumerate(self.Data.Parts) if (i, m) not in self.InfeasibleAssignments]
            items = TransformItems(tmpItems, machine.Width)

            subsetsum = SubsetSumWithRotation(items, machine.Width)
            newWidth = subsetsum.solve()

            if newWidth < machine.Width:
                self.MachinesShrinked = True
                self.MachineWidthShrinkRates[m] = (machine.Width - newWidth)/machine.Width
                machine.Width = newWidth

            items = TransformItems(tmpItems, machine.Length)

            subsetsum = SubsetSumWithRotation(items, machine.Length)
            newLength = subsetsum.solve()

            if newLength < machine.Length:
                self.MachinesShrinked = True
                self.MachineLengthShrinkRates[m] = (machine.Length - newLength)/machine.Length
                machine.Length = newLength

            # This would require new item structures, since dimensions for each machineType need to be stored
            #self.IncreaseItemSizes(self.Data.Parts, machine)

    def Run(self):
        if self.IsCompleted:
            return
          
        self.DetermineInfeasibleAssignments()
        self.DetermineIncompatibleParts()
        self.TransformSizes()

        self.IsCompleted = True


class StrippackingPreprocess:
    def __init__(self, items, container, enableItemEnlargement = False):
        self.Items = items
        self.Container = container

        self.PreprocessedItems = items
        self.PreprocessedContainer = container

        self.EnableItemEnlargement = enableItemEnlargement

        self.IsCompleted = False

        self.InfeasibleAssignments = []
        self.IncompatibleParts = {}

        self.MachinesShrinked = False
        self.MachineLengthShrinkRates = {}
        self.MachineWidthShrinkRates = {}

    def RemoveLargeItems(self, items, container):
        itemRemoved = True
        referenceWidth = container.Width
        adjustedLength = container.Length
        while itemRemoved:
            itemRemoved = False
            itemsToRemove = []
            for i, item in enumerate(items[:int(len(items)/2)]):
                
                if item.Width == referenceWidth:
                    adjustedLength -= item.Length

                    itemsToRemove.append(item)
                    itemsToRemove.append(items[int(len(items)/2) + i])
                    itemRemoved = True

                    break

                if item.Length == referenceWidth:
                    adjustedLength -= item.Width

                    itemsToRemove.append(item)
                    itemsToRemove.append(items[int(len(items)/2) + i])
                    itemRemoved = True

                    break
                
                if itemRemoved:
                    break

            for item in itemsToRemove:
                items.remove(item)
        
        container.Length = adjustedLength

    def ShrinkMachineSizes(self, items, container):

        widths = [item.Width for item in items]
        
        maxSize  = max(container.Width, container.Length)
        subsetsum = SubsetSum(widths, maxSize)
        subsetsum.solve()

        containerWidth = subsetsum.MaximumRealizable(container.Width)
        containerLength = subsetsum.MaximumRealizable(container.Length)

        container.Width = containerWidth
        container.Length = containerLength
    
    def IncreaseItemSizes(self, items, container):
        if not self.EnableItemEnlargement:
            return
        
        for i, fullItem in enumerate(items):
            itemWidth = fullItem.Width
            widths = [item.Width for j, item in enumerate(items) if j != i]

            if itemWidth < container.Width:
                ssp = SubsetSum(widths, container.Width - itemWidth)
                ssp.solve()

                newWidth = ssp.MaximumRealizable(container.Width - itemWidth)

                if itemWidth < container.Width - newWidth:
                    fullItem.Width = container.Width - newWidth
                    # print(f"Increase {fullItem.PartId}")
            
            itemLength = fullItem.Length
            lengths = [item.Length for j, item in enumerate(items) if j != i]

            if itemLength < container.Length:
                ssp = SubsetSum(lengths, container.Length - itemLength)
                ssp.solve()

                newLength = ssp.MaximumRealizable(container.Length - itemLength)

                if itemLength < container.Length - newLength:
                    fullItem.Length = container.Length - newLength
                    # print(f"Increase {fullItem.PartId}")

    
    def Run(self):
        newItems = copy.deepcopy(self.Items)
        newContainer = copy.deepcopy(self.Container)
        
        if self.Container.Width == self.Container.Length:
            self.RemoveLargeItems(newItems, newContainer)

        self.ShrinkMachineSizes(newItems, newContainer)
        self.IncreaseItemSizes(newItems, newContainer)

        
        self.PreprocessedItems = newItems
        self.PreprocessedContainer = newContainer




class SubsetSumWithRotation:

    def __init__(self, nums, m):
        self.nums = nums
        self.m = m
        self.dp = [[False for j in range(m + 1)] for i in range(len(nums) + 1)]
    
    def solve(self):
        for i in range(len(self.nums) + 1):
            self.dp[i][0] = True
        
        for i in range(1, len(self.nums) + 1):
            for j in range(1, self.m + 1):
                val1, val2 = self.nums[i - 1][0], self.nums[i - 1][1]
                if val1 <= j and val2 <= j:
                    self.dp[i][j] = self.dp[i - 1][j] or self.dp[i - 1][j - val1] or self.dp[i - 1][j - val2]
                    # self.dp[i][j] = self.dp[i - 1][j] or self.dp[i - 1][j - val2]
                elif val1 <= j and val2 > j:
                    self.dp[i][j] = self.dp[i - 1][j] or self.dp[i - 1][j - val1]
                elif val1 > j and val2 <= j:
                    self.dp[i][j] = self.dp[i - 1][j] or self.dp[i - 1][j - val2]
                else:
                    self.dp[i][j] = self.dp[i - 1][j]

        for i in range(self.m, -1, -1):
            if self.dp[len(self.nums)][i] == True:
                return i
            
class SubsetSum:
    def __init__(self, nums, m):
        self.nums = nums
        self.m = m
        self.dp = [[False for j in range(m + 1)] for i in range(len(nums) + 1)]
    
    def solve(self):
        for i in range(len(self.nums) + 1):
            self.dp[i][0] = True
        
        for i in range(1, len(self.nums) + 1):
            for j in range(1, self.m + 1):
                if self.nums[i - 1] <= j:
                    self.dp[i][j] = self.dp[i - 1][j] or self.dp[i - 1][j - self.nums[i - 1]]
                else:
                    self.dp[i][j] = self.dp[i - 1][j]

    def Check(self, val):
        return self.dp[len(self.nums)][val]
        
    def MaximumRealizable(self, val):
        for i in range(val, -1, -1):
            if self.dp[len(self.nums)][i] == True:
                return i

if __name__ == "__main__":
    # l = [(76, 50), (50, 50), (40, 40), (40, 40), (50, 50), (50, 50), (75, 40), (100, 140), (60, 50), (80, 80), (40, 75)]
    l = [(23, 28), (23, 26)]
    machine = (28, 49)

    for i, fullItem in enumerate(l):
            redItem = copy.deepcopy(fullItem)
            tmpItems = [p for j, p in enumerate(l) if j != i]

            if redItem[0] < machine[0] and redItem[0] < machine[1]:
                ssp = SubsetSumWithRotation(tmpItems, machine[0] - redItem[0])
                newLength = ssp.solve()

                ssp = SubsetSumWithRotation(tmpItems, machine[1] - redItem[0])
                newWidth = ssp.solve()

                bestIncrease = min(machine[0] - newLength, machine[1] - newWidth)

                if redItem[0] < bestIncrease:
                    print(f"Increase {fullItem}")
            
            elif redItem[0] < machine[0]:
                ssp = SubsetSumWithRotation(tmpItems, machine[0] - redItem[0])
                newLength = ssp.solve()

                bestIncrease = min(machine[0] - newLength,machine[1])

                if redItem[0] < bestIncrease:
                    print(f"Increase {fullItem}")
            
            elif redItem[0] < machine[1]:
                ssp = SubsetSumWithRotation(tmpItems, machine[1] - redItem[0])
                newWidth = ssp.solve()

                bestIncrease = min(machine[1] - newWidth, machine[0])

                if redItem[0] < bestIncrease:
                    print(f"Increase {fullItem}")
            
            if redItem[1] < machine[0] and redItem[1] < machine[1]:
                ssp = SubsetSumWithRotation(tmpItems, machine[0] - redItem[1])
                newLength = ssp.solve()

                ssp = SubsetSumWithRotation(tmpItems, machine[1] - redItem[1])
                newWidth = ssp.solve()

                bestIncrease = min(machine[0] - newLength, machine[1] - newWidth)

                if redItem[1] < bestIncrease:
                    print(f"Increase {fullItem}")
            
            elif redItem[1] < machine[0]:
                ssp = SubsetSumWithRotation(tmpItems, machine[0] - redItem[1])
                newLength = ssp.solve()

                bestIncrease = min(machine[0] - newLength, machine[1])

                if redItem[1] < bestIncrease:
                    print(f"Increase {fullItem}")
            
            elif redItem[1] < machine[1]:
                ssp = SubsetSumWithRotation(tmpItems, machine[1] - redItem[1])
                newWidth = ssp.solve()

                bestIncrease = min(machine[1] - newWidth, machine[0])

                if redItem[1] < bestIncrease:
                    print(f"Increase {fullItem}")
