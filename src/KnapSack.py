from ortools.sat.python import cp_model
import collections

class KnapSack2D:

    def __init__(self):
        self.xStarts = {}
        self.xEnds = {}
        self.Positions = {}

        self.Intervals = collections.defaultdict(list)
        self.PartWidths = {}
        self.PartLengths = {}

        self.VariantUsed = {}
        self.PartInSolution = {}

        self.model = cp_model.CpModel()
        self.solver = cp_model.CpSolver()
        
    def Solve(self, parts, currentPartToFix, profits, machine, timeLimit = 1.0):
        position = collections.namedtuple('position', 'xStart xEnd yStart yEnd')

        currentPartSet = [part for part in parts]

        # for part in currentPartToFix:
        #     currentPartSet.append(part)

        for i, part in enumerate(currentPartSet):
 
            for variant in part.Variants:
                tmpIntervalName = '(' + str(i) + ',' + str(variant.VariantId) + ')'
                self.VariantUsed[(i, variant.VariantId)] = self.model.NewBoolVar('VariantUsed' + tmpIntervalName)
        
        for i, part in enumerate(currentPartSet):
            tmpIntervalName = str(i)

            binVariable = self.model.NewBoolVar('binVar(' + tmpIntervalName + ')')
            if part in currentPartToFix:
                xStart = self.model.NewIntVar(0, machine.Width - min(variant.Width for variant in part.Variants), 'xStart(' + tmpIntervalName + ')')
                xEnd = self.model.NewIntVar(min(variant.Width for variant in part.Variants), machine.Width, 'xnEnd(' + tmpIntervalName + ')')

                
            else:
                machineStart = (i + 1) * machine.Width
                xStart = self.model.NewIntVarFromDomain(cp_model.Domain.FromIntervals([[0, machine.Width - min(variant.Width for variant in part.Variants)], [machineStart, machineStart]]), 'xStart(' + tmpIntervalName + ')')
                xEnd = self.model.NewIntVarFromDomain(cp_model.Domain.FromIntervals([[min(variant.Width for variant in part.Variants), machine.Width], [machineStart + min(variant.Width for variant in part.Variants), machineStart + max(variant.Width for variant in part.Variants)]]), 'xEnd(' + tmpIntervalName + ')')
            
            yStart = self.model.NewIntVar(0, machine.Length - min(variant.Length for variant in part.Variants), 'y_start(' + tmpIntervalName + ')')
            yEnd = self.model.NewIntVar(min(variant.Length for variant in part.Variants), machine.Length, 'y_end(' + tmpIntervalName + ')')
            
            partWidth = self.model.NewIntVarFromDomain(cp_model.Domain.FromIntervals([[min([variant.Width for variant in part.Variants])], [max([variant.Width for variant in part.Variants])]]), 'Width(' + tmpIntervalName + ')')
            partLength = self.model.NewIntVarFromDomain(cp_model.Domain.FromIntervals([[min([variant.Length for variant in part.Variants])], [max([variant.Length for variant in part.Variants])]]), 'Length(' + tmpIntervalName + ')')

            xInterval = self.model.NewIntervalVar(xStart, partWidth, xEnd, 'xInterval(' + tmpIntervalName + ')')
            yInterval = self.model.NewIntervalVar(yStart, partLength, yEnd, 'yInterval(' + tmpIntervalName + ')')

            self.Intervals['x'].append(xInterval)
            self.Intervals['y'].append(yInterval)

            self.xStarts[i] = xStart
            self.xEnds[i] = xEnd
            self.Positions[i] = position(xStart=xStart, xEnd=xEnd, yStart=yStart, yEnd=yEnd)
            self.PartLengths[i] = partLength
            self.PartWidths[i] = partWidth
            self.PartInSolution[i] = binVariable
        
        for i, part in enumerate(currentPartSet):
            for variant in part.Variants:
                self.model.Add(self.PartWidths[i] == variant.Width).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)])
                self.model.Add(self.PartLengths[i] == variant.Length).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)])
                self.model.Add(self.PartWidths[i] == variant.Length).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)].Not())
                self.model.Add(self.PartLengths[i] == variant.Width).OnlyEnforceIf(self.VariantUsed[(i,variant.VariantId)].Not())

        self.model.AddNoOverlap2D(self.Intervals['x'],self.Intervals['y'])
        
        for i, part in enumerate(currentPartSet):
            self.model.Add(sum(self.VariantUsed[(i, variant.VariantId)] for variant in part.Variants) == 1)

            if part in currentPartToFix:
                self.model.Add(self.PartInSolution[i] == 1)
        
        for i, part in enumerate(currentPartSet):
            self.model.Add(self.xEnds[i] <= machine.Width).OnlyEnforceIf(self.PartInSolution[i])
            self.model.Add(self.xEnds[i] >= machine.Width + min(variant.Width for variant in part.Variants)).OnlyEnforceIf(self.PartInSolution[i].Not())
            
        
        self.model.Maximize(sum(self.PartInSolution[i] * int(profits[part.PartId]) for i, part in enumerate(currentPartSet)))


        self.solver.parameters.log_search_progress = False
        self.solver.parameters.max_time_in_seconds = 1
        self.solver.parameters.num_search_workers = 6
        self.solver.Solve(self.model)
        return self.solver.StatusName(), self.solver.BestObjectiveBound()