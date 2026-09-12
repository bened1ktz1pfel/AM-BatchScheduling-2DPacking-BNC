import math
import time
import copy

from OrthogonalPacker import *
from OneDimContBinPacker import *
from BarRelaxation import *
from OrthogonalRelaxation import *
from KnapSack import *
from LowerBounds import *
from Preprocess import *
# from BranchAndBound import *
from OrthogonalPackerCplex import *



class ValidationCallback:
    def __init__(self):
        pass

    @staticmethod
    def AddCut(model, assignedParts, machine, label = ""):
        cuts = 0    
        tmpList = [machine.MachineId]
        for secondMachine in model._Machines:
            if secondMachine.MachineId == machine.MachineId:
                continue
            if (secondMachine.Length <= machine.Length and secondMachine.Width <= machine.Width) or (secondMachine.Width <= machine.Length and secondMachine.Length <= machine.Width):
                tmpList.append(secondMachine.MachineId)
        
        for m in tmpList:
            model._TotalCuts += 1
            cuts += 1
            for b in range(model._Builds):
                model.cbLazy(sum(model._XAssignments[part.PartId][b][m] for part in assignedParts) <= (len(assignedParts) - 1)*model._YBatches[b][m])

        if label == "LB":
            model._LBConstraints += cuts
        elif label == "OR":
            model._ORConstraints += cuts
        elif label == "BR":
            model._BRConstraints += cuts
        elif label == "BP":
            model._BPConstraints += cuts
        else:
            pass
        return

    @staticmethod
    def DetermineFirstLowerBound(parts, machine):
        
        LB1 = BinPackingBound(parts, machine.Width, machine.Length).GetLowerBound()

        return (LB1  - 1 > 1e-5)
    
    @staticmethod
    def DetermineSecondLowerBound(parts, machine):

        relaxedPacking = RelaxedPacker()
        relaxedPacking.AddItems([(item.Variants[0].Width, item.Variants[0].Length) for item in parts])
        relaxedPacking.AddBin(machine.Width, machine.Length)
        relaxedPacking.CreateConservativeScales([0.01, 0.1, 0.2, 0.3, 0.4, 0.49], [0.01, 0.1, 0.2, 0.3, 0.4, 0.49])
        relaxedPacking.CreateConservativeScales2()
        relaxedPacking.DetermineConservativeScaledAreas()

        LB2 = relaxedPacking.GetLowerBound()

        return (LB2  - 1 > 1e-5)

    @staticmethod
    def CreateInitialPatterns(parts):
        initPatterns = []
        size = len(parts)
        for i in range(size):
            pattern = [0]*size
            pattern[i] = 1
            initPatterns.append(pattern)
        return initPatterns
        
    @staticmethod
    def ExtendParts(parts):
        return [{"width": part.Variants[0].Width, "length": part.Variants[0].Length} for part in parts] + [{"width": part.Variants[-1].Width, "length": part.Variants[-1].Length} for part in parts]

    @staticmethod
    def BarRelaxationSolve(model, parts, machine):

        tmpParts = ValidationCallback.ExtendParts(parts)
        tmpMachine = copy.deepcopy(machine)

        initialPatterns = ValidationCallback.CreateInitialPatterns(tmpParts)

        barRelax = BarRelaxationVariant()
        barRelax.AddParts(tmpParts)
        barRelax.AddPattern(initialPatterns)
        barRelax.AddMachine(tmpMachine)
        
        WidthBound = np.round(barRelax.Solve(model._Seed), 5)

        if math.ceil(WidthBound) > tmpMachine.Width:
            return False

        
        if math.ceil(WidthBound) <= tmpMachine.Width:
            return True
        
    @staticmethod
    def DetermineMinimalAreaReduction(parts, machine):
        return math.ceil(sum([part.Area for part in parts]) / machine.Area)

    @staticmethod
    def FindAssignments(model):
        assignmentArray = []

        for b in range(model._Builds):
            for k, machine in enumerate(model._Machines):
                assignedParts = [part for i, part in enumerate(model._Parts) if model.cbGetSolution(model._XAssignments[i][b][k]) > 0.5]

                if len(assignedParts) > 0:
                    assignmentArray.append((assignedParts, machine))
        
        # Sort Assignments based on area consumption?!
        assignmentArray.sort(key = lambda tup: -sum([part.Area for part in tup[0]])/tup[1].Area)
        return assignmentArray
    
    @staticmethod
    def UKnapSack2DSolve(liftedAssignment, currentPart, machine, profits, model):
        currentPartToFix = [currentPart]
        parts = [currentPart]
        for part in liftedAssignment:
            parts.append(part)


        knapSack = KnapSack2D()
        status, upperbound = knapSack.Solve(parts, currentPartToFix, profits, machine, 1.0)

        if status == 'INFEASIBLE' or status == 'INVALID':
            raise ValueError("Something went wrong in Knapsack")
        elif status == 'UNKNOWN' or status == "FEASIBLE":
            return len(liftedAssignment) - 1
        
        return upperbound

    @staticmethod
    def AddLiftedCoverInequality(model, assignedParts, machine, additionalParts, liftingCoefficients, label = ""):
        cuts = 0    
        tmpList = [machine.MachineId]
        for secondMachine in model._Machines:
            if secondMachine.MachineId == machine.MachineId:
                continue
            if (secondMachine.Length <= machine.Length and secondMachine.Width <= machine.Width) or (secondMachine.Width <= machine.Length and secondMachine.Length <= machine.Width):
                tmpList.append(secondMachine.MachineId)
        
        for m in tmpList:
            model._TotalCuts += 1
            cuts += 1
            for b in range(model._Builds):
                expr = gp.LinExpr()
                for part in assignedParts:
                    expr += model._XAssignments[part.PartId][b][machine.MachineId]
                
                for part in additionalParts:
                    expr += liftingCoefficients[part.PartId] * model._XAssignments[part.PartId][b][machine.MachineId]
                
                model.cbLazy(expr <= len(assignedParts) - 1)

        if label == "LB":
            model._LBConstraints += cuts
        elif label == "OR":
            model._ORConstraints += cuts
        elif label == "BR":
            model._BRConstraints += cuts
        elif label == "BP":
            model._BPConstraints += cuts
        else:
            pass
        return

    @staticmethod
    def AddLiftingCut(model, assignedParts, machine, label):
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

        ValidationCallback.AddLiftedCoverInequality(model, assignedParts, machine, additionalParts, liftingCoefficients, label)

    @staticmethod
    def AddStrengthendCut(model, assignedParts, machine, label):
        sortedAssignedParts = sorted(assignedParts, key = lambda part: part.Area)
        
        isFeasible = False

        while not isFeasible:

            currentAssignment = sortedAssignedParts[1:]

            if machine.MachineId in model._FeasibleAssignments.keys():
                if frozenset(currentAssignment) in model._FeasibleAssignments[machine.MachineId]:
                    if model._EnableCutLifting:
                        ValidationCallback.AddLiftingCut(model, sortedAssignedParts, machine, label)
                    else:
                        ValidationCallback.AddCut(model, sortedAssignedParts, machine, label)
                    return
            
            binPacking2D = BinPacking2DStrengthened()
            binPacking2D.AddParts(currentAssignment)
            binPacking2D.AddBin(machine)

            binPacking2D.CreateVariables(model._PlacementStrategy)
            binPacking2D.CreateConstraints()

            isFeasible = binPacking2D.Solve(model, 8)

            if isFeasible:
                if machine.MachineId in model._FeasibleAssignments.keys():
                    model._FeasibleAssignments[machine.MachineId].append(frozenset(currentAssignment))
                else:
                    model._FeasibleAssignments[machine.MachineId] = [frozenset(currentAssignment)]
                
                if len(currentAssignment) != 1:
                    model._PlacementPoints[(machine.MachineId, frozenset([item.PartId for item in currentAssignment]))] = binPacking2D.ExtractSolution()
                else:
                    tmpLength, tmpWidth = max(currentAssignment[0].Variants[0].Length, currentAssignment[0].Variants[0].Width), min(currentAssignment[0].Variants[0].Length, currentAssignment[0].Variants[0].Width)
                    model._PlacementPoints[(machine.MachineId, frozenset([item.PartId for item in currentAssignment]))] = ([0], [0], [tmpLength], [tmpWidth])
            
                if model._EnableCutLifting:
                    ValidationCallback.AddLiftingCut(model, sortedAssignedParts, machine, label)
                else:
                    ValidationCallback.AddCut(model, sortedAssignedParts, machine, label)
                return

            else:
                model._ProvenInfeasibleSets[machine.MachineId].append(frozenset(currentAssignment))
                sortedAssignedParts = [part for part in sortedAssignedParts[1:]]
            
            del binPacking2D
      
    @staticmethod
    def AddCuts(model, assignmentArray, timelimitReached = False):

        if timelimitReached:

            for assignment in assignmentArray:

                assignedParts, machine = assignment

                if machine.MachineId in model._FeasibleAssignments.keys():
                    if frozenset(assignedParts) in model._FeasibleAssignments[machine.MachineId]:
                        continue
                
                ValidationCallback.AddCut(model, assignedParts, machine)
                break
        else:

            for assignment in assignmentArray:
                
                model._BatchChecks += 1
                starttime = time.time()

                assignedParts, machine = assignment

                preprocess = CallbackPreprocess(assignedParts,  machine)
                
                if model._EnablePreprocessing:
                    preprocess.Run()

                tmpDict = {"Check No.": model._BatchChecks, "Obj": model.cbGet(GRB.Callback.MIPSOL_OBJ), 
                                "PartIds": [part.PartId for part in assignedParts], "MachineId": machine.MachineId, 
                                "MachineShrinked": preprocess.IsMachineShrinked, "LengthReduction": preprocess.MachineLengthShrinkRate, 
                                "WidthReduction": preprocess.MachineWidthShrinkRate, "ItemsIncreased": preprocess.ItemsIncreased}#, 
                                # "ItemIncrease": preprocess.IncreasedItems}

                if len(preprocess.PreprocessedParts) == 0:
                    tmpDict["Status"] = "OPTIMAL"
                    if machine.MachineId in model._FeasibleAssignments.keys():
                        model._FeasibleAssignments[machine.MachineId].append(frozenset(assignedParts))
                    else:
                        model._FeasibleAssignments[machine.MachineId] = [frozenset(assignedParts)]
                    
                    if len(assignedParts) != 1:
                        model._PlacementPoints[(machine.MachineId, frozenset([item.PartId for item in assignedParts]))] = (['Preprocessed'], ['Preprocessed'], ['Preprocessed'], ['Preprocessed'])
                    else:
                        tmpLength, tmpWidth = max(assignedParts[0].Variants[0].Length, assignedParts[0].Variants[0].Width), min(assignedParts[0].Variants[0].Length, assignedParts[0].Variants[0].Width)
                        model._PlacementPoints[(machine.MachineId, frozenset([item.PartId for item in assignedParts]))] = ([0], [0], [tmpLength], [tmpWidth])
                    model._Checks.append(tmpDict)
                    continue

                if machine.MachineId in model._FeasibleAssignments.keys():
                    if frozenset(assignedParts) in model._FeasibleAssignments[machine.MachineId]:
                        tmpDict["Status"] = "OPTIMAL"
                        model._Checks.append(tmpDict)
                        continue
                
                timeStampEndPP = time.time()
                model._TimeConPP += timeStampEndPP - starttime
                tmpDict["PP Time"] = timeStampEndPP - starttime

                if model._EnablePackingRelaxations:

                    ### First Check Mechanism based on Lower Bound ###
                    isInfeasible = ValidationCallback.DetermineFirstLowerBound(preprocess.PreprocessedParts, preprocess.PreprocessedMachine)
                    
                    timeStampEndLB = time.time()
                    model._TimeConLB += timeStampEndLB - timeStampEndPP
                    tmpDict["LB Time"] = timeStampEndLB - timeStampEndPP

                    if isInfeasible:
                        model._ProvenInfeasibleSets[machine.MachineId].append(frozenset(assignedParts))
                        
                        if model._EnableCutStrengthening:
                            ValidationCallback.AddStrengthendCut(model, assignedParts, machine, "LB")
                            timeStampEndCS = time.time()
                            model._TimeConSC += timeStampEndCS - timeStampEndLB
                            tmpDict["CS Time"] = timeStampEndCS - timeStampEndLB

                        else:
                            ValidationCallback.AddCut(model, assignedParts, machine, "LB")

                        tmpDict["Status"] = "INFEASIBLE"
                        model._Checks.append(tmpDict)
                        break
                    
                    ### Second Check Mechanism based on DFF Lower Bound ###
                    isInfeasible = ValidationCallback.DetermineSecondLowerBound(preprocess.PreprocessedParts, preprocess.PreprocessedMachine)

                    timeStampEndOR = time.time()
                    model._TimeConOR += timeStampEndOR - timeStampEndLB
                    tmpDict["OR Time"] = timeStampEndOR - timeStampEndLB

                    if isInfeasible:
                        model._ProvenInfeasibleSets[machine.MachineId].append(frozenset(assignedParts))
                        
                        if model._EnableCutStrengthening:
                            ValidationCallback.AddStrengthendCut(model, assignedParts, machine, "OR")
                            timeStampEndCS = time.time()
                            model._TimeConSC += timeStampEndCS - timeStampEndLB
                            tmpDict["CS Time"] = timeStampEndCS - timeStampEndLB

                        else:
                            ValidationCallback.AddCut(model, assignedParts, machine, "OR")
                        
                        tmpDict["Status"] = "INFEASIBLE"
                        model._Checks.append(tmpDict)
                        break
                    
                    ### Third Check Mechanism based on Bar Relaxation ###
                    relaxationFeasible = ValidationCallback.BarRelaxationSolve(model, preprocess.PreprocessedParts, preprocess.PreprocessedMachine)

                    timeStampEndBR = time.time()
                    model._TimeConBR += timeStampEndBR - timeStampEndOR
                    tmpDict["BR Time"] = timeStampEndBR - timeStampEndOR

                    if not relaxationFeasible:
                        model._ProvenInfeasibleSets[machine.MachineId].append(frozenset(assignedParts))
                        
                        if model._EnableCutStrengthening:
                            ValidationCallback.AddStrengthendCut(model, assignedParts, machine, "BR")
                            timeStampEndCS = time.time()
                            model._TimeConSC += timeStampEndCS - timeStampEndLB
                            tmpDict["CS Time"] = timeStampEndCS - timeStampEndLB

                        else:
                            ValidationCallback.AddCut(model, assignedParts, machine, "BR")

                        tmpDict["Status"] = "INFEASIBLE"
                        model._Checks.append(tmpDict)
                        break
                
                timeStampCurrent = time.time()
                bnbFeasible = False
                enableExactPacking = True

                ### Fourth Check Mechanism for heuristically solve the packing problem ###
                if model._EnablePackingHeuristics:
                        if model._OrthogonalPackingMethod == OrthogonalPackingMethod.ORTOOLS:
                            binPacking2D = BinPacking2D()
                            binPacking2D.AddParts(preprocess.PreprocessedParts)
                            binPacking2D.AddBin(preprocess.PreprocessedMachine)

                            binPacking2D.CreateVariables(model._PlacementStrategy)
                            binPacking2D.CreateConstraints()

                            isFeasible = binPacking2D.HeuristicSolve(model, 8)
                        
                        elif model._OrthogonalPackingMethod == OrthogonalPackingMethod.CPLEX:
                            binPacking2D = CplexOrthogonalPacking2D()

                            binPacking2D.AddParts(preprocess.PreprocessedParts)
                            binPacking2D.AddBin(preprocess.PreprocessedMachine)

                            binPacking2D.CreateVariables(model._PlacementStrategy)
                            binPacking2D.CreateConstraints()

                            isFeasible = binPacking2D.HeuristicSolve(model, 8)

                        elif model._OrthogonalPackingMethod == OrthogonalPackingMethod.BNC:
                            binPacking2D = OneDimContBinPacking(model._Seed)

                            binPacking2D.AddParts(preprocess.PreprocessedParts)
                            binPacking2D.AddBoundaries(preprocess.PreprocessedMachine.Width, preprocess.PreprocessedMachine.Length)

                            binPacking2D.Preprocess()
                            binPacking2D.DetermineArcs()
                            binPacking2D.CreateVariables()
                            binPacking2D.CreateConstraints()

                            binPacking2D.SetCallbackData()
                            isFeasible = binPacking2D.HeuristicSolve(model, 8)
                        
                        timeStampEndPreOP = time.time()

                        model._TimeConBP += timeStampEndPreOP - timeStampCurrent
                        tmpDict["OP I Time"] = timeStampEndPreOP - timeStampCurrent

                        timeStampCurrent = timeStampEndPreOP

                        if isFeasible != None:
                            enableExactPacking = False
                            if len(preprocess.PreprocessedParts) > 1:
                                if model._OrthogonalPackingMethod == OrthogonalPackingMethod.ORTOOLS:
                                    tmpDict["Status"] = binPacking2D.solver.StatusName()
                                elif model._OrthogonalPackingMethod == OrthogonalPackingMethod.CPLEX:
                                    tmpDict["Status"] = binPacking2D.result.get_solve_status()
                                elif model._OrthogonalPackingMethod == OrthogonalPackingMethod.BNC:
                                    tmpDict["Status"] = binPacking2D.model.status
                            else:
                                if isFeasible:
                                    tmpDict["Status"] = "OPTIMAL"
                                else:
                                    tmpDict["Status"] = "INFEASIBLE"
                        else:
                            items = [Item(item.PartId, item.Variants[0].Width, item.Variants[0].Length) for item in preprocess.PreprocessedParts]

                            BnB = BranchAndBoundPacker(BranchingRule.GStairCase, SearchStrategy.DepthFirst, True)

                            BnB.AddContainerSize(preprocess.PreprocessedMachine.Width, preprocess.PreprocessedMachine.Length)

                            BnB.AddItems(items)

                            BnB.Initialize(model._StairCaseRestriction)

                            isFeasible, count = BnB.Pack(time.time(), min(model._HeuristicTimeLimit, model._TimeLimit - model.cbGet(GRB.Callback.RUNTIME)), "True")
                            
                            timeStampEndBnBOP = time.time()

                            model._TimeConBP += timeStampEndBnBOP - timeStampCurrent
                            tmpDict["OP II Time"] = timeStampEndBnBOP - timeStampCurrent

                            timeStampCurrent = timeStampEndBnBOP

                            if isFeasible != None:
                                if isFeasible:
                                    tmpDict["Status"] = "OPTIMAL"
                                    enableExactPacking = False
                                    bnbFeasible = True
                        
                ### Fifth Check Mechanism based on Exact Orthogonal Packing ###
                if enableExactPacking:
                    if model._OrthogonalPackingMethod == OrthogonalPackingMethod.ORTOOLS:
                        binPacking2D = BinPacking2D()
                        binPacking2D.AddParts(preprocess.PreprocessedParts)
                        binPacking2D.AddBin(preprocess.PreprocessedMachine)

                        binPacking2D.CreateVariables(model._PlacementStrategy)
                        binPacking2D.CreateConstraints()

                        isFeasible = binPacking2D.Solve(model, 8, max(0, model._TimeLimit - model.cbGet(GRB.Callback.RUNTIME)))
                        
                        timeStampEndOP = time.time()

                        model._TimeConBP += timeStampEndOP - timeStampCurrent
                        tmpDict["OP Time"] = timeStampEndOP - timeStampCurrent
                        
                        timeStampCurrent = timeStampEndOP

                        if len(preprocess.PreprocessedParts) > 1:
                            tmpDict["Status"] = binPacking2D.solver.StatusName()
                        else:
                            if isFeasible:
                                tmpDict["Status"] = "OPTIMAL"
                            else:
                                tmpDict["Status"] = "INFEASIBLE"
                
                    elif model._OrthogonalPackingMethod == OrthogonalPackingMethod.CPLEX:
                        binPacking2D = CplexOrthogonalPacking2D()

                        binPacking2D.AddParts(preprocess.PreprocessedParts)
                        binPacking2D.AddBin(preprocess.PreprocessedMachine)

                        binPacking2D.CreateVariables(model._PlacementStrategy)
                        binPacking2D.CreateConstraints()

                        isFeasible = binPacking2D.Solve(model, 8, max(0, model._TimeLimit - model.cbGet(GRB.Callback.RUNTIME)))

                        timeStampEndOP = time.time()

                        model._TimeConBP += timeStampEndOP - timeStampCurrent
                        tmpDict["OP Time"] = timeStampEndOP - timeStampCurrent
                        
                        timeStampCurrent = timeStampEndOP

                        if len(preprocess.PreprocessedParts) > 1:
                            tmpDict["Status"] = binPacking2D.result.get_solve_status()
                        else:
                            if isFeasible:
                                tmpDict["Status"] = "OPTIMAL"
                            else:
                                tmpDict["Status"] = "INFEASIBLE"
                    
                    elif model._OrthogonalPackingMethod == OrthogonalPackingMethod.BNC:
                        binPacking2D = OneDimContBinPacking(model._Seed)

                        binPacking2D.AddParts(preprocess.PreprocessedParts)
                        binPacking2D.AddBoundaries(preprocess.PreprocessedMachine.Width, preprocess.PreprocessedMachine.Length)

                        binPacking2D.Preprocess()
                        binPacking2D.DetermineArcs()
                        binPacking2D.CreateVariables()
                        binPacking2D.CreateConstraints()

                        binPacking2D.SetCallbackData()
                        isFeasible = binPacking2D.Solve(model, 8, max(0, model._TimeLimit - model.cbGet(GRB.Callback.RUNTIME)))

                        timeStampEndOP = time.time()

                        model._TimeConBP += timeStampEndOP - timeStampCurrent
                        tmpDict["OP Time"] = timeStampEndOP - timeStampCurrent
                        
                        timeStampCurrent = timeStampEndOP

                        if len(preprocess.PreprocessedParts) > 1:
                            tmpDict["Status"] = binPacking2D.model.status
                        else:
                            if isFeasible:
                                tmpDict["Status"] = "OPTIMAL"
                            else:
                                tmpDict["Status"] = "INFEASIBLE"
                    else:
                        raise Exception("Unknown Orthogonal Packing Method")
                
                if isFeasible:
                    if machine.MachineId in model._FeasibleAssignments.keys():
                        model._FeasibleAssignments[machine.MachineId].append(frozenset(assignedParts))
                    else:
                        model._FeasibleAssignments[machine.MachineId] = [frozenset(assignedParts)]
                    
                    if len(preprocess.PreprocessedParts) != 1:
                        if enableExactPacking:
                            model._PlacementPoints[(machine.MachineId, frozenset([item.PartId for item in assignedParts]))] = binPacking2D.ExtractSolution()
                        elif not enableExactPacking and not bnbFeasible:
                            model._PlacementPoints[(machine.MachineId, frozenset([item.PartId for item in assignedParts]))] = binPacking2D.ExtractSolution()
                        else:
                            model._PlacementPoints[(machine.MachineId, frozenset([item.PartId for item in assignedParts]))] = (['TODO'], ['TODO'], ['TODO'], ['TODO'])
                    else:
                        tmpLength, tmpWidth = max(preprocess.PreprocessedParts[0].Variants[0].Length, preprocess.PreprocessedParts[0].Variants[0].Width), min(preprocess.PreprocessedParts[0].Variants[0].Length, preprocess.PreprocessedParts[0].Variants[0].Width)
                        model._PlacementPoints[(machine.MachineId, frozenset([item.PartId for item in assignedParts]))] = ([0], [0], [tmpLength], [tmpWidth])
                    
                    model._Checks.append(tmpDict)
                
                else:
                    try:
                        if model._OrthogonalPackingMethod == OrthogonalPackingMethod.CPLEX:
                            isUnknown = (binPacking2D.result.get_solve_status() == "Unknown")
                        else:
                            isUnknown = (binPacking2D.solver.StatusName() == "UNKNOWN")
                        
                        if isUnknown:
                            ValidationCallback.AddCut(model, assignedParts, machine)
                            model._Checks.append(tmpDict)
                            break
                    except:
                        pass

                    model._ProvenInfeasibleSets[machine.MachineId].append(frozenset(assignedParts))

                    if model._EnableCutStrengthening:
                        ValidationCallback.AddStrengthendCut(model, assignedParts, machine, "BP")
                        timeStampEndCS = time.time()
                        model._TimeConSC += timeStampEndCS - timeStampCurrent
                        tmpDict["CS Time"] = timeStampEndCS - timeStampCurrent
                        model._Checks.append(tmpDict)
                        break
                    else:
                        ValidationCallback.AddCut(model, assignedParts, machine, "BP")
                        model._Checks.append(tmpDict)
                        break
          
    @staticmethod
    def Callback(model, where):
        if model._IsPreSolve and where == GRB.Callback.MIPNODE:

            obj = model.cbGet(GRB.Callback.MIPNODE_OBJBST)
            if abs(obj - model._CurrentObj) > 1e-5:
                model._CurrentObj = obj
                model._CurrentTime = time.time()
        
            if time.time() - model._CurrentTime > model._EarlyStop:
                model.terminate()

        if where == GRB.Callback.MIPSOL:
            timelimitReached = False
            if model.cbGet(GRB.Callback.RUNTIME) > model._TimeLimit:
                timelimitReached = True
                model.terminate()

            assignmentArray = ValidationCallback.FindAssignments(model)
            currentTime = model.cbGet(GRB.Callback.RUNTIME)
            
            model._SolutionChecks += 1
            ValidationCallback.AddCuts(model, assignmentArray, timelimitReached)