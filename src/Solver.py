from DataManager import *
from Scheduler import *
from ConstructiveAlgorithms import *
from Preprocess import *
from Enums import *

import os
import time
import numpy as np


class SchedulingAndPackingBnCSolver:
    def __init__(self, outputPath, seed, preprocess = None):
        self.RNG = np.random.default_rng(seed)
        self.Seed = seed
        self.OutputPath = outputPath

        self.Preprocess = preprocess

        self.IncompatibleParts = {}
        self.InfeasibleAssignments = []

        self.SchedulingModel = MachineScheduling(outputPath, seed)
        self.PreSolveSchedulingModel = MachineScheduling(outputPath, seed)

        self.IsOptimal = False
        self.LowerBound = -1
        self.UpperBound = -1
        self.ModelRuntime = -1
        self.OverallRuntime = -1
        self.PreprocessTime = -1
        self.SolverType = ""
        self.CPTimeOuts = -1
        self.TOAssignments = -1
        self.LBCuts = -1
        self.BRCuts = -1
        self.BPCuts = -1
    
    def Initialize(self, solverParameters):
        self.Threads = solverParameters["Threads"]
        self.TimeLimit = solverParameters["TimeLimit"]
        self.EnablePreprocess = solverParameters["Preprocess"]
        self.EnableInitialSolution = solverParameters["InitialSolution"]
        self.EnableModelImprovements = solverParameters["ModelImprovements"]
        self.EnablePackingRelaxations = solverParameters["PackingRelaxations"]
        self.EnablePackingHeuristics = solverParameters["PackingHeuristics"]
        self.EnablePreSolve = solverParameters["PreSolve"]
        self.EnableCutLifting = solverParameters["CutLifting"]
        self.EnableCutStrengthening = solverParameters["CutStrengthening"]
        self.HeuristicTimeLimit = solverParameters["HeuristicTimeLimit"]
        self.StairCaseRestriction = solverParameters["StairCaseRestriction"]
        self.PreSolveTimeFraction = solverParameters["PreSolveTimeFraction"]
        self.PreSolveAreaRestriction = solverParameters["PreSolveAreaRestriction"]
        self.AssignmentStrategy = solverParameters["AssignmentStrategy"]
        self.SortingStrategies = solverParameters["SortingStrategies"]
        self.AllocationStrategies = solverParameters["AllocationStrategies"]
        self.LocalSearchIterations = solverParameters["LocalSearchIterations"]
        self.OrthongonalPackingMethod = solverParameters["OrthogonalPackingMethod"]

        self.EnablePreprocessOP = solverParameters["PreprocessOP"]
        self.MIPTolerance = solverParameters["MIPTolerance"]

    def RetrieveSolutionStatistics(self):
        
        model = self.SchedulingModel.model

        self.IsOptimal = 1 if model.Status == GRB.OPTIMAL else 0
        self.LowerBound = model.objBound
        self.UpperBound = model.objVal
        self.ModelRuntime = model.Runtime
        self.SolverType = "B&C"
        self.CPTimeOuts = model._BPTimeOuts
        self.LBCuts = model._LBConstraints
        self.ORCuts = model._ORConstraints
        self.BRCuts = model._BRConstraints
        self.BPCuts = model._BPConstraints
        self.TotalCuts = model._TotalCuts
        self.TOAssignments = model._TOAssignments

        self.FeasibleAssignments = model._FeasibleAssignments
        self.ProvenInfeasibleAssignments = model._ProvenInfeasibleSets

        self.TimeConPP = model._TimeConPP
        self.TimeConLB = model._TimeConLB
        self.TimeConOR = model._TimeConOR
        self.TimeConBR = model._TimeConBR
        self.TimeConBP = model._TimeConBP
        self.TimeConSC = model._TimeConSC
        self.CheckData = model._Checks

        self.ExploredAssignments = model._BatchChecks
        self.IntegerSolutions = model._SolutionChecks

        self.Placements = {machine.MachineId: [] for machine in model._Machines}

        if self.UpperBound < np.inf:
            for b in range(model._Builds):
                for m, machine in enumerate(model._Machines): 
                    if self.SchedulingModel.y[b][m].X > 0.5:
                        currentAssigment = [i for i, item in enumerate(self.SchedulingModel.Parts) if self.SchedulingModel.x[i][b][m].X > 0.5]
                        currentAssigment.sort()
                        if currentAssigment != []:
                            placements = model._PlacementPoints[(machine.MachineId, frozenset(currentAssigment))]

                            self.Placements[m].append((b, placements))

    def WriteSolutionStatistics(self, inputPath, fileName):
        solutions = {}
        currentWorkingDirectory = os.path.basename(inputPath.replace("\\", "/").rstrip("/"))

        if currentWorkingDirectory != "Test":
            mipSolutionPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'results', 'MIP', f'{currentWorkingDirectory}')
            fullmipPath = os.path.join(mipSolutionPath,f'{fileName}_Sol_21.json')

            with open(fullmipPath, "r") as mipFile:
                mipData = json.load(mipFile)

            key = [i for i in mipData.keys()][0]
            mipUB = float(np.round(mipData[key]["UB"], decimals = 3))
            mipLB = float(np.round(mipData[key]["LB"], decimals = 3))
        

            solutions[fileName + "_" +str(self.Seed)] = {"Optimality": self.IsOptimal, "MIPFeasible": float(np.round(self.UpperBound, decimals = 3)) >= mipLB, "UB": self.UpperBound, "LB": self.LowerBound, 
                                    "Model Runtime": self.ModelRuntime, "Overall Runtime": self.OverallRuntime, "Preprocess Runtime": self.PreprocessTime, 
                                    "MachinesShrinked": self.Preprocess.MachinesShrinked, "MachineLengthsReductions": self.Preprocess.MachineLengthShrinkRates,
                                    "MachineWidthReductions": self.Preprocess.MachineWidthShrinkRates, 
                                    "Time Consumption PP": self.TimeConPP,"Time Consumption LB": self.TimeConLB, "Time Consumption OR": self.TimeConOR, 
                                    "Time Consumption BR": self.TimeConBR, "Time Consumption SC": self.TimeConSC, "Time Consumption OP": self.TimeConBP, 
                                    "Checks": self.CheckData, "Constructive StartValue": self.MIPStartValue, "PreSolved StartValue": self.PreSolvedValue, 
                                    "Explored Solutions": self.IntegerSolutions, "Explored Assignments": self.ExploredAssignments, 
                                    "Feasible Assignments": sum([len(l) for l in self.FeasibleAssignments.values()]), 
                                    "Infeasible Assignments": sum([len(l) for l in self.ProvenInfeasibleAssignments.values()]), 
                                    "MaxBatchesPerMachine": self.SchedulingModel.Builds, "NumberBatchesInSolution": sum(len(value) for key, value in self.Placements.items()),
                                    "Total Cuts": self.TotalCuts, "LB Cuts": self.LBCuts, "OR Cuts": self.ORCuts, "BR Cuts": self.BRCuts, "BP Cuts": self.BPCuts, 
                                    "CP TimeOuts": self.CPTimeOuts, "CP Time Out Assignments": self.TOAssignments, "Placements": self.Placements}
        
        else:

            solutions[fileName + "_" +str(self.Seed)] = {"Optimality": self.IsOptimal, "UB": self.UpperBound, "LB": self.LowerBound, 
                                    "Model Runtime": self.ModelRuntime, "Overall Runtime": self.OverallRuntime, "Preprocess Runtime": self.PreprocessTime, 
                                    "MachinesShrinked": self.Preprocess.MachinesShrinked, "MachineLengthsReductions": self.Preprocess.MachineLengthShrinkRates,
                                    "MachineWidthReductions": self.Preprocess.MachineWidthShrinkRates, 
                                    "Time Consumption PP": self.TimeConPP,"Time Consumption LB": self.TimeConLB, "Time Consumption OR": self.TimeConOR, 
                                    "Time Consumption BR": self.TimeConBR, "Time Consumption SC": self.TimeConSC, "Time Consumption OP": self.TimeConBP, 
                                    "Checks": self.CheckData, "Constructive StartValue": self.MIPStartValue, "PreSolved StartValue": self.PreSolvedValue, 
                                    "Explored Solutions": self.IntegerSolutions, "Explored Assignments": self.ExploredAssignments, 
                                    "Feasible Assignments": sum([len(l) for l in self.FeasibleAssignments.values()]), 
                                    "Infeasible Assignments": sum([len(l) for l in self.ProvenInfeasibleAssignments.values()]), 
                                    "MaxBatchesPerMachine": self.SchedulingModel.Builds, "NumberBatchesInSolution": sum(len(value) for key, value in self.Placements.items()),
                                    "Total Cuts": self.TotalCuts, "LB Cuts": self.LBCuts, "OR Cuts": self.ORCuts, "BR Cuts": self.BRCuts, "BP Cuts": self.BPCuts, 
                                    "CP TimeOuts": self.CPTimeOuts, "CP Time Out Assignments": self.TOAssignments, "Placements": self.Placements}

        solutionsJson = json.dumps(solutions, indent= 4)

        with open(os.path.join(self.OutputPath, fileName + '_Sol_' + str(self.Seed) + '.json'), "w") as outFile:
            outFile.write(solutionsJson)

    def DetermineConstructiveStartSolution(self):
        constructiveHeuristic = ConstructiveHeuristic(self.RNG)
        if self.EnablePreSolve:
            constructiveHeuristic.InitializeAreaRestriction(self.PreSolveAreaRestriction)

        startSolution = constructiveHeuristic.Run(self.Data, self.SortingStrategies, self.AllocationStrategies, self.LocalSearchIterations)
        
        self.InitialPartAssignment, self.MIPStartValue = startSolution.PartAssignment, startSolution.Makespan

        print(f'Start Solution should be {self.MIPStartValue}')

    def RunSolver(self, data):
        self.Data = data
        start = time.time()

        ## Preprocessing
        self.Preprocess = SchedulingPreprocess(self.Data, self.EnablePreprocess)
        self.Preprocess.Run()
        self.PreprocessTime = float(np.round(time.time() - start, decimals=3))

        # Constructive Heuristic
        if self.EnableInitialSolution:
            self.DetermineConstructiveStartSolution()
            
        else:
            self.InitialPartAssignment, self.MIPStartValue = None, np.inf

        self.ConstructionTime = float(np.round(time.time() - self.PreprocessTime, decimals=3))

        # PreSolving the Model with enforced restrictions
        isPreSolveFeasible = False
        if self.EnablePreSolve:
        
            ## Initialize Pre-Solve
            self.PreSolveSchedulingModel.AddParts(self.Data.Parts)
            self.PreSolveSchedulingModel.AddMachines(self.Data.Machines)
            self.PreSolveSchedulingModel.DetermineMaximumBuilds(self.AssignmentStrategy)

            self.PreSolveSchedulingModel.CreateVariables()
            self.PreSolveSchedulingModel.CreateConstraints()

            self.PreSolveSchedulingModel.InitializePreprocessResults(self.Preprocess.IncompatibleParts, self.Preprocess.InfeasibleAssignments, self.InitialPartAssignment,
                                                                     self.EnablePreprocessOP, self.EnablePackingRelaxations, self.EnablePackingHeuristics, 
                                                                     self.EnableCutLifting, self.EnableCutStrengthening, self.OrthongonalPackingMethod, self.HeuristicTimeLimit, 
                                                                     self.StairCaseRestriction)

            self.PreSolveSchedulingModel.AddEnforcedAreaRestriction()
            self.PreSolveSchedulingModel.AddMIPStartSolution()

            if self.EnableModelImprovements:
                self.PreSolveSchedulingModel.AddSymmetryBreakingConstraints()

            preSolveTimeLimit = int(self.TimeLimit*self.PreSolveTimeFraction)
            isPreSolveFeasible = self.PreSolveSchedulingModel.PreSolve(self.Threads, preSolveTimeLimit, self.Data.ModelId)

        if isPreSolveFeasible:
            self.PresolvedSolution = self.PreSolveSchedulingModel.ExtractSolution()
            self.PreSolvedValue = self.PreSolveSchedulingModel.model.objVal
        else:
            self.PresolvedSolution = self.InitialPartAssignment
            self.PreSolvedValue = self.MIPStartValue

        self.PreSolveTime = float(np.round(time.time() - self.ConstructionTime, decimals=3))

        ## Initialize Main Model
        self.SchedulingModel.AddParts(self.Data.Parts)
        self.SchedulingModel.AddMachines(self.Data.Machines)
        self.SchedulingModel.DetermineMaximumBuilds(self.AssignmentStrategy)

        self.SchedulingModel.CreateVariables()
        self.SchedulingModel.CreateConstraints()

        self.SchedulingModel.InitializePreprocessResults(self.Preprocess.IncompatibleParts, self.Preprocess.InfeasibleAssignments, self.PresolvedSolution,
                                                                     self.EnablePreprocessOP, self.EnablePackingRelaxations, self.EnablePackingHeuristics, 
                                                                     self.EnableCutLifting, self.EnableCutStrengthening, self.OrthongonalPackingMethod, 
                                                                     self.HeuristicTimeLimit, self.StairCaseRestriction)

        self.SchedulingModel.AddMIPStartSolution()

        if self.EnableModelImprovements:
            self.SchedulingModel.AddAreaRestriction()
            self.SchedulingModel.AddSymmetryBreakingConstraints()

        ## Solve Main Model
        self.SchedulingModel.Solve(self.Threads, self.TimeLimit - self.PreSolveTime, self.MIPTolerance, self.Data.ModelId)

        self.OverallRuntime = float(np.round(time.time() - start, decimals=3))
        self.SolveTime = float(np.round(time.time() - self.PreSolveTime, decimals=3))


def main(inputPath, outputPath, dataScaling, solverParams):
    seeds = solverParams["Seeds"]
    enableVisualization = solverParams["Visualization"]

    for file in os.listdir(inputPath):
        # if not file.startswith("Instance_Class3"):
        #     continue
        if file.endswith(".json") and not file.endswith("Che.json"):
            for seed in seeds:
                
                fileName = file[:-5]

                if os.path.exists(os.path.join(outputPath, fileName + '_Sol_' + str(seed) + '.json')):
                    continue
                           
                data = DataModel(fileName, dataScaling)
                data.BuildData(os.path.join(inputPath, file))

                newSolver = SchedulingAndPackingBnCSolver(outputPath, int(seed))
                newSolver.Initialize(solverParams)
                newSolver.RunSolver(data)

                newSolver.RetrieveSolutionStatistics()
                newSolver.WriteSolutionStatistics(inputPath, fileName)

                if enableVisualization:
                    # TODO: Revise Visualization-Methods in Visualization.py
                    #newSolver.CreateGanttChart()
                    #newSolver.RetrievePackingPlots()
                    pass
                
                del data
                del newSolver

# Setup for full tests 
if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("Provide specification as follows: codepath, folder specs (e.g., n10m2), time limit (e.g., 3600), and decision about two step procedure (True/False)!")
        sys.exit()

    SOLVERVARIANTS = {
        1: {
            'Name': 'TestVariant',
            'ShortName': 'TV',
            'Preprocess': True,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': True,
            'PackingHeuristics': True,
            'PreSolve': True,
            'PreprocessOP': True,
            'CutStrengthening': True,
            'CutLifting': True,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.CPLEX,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [22]
        },
        2: {
            'Name': 'BaseVariant',
            'ShortName': 'BV',
            'Preprocess': False,
            'InitialSolution': True,
            'ModelImprovements': False,
            'PackingRelaxations': False,
            'PackingHeuristics': False,
            'PreSolve': False,
            'PreprocessOP': False,
            'CutStrengthening': False,
            'CutLifting': False,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        3: { 
            'Name': 'FullVariantORTOOLS',
            'ShortName': 'FORT',
            'Preprocess': True,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': True,
            'PackingHeuristics': True,
            'PreSolve': False,
            'PreprocessOP': True,
            'CutStrengthening': True,
            'CutLifting': True,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        6: {
            'Name': 'PreSolveVariantORTOOLS',
            'ShortName': 'PORT',
            'Preprocess': True,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': True,
            'PackingHeuristics': True,
            'PreSolve': True,
            'PreprocessOP': True,
            'CutStrengthening': True,
            'CutLifting': True,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        7: { 
            'Name': 'FullVariantORTOOLSNCL',
            'ShortName': 'FORTNCL',
            'Preprocess': True,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': True,
            'PackingHeuristics': True,
            'PreSolve': False,
            'PreprocessOP': True,
            'CutStrengthening': True,
            'CutLifting': False,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        8: {
            'Name': 'FinalVariantORTOOLS',
            'ShortName': 'FINORT',
            'Preprocess': True,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': True,
            'PackingHeuristics': False,
            'PreSolve': True,
            'PreprocessOP': True,
            'CutStrengthening': True,
            'CutLifting': True,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        9: {
            'Name': 'FinalVariantCPLEX',
            'ShortName': 'FINCP',
            'Preprocess': True,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': True,
            'PackingHeuristics': False,
            'PreSolve': True,
            'PreprocessOP': True,
            'CutStrengthening': True,
            'CutLifting': True,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.CPLEX,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        10: {
            'Name': 'FinalVariantBNC',
            'ShortName': 'FINBNC',
            'Preprocess': True,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': True,
            'PackingHeuristics': False,
            'PreSolve': True,
            'PreprocessOP': True,
            'CutStrengthening': True,
            'CutLifting': True,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.BNC,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        11: {
            'Name': 'BasePlusVariant',
            'ShortName': 'BPV',
            'Preprocess': False,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': False,
            'PackingHeuristics': False,
            'PreSolve': False,
            'PreprocessOP': False,
            'CutStrengthening': False,
            'CutLifting': False,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        12: {
            'Name': 'NoInitialSolutionVariant',
            'ShortName': 'NISV',
            'Preprocess': False,
            'InitialSolution': False,
            'ModelImprovements': False,
            'PackingRelaxations': False,
            'PackingHeuristics': False,
            'PreSolve': False,
            'PreprocessOP': False,
            'CutStrengthening': False,
            'CutLifting': False,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        13: {
            'Name': 'NoPackingHeuristicNoLiftingVariant',
            'ShortName': 'NPHNLV',
            'Preprocess': True,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': True,
            'PackingHeuristics': False,
            'PreSolve': False,
            'PreprocessOP': True,
            'CutStrengthening': True,
            'CutLifting': False,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
        14: {
            'Name': 'NoPackingHeuristicVariant',
            'ShortName': 'NPHV',
            'Preprocess': True,
            'InitialSolution': True,
            'ModelImprovements': True,
            'PackingRelaxations': True,
            'PackingHeuristics': False,
            'PreSolve': False,
            'PreprocessOP': True,
            'CutStrengthening': True,
            'CutLifting': True,
            'Visualization': False, # Change to True if visualization is desired
            'AssignmentStrategy': 'FirstFit',
            'SortingStrategies': [SortingStrategy.NoStrategy, SortingStrategy.Area, SortingStrategy.Height], # Item sorting in constructive heuristic
            'AllocationStrategies': [AllocationStrategy.LocalSearch, AllocationStrategy.AreaShare], # Allocation strategy in constructive heuristic
            'LocalSearchIterations': 100, # Only relevant if AllocationStrategy is LocalSearch
            'HeuristicTimeLimit': 30,
            'StairCaseRestriction': 3,
            'TimeLimit': 3600,
            'PreSolveTimeFraction': 0.1, # Only relevant if PreSolve is True
            'PreSolveAreaRestriction': 0.9, # Only relevant if PreSolve is True
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [11, 22, 33]
        },
    }

    baseDir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(baseDir, '..', 'data')
    baseOutputPath = os.path.join(baseDir, '..', 'results')

    specs = sys.argv[1]

    solverParams = SOLVERVARIANTS[int(sys.argv[2])]

    for folder in os.listdir(path):
        if folder.startswith(specs):
            if folder.startswith("Che"):
                dataScaling = True
            else:
                dataScaling = False

            inputPath = os.path.join(path, folder)

            outputFolderPath = os.path.join(baseOutputPath, f'BNC_{solverParams["ShortName"]}')
            if not os.path.exists(outputFolderPath):
                os.mkdir(outputFolderPath)

            outputPath = os.path.join(outputFolderPath, folder)
            if not os.path.exists(outputPath):
                os.mkdir(outputPath)

            main(inputPath, outputPath, dataScaling, solverParams)