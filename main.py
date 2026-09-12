import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from Solver import *

def getDemoInstance():
    folder = "n10m2"
    instance = "Instance_Class1_Parts10_Machines2_Instance1.json"

    return folder, instance

def runDemo(inputPath, file, outputPath, dataScaling, solverParams):
    seeds = solverParams["Seeds"]
    enableVisualization = solverParams["Visualization"]

    if file.endswith(".json"):
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

if __name__ == "__main__":

    print("This is a demo run of the BNC solver for scheduling with 2D orthogonal packing.")
    print("================================")

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
            'OrthogonalPackingMethod': OrthogonalPackingMethod.ORTOOLS,
            'Threads': 8,
            'MIPTolerance': 0.00001,
            'Seeds': [22]
        }
    }

    folder, instance = getDemoInstance()

    inputPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data', folder)

    baseOutputPath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

    solverParams = SOLVERVARIANTS[1]

    if folder.startswith("Che"):
        dataScaling = True
    else:
        dataScaling = False

    outputFolderPath = os.path.join(baseOutputPath, f'BNC_{solverParams["ShortName"]}_demo')
    if not os.path.exists(outputFolderPath):
        os.mkdir(outputFolderPath)

    outputPath = os.path.join(outputFolderPath, folder)
    if not os.path.exists(outputPath):
        os.mkdir(outputPath)

    runDemo(inputPath, instance, outputPath, dataScaling, solverParams)
    print(f"Finished processing folder: {folder}. Results saved in: {outputPath}")
    print("================================")  