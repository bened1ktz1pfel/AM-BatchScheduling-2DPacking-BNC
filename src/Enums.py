from enum import IntEnum

class PlacementPointStrategy(IntEnum):
    NoStrategy = 0
    NormalPatterns = 1
    MeetInTheMiddle = 2
    MinimalMeetInTheMiddle = 3

class OrthogonalPackingMethod(IntEnum):
    ORTOOLS = 0
    CPLEX = 1
    BNC = 2
    HORTOOLS = 3
    BNB = 4