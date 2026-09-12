
class PlacementPointGenerator():

        # Pattern Approaches are adopted from https://github.com/ktnr/PlacementPointGenerator/blob/master/BinPacking.py and modified for rotation-purposes
    @staticmethod
    def DetermineNormalPatternsX(parts, machineWidth):
        if machineWidth < 0:
            return [0]
        T = [0]* (machineWidth + 1)
        T[0] = 1

        for i, part in enumerate(parts):
            for p in range(machineWidth - min(variant.Width for variant in part.Variants), -1, -1):
                if T[p] == 1:
                    for variant in part.Variants:
                        if p + variant.Width <= machineWidth:
                            T[p + variant.Width] = 1

        normalPatternsX = []
        for p in range (machineWidth, -1, -1):
            if T[p] == 1:
                normalPatternsX.append(p)

        return normalPatternsX

    @staticmethod
    def DetermineNormalPatternsY(parts, machineLength):
        if machineLength < 0:
            return [0]
        T = [0]* (machineLength + 1)
        T[0] = 1

        for i, part in enumerate(parts):
            for p in range(machineLength - min(variant.Length for variant in part.Variants), -1, -1):
                if T[p] == 1:
                    for variant in part.Variants:
                        if p + variant.Length <= machineLength:
                            T[p + variant.Length] = 1

        normalPatternsY = []
        for p in range (machineLength, -1, -1):
            if T[p] == 1:
                normalPatternsY.append(p)

        return normalPatternsY
    
    @staticmethod
    def DetermineMeetInTheMiddlePatternsX(partsReduced, part, machineWidth, t):
        meetInTheMiddlePoints = PlacementPointGenerator.DetermineNormalPatternsX(partsReduced, min(t - 1, machineWidth - min(variant.Width for variant in part.Variants))) # placemenetPointsLeft
        placemenetPointsRightPrime = PlacementPointGenerator.DetermineNormalPatternsX(partsReduced, machineWidth - min(variant.Width for variant in part.Variants) - t)

        for p in placemenetPointsRightPrime:
            meetInTheMiddlePoints.append(machineWidth - min(variant.Width for variant in part.Variants) - p)

        return meetInTheMiddlePoints

    @staticmethod
    def DetermineMeetInTheMiddlePatternsY(partsReduced, part, machineLength, t):
        meetInTheMiddlePoints = PlacementPointGenerator.DetermineNormalPatternsY(partsReduced, min(t - 1, machineLength - min(variant.Length for variant in part.Variants))) # placemenetPointsLeft
        placemenetPointsRightPrime = PlacementPointGenerator.DetermineNormalPatternsY(partsReduced, machineLength - min(variant.Length for variant in part.Variants) - t)

        for p in placemenetPointsRightPrime:
            meetInTheMiddlePoints.append(machineLength - min(variant.Length for variant in part.Variants) - p)

        return meetInTheMiddlePoints

    @staticmethod
    def DetermineMeetInTheMiddlePatterns(partsReduced, part, machineWidth, machineLength):
        meetInTheMiddleX = set()
        meetInTheMiddleY = set()

        for t in range(1, machineWidth + 1, 1):
            meetInTheMiddlePoints = PlacementPointGenerator.DetermineMeetInTheMiddlePatternsX(partsReduced, part, machineWidth, t)
            meetInTheMiddleX.update(meetInTheMiddlePoints)
        
        for t in range(1, machineLength + 1, 1):
            meetInTheMiddlePoints = PlacementPointGenerator.DetermineMeetInTheMiddlePatternsX(partsReduced, part, machineLength, t)
            meetInTheMiddleY.update(meetInTheMiddlePoints)

        return list(meetInTheMiddleX), list(meetInTheMiddleY)

    @staticmethod
    def DetermineMinimalMeetInTheMiddlePatternsX(tmpParts, machine):

        def DeterminePlacementArraysX():
            leftT, rightT =  [0]* (machine.Width + 1), [0]*(machine.Width + 1) #two arrays with all entries initialized at zero

            allNormalPatterns = []
    
            for part in tmpParts:
                minimalWidth = min(variant.Width for variant in part.Variants)
                normalPattern = PlacementPointGenerator.DetermineNormalPatternsX([item for item in tmpParts if item != part], machine.Width - minimalWidth)
                
                for p in normalPattern:
                    leftT[p] += 1
                    # rightT[machine.Width - minimalWidth - p] += 1
                    if (part.Variants[0].Width == part.Variants[0].Length) or any([variant.Width > machine.Width for variant in part.Variants]):
                        rightT[machine.Width - minimalWidth - p] += 1
                    
                    else:
                        rightT[machine.Width - part.Variants[0].Width - p] += 1
                        rightT[machine.Width - part.Variants[0].Length - p] += 1

                allNormalPatterns.append(normalPattern)

            for p in range(1, machine.Width + 1):
                leftT[p] = leftT[p] + leftT[p - 1]
                rightT[machine.Width - p] = rightT[machine.Width - p] + rightT[machine.Width - (p - 1)]
            
            return leftT, rightT, allNormalPatterns
        
        def DetermineMinimalT(leftT, rightT):
            minT = 1
            minValue = leftT[0] + rightT[1]

            for t in range(2, machine.Width + 1):
                if leftT[t - 1] + rightT[t] < minValue:
                    minValue = leftT[t - 1] + rightT[t]
                    minT = t
            
            return minT
        
        def CreatingAllPatterns(allNormalPatterns, minT):
            allPatterns = []

            for i, part in enumerate(tmpParts):
                minimalWidth = min(variant.Width for variant in part.Variants)
                M_i = set()

                for p in allNormalPatterns[i]:
                    if p < minT:
                        M_i.add(p)
                        
                    for variant in part.Variants:

                        if (machine.Width - variant.Width - p) >= minT:
                            M_i.add(machine.Width - variant.Width - p)
                
                allPatterns.append(list(M_i))
            
            return allPatterns
        
        leftT, rightT, allNormalPatterns = DeterminePlacementArraysX()
        t = DetermineMinimalT(leftT, rightT)

        return CreatingAllPatterns(allNormalPatterns, t)

    
    @staticmethod
    def DetermineMinimalMeetInTheMiddlePatternsY(tmpParts, machine):

        def DeterminePlacementArraysY():
            leftT, rightT =  [0]* (machine.Length + 1), [0]*(machine.Length + 1) #two arrays with all entries initialized at zero

            allNormalPatterns = []
    
            for part in tmpParts:
                minimalLength = min(variant.Length for variant in part.Variants)
                normalPattern = PlacementPointGenerator.DetermineNormalPatternsY([item for item in tmpParts if item != part], machine.Length - minimalLength)
                
                for p in normalPattern:
                    leftT[p] += 1
                    #rightT[machine.Length - minimalLength - p] += 1
                    if (part.Variants[0].Width == part.Variants[0].Length) or any([variant.Length > machine.Length for variant in part.Variants]):
                        rightT[machine.Length - minimalLength - p] += 1
                    
                    else:
                        rightT[machine.Length - part.Variants[0].Width - p] += 1
                        rightT[machine.Length - part.Variants[0].Length - p] += 1


                allNormalPatterns.append(normalPattern)

            for p in range(1, machine.Length + 1):
                leftT[p] = leftT[p] + leftT[p - 1]
                rightT[machine.Length - p] = rightT[machine.Length - p] + rightT[machine.Length - (p - 1)]
            
            return leftT, rightT, allNormalPatterns
        
        def DetermineMinimalT(leftT, rightT):
            minT = 1
            minValue = leftT[0] + rightT[1]

            for t in range(2, machine.Length + 1):
                if leftT[t - 1] + rightT[t] < minValue:
                    minValue = leftT[t - 1] + rightT[t]
                    minT = t
            
            return minT
        
        def CreatingAllPatterns(allNormalPatterns, minT):
            allPatterns = []

            for i, part in enumerate(tmpParts):
                minimalLength = min(variant.Length for variant in part.Variants)
                M_i = set()

                for p in allNormalPatterns[i]:
                    if p < minT:
                        M_i.add(p)
                    
                    # if (machine.Length - minimalLength - p) >= minT:
                    #     M_i.add(machine.Length - minimalLength - p)
                    for variant in part.Variants:

                        if (machine.Length - variant.Length - p) >= minT:
                            M_i.add(machine.Length - variant.Length - p)
                
                allPatterns.append(list(M_i))
            
            return allPatterns
        
        leftT, rightT, allNormalPatterns = DeterminePlacementArraysY()
        t = DetermineMinimalT(leftT, rightT)

        return CreatingAllPatterns(allNormalPatterns, t)
   
    @staticmethod
    def DetermineMinimalMeetInTheMiddlePatterns(tmpParts, machine):
        
        return PlacementPointGenerator.DetermineMinimalMeetInTheMiddlePatternsX(tmpParts, machine), PlacementPointGenerator.DetermineMinimalMeetInTheMiddlePatternsY(tmpParts, machine)

    @staticmethod
    def DetermineNormalPatterns(parts, machineWidth, machineLength):
        normalPatternsX = PlacementPointGenerator.DetermineNormalPatternsX(parts, machineWidth)
        normalPatternsY = PlacementPointGenerator.DetermineNormalPatternsY(parts, machineLength)

        return normalPatternsX, normalPatternsY
        
    @staticmethod
    def DetermineNormalPatternsWithoutRotation(itemSizes, containerSize):
        if containerSize < 0:
            return [0]
        T = [0]* (containerSize + 1)
        T[0] = 1

        for i, size in enumerate(itemSizes):
            for p in range(containerSize - size, -1, -1):
                if T[p] == 1:
                    T[p + size] = 1

        normalPatternsX = []
        for p in range (containerSize, -1, -1):
            if T[p] == 1:
                normalPatternsX.append(p)

        return normalPatternsX
    
    @staticmethod
    def DetermineMeetInTheMiddlePatternWoR(itemSizes, selectedItemSize, containerSize):
        meetInTheMiddleX = set()

        for t in range(1, containerSize + 1, 1):
            meetInTheMiddlePoints = PlacementPointGenerator.MeetInTheMiddlePatternWoR(itemSizes, selectedItemSize, containerSize, t)
            meetInTheMiddleX.update(meetInTheMiddlePoints)
        
        return list(meetInTheMiddleX)
    
    @staticmethod
    def MeetInTheMiddlePatternWoR(itemSizes, selectedItemSize, containerSize, t):
        meetInTheMiddlePoints = PlacementPointGenerator.DetermineNormalPatternsWithoutRotation(itemSizes, min(t - 1, containerSize - selectedItemSize)) # placemenetPointsLeft
        placemenetPointsRightPrime = PlacementPointGenerator.DetermineNormalPatternsWithoutRotation(itemSizes, containerSize - selectedItemSize - t)

        for p in placemenetPointsRightPrime:
            meetInTheMiddlePoints.append(containerSize - selectedItemSize - p)

        return meetInTheMiddlePoints
    
    @staticmethod
    def DetermineMinimalMeetInTheMiddlePatternsWoR(itemSizes, containerSize):
        def DeterminePlacementArrays():
            leftT, rightT =  [0]* (containerSize + 1), [0]*(containerSize + 1) #two arrays with all entries initialized at zero

            allNormalPatterns = []
    
            for i, size in enumerate(itemSizes):
                normalPattern = PlacementPointGenerator.DetermineNormalPatternsWithoutRotation([other for j, other in enumerate(itemSizes) if j != i], 
                                                                                               containerSize - size)
                for p in normalPattern:
                    leftT[p] += 1
                    if size <= containerSize:
                        rightT[containerSize - size - p] += 1

                allNormalPatterns.append(normalPattern)

            for p in range(1, containerSize + 1):
                leftT[p] = leftT[p] + leftT[p - 1]
                rightT[containerSize - p] = rightT[containerSize - p] + rightT[containerSize - (p - 1)]
            
            return leftT, rightT, allNormalPatterns
        
        def DetermineMinimalT(leftT, rightT):
            minT = 1
            minValue = leftT[0] + rightT[1]

            for t in range(2, containerSize + 1):
                if leftT[t - 1] + rightT[t] < minValue:
                    minValue = leftT[t - 1] + rightT[t]
                    minT = t
            
            return minT
        
        def CreatingAllPatterns(allNormalPatterns, minT):
            allPatterns = []

            for i, size in enumerate(itemSizes):
                M_i = set()

                for p in allNormalPatterns[i]:
                    if p < minT:
                        M_i.add(p)
                    
                    if (containerSize - size - p) >= minT:
                        M_i.add(containerSize - size - p)
                
                allPatterns.append(list(M_i))
            
            return allPatterns
        
        leftT, rightT, allNormalPatterns = DeterminePlacementArrays()
        t = DetermineMinimalT(leftT, rightT)

        return CreatingAllPatterns(allNormalPatterns, t)
