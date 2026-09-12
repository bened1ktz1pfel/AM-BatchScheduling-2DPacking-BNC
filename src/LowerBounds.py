import math
# See M. Dell’Amico et al.: A lower bound for the non-oriented two-dimensional bin packing problem (2002)
class BinPackingBound:

    def __init__(self, items, width, length):
        self.length = length
        self.width = width
        self.l = {i: part.Variants[0].Length for i, part in enumerate(items)}
        self.w = {i: part.Variants[0].Width for i, part in enumerate(items)}
        self.items = [i for i, part in enumerate(items)]

    def cutInSquares(self):
            squares = {}
            i = 1
            for j in self.items:
                s = {}
                w, l = max(self.w[j], self.l[j]), min(self.w[j], self.l[j])

                while l > 1:  
                    k = math.floor(w / l)
                    for num in range(k):
                        s[i + num] = l
                    i += k
                    w -= k * l
                    w, l = l, w
                squares.update(s)
            return squares

    def GetLowerBound(self):
        squares = self.cutInSquares()  # {id: l}
        if self.length > self.width:
            self.width, self.length = self.length, self.width

        def getIndividualLowerBound(q):
            S1 = {j for j in squares if squares[j] > self.width - q}
            S2 = {j for j in squares if self.width - q >= squares[j] > self.width / 2}
            S3 = {j for j in squares if self.width / 2 >= squares[j] > self.length / 2}
            S4 = {j for j in squares if self.length / 2 >= squares[j] >= q}

            if self.width == self.length:
                low_q = len(S1 | S2) + max(0, math.ceil(sum(squares[j] ** 2 for j in S2 | S4) /
                                                        self.width ** 2 - len(S2)))
                return low_q

            S_3 = set()
            _S2, _S3 = sorted(list(S2), key=lambda x: squares[x], reverse=True), sorted(
                list(S3), key=lambda x: squares[x], reverse=True)
            q = 0
            for i in _S2:
                for _j in range(q, len(_S3)):
                    j = _S3[_j]
                    if squares[i] + squares[j] <= self.width:
                        S_3.add(j)
                        q = _j + 1
                        break
                else:  
                    break

            low = len(S2) + max(math.ceil(sum(squares[j] for j in S3 if j not in S_3) / self.width), math.ceil(
                (len(S3) - len(S_3)) / math.floor(self.width / math.floor(1 + self.length / 2))))
            S23 = {j for j in S2 | S3 if squares[j] > self.length - q}
            low_q = len(S1) + low + max(0, math.ceil((sum(squares[j] ** 2 for j in S2 | S3 | S4) - (
                    self.width * self.length * low - sum(squares[j] * (self.length - squares[j]) for j in S23))) / (
                    self.width * self.length)))
            return low_q

        return max(getIndividualLowerBound(q) for q in range(math.floor(self.length / 2) + 1))

class BinPackingLowerBoundBoschetti:

    def __init__(self, items, width, length):
        self.length = length
        self.width = width
        self.h = {i: part.Variants[0].Length for i, part in enumerate(items)}
        self.w = {i: part.Variants[0].Width for i, part in enumerate(items)}
        self.items = [i for i, part in enumerate(items)]

        self.ModifiedSizes()

    def ModifiedSizes(self):
        self.changedItems = []
        self.unchangedItems = []
        for i in self.items:
            if self.w[i] <= self.length and self.h[i] <= self.width:
                self.w[i] = min(self.w[i], self.h[i])
                self.changedItems.append(i)
            else:
                self.unchangedItems.append(i)


    def getLowerBound(self):

        def nu(s, z, Z):
            if z > (Z / 2):
                return math.floor(Z / s) - math.floor((Z - z) / s)
            else:
                return math.floor(z / s)
        
        def mu(j, p, q):
            if j in self.changedItems:
                return min((nu(q, self.w[j], self.width) * nu(p, self.h[j], self.length)), (nu(q, self.h[j], self.width) * nu(p, self.w[j], self.length)))
            else:
                return (nu(q, self.w[j], self.width) * nu(p, self.h[j], self.length))
        
        lowerBound = max(math.ceil((sum(mu(j, p, q) for j in self.items)) / (math.floor(self.length / p) * math.floor(self.width / q))) for p in range(1, math.floor(self.length / 2) + 1) for q in range(1, math.floor(self.width / 2) + 1))

        return lowerBound
