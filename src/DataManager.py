import sys
import json
import random

class Item:
    # Class for the items in StripPacking-Problem
    def __init__(self, itemId, width, length):
        self.Id = itemId
        self.Length = length
        self.Width = width
        self.Area = self.Length * self.Width

class Container:
    # Class for the container in StripPacking-Problem
    def __init__(self, width, length):
        self.Length = length
        self.Width = width
        self.Area = self.Length * self.Width

class Machine:

    def __init__(self, machineId, length, width, height, setup, scanning, recoating):
        self.MachineId = machineId
        self.Length = length
        self.Width = width
        self.Height = height
        self.Setup = setup
        self.ScanTime = scanning
        self.RecoatTime = recoating
        self.Area = self.Width * self.Length 
    
class Variant:

    def __init__(self, variantId, length, width, height, support):
        self.VariantId = variantId
        self.Length = length
        self.Width = width
        self.Height = height
        self.SupportVolume = support

class Part:

    def __init__(self, partId, origin, variants, volume, color):
        self.PartId = partId
        self.OriginId = origin
        self.Variants = variants
        self.Volume = volume + self.Variants[0].SupportVolume
        self.Color = color
        self.Area = self.Variants[0].Length * self.Variants[0].Width
        self.Height = self.Variants[0].Height
    
    def createNewVariant(self):
        # Creating a new Variant by rotating first variant by 90 Degrees
        if self.Variants[-1].Width != self.Variants[-1].Length:
            self.Variants.append(Variant(self.Variants[-1].VariantId + 1, self.Variants[-1].Width, self.Variants[-1].Length, self.Variants[-1].Height, self.Variants[-1].SupportVolume))

class DataModel:

    def __init__(self, modelId, dataScaling = True):
        self.ModelId = modelId
        self.DataScaling = dataScaling
    
    def BuildData(self, path=None):
        
        if path == None:
            print("No path was given. Continuing with instance ht1_1!")
            sys.exit()            

        with open(path, "r") as inputFile:
            inputData = json.load(inputFile)          

        self.Machines = self.createMachineList(inputData)
        self.Parts = self.createPartList(inputData)

        self.M1 = len(self.Parts)

        self.NumberOfMachines = len(self.Machines)
        self.NumberOfParts = len(self.Parts)
                
    def createMachineList(self, inputData):

        tmpId = 0
        tmpMachineList = []

        if self.DataScaling:
            for mtype in range(inputData["MachineTypes"]):
                for _ in range(int(inputData["Machine " + str(mtype)]["Count"])):
                    tmpMachineList.append(Machine(tmpId, int(inputData["Machine " + str(mtype)]["Length"]*10), 
                        int(inputData["Machine " + str(mtype)]["Width"]*10), int(inputData["Machine " + str(mtype)]["Height"]*10), 
                        inputData["Machine " + str(mtype)]["Setup"], inputData["Machine " + str(mtype)]["ScanTime"]/1000, round(inputData["Machine " + str(mtype)]["RecoatTime"]/10, ndigits=3)))
                    tmpId += 1
        
        else:
            for mtype in range(inputData["MachineTypes"]):
                for _ in range(int(inputData["Machine " + str(mtype)]["Count"])):
                    tmpMachineList.append(Machine(tmpId, int(inputData["Machine " + str(mtype)]["Length"]), 
                        int(inputData["Machine " + str(mtype)]["Width"]), int(inputData["Machine " + str(mtype)]["Height"]), 
                        inputData["Machine " + str(mtype)]["Setup"], inputData["Machine " + str(mtype)]["ScanTime"], round(inputData["Machine " + str(mtype)]["RecoatTime"], ndigits=3)))
                    tmpId += 1
 
        return tmpMachineList
    
    def createPartList(self, inputData):
        
        colors = []

        for i in range(inputData["PartCount"]):
            colors.append('#%06X' % random.randint(0, 0xFFFFFF))
        
        tmpId = 0
        tmpPartList = []

        if self.DataScaling:
            for ptype in range(inputData["PartTypes"]):
                for _ in range(inputData["Part " + str(ptype)]["Count"]):
                    tmpVariant = Variant(0, int(inputData["Part " + str(ptype)]["Length"]*10), int(inputData["Part " + str(ptype)]["Width"]*10), 
                        int(inputData["Part " + str(ptype)]["Height"]*10), int(inputData["Part " + str(ptype)]["SupportVolume"]*1000))
                    tmpPartList.append(Part(tmpId, inputData["Part " + str(ptype)]["PartId"], [tmpVariant], 
                        int(inputData["Part " + str(ptype)]["Volume"]*1000), colors[tmpId]))
                    tmpId += 1
        
        else:
            for ptype in range(inputData["PartTypes"]):
                for _ in range(inputData["Part " + str(ptype)]["Count"]):
                    tmpVariant = Variant(0, int(inputData["Part " + str(ptype)]["Length"]), int(inputData["Part " + str(ptype)]["Width"]), 
                        int(inputData["Part " + str(ptype)]["Height"]), int(inputData["Part " + str(ptype)]["SupportVolume"]))
                    tmpPartList.append(Part(tmpId, inputData["Part " + str(ptype)]["PartId"], [tmpVariant], 
                        int(inputData["Part " + str(ptype)]["Volume"]), colors[tmpId]))
                    tmpId += 1
        
        for part in tmpPartList:
            part.createNewVariant()

        return tmpPartList