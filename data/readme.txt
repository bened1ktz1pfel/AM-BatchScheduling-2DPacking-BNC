This file explains how to read the data provided within this folder.
All instance files are orgainzed using python dictionaries.
First, basic instance information are given.
Afterward, detailed information about the machines and parts is given within subdictionaries.

{
  "MachineTypes": 2, 		% Number of different machine types
  "PartTypes": 12, 		% Number of different part types
  "MachineCount": 2, 		% Total number of machines 
  "PartCount": 20, 		% Total number of parts to be produced
  "Machine 0": { 		% Subdictionary of the first machine type
    "MachineId": 0, 		% Machine type ID
    "Count": 1, 		% Number of machines from respective type
    "ScanTime": 0.030864,	% Scan time of machine type
    "RecoatTime": 0.16,		% Recoat time of machine type
    "Setup": 1.0,		% Setup time of machine type
    "Length": 60.0,		% Length of the build space 
    "Width": 40.0,		% Width of the build space
    "Height": 45.0		% Height of the build space
  },
   .
   .
   .
  },
  "Part 0": {			% Subdictionary of the first part type
    "PartId": 0,		% Part type ID
    "Count": 1,			% Number of parts from respective type 
    "Volume": 27.5,		% Volume of the part type
    "Length": 7.5,		% Length of the part type
    "Width": 7.5,		% Width of the part type
    "Height": 5.0,		% Height of the part type
    "SupportVolume": 10.75	% Support volumne of the part type
  },
  .
  .
  .
}