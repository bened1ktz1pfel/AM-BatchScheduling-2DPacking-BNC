def TransformItems(items, machineSize):
    newItems = []
    for item in items:
        if item.Variants[0].Width <= machineSize and item.Variants[0].Length <= machineSize:
            newItems.append((item.Variants[0].Width, item.Variants[0].Length))
        elif item.Variants[0].Width <= machineSize and item.Variants[0].Length > machineSize:
            newItems.append((item.Variants[0].Width, item.Variants[0].Width))
        elif item.Variants[0].Width > machineSize and item.Variants[0].Length <= machineSize:
            newItems.append((item.Variants[0].Length, item.Variants[0].Length))
        else:
            print(f"Something went wrong in transforming the items. Item {item.Variants[0].Width, item.Variants[0].Length}, machine {machineSize}")
            return None
            newItems.append((0, 0))
    
    return newItems