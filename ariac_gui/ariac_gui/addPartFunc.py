import tkinter as tk
from time import sleep
from functools import partial
from ariac_gui.addNewBin import addBin
from ariac_gui.addConvPart import addPartConv
from ariac_gui.checkCancel import *
from ariac_gui.validationFunctions import *
from ariac_gui.updateAGVFuncs import *
from ariac_gui.newClasses import *
from ariac_msgs.msg import *

agvTrayIds=["0","1","2","3","4","5","6"] # all options for tray ids for agvs
agvList=["agv1", "agv2", "agv3", "agv4"]
partTypes=["sensor", "pump", "regulator", "battery"]
partColors=['green', 'red', 'purple','blue','orange']
leftColumn=1
middleColumn=2 
middleColumnWidth=62  # width of middle margin for live yaml windows
rightColumn=3
rightMiddleColumn=4
farRightColumn=5
convOrders=["random", "sequential"]

def addAGVPart(agv1Parts, agv2Parts, agv3Parts, agv4Parts, agv1Quadrants,agv2Quadrants,agv3Quadrants,agv4Quadrants, mainWind):
    '''Adds a part to the agv'''
    newPartWind=tk.Toplevel(mainWind)
    #newPartWind.geometry("850x600")
    newPartWind.attributes('-fullscreen', True)
    #agv selection
    agvSelection=tk.StringVar()
    agvSelection.set(agvList[0])
    agvSelectLabel=tk.Label(newPartWind, text="Select the agv for the part")
    agvSelectLabel.pack()
    agvSelectMenu=tk.OptionMenu(newPartWind, agvSelection, *agvList)
    agvSelectMenu.pack()
    #part type selection
    partType=tk.StringVar()
    partType.set(partTypes[0])
    partTypeSelectLabel=tk.Label(newPartWind, text="Select the type of part")
    partTypeSelectLabel.pack()
    partTypeSelectMenu=tk.OptionMenu(newPartWind, partType, *partTypes)
    partTypeSelectMenu.pack()
    #part color selection
    partColor=tk.StringVar()
    partColor.set(partColors[0])
    partColorSelectLabel=tk.Label(newPartWind, text="Select the color of the part")
    partColorSelectLabel.pack()
    partColorSelectMenu=tk.OptionMenu(newPartWind, partColor, *partColors)
    partColorSelectMenu.pack()
    #quadrant selection
    partQuadrant=tk.StringVar()
    partQuadrant.set(agv1Quadrants[0])
    partQuadrantSelectLabel=tk.Label(newPartWind, text="Select the quadrant of the tray for the part")
    partQuadrantSelectLabel.pack()
    partQuadrantSelectMenu=tk.OptionMenu(newPartWind, partQuadrant, *agv1Quadrants)
    partQuadrantSelectMenu.pack()
    #rotation entry
    partRotation=tk.StringVar()
    partRotation.set('0')
    partRotationLabel=tk.Label(newPartWind, text="Enter the rotation of the part")
    partRotationLabel.pack()
    partRotationEntry=tk.Entry(newPartWind, textvariable=partRotation)
    partRotationEntry.pack()
    #save and cancel buttons
    save_new_part=partial(saveAndExitParts,agvSelection, partQuadrant, agv1Quadrants, agv2Quadrants, agv3Quadrants, agv4Quadrants, newPartWind, mainWind)
    saveNewPartButton=tk.Button(newPartWind, text="Save", command=save_new_part)
    saveNewPartButton.pack(pady=20)
    newPartCancelFlag=tk.StringVar()
    newPartCancelFlag.set("0")
    cancel_new_part=partial(cancel_func, newPartWind, newPartCancelFlag)
    cancelNewPartButton=tk.Button(newPartWind, text="Cancel", command=cancel_new_part)
    cancelNewPartButton.pack(pady=20)
    #trace functions
    validate_rotation=partial(validateRotationValue, partRotation, saveNewPartButton)
    partRotation.trace('w', validate_rotation)
    agv_update_menu=partial(updateAgvQudrants,agvSelection, partQuadrantSelectMenu, partQuadrant, agv1Quadrants,agv2Quadrants,agv3Quadrants,agv4Quadrants)
    agvSelection.trace('w', agv_update_menu)
    newPartWind.mainloop()
    if newPartCancelFlag.get()=="0":
        add_quotes(partType)
        add_quotes(partColor)
        if 'pi' in partRotation.get():
            add_quotes(partRotation)
        if agvSelection.get()=='agv1': # saves the part to the correct agv
            agv1Parts.append(PartsClass(partType.get(), partColor.get(), partQuadrant.get(), partRotation.get()))
        elif agvSelection.get()=='agv2':
            agv2Parts.append(PartsClass(partType.get(), partColor.get(), partQuadrant.get(), partRotation.get()))
        elif agvSelection.get()=='agv3':
            agv3Parts.append(PartsClass(partType.get(), partColor.get(), partQuadrant.get(), partRotation.get()))
        else:
            agv4Parts.append(PartsClass(partType.get(), partColor.get(), partQuadrant.get(), partRotation.get()))      

def savePart(wind, partFlag): # allows the while loop in main to stop so the parts window stops when the user saves
    partFlag.set('0')
    wind.destroy()

def addPart(partVals,agv1Parts, agv2Parts, agv3Parts, agv4Parts, 
        agv1Quadrants, agv2Quadrants, agv3Quadrants, agv4Quadrants,bins,
        bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots, 
        convParts, cancelFlag, pathIncrement,fileName,createdDir, partFlag, mainWind):
    partsWind=tk.Toplevel(mainWind)
    partFlag.set('1')
    partsWind.grid_columnconfigure(0, weight=1)
    partsWind.grid_columnconfigure(6, weight=1)
    #variables for parts
    agv1TrayId=tk.StringVar()
    agv1TrayId.set(partVals[0])
    agv2TrayId=tk.StringVar()
    agv2TrayId.set(partVals[1])
    agv3TrayId=tk.StringVar()
    agv3TrayId.set(partVals[2])
    agv4TrayId=tk.StringVar()
    agv4TrayId.set(partVals[3])
    spawnRate=tk.StringVar()
    spawnRate.set(partVals[5])
    convOrder=tk.StringVar()
    convOrder.set(partVals[6])
    convActive=tk.StringVar()
    if len(convParts)==1:#changes to active if a part is placed
        convActive.set('1')
    else:
        convActive.set(partVals[4])
    #agv settings
    partsWind.attributes('-fullscreen', True)
    agv1TrayLabel=tk.Label(partsWind, text="Select the tray Id for agv1")
    agv1TrayLabel.grid(column=leftColumn,padx=20)
    agv1TrayIdSelect=tk.OptionMenu(partsWind, agv1TrayId, *agvTrayIds)
    agv1TrayIdSelect.grid(column=leftColumn)
    agv2TrayLabel=tk.Label(partsWind, text="Select the tray Id for agv2")
    agv2TrayLabel.grid(column=leftColumn)
    agv2TrayIdSelect=tk.OptionMenu(partsWind, agv2TrayId, *agvTrayIds)
    agv2TrayIdSelect.grid(column=leftColumn)
    agv3TrayLabel=tk.Label(partsWind, text="Select the tray Id for agv3")
    agv3TrayLabel.grid(column=leftColumn)
    agv3TrayIdSelect=tk.OptionMenu(partsWind, agv3TrayId, *agvTrayIds)
    agv3TrayIdSelect.grid(column=leftColumn)
    agv4TrayLabel=tk.Label(partsWind, text="Select the tray Id for agv4")
    agv4TrayLabel.grid(column=leftColumn)
    agv4TrayIdSelect=tk.OptionMenu(partsWind, agv4TrayId, *agvTrayIds)
    agv4TrayIdSelect.grid(column=leftColumn)
    #agv parts
    add_new_part=partial(addAGVPart,agv1Parts, agv2Parts, agv3Parts, agv4Parts, agv1Quadrants,agv2Quadrants,agv3Quadrants,agv4Quadrants, mainWind)
    addPartsButton=tk.Button(partsWind, text="Add AGV Part", command=add_new_part)
    addPartsButton.grid(column=leftColumn)
    leftSpaceLabel=tk.Label(partsWind, text="")
    leftSpaceLabel.grid(column=leftColumn, pady=30)
    #bin part button
    add_bin_func=partial(addBin,bins,bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots,mainWind)
    addBinsButton=tk.Button(partsWind, text="Add Bin Part", command=add_bin_func)
    addBinsButton.grid(column=leftColumn, pady=20)
    #middle buffer to split the window
    middleSpaceLabel=tk.Label(partsWind, text="", justify=tk.CENTER)
    middleSpaceLabel.grid(column=middleColumn,row=0, padx=30)
    #conveyor belt settings and add parts
    conveyorBeltLabel=tk.Label(partsWind, text="Conveyor Belt Settings")
    conveyorBeltLabel.grid(column=rightColumn, row=0)
    activeCheck=tk.Checkbutton(partsWind, text="Active", variable=convActive, onvalue="1", offvalue="0", height=3, width=20)
    activeCheck.grid(column=rightColumn, row=1)
    spawnRateEntryLabel=tk.Label(partsWind, text="Enter the spawn rate for the conveyor belt")
    spawnRateEntryLabel.grid(column=rightColumn, row=2)
    spawnRateEntry=tk.Entry(partsWind, textvariable=spawnRate)
    spawnRateEntry.grid(column=rightColumn, row=3)
    convOrderLabel=tk.Label(partsWind, text="Order of the conveyor belt")
    convOrderLabel.grid(column=rightColumn, row=4)
    convOrderMenu=tk.OptionMenu(partsWind, convOrder, *convOrders)
    convOrderMenu.grid(column=rightColumn, row=5)
    add_conv_part=partial(addPartConv, convParts, mainWind)
    addPartConvButton=tk.Button(partsWind, text="Add conveyor belt part", command=add_conv_part)
    addPartConvButton.grid(column=rightColumn, row=6)
    validate_spawn_rate=partial(require_num, spawnRate)
    spawnRate.trace('w', validate_spawn_rate)
    rightSpaceLabel=tk.Label(partsWind, text="")
    rightSpaceLabel.grid(column=rightColumn, row=8, pady=30)
    #save and cancel button
    save_part=partial(savePart, mainWind, partFlag)#needs to change the save flag to exit the window and not refresh
    savePartsButton=tk.Button(partsWind, text="Save and return to main menu", command=save_part)
    savePartsButton.grid(column=rightColumn, row=10, pady=20)
    #part labels
    #Label for parts currently selected for agv1
    currentAGV1Parts="Current AGV1 Parts:\n"
    for part in agv1Parts:
        currentAGV1Parts+=part.color+" "+part.pType+"\n"
    currentAGV1Label=tk.Label(partsWind, text=currentAGV1Parts)
    currentAGV1Label.grid(column=rightMiddleColumn,row=0,padx=40)
    #Label for parts currently selected for agv2
    currentAGV2Parts="Current AGV2 Parts:\n"
    for part in agv2Parts:
        currentAGV2Parts+=part.color+" "+part.pType+"\n"
    currentAGV2Label=tk.Label(partsWind, text=currentAGV2Parts)
    currentAGV2Label.grid(column=rightMiddleColumn,row=2,padx=40)
    #Label for parts currently selected for agv3
    currentAGV3Parts="Current AGV3 Parts:\n"
    for part in agv3Parts:
        currentAGV3Parts+=part.color+" "+part.pType+"\n"
    currentAGV3Label=tk.Label(partsWind, text=currentAGV3Parts)
    currentAGV3Label.grid(column=rightMiddleColumn,row=4,padx=40)
    #Label for parts currently selected for agv4
    currentAGV4Parts="Current AGV4 Parts:\n"
    for part in agv4Parts:
        currentAGV4Parts+=part.color+" "+part.pType+"\n"
    currentAGV4Label=tk.Label(partsWind, text=currentAGV4Parts)
    currentAGV4Label.grid(column=rightMiddleColumn,row=6,padx=40)
    #Label for parts currently selected for bins
    currentBins="Current Bins:\n"
    for bin in bins:
        currentBins+=bin.binName+" "+bin.color+" "+bin.type+"\n"
    currentBinLabel=tk.Label(partsWind, text=currentBins)
    currentBinLabel.grid(column=farRightColumn, row=0,padx=40)
    #Label for parts currently selected for the conveyor belt 
    currentConv="Current Conveyor Belt Parts:\n"
    for convPart in convParts:
        currentConv+=convPart.color+" "+convPart.type+"\n"
    currentConLabel=tk.Label(partsWind, text=currentConv)
    currentConLabel.grid(column=farRightColumn, row=2, padx=40)
    partsWind.mainloop()
    partVals.clear()
    partVals.append(agv1TrayId.get())
    partVals.append(agv2TrayId.get())
    partVals.append(agv3TrayId.get())
    partVals.append(agv4TrayId.get())
    partVals.append(convActive.get())
    partVals.append(spawnRate.get())
    partVals.append(convOrder.get())

def writePartsToFile(name, id, partsList, saveFileName): # writes the part information to the file for a given agv
    '''Writes agv info and parts on the agv to the file'''
    with open(saveFileName, "a") as o:
        o.write("    "+name+":\n")
        o.write("      tray_id: "+ id+"\n")
        o.write("      parts:\n")
        for i in partsList:
            o.write("      - type: \'"+i.pType+"\'\n")
            o.write("        color: \'"+i.color+"\'\n")
            o.write("        quadrant: "+i.quadrant+"\n")
            o.write("        rotation: "+i.rotation+"\n")
