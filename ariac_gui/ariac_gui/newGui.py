import tkinter as tk
import platform
from datetime import datetime
from os import chdir
from functools import partial
from PIL import Image, ImageTk  # needed for images in gui
from ariac_gui.checkCancel import *
from ariac_gui.updateRanges import *
from ariac_gui.validationFunc import *
from ariac_gui.fileFunc import *
from ariac_gui.buttonFuncs import *
from ariac_gui.timeFunctions import *
from ariac_gui.addPartFunc import *
from ariac_gui.newClasses import *
from ariac_gui.addNewBin import *
from ariac_gui.orderFuncs import *
from ariac_gui.challengesFuncs import *
from ariac_gui.msgNames import *
from ariac_gui.kittingTrayFunctions import *
from ariac_msgs.msg import *

def saveMainWind(window, flag):
    flag.set('1')
    window.destroy()

def runGUI():
    pathIncrement = []  # gives the full path for recursive deletion
    createdDir = []  # to deleted directories made if canceled
    nameLabels = []  # holds temporary flags to be deleted
    agv1Parts=[]
    agv2Parts=[]
    agv3Parts=[]
    agv4Parts=[]
    agv1Quadrants=["1","2","3","4"] # available quadrants for agv1
    agv2Quadrants=["1","2","3","4"] # available quadrants for agv2
    agv3Quadrants=["1","2","3","4"] # available quadrants for agv3
    agv4Quadrants=["1","2","3","4"] # available quadrants for agv4
    bins=[] # holds the bins
    bin1Slots=[] # holds the available slots for bin1
    bin2Slots=[] # holds the available slots for bin2
    bin3Slots=[] # holds the available slots for bin3
    bin4Slots=[] # holds the available slots for bin4
    bin5Slots=[] # holds the available slots for bin5
    bin6Slots=[] # holds the available slots for bin6
    bin7Slots=[] # holds the available slots for bin7
    bin8Slots=[] # holds the available slots for bin8
    convParts=[] # holds conveyor belt parts
    for i in range(9):
        bin1Slots.append(str(i+1))
        bin2Slots.append(str(i+1))
        bin3Slots.append(str(i+1))
        bin4Slots.append(str(i+1))
        bin5Slots.append(str(i+1))
        bin6Slots.append(str(i+1))
        bin7Slots.append(str(i+1))
        bin8Slots.append(str(i+1))
    orderCounter=[] # for counting the number of orders
    orderMSGS=[] # holds all order ros2 messages
    orderConditions=[] #holds all order condition ros2 messages
    binPresentFlags=[] # to hold which bins are present 
    for i in range(8):
        binPresentFlags.append(0)
    usedIDs=[] # holds the ids that have already been used to avoid repeated ids
    robotMalfunctions=[] # holds all robot malfunctions
    faultyParts=[] # holds all faulty parts
    droppedParts=[] # holds all dropped parts
    sensorBlackouts=[] # holds all sensor blackouts
    robotsToDisable=[] # holds robots to be disabled
    faultyPartQuadrants=[] # holds quadrants for dropped parts
    sensorsToDisable=[] # holds sensors for sensor blackout
    availableTrays=["Tray 0","Tray 1","Tray 2","Tray 3","Tray 4","Tray 5","Tray 6","Tray 7","Tray 8","Tray 9"]
    availableSlots=["Slot 1", "Slot 2", "Slot 3", "Slot 4", "Slot 5", "Slot 6"]
    kittingTrayCounter=[]
    # window outputs
    timeVal="0"
    noTimeVal="0"
    timeList=[timeVal, noTimeVal]
    trayVals=[]
    slotVals=[]
    for i in range(6):
        trayVals.append("")
        slotVals.append("")
    agv1TrayIdVal=agvTrayIds[0]
    agv2TrayIdVal=agvTrayIds[0]
    agv3TrayIdVal=agvTrayIds[0]
    agv4TrayIdVal=agvTrayIds[0]
    convActiveVal='0'
    spawnRateVal='0'
    convOrderVal='random'
    partVals=[agv1TrayIdVal, agv2TrayIdVal, agv3TrayIdVal, agv4TrayIdVal, convActiveVal, spawnRateVal, convOrderVal]
    # END OF DEFINITIONS
    # ----------------------------------------------------------------------------------------------
    # START OF GUI
    getFileName = tk.Tk() #window to create and get the file
    fileNameVar = tk.StringVar()
    faultySkipFlag = tk.StringVar()
    faultySkipFlag.set('0')
    dropsSkipFlag = tk.StringVar()
    dropsSkipFlag.set('0')
    sensor_blackout_skip_flag = tk.StringVar()
    sensor_blackout_skip_flag.set('0')
    secondOrderFlag = tk.StringVar()
    secondOrderFlag.set('0')
    assembProdsFlag = tk.StringVar()
    assembProdsFlag.set('0')
    kitProdsFlag = tk.StringVar()
    kitProdsFlag.set('0')
    orderFlag = tk.StringVar()
    orderFlag.set('0')
    orderNextFlag = tk.StringVar()
    orderNextFlag.set('0')
    kittingFlag = tk.StringVar()
    kittingFlag.set('0')
    assembFlag = tk.StringVar()
    assembFlag.set('0')
    frame = tk.Frame(getFileName)
    #getFileName.geometry("850x600")
    getFileName.attributes('-fullscreen', True)
    frame.pack()
    nistLogo = ImageTk.PhotoImage(Image.open("src/ARIAC/ariac_gui/ariac_gui/GUI_Images/new_NIST_logo.png"))
    logoImgLabel = tk.Label(frame, image=nistLogo)
    logoImgLabel.pack(pady=40)
    cancelFlag = tk.StringVar()
    cancelFlag.set('0')
    ordersFlag=tk.StringVar()
    ordersFlag.set('0')
    challengesFlag=tk.StringVar()
    challengesFlag.set('0')
    saveOrdersFlag=tk.StringVar()
    saveOrdersFlag.set('0')
    saveMainFlag=tk.StringVar()
    saveMainFlag.set('0')
    partFlag=tk.StringVar()
    partFlag.set('0')
    getFileName.title("NIST ARIAC CONFIG GUI")
    fileName = tk.StringVar()
    fileName.set("")
    invalidFlag = tk.StringVar()
    invalidFlag.set('0')
    reqFlag = tk.StringVar()
    reqFlag.set("0")
    existFlag = tk.StringVar()
    existFlag.set("0")
    fileNameCorrectFunc = partial(correct_file_name, fileName)
    saveAndExit = partial(make_file, getFileName, fileNameVar)
    openFileExp = tk.Button(getFileName, text="Create file", command=saveAndExit)
    openFileExp.pack()
    cancel_file = partial(cancel_wind, getFileName, cancelFlag)
    cancelFile = tk.Button(getFileName, text="Cancel and Exit", command=cancel_file)
    cancelFile.pack(side=tk.BOTTOM, pady=20)
    fileFunc=partial(get_file_name_next, fileName, invalidFlag, nameLabels, getFileName, reqFlag, existFlag)
    fileExit = tk.Button(getFileName, text="Next", command=fileFunc)
    fileExit.pack(side=tk.BOTTOM, pady=20)
    fileName.trace('w', fileNameCorrectFunc)
    getFileName.mainloop()
    if cancelFlag.get()=='1':
        quit()
    tempFilePath=''
    brokenPath=fileNameVar.get().split("/")
    for i in brokenPath[:-1]:
        tempFilePath+=i+"/"
    fileNameStr=brokenPath[len(brokenPath)-1]
    chdir(tempFilePath)
    saveFileName=fileNameStr
    fileName.set(saveFileName)
    # END OF GETTING THE NAME OF THE FILE
    # ----------------------------------------------------------------------------------------------
    # START OF MAINWIND
    while (saveMainFlag.get()=="0"):
        mainWind=tk.Tk()
        mainWind.attributes('-fullscreen', True)
        #Time limit
        get_time_limit=partial(guiTimeWindow, timeList, mainWind)
        mainTimeButton=tk.Button(mainWind, text="Time Limit", command=get_time_limit)
        mainTimeButton.pack()
        #Kitting trays and slots
        get_kitting_trays=partial(runKittingTrayWind,kittingTrayCounter, availableTrays, availableSlots, cancelFlag,pathIncrement, fileName, createdDir,trayVals, slotVals, mainWind)
        mainKittingTraysButton=tk.Button(mainWind, text="Kitting Trays", command=get_kitting_trays)
        mainKittingTraysButton.pack()
        #Parts
        get_parts=partial(addPart,partVals,agv1Parts, agv2Parts, agv3Parts, agv4Parts, 
        agv1Quadrants, agv2Quadrants, agv3Quadrants, agv4Quadrants,bins,
        bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots, 
        convParts, cancelFlag, pathIncrement,fileName,createdDir, partFlag, mainWind)
        mainPartButton=tk.Button(mainWind, text="Parts", command=get_parts)
        mainPartButton.pack()
        #Orders
        get_order=partial(runOrdersWind, orderMSGS,  orderConditions, orderCounter, usedIDs, ordersFlag, mainWind)
        mainOrderButton=tk.Button(mainWind, text="Orders", command=get_order)
        mainOrderButton.pack()
        #Challenges
        get_challenge=partial(runChallengeWind,robotMalfunctions, usedIDs, faultyParts, droppedParts, sensorBlackouts,cancelFlag, pathIncrement,fileName, createdDir, challengesFlag, mainWind)
        mainChallengeButton=tk.Button(mainWind, text="Challenges", command=get_challenge)
        mainChallengeButton.pack()
        #save button
        save_main_wind=partial(saveMainWind, mainWind, saveMainFlag)
        saveMainButton=tk.Button(mainWind, text="Save and Continue", command=save_main_wind)
        saveMainButton.pack()
        #cancel button
        cancel_main_command=partial(cancel_wind, mainWind, cancelFlag)
        cancelMainButton=tk.Button(mainWind, text="Cancel and Exit", command=cancel_main_command)
        cancelMainButton.pack()
        if partFlag.get()=="1": # checks if the user is still in parts
            mainWind.withdraw()
            addPart(partVals,agv1Parts, agv2Parts, agv3Parts, agv4Parts, 
            agv1Quadrants, agv2Quadrants, agv3Quadrants, agv4Quadrants,bins,
            bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots, 
            convParts, cancelFlag, pathIncrement,fileName,createdDir, partFlag, mainWind)
        if ordersFlag.get()=="1": # checks if the user is still in orders
            mainWind.withdraw()
            runOrdersWind(orderMSGS, orderConditions, orderCounter, usedIDs, ordersFlag, mainWind)
        if challengesFlag.get()=="1":# checks if the user is still in challenges
            mainWind.withdraw()
            runChallengeWind(robotMalfunctions, usedIDs, faultyParts, droppedParts, sensorBlackouts,cancelFlag, pathIncrement,fileName, createdDir, challengesFlag, mainWind)
        mainWind.mainloop()
        check_cancel(cancelFlag.get(), pathIncrement, fileName, createdDir)
    # END OF MAIN WIND
    # ----------------------------------------------------------------------------------------------
    #Finds which bins are present
    for i in bins:
        if i.binName=="bin1":
            binPresentFlags[0]=1
        if i.binName=="bin2":
            binPresentFlags[1]=1
        if i.binName=="bin3":
            binPresentFlags[2]=1
        if i.binName=="bin4":
            binPresentFlags[3]=1
        if i.binName=="bin5":
            binPresentFlags[4]=1
        if i.binName=="bin6":
            binPresentFlags[5]=1
        if i.binName=="bin7":
            binPresentFlags[6]=1
        if i.binName=="bin8":
            binPresentFlags[7]=1
    KTraysSTR=""
    for i in trayVals:
        KTraysSTR+=i
    chosenKTrays=[]
    for i in KTraysSTR:
        if i.isnumeric():
            chosenKTrays.append(i)
    KSlotsSTR=""
    for i in slotVals:
        KSlotsSTR+=i
    chosenKSlots=[]
    for i in KSlotsSTR:
        if i.isnumeric():
            chosenKSlots.append(i)
    #  WRITE TO FILE
    with open(saveFileName, "a") as o:
        o.write("# Trial Name: "+saveFileName+"\n")
        o.write("# ARIAC2023\n")
        o.write("# "+datetime.now().strftime("%Y-%m-%d %H:%M:%S")+"\n\n") #writes the time and date
        o.write("# ENVIRONMENT SETUP\n") 
        if noTimeVal=="1": # runs if the user selects no time limit
            o.write("time_limit: -1")
        else: #writes the time limit
            o.write("time_limit: "+timeVal)
        o.write(" # options: -1 (no time limit) or number of seconds\n")
        if len(chosenKTrays)>0:
            o.write("\nkitting_trays: # Which kitting trays will be spawned\n")
            o.write("  tray_ids: ["+", ".join(chosenKTrays)+"]\n")
            o.write("  slots: ["+", ".join(chosenKSlots)+"]\n")
        o.write("\nparts:\n")
        o.write("  agvs:\n")
    if len(agv1Parts)>0:
        writePartsToFile("agv1", agv1TrayIdVal, agv1Parts, saveFileName)
    if len(agv2Parts)>0:
        writePartsToFile("agv2", agv2TrayIdVal, agv2Parts, saveFileName)
    if len(agv3Parts)>0:
        writePartsToFile("agv3", agv3TrayIdVal, agv3Parts, saveFileName)
    if len(agv4Parts)>0:
        writePartsToFile("agv4", agv4TrayIdVal, agv4Parts, saveFileName)
    with open(saveFileName, "a") as o:
        o.write("\n  bins: # bin params - 8 total bins each bin has nine total slots (1-9)\n")
    if binPresentFlags[0]==1:
        writeBinsToFile("bin1", bins, saveFileName)
    if binPresentFlags[1]==1:
        writeBinsToFile("bin2", bins, saveFileName)
    if binPresentFlags[2]==1:
        writeBinsToFile("bin3", bins, saveFileName)
    if binPresentFlags[3]==1:
        writeBinsToFile("bin4", bins, saveFileName)
    if binPresentFlags[4]==1:
        writeBinsToFile("bin5", bins, saveFileName)
    if binPresentFlags[5]==1:
        writeBinsToFile("bin6", bins, saveFileName)
    if binPresentFlags[6]==1:
        writeBinsToFile("bin7", bins, saveFileName)
    if binPresentFlags[7]==1:
        writeBinsToFile("bin8", bins, saveFileName)
    with open(saveFileName, "a") as o:
        o.write("\n  conveyor_belt: #population params for conveyor belt\n")
        if convActiveVal=="1":
            o.write("    active: true\n")
        else:
            o.write("    active: false\n")
        o.write("    spawn_rate: "+spawnRateVal+" # seconds between spawn\n")
        o.write("    order: "+convOrderVal+" # random or sequential\n")
        if len(convParts)>0:
            o.write("    parts_to_spawn:\n")
            for part in convParts:
                o.write("      - type: \'"+part.type+"\'")
                o.write("\n        color: \'"+part.color+"\'")
                o.write("\n        number: "+part.number)
                o.write("\n        offset: "+part.offset+" # between -1 and 1")
                o.write("\n        rotation: "+ part.rotation)
                o.write("\n        # time_before_next_part: 2 # seconds\n")
        
        #Beginning of order writing to file
        counter=0
        o.write("\n# ORDER SETUP\n")
        o.write("orders:\n")
        for order in orderMSGS:
            o.write("  - id: \'"+order.id+"\'\n")
            o.write("    type: \'"+getOrderType(order.type)+"\'\n")
            o.write("    announcement:\n")
            if orderConditions[counter].type==0:
                o.write("      time_condition: "+str(orderConditions[counter].time_condition.seconds)+"\n")
            elif malf.condition.type==1:
                o.write("      part_type: \'"+getPartName(orderConditions[counter].condition.part_place_condition.part.type)+"\'\n")
                o.write("      part_color: \'"+getPartColor(orderConditions[counter].condition.part_place_condition.part.color)+"\'\n")
                o.write("      agv: "+str(orderConditions[counter].condition.part_place_condition.agv))
            elif malf.condition.type==2:
                o.write("      submission_condition:\n")
                o.write("        order_id: \'"+orderConditions[counter].condition.submission_condition.order_id+"\'\n")
            counter+=1
            o.write("    priority: " + str(order.priority).lower()+"\n")
            if order.type==0:
                o.write("    kitting_task:\n")
                o.write("      agv_number: "+str(order.kitting_task.agv_number)+"\n")
                o.write("      tray_id: "+str(order.kitting_task.tray_id)+"\n")
                o.write("      destination: \'"+getKittingDestName(order.kitting_task.destination)+"\'\n")
                o.write("      products:\n")
                for kittingPart in order.kitting_task.parts:
                    o.write("        - type: \'"+getPartName(kittingPart.part.type)+"\'\n")
                    o.write("          color: \'"+getPartColor(kittingPart.part.color)+"\'\n")
                    o.write("          quadrant: "+str(kittingPart.quadrant)+"\n")
            elif order.type==1:
                o.write("    assembly_task:\n")
                o.write("      agv_number: ["+", ".join(str(agvNum) for agvNum in order.assembly_task.agv_numbers)+"]\n")
                o.write("      station: \'as"+str(order.assembly_task.station)+"\'\n")
                o.write("      products:\n")
                for assemblyPart in order.assembly_task.parts:
                    o.write("        - type: \'"+getPartName(assemblyPart.part.type)+"\'\n")
                    o.write("          color: \'"+getPartColor(assemblyPart.part.color)+"\'\n")
                    o.write("          assembled_pose: # relative to briefcase frame\n")
                    o.write("            xyz: FIXED\n")
                    o.write("            rpy: FIXED\n")
                    o.write("          assembly_direction: ["+str(assemblyPart.install_direction.x)+", "+str(assemblyPart.install_direction.y)+", "+str(assemblyPart.install_direction.z)+"]\n")
            else:
                o.write("    combined_task:\n")
                o.write("      station: \'as"+str(order.combined_task.station)+"\'\n")
                o.write('      products:\n')
                for combinedPart in order.combined_task.parts:
                    o.write("        - type: \'"+getPartName(combinedPart.part.type)+"\'\n")
                    o.write("          color: \'"+getPartColor(combinedPart.part.color)+"\'\n")
                    '''o.write("          assembled_pose: # relative to briefcase frame\n")
                    o.write("            xyz: "+prod.xyz+"\n")
                    o.write("            rpy: "+prod.rpy+"\n")'''
                    o.write("          assembly_direction: ["+str(combinedPart.install_direction.x)+", "+str(combinedPart.install_direction.y)+", "+str(combinedPart.install_direction.z)+"]\n")
        #end of order writing to file
        
        #writes orders to file
        if len(robotMalfunctions)+len(faultyParts)+len(droppedParts)+len(sensorBlackouts)>0:
            o.write("\n# GLOBAL CHALLENGES\n")
            o.write("challenges:\n")
            #robot malfunctions
            for malf in robotMalfunctions:
                o.write("  - robot_malfunction:\n")
                o.write("      duration: "+str(malf.duration)+"\n")
                robotsToDisable=[]
                if malf.robots_to_disable.floor_robot:
                    robotsToDisable.append("\'floor_robot\'")
                if malf.robots_to_disable.ceiling_robot:
                    robotsToDisable.append("\'ceiling_robot\'")
                o.write("      robots_to_disable: ["+", ".join(robotsToDisable)+"]\n")
                if malf.condition.type==0:
                    o.write("      time: "+str(malf.condition.time_condition.seconds)+"\n")
                elif malf.condition.type==1:
                    o.write("      part_type: \'"+getPartName(malf.condition.part_place_condition.part.type)+"\'\n")
                    o.write("      part_color: \'"+getPartColor(malf.condition.part_place_condition.part.color)+"\'\n")
                    o.write("      agv: "+str(malf.condition.part_place_condition.agv))
                elif malf.condition.type==2:
                    o.write("      order_id: \'"+malf.condition.submission_condition.order_id+"\'\n")
            #faulty parts
            for part in faultyParts:
                o.write("  - faulty_part:\n")
                o.write("      order_id: \'"+part.order_id+"\'\n")
                faultyPartQuadrants=[]
                if part.quadrant1:
                    faultyPartQuadrants.append("1")
                if part.quadrant2:
                    faultyPartQuadrants.append("2")
                if part.quadrant3:
                    faultyPartQuadrants.append("3")
                if part.quadrant4:
                    faultyPartQuadrants.append("4")
                o.write("      quadrant: ["+", ".join(faultyPartQuadrants)+"]\n")
            #dropped parts
            for part in droppedParts:
                o.write("  - dropped_part:\n")
                o.write("      robot: \'"+part.robot+"\'\n")
                o.write("      type: \'"+getPartName(part.part_to_drop.type)+"\'\n")
                o.write("      color: \'"+getPartColor(part.part_to_drop.color)+"\'\n")
                o.write("      drop_after_num: "+str(part.drop_after_num)+" # first part the robot successfully picks\n")
                o.write("      drop_after_time: "+str(part.drop_after_time)+" # secons\n")
            #sensor blackouts
            for blackout in sensorBlackouts:
                o.write("  - sensor_blackout:\n")
                o.write("    duration: "+str(blackout.duration)+"\n")
                #gets the list of sensors to disable
                if blackout.sensors_to_disable.break_beam:
                    sensorsToDisable.append("break_beam")
                if blackout.sensors_to_disable.proximity:
                    sensorsToDisable.append("proximity")
                if blackout.sensors_to_disable.laser_profiler:
                    sensorsToDisable.append("laser_profiler")
                if blackout.sensors_to_disable.lidar:
                    sensorsToDisable.append("lidar")
                if blackout.sensors_to_disable.camera:
                    sensorsToDisable.append("camera")
                if blackout.sensors_to_disable.logical_camera:
                    sensorsToDisable.append("logical_camera")
                o.write("    sensors_to_disable: ["+", ".join(sensorsToDisable)+"]\n")
                if blackout.condition.type==0:
                    o.write("      time: "+str(blackout.condition.time_condition.seconds)+"\n")
                elif blackout.condition.type==1:
                    o.write("      part_type: \'"+getPartName(blackout.condition.part_place_condition.part.type)+"\'\n")
                    o.write("      part_color: \'"+getPartColor(blackout.condition.part_place_condition.part.color)+"\'\n")
                    o.write("      agv: "+str(blackout.condition.part_place_condition.agv))
                elif blackout.condition.type==2:
                    o.write("      order_id: \'"+blackout.condition.submission_condition.order_id+"\'\n")