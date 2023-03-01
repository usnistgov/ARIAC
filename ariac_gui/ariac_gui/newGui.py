import tkinter as tk
from datetime import datetime
from os import chdir
from tkinter import ttk
from functools import partial
from ariac_gui.notebookSetup import timeEntry, kittingTrayWidgets
from ariac_gui.notebookParts import partsWidgets, agvTrayWidgets, writePartsToFile
from ariac_gui.notebookBins import binWidgets, writeBinsToFile
from ariac_gui.notebookConveyor import convWidgets
from ariac_gui.notebookOrders import orderWidgets
from ariac_gui.notebookChallenges import allChallengeWidgets, chooseChallenge
from PIL import Image, ImageTk  # needed for images in gui
from ariac_gui.checkCancel import *
from ariac_gui.validationFunctions import *
from ariac_gui.fileFunc import *
from ariac_gui.timeFunctions import *
from ariac_gui.newClasses import *
from ariac_gui.msgNames import *
from ariac_msgs.msg import *
from ament_index_python.packages import get_package_share_directory
FRAMEWIDTH=1000
FRAMEHEIGHT=750
WINDOWMEASURE="900x800"
LEFTCOLUMN=1
def updatePartOrdLabel(agv1Parts, agv2Parts, agv3Parts, agv4Parts,bins,convParts, orderMSGS,partOrdLabel,a,b,c):
    newText="Parts:\nAGV Parts present:\n"
    for part in agv1Parts:
        newText+=part.color+" "+part.pType+" on AGV 1\n"
    for part in agv2Parts:
        newText+=part.color+" "+part.pType+" on AGV 2\n"
    for part in agv3Parts:
        newText+=part.color+" "+part.pType+" on AGV 3\n"
    for part in agv4Parts:
        newText+=part.color+" "+part.pType+" on AGV 4\n"
    if len(agv1Parts)+len(agv2Parts)+len(agv3Parts)+len(agv4Parts)==0:
        newText+="NONE\n"
    
    newText+="\nBin Parts present:\n"
    for bin in bins:
        newText+=bin.binName+" "+bin.color+" "+bin.type+"\n"
    if len(bins)==0:
        newText+="NONE\n"
    
    newText+="\nConveyor Parts present:\n"
    for convPart in convParts:
        newText+=convPart.color+" "+convPart.type+"\n"
    if len(convParts)==0:
        newText+="NONE\n"
    newText+="\nOrders Present:\n"
    if len(orderMSGS)==0:
        newText+="NONE"
    else:
        c=0
        for order in orderMSGS:
            newText+="Order "+order.id+":\n"
            newText+="Type: "+getOrderType(order.type)+"\n"
            newText+="Number of parts: "
            if order.type==0:
                newText+=str(len(order.kitting_task.parts))
            elif order.type==1:
                newText+=str(len(order.assembly_task.parts))
            else:
                newText+=str(len(order.combined_task.parts))
            newText+="\n"
            c+=1
    partOrdLabel.config(text=newText)

def updateChallengeLabel(robotMalfunctions, faultyParts, droppedParts, sensorBlackouts,humanChallenges,challengesLabel, a,b,c):
    newText="Challenges:\nRobot Malfunction challenges:\n"
    for rm in robotMalfunctions:
        robotsToDisable=[]
        newText+="Robots: "
        if rm.robots_to_disable.floor_robot:
            robotsToDisable.append("\'floor_robot\'")
        if rm.robots_to_disable.ceiling_robot:
            robotsToDisable.append("\'ceiling_robot\'")
        newText+=",".join(robotsToDisable)
        newText+=" Duration: "+str(rm.duration)+"\n"
    if len(robotMalfunctions)==0:
        newText+="NONE\n"
    newText+="\nFaulty Part challenges:\n"
    for fp in faultyParts:
        newText+="order_id: \'"+fp.order_id+"\'"
        faultyPartQuadrants=[]
        if fp.quadrant1:
            faultyPartQuadrants.append("1")
        if fp.quadrant2:
            faultyPartQuadrants.append("2")
        if fp.quadrant3:
            faultyPartQuadrants.append("3")
        if fp.quadrant4:
            faultyPartQuadrants.append("4")
        newText+=" Quadrants: "+",".join(faultyPartQuadrants)+"\n"
    if len(faultyParts)==0:
        newText+="NONE\n"
    newText+="\nDropped Part challenges:\n"
    for dp in droppedParts:
        newText+=dp.robot+" type: \'"+getPartName(dp.part_to_drop.type)+" color: \'"+getPartColor(dp.part_to_drop.color)+"\n"
    if len(droppedParts)==0:
        newText+="NONE\n"
    newText+="\nSensor Blackout challenges:\n"
    for sb in sensorBlackouts:
        sensorsToDisable=[]
        newText+="Duration: "+str(sb.duration)
        #gets the list of sensors to disable
        if sb.sensors_to_disable.break_beam:
            sensorsToDisable.append("break_beam")
        if sb.sensors_to_disable.proximity:
            sensorsToDisable.append("proximity")
        if sb.sensors_to_disable.laser_profiler:
            sensorsToDisable.append("laser_profiler")
        if sb.sensors_to_disable.lidar:
            sensorsToDisable.append("lidar")
        if sb.sensors_to_disable.camera:
            sensorsToDisable.append("camera")
        if sb.sensors_to_disable.logical_camera:
            sensorsToDisable.append("logical_camera")
        newText+=" Sensors: "+", ".join(sensorsToDisable)+"\n"
    if len(sensorBlackouts)==0:
        newText+="NONE\n"
    behaviorOptions=["antagonistic", "indifferent","helpful"]
    newText+="\nHuman challenges:\n"
    for ch in humanChallenges:
        newText+="Behavior: "+behaviorOptions[ch.behavior]+"\n"
    if len(humanChallenges)==0:
        newText+="NONE\n"
    challengesLabel.config(text=newText)



def runGUI(): # runs the entire gui

    pathIncrement = []  # gives the full path for recursive deletion
    createdDir = []  # to deleted directories made if canceled
    nameLabels = []  # holds temporary flags to be deleted

    trayValsMain=[]
    slotValsMain=[]

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
    for i in range(9): # writes all slots to each of the bins
        bin1Slots.append(str(i+1))
        bin2Slots.append(str(i+1))
        bin3Slots.append(str(i+1))
        bin4Slots.append(str(i+1))
        bin5Slots.append(str(i+1))
        bin6Slots.append(str(i+1))
        bin7Slots.append(str(i+1))
        bin8Slots.append(str(i+1))
    binPresentFlags=[] # to hold which bins are present 
    for i in range(8):
        binPresentFlags.append(0)

    orderCounter=[] # for counting the number of orders
    orderMSGS=[] # holds all order ros2 messages
    orderConditions=[] #holds all order condition ros2 messages
    usedIDs=[] # holds the ids that have already been used to avoid repeated ids

    robotMalfunctions=[] # holds all robot malfunctions
    faultyParts=[] # holds all faulty parts
    droppedParts=[] # holds all dropped parts
    sensorBlackouts=[] # holds all sensor blackouts
    humanChallenges=[]
    robotsToDisable=[] # holds robots to be disabled
    faultyPartQuadrants=[] # holds quadrants for dropped parts
    sensorsToDisable=[] # holds sensors for sensor blackout
    rmVals=[]
    fpVals=[]
    dpVals=[]
    sbVals=[]
    huVals=[]
    chCondVals=[]
    behaviorOptions=["antagonistic", "indifferent","helpful"]

    availableTrays=["Tray 0","Tray 1","Tray 2","Tray 3","Tray 4","Tray 5","Tray 6","Tray 7","Tray 8","Tray 9"] #list of trays to hold available trays for kitting trays
    availableSlots=["Slot 1", "Slot 2", "Slot 3", "Slot 4", "Slot 5", "Slot 6"] #list of slots to hold available slots for kitting trays
    kittingTrayCounter=[] #holds the number of trays present to avoid overflow in the kitting tray window

    # window outputs
    timeVal="0"
    noTimeVal="0"
    timeList=[timeVal, noTimeVal]

    trayVals=[]
    slotVals=[]
    for i in range(6):
        trayVals.append("")
        slotVals.append("")
    # END OF DEFINITIONS
    # ----------------------------------------------------------------------------------------------
    # START OF GUI
    getFileName = tk.Tk() #window to create and get the file
    getFileName.geometry(WINDOWMEASURE)
    getFileName.title("NIST ARIAC CONFIG GUI")

    frame = tk.Frame(getFileName)
    #getFileName.geometry("850x600")
    frame.pack()
    pkg_share = get_package_share_directory('ariac_gui')
    nistLogo = ImageTk.PhotoImage(Image.open(pkg_share + "/resource/NIST_logo.png"))
    logoImgLabel = tk.Label(frame, image=nistLogo)
    logoImgLabel.pack(pady=40)
    
    cancelFlag = tk.StringVar()
    cancelFlag.set('0')
    ordersFlag=tk.StringVar()
    ordersFlag.set('0')
    challengesFlag=tk.StringVar()
    challengesFlag.set('0')
    saveMainFlag=tk.StringVar()
    saveMainFlag.set('0')
    partFlag=tk.StringVar()
    partFlag.set('0')
    fileName = tk.StringVar()
    fileName.set("")
    invalidFlag = tk.StringVar()
    invalidFlag.set('0')
    reqFlag = tk.StringVar()
    reqFlag.set("0")
    existFlag = tk.StringVar()
    existFlag.set("0")
    
    fileNameVar = tk.StringVar()
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
    presentChallengeWidgets=[]
    allChallengeWidgetsArr=[]
    agvTrayWidgetsArr = []
    agvTrayValsArr = []
    kittingParts=[] 
    assemblyParts=[]
    convSettingsVals=[]
    mainWind=tk.Tk()
    partOrdCounter=tk.StringVar()
    partOrdCounter.set("0")
    challengeCounter=tk.StringVar()
    challengeCounter.set("0")
    mainWind.geometry(WINDOWMEASURE)
    mainWind.title('Main Window')
    mainFrame=ttk.Frame(mainWind)
    mainFrame.grid(row=0, column=1, columnspan=3, padx=10, pady=10, sticky=tk.E+tk.W+tk.N+tk.S)
    notebook=ttk.Notebook(mainFrame)
    notebook.grid(pady=10, column=LEFTCOLUMN,sticky=tk.E+tk.W+tk.N+tk.S)

    setupFrame = ttk.Frame(notebook, width=800, height=600)
    

    setupFrame.pack(fill='both', expand=True)
    notebook.add(setupFrame, text='Setup')
    timeVar=tk.StringVar()
    timeEntry(setupFrame, timeVar, timeVal[0])
    # add frames to notebook

    #kitting trays
    kittingTrayFrame=ttk.Frame(notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
    kittingTrayFrame.pack(fill='both', expand=True)
    notebook.add(kittingTrayFrame, text='Kitting Trays')
    kittingTrayWidgets(kittingTrayFrame, kittingTrayCounter, availableSlots, availableTrays, trayVals, slotVals,trayValsMain, slotValsMain)

    partsFrame = ttk.Frame(notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
    partsFrame.pack(fill='both', expand=True)
    notebook.add(partsFrame, text='AGV Parts')

    binFrame=ttk.Frame(notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
    binFrame.pack(fill='both', expand=True)
    notebook.add(binFrame, text="Bins")

    convFrame=ttk.Frame(notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
    convFrame.pack(fill='both', expand=True)
    notebook.add(convFrame, text="Conveyor Belt")

    ordersFrame=ttk.Frame(notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
    ordersFrame.pack(fill='both', expand=True)
    notebook.add(ordersFrame, text="Orders")

    challengesFrame=ttk.Frame(notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
    challengesFrame.pack(fill='both', expand=True)
    notebook.add(challengesFrame, text="Challenges")

    #Parts frame
    partFlag=tk.StringVar()
    partFlag.set('0')
    agvTrayWidgets(partsFrame, agvTrayWidgetsArr, agvTrayValsArr)
    partsWidgets(partsFrame, partFlag, agv1Quadrants,agv2Quadrants,agv3Quadrants,agv4Quadrants,agvTrayWidgetsArr, agvTrayValsArr,agv1Parts, agv2Parts, agv3Parts, agv4Parts, partOrdCounter)

    #Bins frame
    bin1Slots=[] # holds the available slots for bin1
    bin2Slots=[] # holds the available slots for bin2
    bin3Slots=[] # holds the available slots for bin3
    bin4Slots=[] # holds the available slots for bin4
    bin5Slots=[] # holds the available slots for bin5
    bin6Slots=[] # holds the available slots for bin6
    bin7Slots=[] # holds the available slots for bin7
    bin8Slots=[] # holds the available slots for bin8
    for i in range(9):
        bin1Slots.append(str(i+1))
        bin2Slots.append(str(i+1))
        bin3Slots.append(str(i+1))
        bin4Slots.append(str(i+1))
        bin5Slots.append(str(i+1))
        bin6Slots.append(str(i+1))
        bin7Slots.append(str(i+1))
        bin8Slots.append(str(i+1))
    binWidgets(binFrame,bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots,bins, partOrdCounter)
    
    #Conveyor fame
    convWidgets(convFrame, convParts, partOrdCounter, convSettingsVals)
    
    #Orders frame
    orderWidgets(ordersFrame, orderMSGS,orderConditions, usedIDs, kittingParts, assemblyParts, partOrdCounter,agv1Parts, agv2Parts, agv3Parts, agv4Parts)
    #Challenges frame
    conditionVal=[]
    chooseChallenge(challengesFrame, allChallengeWidgetsArr,presentChallengeWidgets,rmVals,fpVals,dpVals, sbVals,huVals, chCondVals,usedIDs, partOrdCounter, conditionVal)
    allChallengeWidgets(challengesFrame,allChallengeWidgetsArr,presentChallengeWidgets,robotMalfunctions,faultyParts, droppedParts, sensorBlackouts,humanChallenges,rmVals,fpVals, dpVals, sbVals,huVals, chCondVals, challengeCounter, partOrdCounter, orderMSGS,usedIDs, conditionVal)
    
    partOrdText="Parts:\nAGV Parts present:\nNONE\nBin Parts presents:\nNONE\nConveyor Parts present:\nNONE\n\n"
    partOrdText+="Orders Present:\nNONE"
    partOrdLabel=tk.Label(mainFrame, text=partOrdText)
    partOrdLabel.grid(column=2, row=0,sticky=tk.E+tk.W+tk.N+tk.S)

    challengesText="Challenges:\nRobot Malfunction challenges:\nNONE\nFaulty Part challenges:\nNONE\nDropped Part challenges:\nNONE\nSensor Blackout challenges:\nNONE\nHuman challenges:\nNONE\n"
    challengesLabel=tk.Label(mainFrame, text=challengesText)
    challengesLabel.grid(column=3, row=0,sticky=tk.E+tk.W+tk.N+tk.S)
    cancel_main_command=partial(cancel_wind, mainWind, cancelFlag)
    cancelMainButton=tk.Button(mainFrame, text="Cancel and Exit", command=cancel_main_command)
    cancelMainButton.grid(column=1)
    mainSaveButton=tk.Button(mainFrame, text="Save and Exit", command=mainWind.destroy)
    mainSaveButton.grid(column=3, row=1)

    #Trace functions
    update_part_ord_label=partial(updatePartOrdLabel,agv1Parts, agv2Parts, agv3Parts, agv4Parts,bins,convParts, orderMSGS,partOrdLabel)
    partOrdCounter.trace('w',update_part_ord_label)
    update_challenge_label=partial(updateChallengeLabel,robotMalfunctions, faultyParts, droppedParts, sensorBlackouts,humanChallenges,challengesLabel)
    challengeCounter.trace('w', update_challenge_label)
    
    #Formatting
    mainWind.columnconfigure(0, weight=1)
    mainWind.columnconfigure(4, weight=1)
    mainFrame.rowconfigure(0, weight=1)
    mainFrame.columnconfigure(0, weight=1)
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
        
    #gets the kitting trays and slots to write to the file
    KTraysSTR=""
    for i in trayValsMain:
        KTraysSTR+=i.get()
    chosenKTrays=[]
    for i in KTraysSTR:
        if i.isnumeric():
            chosenKTrays.append(i)
    KSlotsSTR=""
    for i in slotValsMain:
        KSlotsSTR+=i.get()
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
        o.write("time_limit: "+timeVar.get()+" # options: -1 (no time limit) or number of seconds\n")
        if len(chosenKTrays)>0:
            o.write("\nkitting_trays: # Which kitting trays will be spawned\n")
            o.write("  tray_ids: ["+", ".join(chosenKTrays)+"]\n")
            o.write("  slots: ["+", ".join(chosenKSlots)+"]\n")
        o.write("\nparts:\n")
        o.write("  agvs:\n")
    if len(agv1Parts)>0:
        writePartsToFile("agv1", agvTrayValsArr[0].get(), agv1Parts, saveFileName)
    if len(agv2Parts)>0:
        writePartsToFile("agv2", agvTrayValsArr[1].get(), agv2Parts, saveFileName)
    if len(agv3Parts)>0:
        writePartsToFile("agv3", agvTrayValsArr[2].get(), agv3Parts, saveFileName)
    if len(agv4Parts)>0:
        writePartsToFile("agv4", agvTrayValsArr[3].get(), agv4Parts, saveFileName)
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
    if len(convParts)>0:
        with open(saveFileName, "a") as o:
            o.write("\n  conveyor_belt: #population params for conveyor belt\n")
            if convSettingsVals[0].get()=="1":
                o.write("    active: true\n")
            else:
                o.write("    active: false\n")
            o.write("    spawn_rate: "+convSettingsVals[1].get()+" # seconds between spawn\n")
            o.write("    order: \'"+convSettingsVals[2].get()+"\' # random or sequential\n")
            if len(convParts)>0:
                o.write("    parts_to_spawn:\n")
                for part in convParts:
                    o.write("      - type: \'"+part.type+"\'")
                    o.write("\n        color: \'"+part.color+"\'")
                    o.write("\n        number: "+part.number)
                    o.write("\n        offset: "+part.offset+" # between -1 and 1")
                    try:
                        val=float(part.rotation)
                    except:
                        val=1
                    if val!=0:
                        o.write("\n        rotation: "+ part.rotation)
        
        #Beginning of order writing to file
    with open(saveFileName, "a") as o:
        counter=0
        o.write("\n\n# ORDER SETUP\n")
        o.write("orders:\n")
        for order in orderMSGS:
            o.write("  - id: \'"+order.id+"\'\n")
            o.write("    type: \'"+getOrderType(order.type)+"\'\n")
            o.write("    announcement:\n")
            if orderConditions[counter].type==0:
                o.write("      time_condition: "+str(orderConditions[counter].time_condition.seconds)+"\n")
            elif orderConditions[counter].type==1:
                o.write("      part_type: \'"+getPartName(orderConditions[counter].part_place_condition.part.type)+"\'\n")
                o.write("      part_color: \'"+getPartColor(orderConditions[counter].part_place_condition.part.color)+"\'\n")
                o.write("      agv: "+str(orderConditions[counter].part_place_condition.agv))
            elif orderConditions[counter].type==2:
                o.write("      submission_condition:\n")
                o.write("        order_id: \'"+orderConditions[counter].submission_condition.order_id+"\'\n")
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
                    if getPartName(assemblyPart.part.type)=="battery":
                        o.write("            xyz: [-0.15, 0.035, 0.043]\n")
                        o.write("            rpy: [0, 0, \'pi/2\']\n")
                        o.write("          assembly_direction: [0, 1, 0]\n")
                    elif getPartName(assemblyPart.part.type)=="pump":
                        o.write("            xyz: [0.14, 0.0, 0.02]\n")
                        o.write("            rpy: [0, 0, \'-pi/2\']\n")
                        o.write("          assembly_direction: [0, 0, -1]\n")
                    elif getPartName(assemblyPart.part.type)=="sensor":
                        o.write("            xyz: [-0.1, 0.395, 0.045]\n")
                        o.write("            rpy: [0, 0, \'-pi/2\']\n")
                        o.write("          assembly_direction: [0, -1, 0]\n")
                    else:
                        o.write("            xyz: [0.175, -0.223, 0.215]\n")
                        o.write("            rpy: [\'pi/2\', 0, \'-pi/2\']\n")
                        o.write("          assembly_direction: [0, 0, -1]\n")
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
        
        #writes challenges to file
        if len(robotMalfunctions)+len(faultyParts)+len(droppedParts)+len(sensorBlackouts)+len(humanChallenges)>0:
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
                    o.write("      agv: "+str(malf.condition.part_place_condition.agv)+"\n")
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
            for human in humanChallenges:
                o.write("  - human:\n")
                o.write("      behavior: \'"+behaviorOptions[human.behavior]+"\'\n")
                if human.condition.type==0:
                    o.write("      time: "+str(human.condition.time_condition.seconds)+"\n")
                elif human.condition.type==1:
                    o.write("      part_type: \'"+getPartName(human.condition.part_place_condition.part.type)+"\'\n")
                    o.write("      part_color: \'"+getPartColor(human.condition.part_place_condition.part.color)+"\'\n")
                    o.write("      agv: "+str(human.condition.part_place_condition.agv))
                elif human.condition.type==2:
                    o.write("      order_id: \'"+human.condition.submission_condition.order_id+"\'\n")
