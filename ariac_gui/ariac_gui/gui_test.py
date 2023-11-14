from os import chdir
from functools import partial
from PIL import Image, ImageTk  # needed for images in gui
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import customtkinter as ctk
from ariac_gui.fileFunc import *
from ariac_gui.timeFunctions import *
from ariac_gui.gui_class import ARIAC_GUI
from ariac_gui.msgNames import *
from ariac_gui.notebookParts import writePartsToFile
from ariac_gui.notebookBins import writeBinsToFile

FRAMEWIDTH=1000
FRAMEHEIGHT=750
WINDOWMEASURE="900x800"
LEFTCOLUMN=1

def runGUI(): # runs the entire gui
    nameLabels = []  # holds temporary flags to be deleted

    getFileName = ctk.CTk() #window to create and get the file
    getFileName.geometry(WINDOWMEASURE)
    getFileName.title("NIST ARIAC CONFIG GUI")
    
    fileName = tk.StringVar()
    fileName.set("")
    cancelFlag = tk.StringVar()
    cancelFlag.set('0')
    invalidFlag = tk.StringVar()
    invalidFlag.set('0')
    reqFlag = tk.StringVar()
    reqFlag.set("0")
    existFlag = tk.StringVar()
    existFlag.set("0")

    frame = tk.Frame(getFileName)
    #getFileName.geometry("850x600")
    frame.pack()
    pkg_share = get_package_share_directory('ariac_gui')
    nistLogo = ctk.CTkImage(Image.open(pkg_share + "/resource/NIST_logo.png"),size=(500,89))
    logoImgLabel = ctk.CTkLabel(frame, text="", image=nistLogo)
    logoImgLabel.pack(pady=40)
    
    fileNameVar = ctk.StringVar()
    fileNameCorrectFunc = partial(correct_file_name, fileName)
    saveAndExit = partial(make_file, getFileName, fileNameVar)
    openFileExp = ctk.CTkButton(getFileName, text="Create file", command=saveAndExit)
    openFileExp.pack()
    cancel_file = partial(cancel_wind, getFileName, cancelFlag)
    cancelFile = ctk.CTkButton(getFileName, text="Cancel and Exit", command=cancel_file)
    cancelFile.pack(side=ctk.BOTTOM, pady=20)
    fileFunc=partial(get_file_name_next, fileName, invalidFlag, nameLabels, getFileName, reqFlag, existFlag)
    fileExit = ctk.CTkButton(getFileName, text="Next", command=fileFunc)
    fileExit.pack(side=ctk.BOTTOM, pady=20)
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
    save_file_name=fileNameStr
    fileName.set(save_file_name)
    
    main_gui = ARIAC_GUI()
    main_gui.mainloop()

    check_cancel(main_gui.cancelFlag.get(), main_gui.pathIncrement, fileName, main_gui.createdDir)

    binPresentFlags=[0 for i in range(8)]
    #Finds which bins are present
    for bin in main_gui.bins:
        if bin.binName=="bin1":
            binPresentFlags[0]=1
        if bin.binName=="bin2":
            binPresentFlags[1]=1
        if bin.binName=="bin3":
            binPresentFlags[2]=1
        if bin.binName=="bin4":
            binPresentFlags[3]=1
        if bin.binName=="bin5":
            binPresentFlags[4]=1
        if bin.binName=="bin6":
            binPresentFlags[5]=1
        if bin.binName=="bin7":
            binPresentFlags[6]=1
        if bin.binName=="bin8":
            binPresentFlags[7]=1
        
    #gets the kitting trays and slots to write to the file
    KTraysSTR=""
    for i in main_gui.trayValsMain:
        KTraysSTR+=i.get()
    chosenKTrays=[]
    for i in KTraysSTR:
        if i.isnumeric():
            chosenKTrays.append(i)
    KSlotsSTR=""
    for i in main_gui.slotValsMain:
        KSlotsSTR+=i.get()
    chosenKSlots=[]
    for i in KSlotsSTR:
        if i.isnumeric():
            chosenKSlots.append(i)

    #  WRITE TO FILE
    with open(save_file_name, "a") as o:
        o.write("# Trial Name: "+save_file_name+"\n")
        o.write("# ARIAC2023\n")
        o.write("# "+datetime.now().strftime("%Y-%m-%d %H:%M:%S")+"\n\n") #writes the time and date
        o.write("# ENVIRONMENT SETUP\n")
        o.write("time_limit: "+main_gui.timeVar.get()+" # options: -1 (no time limit) or number of seconds\n")
        if len(chosenKTrays)>0:
            o.write("\nkitting_trays: # Which kitting trays will be spawned\n")
            o.write("  tray_ids: ["+", ".join(chosenKTrays)+"]\n")
            o.write("  slots: ["+", ".join(chosenKSlots)+"]\n")
        o.write("\nparts:\n")
        if len(main_gui.agv1Parts)+len(main_gui.agv2Parts)+len(main_gui.agv3Parts)+len(main_gui.agv4Parts)>0:
            o.write("  agvs:\n")
    if len(main_gui.agv1Parts)>0:
        writePartsToFile("agv1", main_gui.agvTrayValsArr[0].get(), main_gui.agv1Parts, save_file_name)
    if len(main_gui.agv2Parts)>0:
        writePartsToFile("agv2", main_gui.agvTrayValsArr[1].get(), main_gui.agv2Parts, save_file_name)
    if len(main_gui.agv3Parts)>0:
        writePartsToFile("agv3", main_gui.agvTrayValsArr[2].get(), main_gui.agv3Parts, save_file_name)
    if len(main_gui.agv4Parts)>0:
        writePartsToFile("agv4", main_gui.agvTrayValsArr[3].get(), main_gui.agv4Parts, save_file_name)
    with open(save_file_name, "a") as o:
        if len(main_gui.bins)>0:
            o.write("\n  bins: # bin params - 8 total bins each bin has nine total slots (1-9)\n")
    if binPresentFlags[0]==1:
        writeBinsToFile("bin1", main_gui.bins, save_file_name)
    if binPresentFlags[1]==1:
        writeBinsToFile("bin2", main_gui.bins, save_file_name)
    if binPresentFlags[2]==1:
        writeBinsToFile("bin3", main_gui.bins, save_file_name)
    if binPresentFlags[3]==1:
        writeBinsToFile("bin4", main_gui.bins, save_file_name)
    if binPresentFlags[4]==1:
        writeBinsToFile("bin5", main_gui.bins, save_file_name)
    if binPresentFlags[5]==1:
        writeBinsToFile("bin6", main_gui.bins, save_file_name)
    if binPresentFlags[6]==1:
        writeBinsToFile("bin7", main_gui.bins, save_file_name)
    if binPresentFlags[7]==1:
        writeBinsToFile("bin8", main_gui.bins, save_file_name)
    if len(main_gui.convParts)>0:
        with open(save_file_name, "a") as o:
            o.write("\n  conveyor_belt: #population params for conveyor belt\n")
            if main_gui.convSettingsVals[0].get()=="1":
                o.write("    active: true\n")
            else:
                o.write("    active: false\n")
            o.write("    spawn_rate: "+main_gui.convSettingsVals[1].get()+" # seconds between spawn\n")
            o.write("    order: \'"+main_gui.convSettingsVals[2].get()+"\' # random or sequential\n")
            if len(main_gui.convParts)>0:
                o.write("    parts_to_spawn:\n")
                for part in main_gui.convParts:
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
    with open(save_file_name, "a") as o:
        counter=0
        o.write("\n\n# ORDER SETUP\n")
        if len(main_gui.orderMSGS)>0:
            o.write("orders:\n")
        for order in main_gui.orderMSGS:
            o.write("  - id: \'"+order.id+"\'\n")
            o.write("    type: \'"+getOrderType(order.type)+"\'\n")
            o.write("    announcement:\n")
            if main_gui.orderConditions[counter].type==0:
                o.write("      time_condition: "+str(main_gui.orderConditions[counter].time_condition.seconds)+"\n")
            elif main_gui.orderConditions[counter].type==1:
                o.write("      part_type: \'"+getPartName(main_gui.orderConditions[counter].part_place_condition.part.type)+"\'\n")
                o.write("      part_color: \'"+getPartColor(main_gui.orderConditions[counter].part_place_condition.part.color)+"\'\n")
                o.write("      agv: "+str(main_gui.orderConditions[counter].part_place_condition.agv))
            elif main_gui.orderConditions[counter].type==2:
                o.write("      submission_condition:\n")
                o.write("        order_id: \'"+main_gui.orderConditions[counter].submission_condition.order_id+"\'\n")
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
        if len(main_gui.robotMalfunctions)+len(main_gui.faultyParts)+len(main_gui.droppedParts)+len(main_gui.sensorBlackouts)+len(main_gui.humanChallenges)>0:
            o.write("\n# GLOBAL CHALLENGES\n")
            o.write("challenges:\n")
            #robot malfunctions
            for malf in main_gui.robotMalfunctions:
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
            for part in main_gui.faultyParts:
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
            for part in main_gui.droppedParts:
                o.write("  - dropped_part:\n")
                o.write("      robot: \'"+part.robot+"\'\n")
                o.write("      type: \'"+getPartName(part.part_to_drop.type)+"\'\n")
                o.write("      color: \'"+getPartColor(part.part_to_drop.color)+"\'\n")
                o.write("      drop_after_num: "+str(part.drop_after_num)+" # first part the robot successfully picks\n")
                o.write("      drop_after_time: "+str(part.drop_after_time)+" # secons\n")
            #sensor blackouts
            for blackout in main_gui.sensorBlackouts:
                o.write("  - sensor_blackout:\n")
                o.write("    duration: "+str(blackout.duration)+"\n")
                #gets the list of sensors to disable
                if blackout.sensors_to_disable.break_beam:
                    main_gui.sensorsToDisable.append("break_beam")
                if blackout.sensors_to_disable.proximity:
                    main_gui.sensorsToDisable.append("proximity")
                if blackout.sensors_to_disable.laser_profiler:
                    main_gui.sensorsToDisable.append("laser_profiler")
                if blackout.sensors_to_disable.lidar:
                    main_gui.sensorsToDisable.append("lidar")
                if blackout.sensors_to_disable.camera:
                    main_gui.sensorsToDisable.append("camera")
                if blackout.sensors_to_disable.logical_camera:
                    main_gui.sensorsToDisable.append("logical_camera")
                o.write("    sensors_to_disable: ["+", ".join(main_gui.sensorsToDisable)+"]\n")
                if blackout.condition.type==0:
                    o.write("      time: "+str(blackout.condition.time_condition.seconds)+"\n")
                elif blackout.condition.type==1:
                    o.write("      part_type: \'"+getPartName(blackout.condition.part_place_condition.part.type)+"\'\n")
                    o.write("      part_color: \'"+getPartColor(blackout.condition.part_place_condition.part.color)+"\'\n")
                    o.write("      agv: "+str(blackout.condition.part_place_condition.agv))
                elif blackout.condition.type==2:
                    o.write("      order_id: \'"+blackout.condition.submission_condition.order_id+"\'\n")
            for human in main_gui.humanChallenges:
                o.write("  - human:\n")
                o.write("      behavior: \'"+main_gui.behaviorOptions[human.behavior]+"\'\n")
                if human.condition.type==0:
                    o.write("      time: "+str(human.condition.time_condition.seconds)+"\n")
                elif human.condition.type==1:
                    o.write("      part_type: \'"+getPartName(human.condition.part_place_condition.part.type)+"\'\n")
                    o.write("      part_color: \'"+getPartColor(human.condition.part_place_condition.part.color)+"\'\n")
                    o.write("      agv: "+str(human.condition.part_place_condition.agv))
                elif human.condition.type==2:
                    o.write("      order_id: \'"+human.condition.submission_condition.order_id+"\'\n")