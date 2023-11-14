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

class ARIAC_GUI(tk.Tk):

    def __init__(self):
        super().__init__()
        self.selected_commands = []
        self.title('FR3 control gui')
        self.resizable(width=False, height=False)
        self.grid_columnconfigure(0, weight=1)
        
        self.yaml_dict = {'time_limit':-1,
                          'parts':{'agvs':
                                   {'agv1':{'tray_id':0,"parts":[]},
                                    'agv2':{'tray_id':0,"parts":[]},
                                    'agv3':{'tray_id':0,"parts":[]},
                                    'agv4':{'tray_id':0,"parts":[]}
                                   },
                                   'bins':
                                   {'bin1':[],'bin2':[],'bin3':[],'bin4':[],'bin5':[],'bin6':[],'bin7':[],'bin8':[]},
                                   'conveyor_belt':
                                   {'active':True,'spawn_rate':3.0,'order':'sequential','parts_to_spawn':[]}
                                   },
                          'challenges':[],
                          'orders':[]}

        self.pathIncrement = []  # gives the full path for recursive deletion
        self.createdDir = []  # to deleted directories made if canceled

        self.trayValsMain=[]
        self.slotValsMain=[]

        self.agv1Parts=[]
        self.agv2Parts=[]
        self.agv3Parts=[]
        self.agv4Parts=[]
        self.agv1Quadrants=["1","2","3","4"] # available quadrants for agv1
        self.agv2Quadrants=["1","2","3","4"] # available quadrants for agv2
        self.agv3Quadrants=["1","2","3","4"] # available quadrants for agv3
        self.agv4Quadrants=["1","2","3","4"] # available quadrants for agv4

        self.bins=[] # holds the bins
        self.bin1Slots=[str(i+1) for i in range(9)] # holds the available slots for bin1
        self.bin2Slots=[str(i+1) for i in range(9)] # holds the available slots for bin2
        self.bin3Slots=[str(i+1) for i in range(9)] # holds the available slots for bin3
        self.bin4Slots=[str(i+1) for i in range(9)] # holds the available slots for bin4
        self.bin5Slots=[str(i+1) for i in range(9)] # holds the available slots for bin5
        self.bin6Slots=[str(i+1) for i in range(9)] # holds the available slots for bin6
        self.bin7Slots=[str(i+1) for i in range(9)] # holds the available slots for bin7
        self.bin8Slots=[str(i+1) for i in range(9)] # holds the available slots for bin8
        self.convParts=[] # holds conveyor belt parts

        self.orderCounter=[] # for counting the number of orders
        self.orderMSGS=[] # holds all order ros2 messages
        self.orderConditions=[] #holds all order condition ros2 messages
        self.usedIDs=[] # holds the ids that have already been used to avoid repeated ids

        self.robotMalfunctions=[] # holds all robot malfunctions
        self.faultyParts=[] # holds all faulty parts
        self.droppedParts=[] # holds all dropped parts
        self.sensorBlackouts=[] # holds all sensor blackouts
        self.humanChallenges=[]
        self.robotsToDisable=[] # holds robots to be disabled
        self.faultyPartQuadrants=[] # holds quadrants for dropped parts
        self.sensorsToDisable=[] # holds sensors for sensor blackout
        self.rmVals=[]
        self.fpVals=[]
        self.dpVals=[]
        self.sbVals=[]
        self.huVals=[]
        self.chCondVals=[]
        self.behaviorOptions=["antagonistic", "indifferent","helpful"]

        self.availableTrays=["Tray 0","Tray 1","Tray 2","Tray 3","Tray 4","Tray 5","Tray 6","Tray 7","Tray 8","Tray 9"] #list of trays to hold available trays for kitting trays
        self.availableSlots=["Slot 1", "Slot 2", "Slot 3", "Slot 4", "Slot 5", "Slot 6"] #list of slots to hold available slots for kitting trays
        self.kittingTrayCounter=[] #holds the number of trays present to avoid overflow in the kitting tray window

        self.presentChallengeWidgets=[]
        self.allChallengeWidgetsArr=[]
        self.agvTrayWidgetsArr = []
        self.agvTrayValsArr = []
        self.kittingParts=[] 
        self.assemblyParts=[]
        self.convSettingsVals=[]

        # window outputs
        self.timeVal="0"
        self.noTimeVal="0"
        self.timeList=[self.timeVal, self.noTimeVal]

        self.trayVals=["" for i in range(6)]
        self.slotVals=["" for i in range(6)]
        
        # END OF GETTING THE NAME OF THE FILE
        # ----------------------------------------------------------------------------------------------
        # START OF MAINWIND
        self.cancelFlag = tk.StringVar()
        self.cancelFlag.set('0')
        self.ordersFlag=tk.StringVar()
        self.ordersFlag.set('0')
        self.challengesFlag=tk.StringVar()
        self.challengesFlag.set('0')
        self.saveMainFlag=tk.StringVar()
        self.saveMainFlag.set('0')
        self.partFlag=tk.StringVar()
        self.partFlag.set('0')

        partOrdCounter=tk.StringVar()
        partOrdCounter.set("0")
        challengeCounter=tk.StringVar()
        challengeCounter.set("0")
        self.geometry(WINDOWMEASURE)
        self.title('Main Window')
        mainFrame=ttk.Frame(self)
        mainFrame.grid(row=0, column=1, columnspan=3, padx=10, pady=10, sticky=tk.E+tk.W+tk.N+tk.S)
        notebook=ttk.Notebook(mainFrame)
        notebook.grid(pady=10, column=LEFTCOLUMN,sticky=tk.E+tk.W+tk.N+tk.S)

        setupFrame = ttk.Frame(notebook, width=800, height=600)
        

        setupFrame.pack(fill='both', expand=True)
        notebook.add(setupFrame, text='Setup')
        self.timeVar=tk.StringVar()
        timeEntry(setupFrame, self.timeVar, self.timeVal[0])
        # add frames to notebook

        #kitting trays
        kittingTrayFrame=ttk.Frame(notebook, width=FRAMEWIDTH, height=FRAMEHEIGHT)
        kittingTrayFrame.pack(fill='both', expand=True)
        notebook.add(kittingTrayFrame, text='Kitting Trays')
        kittingTrayWidgets(kittingTrayFrame, self.kittingTrayCounter, self.availableSlots, self.availableTrays, self.trayVals, self.slotVals,self.trayValsMain, self.slotValsMain)

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
        agvTrayWidgets(partsFrame, self.agvTrayWidgetsArr, self.agvTrayValsArr)
        partsWidgets(partsFrame, partFlag, self.agv1Quadrants,self.agv2Quadrants,self.agv3Quadrants,self.agv4Quadrants,self.agvTrayWidgetsArr, self.agvTrayValsArr,self.agv1Parts, self.agv2Parts, self.agv3Parts, self.agv4Parts, partOrdCounter)

        binWidgets(binFrame,self.bin1Slots,self.bin2Slots,self.bin3Slots,self.bin4Slots,self.bin5Slots,self.bin6Slots,self.bin7Slots,self.bin8Slots,self.bins, partOrdCounter)
        
        #Conveyor fame
        convWidgets(convFrame, self.convParts, partOrdCounter, self.convSettingsVals)
        
        #Orders frame
        orderWidgets(ordersFrame, self.orderMSGS,self.orderConditions, self.usedIDs, self.kittingParts, self.assemblyParts, partOrdCounter,self.agv1Parts, self.agv2Parts, self.agv3Parts, self.agv4Parts)
        #Challenges frame
        conditionVal=[]
        chooseChallenge(challengesFrame, self.allChallengeWidgetsArr,self.presentChallengeWidgets,self.rmVals,self.fpVals,self.dpVals, self.sbVals,self.huVals, self.chCondVals,self.usedIDs, partOrdCounter, conditionVal)
        allChallengeWidgets(challengesFrame,self.allChallengeWidgetsArr,self.presentChallengeWidgets,self.robotMalfunctions,self.faultyParts, self.droppedParts, self.sensorBlackouts,self.humanChallenges,self.rmVals,self.fpVals, self.dpVals, self.sbVals,self.huVals, self.chCondVals, challengeCounter, partOrdCounter, self.orderMSGS,self.usedIDs, conditionVal)
        
        partOrdText="Parts:\nAGV Parts present:\nNONE\nBin Parts presents:\nNONE\nConveyor Parts present:\nNONE\n\n"
        partOrdText+="Orders Present:\nNONE"
        self.partOrdLabel=tk.Label(mainFrame, text=partOrdText)
        self.partOrdLabel.grid(column=2, row=0,sticky=tk.E+tk.W+tk.N+tk.S)

        challengesText="Challenges:\nRobot Malfunction challenges:\nNONE\nFaulty Part challenges:\nNONE\nDropped Part challenges:\nNONE\nSensor Blackout challenges:\nNONE\nHuman challenges:\nNONE\n"
        self.challengesLabel=tk.Label(mainFrame, text=challengesText)
        self.challengesLabel.grid(column=3, row=0,sticky=tk.E+tk.W+tk.N+tk.S)
        cancel_main_command=partial(cancel_wind, self, self.cancelFlag)
        cancelMainButton=tk.Button(mainFrame, text="Cancel and Exit", command=cancel_main_command)
        cancelMainButton.grid(column=1)
        mainSaveButton=tk.Button(mainFrame, text="Save and Exit", command=self.destroy)
        mainSaveButton.grid(column=3, row=1)

        #Trace functions
        partOrdCounter.trace('w',self.updatePartOrdLabel)
        challengeCounter.trace('w', self.updateChallengeLabel)
        
        #Formatting
        self.columnconfigure(0, weight=1)
        self.columnconfigure(4, weight=1)
        mainFrame.rowconfigure(0, weight=1)
        mainFrame.columnconfigure(0, weight=1)
        
    def updatePartOrdLabel(self,_,__,___):
        newText="Parts:\nAGV Parts present:\n"
        for part in self.agv1Parts:
            newText+=part.color+" "+part.pType+" on AGV 1\n"
        for part in self.agv2Parts:
            newText+=part.color+" "+part.pType+" on AGV 2\n"
        for part in self.agv3Parts:
            newText+=part.color+" "+part.pType+" on AGV 3\n"
        for part in self.agv4Parts:
            newText+=part.color+" "+part.pType+" on AGV 4\n"
        if len(self.agv1Parts)+len(self.agv2Parts)+len(self.agv3Parts)+len(self.agv4Parts)==0:
            newText+="NONE\n"
        
        newText+="\nBin Parts present:\n"
        for bin in self.bins:
            newText+=bin.binName+" "+bin.color+" "+bin.type+"\n"
        if len(self.bins)==0:
            newText+="NONE\n"
        
        newText+="\nConveyor Parts present:\n"
        for convPart in self.convParts:
            newText+=convPart.color+" "+convPart.type+"\n"
        if len(self.convParts)==0:
            newText+="NONE\n"
        newText+="\nOrders Present:\n"
        if len(self.orderMSGS)==0:
            newText+="NONE"
        else:
            counter=0
            for order in self.orderMSGS:
                newText+="Order "+order.id+":\n"
                newText+="Type: "+self.getOrderType(order.type)+"\n"
                newText+="Number of parts: "
                if order.type==0:
                    newText+=str(len(order.kitting_task.parts))
                elif order.type==1:
                    newText+=str(len(order.assembly_task.parts))
                else:
                    newText+=str(len(order.combined_task.parts))
                newText+="\n"
                counter+=1
        self.partOrdLabel.config(text=newText)

    def updateChallengeLabel(self,_,__,___):
        newText="Challenges:\nRobot Malfunction challenges:\n"
        for rm in self.robotMalfunctions:
            robotsToDisable=[]
            newText+="Robots: "
            if rm.robots_to_disable.floor_robot:
                robotsToDisable.append("\'floor_robot\'")
            if rm.robots_to_disable.ceiling_robot:
                robotsToDisable.append("\'ceiling_robot\'")
            newText+=",".join(robotsToDisable)
            newText+=" Duration: "+str(rm.duration)+"\n"
        if len(self.robotMalfunctions)==0:
            newText+="NONE\n"
        newText+="\nFaulty Part challenges:\n"
        for fp in self.faultyParts:
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
        if len(self.faultyParts)==0:
            newText+="NONE\n"
        newText+="\nDropped Part challenges:\n"
        for dp in self.droppedParts:
            newText+=dp.robot+" type: \'"+self.getPartName(dp.part_to_drop.type)+" color: \'"+self.getPartColor(dp.part_to_drop.color)+"\n"
        if len(self.droppedParts)==0:
            newText+="NONE\n"
        newText+="\nSensor Blackout challenges:\n"
        for sb in self.sensorBlackouts:
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
        if len(self.sensorBlackouts)==0:
            newText+="NONE\n"
        behaviorOptions=["antagonistic", "indifferent","helpful"]
        newText+="\nHuman challenges:\n"
        for ch in self.humanChallenges:
            newText+="Behavior: "+behaviorOptions[ch.behavior]+"\n"
        if len(self.humanChallenges)==0:
            newText+="NONE\n"
        self.challengesLabel.config(text=newText)


          
        def pack_and_append(self, widget, pady=5, side = tk.TOP):
            widget.pack(pady=pady,side=side)
            self.current_widgets.append(widget)