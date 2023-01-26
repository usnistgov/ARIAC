import tkinter as tk
from functools import partial
from ariac_gui.checkCancel import *
from ariac_gui.newClasses import *
from ariac_gui.timeFunctions import validateTime
from ariac_gui.validationFunctions import *
from ariac_msgs.msg import *

agvOptions=["1","2","3","4"]
allPartTypes=["sensor", "pump", "regulator", "battery"]
allPartColors=['green', 'red', 'purple','blue','orange']
robotTypes=["ceiling_robot","floor_robot"]
destinations=["warehouse", "as1", "as2","as3","as4","kitting"]
stations=["as1","as2","as3","as4"]
sensBOCategories=["time-based","during kitting", "during assembly","after kitting", "after assembly"]
conditionTypes=['','time','partPlace','submission']

def showCorrectMenu(condition, conditionMenu, time, timeLabel, timeEntry, agv, agvLabel, agvMenu, partType, partTypeLabel, partTypeMenu, partColor, partColorLabel, partColorMenu, annID, annIDLabel, annIDMenu,tempIDs,a,b,c):
    if condition.get()=="":
        time.set('')
        agv.set('')
        partType.set('')
        partColor.set('')
        annID.set('')
        timeLabel.pack_forget()
        timeEntry.pack_forget()
        agvLabel.pack_forget()
        agvMenu.pack_forget()
        partTypeLabel.pack_forget()
        partTypeMenu.pack_forget()
        partColorLabel.pack_forget()
        partColorMenu.pack_forget()
        annIDLabel.pack_forget()
        annIDMenu.pack_forget()
    elif condition.get()==conditionTypes[1]:
        timeLabel.pack(after=conditionMenu)
        timeEntry.pack(after=timeLabel)
        time.set('0')
        agv.set('')
        partType.set('')
        partColor.set('')
        annID.set('')
        agvLabel.pack_forget()
        agvMenu.pack_forget()
        partTypeLabel.pack_forget()
        partTypeMenu.pack_forget()
        partColorLabel.pack_forget()
        partColorMenu.pack_forget()
        annIDLabel.pack_forget()
        annIDMenu.pack_forget()
    elif condition.get()==conditionTypes[2]:
        agvLabel.pack(after=conditionMenu)
        agvMenu.pack(after=agvLabel)
        agv.set(agvOptions[0])
        partTypeLabel.pack(after=agvMenu)
        partTypeMenu.pack(after=partTypeLabel)
        partColorLabel.pack(after=partTypeMenu)
        partColorMenu.pack(after=partColorLabel)
        partType.set(allPartTypes[0])
        partColor.set(allPartColors[0])
        time.set('')
        timeLabel.pack_forget()
        timeEntry.pack_forget()
        annID.set('')
        annIDLabel.pack_forget()
        annIDMenu.pack_forget()
    else:
        annIDLabel.pack(after=conditionMenu)
        annIDMenu.pack(after=annIDLabel)
        annID.set(tempIDs[0])
        time.set('')
        agv.set('')
        partType.set('')
        partColor.set('')
        timeLabel.pack_forget()
        timeEntry.pack_forget()
        agvLabel.pack_forget()
        agvMenu.pack_forget()
        partTypeLabel.pack_forget()
        partTypeMenu.pack_forget()
        partColorLabel.pack_forget()
        partColorMenu.pack_forget()

def newRobotMalfunction(robotMalfunctions, usedIds):
    robotMalfunctionWind=tk.Toplevel()
    #robotMalfunctionWind.geometry("850x600")
    robotMalfunctionWind.attributes('-fullscreen', True)
    #duration
    duration=tk.StringVar()
    duration.set("0")
    durationLabel=tk.Label(robotMalfunctionWind, text="Enter the duration of the robot malfunction")
    durationLabel.pack()
    durationEntry=tk.Entry(robotMalfunctionWind, textvariable=duration)
    durationEntry.pack()
    #robots to disable
    floorRobot=tk.StringVar()
    floorRobot.set("0")
    ceilRobot=tk.StringVar()
    ceilRobot.set("0")
    floorRobotCB=tk.Checkbutton(robotMalfunctionWind, text="Floor robot", variable=floorRobot, onvalue="1", offvalue="0", height=1, width=20)
    floorRobotCB.pack()
    ceilRobotCB=tk.Checkbutton(robotMalfunctionWind, text="Ceiling robot", variable=ceilRobot, onvalue="1", offvalue="0", height=1, width=20)
    ceilRobotCB.pack()
    #condition
    condition=tk.StringVar()
    condition.set(conditionTypes[0])
    conditionLabel=tk.Label(robotMalfunctionWind, text="Select a condition for the order")
    conditionLabel.pack()
    conditionMenu=tk.OptionMenu(robotMalfunctionWind, condition, *conditionTypes)
    conditionMenu.pack()
    time=tk.StringVar()
    time.set('')
    timeLabel=tk.Label(robotMalfunctionWind, text="Enter the time")
    timeLabel.pack_forget()
    timeEntry=tk.Entry(robotMalfunctionWind, textvariable=time)
    timeEntry.pack_forget()
    agv=tk.StringVar()
    agv.set("")
    agvLabel=tk.Label(robotMalfunctionWind, text="Choose the agv")
    agvLabel.pack_forget()
    agvMenu=tk.OptionMenu(robotMalfunctionWind, agv, *agvOptions)
    agvMenu.pack_forget()
    partType=tk.StringVar()
    partType.set("")
    partTypeLabel=tk.Label(robotMalfunctionWind, text="Select the type of part")
    partTypeLabel.pack_forget()
    partTypeMenu=tk.OptionMenu(robotMalfunctionWind, partType, *allPartTypes)
    partTypeMenu.pack_forget()
    partColor=tk.StringVar()
    partColor.set("")
    partColorLabel=tk.Label(robotMalfunctionWind, text="Select the color of the part")
    partColorLabel.pack_forget()
    partColorMenu=tk.OptionMenu(robotMalfunctionWind, partColor, *allPartColors)
    partColorMenu.pack_forget()
    annID=tk.StringVar()
    annID.set("")
    annIDLabel=tk.Label(robotMalfunctionWind, text="Select the order ID")
    annIDLabel.pack_forget()
    annIDMenu=tk.OptionMenu(robotMalfunctionWind, annID, *usedIds)
    annIDMenu.pack_forget()
    #save and cancel buttons
    robotMalfSave=tk.Button(robotMalfunctionWind, text="Save", command=robotMalfunctionWind.destroy)
    robotMalfSave.pack(pady=20)
    robotMalfCancelFlag=tk.StringVar()
    robotMalfCancelFlag.set('0')
    cancel_robot_malf_challenge=partial(exitAndFlag, robotMalfunctionWind, robotMalfCancelFlag)
    robotMalfCancel=tk.Button(robotMalfunctionWind, text="Cancel", command=cancel_robot_malf_challenge)
    robotMalfCancel.pack()
    validate_duration=partial(validateTime, duration)
    duration.trace('w', validate_duration)
    updateConditionMenu=partial(showCorrectMenu,condition, conditionMenu, time, timeLabel, timeEntry, agv, agvLabel, agvMenu, partType, partTypeLabel, partTypeMenu, partColor, partColorLabel, partColorMenu, annID, annIDLabel, annIDMenu,usedIds)
    condition.trace('w', updateConditionMenu)
    robotMalfunctionWind.mainloop()
    if robotMalfCancelFlag.get()=="0":
        bothRobots=Robots()
        if floorRobot.get()=="1":
            bothRobots.floor_robot=True
        else:
            bothRobots.floor_robot=False
        if ceilRobot.get()=="1":
            bothRobots.ceiling_robot=True
        else:
            bothRobots.ceiling_robot=False
        newRobotMalf=RobotMalfunctionChallenge()
        newRobotMalf.robots_to_disable=bothRobots
        newRobotMalf.duration=float(duration.get())
        newRobotMalfCondition=Condition()
        newRobotMalfCondition.type=conditionTypes.index(condition.get())-1
        if condition.get()==conditionTypes[1]:
            newRobotMalfCondition.time_condition.seconds=float(time.get())
        elif condition.get()==conditionTypes[2]:
            newPart=Part()
            if partType.get()=="sensor":
                newPart.type=newPart.SENSOR
            elif partType.get()=="pump":
                newPart.type=newPart.PUMP
            elif partType.get()=="battery":
                newPart.type=newPart.BATTERY
            else:
                newPart.type=newPart.REGULATOR
            if partColor.get()=="red":
                newPart.color=newPart.RED
            elif partColor.get()=="green":
                newPart.color=newPart.GREEN
            elif partColor.get()=="blue":
                newPart.color=newPart.BLUE
            elif partColor.get()=="orange":
                newPart.color=newPart.ORANGE
            else:
                newPart.color=newPart.PURPLE
            newRobotMalfCondition.part_place_condition.part=newPart
            newRobotMalfCondition.part_place_condition.agv=int(agv.get())
        elif condition.get()==conditionTypes[2]:
            newRobotMalfCondition.submission_condition.order_id=annID.get()
        newRobotMalf.condition=newRobotMalfCondition
        robotMalfunctions.append(newRobotMalf)
    
def newFaultyPart(faultyParts, usedIds):
    faultyPartWind=tk.Toplevel()
    faultyPartWind.attributes('-fullscreen', True)
    #choose order
    currentOrderID=tk.StringVar()
    currentOrderID.set(usedIds[0])
    orderIDLabel=tk.Label(faultyPartWind, text="Select the order ID for the faulty part")
    orderIDLabel.pack()
    orderIDMenu=tk.OptionMenu(faultyPartWind, currentOrderID, *usedIds)
    orderIDMenu.pack()
    #quadrants
    q1=tk.StringVar()
    q1.set('0')
    q2=tk.StringVar()
    q2.set('0')
    q3=tk.StringVar()
    q3.set('0')
    q4=tk.StringVar()
    q4.set('0')
    q1CB=tk.Checkbutton(faultyPartWind, text="Quadrant 1", variable=q1, onvalue="1", offvalue="0", height=1, width=20)
    q1CB.pack()
    q2CB=tk.Checkbutton(faultyPartWind, text="Quadrant 2", variable=q2, onvalue="1", offvalue="0", height=1, width=20)
    q2CB.pack()
    q3CB=tk.Checkbutton(faultyPartWind, text="Quadrant 3", variable=q3, onvalue="1", offvalue="0", height=1, width=20)
    q3CB.pack()
    q4CB=tk.Checkbutton(faultyPartWind, text="Quadrant 4", variable=q4, onvalue="1", offvalue="0", height=1, width=20)
    q4CB.pack()
    #save and cancel buttons
    faultyPartSave=tk.Button(faultyPartWind, text="Save", command=faultyPartWind.destroy)
    faultyPartSave.pack(pady=20)
    faultyPartCancelFlag=tk.StringVar()
    faultyPartCancelFlag.set('0')
    cancel_faulty_part_challenge=partial(exitAndFlag, faultyPartWind, faultyPartCancelFlag)
    faultyPartCancel=tk.Button(faultyPartWind, text="Cancel", command=cancel_faulty_part_challenge)
    faultyPartCancel.pack()
    faultyPartWind.mainloop()
    if faultyPartCancelFlag.get()=="0":
        faultyPartMSG=FaultyPartChallenge()
        faultyPartMSG.order_id=currentOrderID.get()
        if q1.get()=="1":
            faultyPartMSG.quadrant1 = True
        else:
            faultyPartMSG.quadrant1 = False
        if q2.get()=="1":
            faultyPartMSG.quadrant2 = True
        else:
            faultyPartMSG.quadrant2 = False
        if q3.get()=="1":
            faultyPartMSG.quadrant3 = True
        else:
            faultyPartMSG.quadrant3 = False
        if q4.get()=="1":
            faultyPartMSG.quadrant4 = True
        else:
            faultyPartMSG.quadrant4 = False
        faultyParts.append(faultyPartMSG)

def newDroppedPart(droppedParts):
    dropPartWind=tk.Toplevel()
    dropPartWind.attributes('-fullscreen', True)
    #robot types
    robotType=tk.StringVar()
    robotType.set(robotTypes[0])
    robotTypeLabel=tk.Label(dropPartWind, text="Select the robot type for the dropped part")
    robotTypeLabel.pack()
    robotTypeMenu=tk.OptionMenu(dropPartWind, robotType, *robotTypes)
    robotTypeMenu.pack()
    #part type
    partType=tk.StringVar()
    partType.set(allPartTypes[0])
    partTypeLabel=tk.Label(dropPartWind, text="Select the type of part")
    partTypeLabel.pack()
    partTypeMenu=tk.OptionMenu(dropPartWind, partType, *allPartTypes)
    partTypeMenu.pack()
    #part color
    partColor=tk.StringVar()
    partColor.set(allPartColors[0])
    partColorLabel=tk.Label(dropPartWind, text="Select the color of the part")
    partColorLabel.pack()
    partColorMenu=tk.OptionMenu(dropPartWind, partColor, *allPartColors)
    partColorMenu.pack()
    #drop after
    dropAfterNum=tk.StringVar()
    dropAfterNum.set("0")
    dropAfterNumLabel=tk.Label(dropPartWind, text="Set the number to drop the part after")
    dropAfterNumLabel.pack()
    dropAfterNumEntry=tk.Entry(dropPartWind, textvariable=dropAfterNum)
    dropAfterNumEntry.pack()
    #delay
    dropAfterTime=tk.StringVar()
    dropAfterTime.set('0')
    dropAfterTimeLabel=tk.Label(dropPartWind,text="Set the time to drop the part after")
    dropAfterTimeLabel.pack()
    dropAfterTimeEntry=tk.Entry(dropPartWind, textvariable=dropAfterTime)
    dropAfterTimeEntry.pack()
    #save and cancel buttons
    dropPartSave=tk.Button(dropPartWind, text="Save", command=dropPartWind.destroy)
    dropPartSave.pack(pady=20)
    dropPartCancelFlag=tk.StringVar()
    dropPartCancelFlag.set('0')
    cancel_drop_part_challenge=partial(exitAndFlag, dropPartWind, dropPartCancelFlag)
    dropPartCancel=tk.Button(dropPartWind, text="Cancel", command=cancel_drop_part_challenge)
    dropPartCancel.pack()
    #validation
    validate_drop_after=partial(require_num,dropAfterNum)
    dropAfterNum.trace('w',validate_drop_after)
    validate_delay=partial(require_num, dropAfterTime)
    dropAfterTime.trace('w', validate_delay)
    dropPartWind.mainloop()
    if dropPartCancelFlag.get()=="0":
        droppedPartMSG=DroppedPartChallenge()
        droppedPartMSG.robot=robotType.get()
        partToDrop=Part()
        if partType.get()=="sensor":
            partToDrop.type=partToDrop.SENSOR
        elif partType.get()=="pump":
            partToDrop.type=partToDrop.PUMP
        elif partType.get()=="battery":
            partToDrop.type=partToDrop.BATTERY
        else:
            partToDrop.type=partToDrop.REGULATOR
        if partColor.get()=="red":
            partToDrop.color=partToDrop.RED
        elif partColor.get()=="green":
            partToDrop.color=partToDrop.GREEN
        elif partColor.get()=="blue":
            partToDrop.color=partToDrop.BLUE
        elif partColor.get()=="orange":
            partToDrop.color=partToDrop.ORANGE
        else:
            partToDrop.color=partToDrop.PURPLE
        droppedPartMSG.part_to_drop=partToDrop
        droppedPartMSG.drop_after_num=int(dropAfterNum.get())
        droppedPartMSG.drop_after_time=float(dropAfterTime.get())
        droppedParts.append(droppedPartMSG)
        #droppedParts.append(DroppedPart(robotType.get(), partType.get(), partColor.get(), dropAfter.get(), delayVal.get()))

def showAGVMenu(agvShow,agvShowCB, agvMenu, agvLabel, agv, a,b,c):
    if agvShow.get()=="1":
        agvLabel.pack(after=agvShowCB)
        agvMenu.pack(after=agvLabel)
        agv.set(agvOptions[0])
    else:
        agvLabel.pack_forget()
        agvMenu.pack_forget()
        agv.set('')

def showDestMenu(destShow, destShowCB, destMenu, destLabel, destination,a,b,c):
    if destShow.get()=="1":
        destLabel.pack(after=destShowCB)
        destMenu.pack(after=destLabel)
        destination.set(destinations[0])
    else:
        destLabel.pack_forget()
        destMenu.pack_forget()
        destination.set('')

def showStatMenu(statShow, statShowCB, statMenu, statLabel, station, a,b,c):
    if statShow.get()=="1":
        statLabel.pack(after=statShowCB)
        statMenu.pack(after=statLabel)
        station.set(stations[0])
    else:
        statLabel.pack_forget()
        statMenu.pack_forget()
        station.set('')

def showTimeMenu(timeShow, timeShowCB, timeEntry, timeLabel, time, a,b,c):
    if timeShow.get()=="1":
        timeLabel.pack(after=timeShowCB)
        timeEntry.pack(after=timeLabel)
        time.set('0')
    else:
        timeLabel.pack_forget()
        timeEntry.pack_forget()
        time.set('')

def showPartMenu(partShow, partShowCB, partTypeLabel, partTypeMenu, partColorLabel, partColorMenu,partType, partColor,a,b,c):
    if partShow.get()=="1":
        partTypeLabel.pack(after=partShowCB)
        partTypeMenu.pack(after=partTypeLabel)
        partColorLabel.pack(after=partTypeMenu)
        partColorMenu.pack(after=partColorLabel)
        partType.set(allPartTypes[0])
        partColor.set(allPartColors[0])
    else:
        partTypeLabel.pack_forget()
        partTypeMenu.pack_forget()
        partColorLabel.pack_forget()
        partColorMenu.pack_forget()
        partType.set("")
        partColor.set("")

def newSensorBlackout(sensorBlackouts, usedIds):
    sensBOWind=tk.Toplevel()
    sensBOWind.attributes('-fullscreen', True)
    #category
    category=tk.StringVar()
    category.set(sensBOCategories[0])
    categoryLabel=tk.Label(sensBOWind, text="Choose the category for the sensor blackout")
    categoryLabel.pack()
    categoryMenu=tk.OptionMenu(sensBOWind, category, *sensBOCategories)
    categoryMenu.pack()
    #duration
    duration=tk.StringVar()
    duration.set('0')
    durationLabel=tk.Label(sensBOWind, text="Enter the duration for the sensor blackout")
    durationLabel.pack()
    durationEntry=tk.Entry(sensBOWind, textvariable=duration)
    durationEntry.pack()
    #sensors to disable
    sensor1=tk.StringVar()
    sensor2=tk.StringVar()
    sensor3=tk.StringVar()
    sensor4=tk.StringVar()
    sensor5=tk.StringVar()
    sensor6=tk.StringVar()
    sensor1.set('0')
    sensor2.set('0')
    sensor3.set('0')
    sensor4.set('0')
    sensor5.set('0')
    sensor6.set('0')
    sensor1CB=tk.Checkbutton(sensBOWind, text="break beam", variable=sensor1, onvalue="1", offvalue="0", height=1, width=20)
    sensor1CB.pack()
    sensor2CB=tk.Checkbutton(sensBOWind, text="proximity", variable=sensor2, onvalue="1", offvalue="0", height=1, width=20)
    sensor2CB.pack()
    sensor3CB=tk.Checkbutton(sensBOWind, text="laser profiler", variable=sensor3, onvalue="1", offvalue="0", height=1, width=20)
    sensor3CB.pack()
    sensor4CB=tk.Checkbutton(sensBOWind, text="lidar", variable=sensor4, onvalue="1", offvalue="0", height=1, width=20)
    sensor4CB.pack()
    sensor5CB=tk.Checkbutton(sensBOWind, text="camera", variable=sensor5, onvalue="1", offvalue="0", height=1, width=20)
    sensor5CB.pack()
    sensor6CB=tk.Checkbutton(sensBOWind, text="logical camera", variable=sensor6, onvalue="1", offvalue="0", height=1, width=20)
    sensor6CB.pack()
    #optional items
    #condition
    condition=tk.StringVar()
    condition.set(conditionTypes[0])
    conditionLabel=tk.Label(sensBOWind, text="Select a condition for the order")
    conditionLabel.pack()
    conditionMenu=tk.OptionMenu(sensBOWind, condition, *conditionTypes)
    conditionMenu.pack()
    time=tk.StringVar()
    time.set('')
    timeLabel=tk.Label(sensBOWind, text="Enter the time")
    timeLabel.pack_forget()
    timeEntry=tk.Entry(sensBOWind, textvariable=time)
    timeEntry.pack_forget()
    agv=tk.StringVar()
    agv.set("")
    agvLabel=tk.Label(sensBOWind, text="Choose the agv")
    agvLabel.pack_forget()
    agvMenu=tk.OptionMenu(sensBOWind, agv, *agvOptions)
    agvMenu.pack_forget()
    partType=tk.StringVar()
    partType.set("")
    partTypeLabel=tk.Label(sensBOWind, text="Select the type of part")
    partTypeLabel.pack_forget()
    partTypeMenu=tk.OptionMenu(sensBOWind, partType, *allPartTypes)
    partTypeMenu.pack_forget()
    partColor=tk.StringVar()
    partColor.set("")
    partColorLabel=tk.Label(sensBOWind, text="Select the color of the part")
    partColorLabel.pack_forget()
    partColorMenu=tk.OptionMenu(sensBOWind, partColor, *allPartColors)
    partColorMenu.pack_forget()
    annID=tk.StringVar()
    annID.set("")
    annIDLabel=tk.Label(sensBOWind, text="Select the order ID")
    annIDLabel.pack_forget()
    annIDMenu=tk.OptionMenu(sensBOWind, annID, *usedIds)
    annIDMenu.pack_forget()
    #save and cancel buttons
    sensBOSave=tk.Button(sensBOWind, text="Save", command=sensBOWind.destroy)
    sensBOSave.pack(pady=20)
    sensBOCancelFlag=tk.StringVar()
    sensBOCancelFlag.set('0')
    cancel_sens_bo_challenge=partial(exitAndFlag, sensBOWind, sensBOCancelFlag)
    sensBOCancel=tk.Button(sensBOWind, text="Cancel", command=cancel_sens_bo_challenge)
    sensBOCancel.pack()
    #variable tracing
    updateConditionMenu=partial(showCorrectMenu,condition, conditionMenu, time, timeLabel, timeEntry, agv, agvLabel, agvMenu, partType, partTypeLabel, partTypeMenu, partColor, partColorLabel, partColorMenu, annID, annIDLabel, annIDMenu, usedIds)
    condition.trace('w', updateConditionMenu)
    validate_duration=partial(validateTime, duration)
    duration.trace('w', validate_duration)
    sensBOWind.mainloop()
    if sensBOCancelFlag.get()=="0": # name of sensor
        newSensorBO=SensorBlackoutChallenge()
        newSensorBO.duration=float(duration.get())
        sensorsToDisable=Sensors()
        if sensor1.get()=="1":
            sensorsToDisable.break_beam=True
        else:
            sensorsToDisable.break_beam=False
        if sensor2.get()=="1":
            sensorsToDisable.proximity=True
        else:
            sensorsToDisable.proximity=False
        if sensor3.get()=="1":
            sensorsToDisable.laser_profiler=True
        else:
            sensorsToDisable.laser_profiler=False
        if sensor4.get()=="1":
            sensorsToDisable.lidar=True
        else:
            sensorsToDisable.lidar=False
        if sensor5.get()=="1":
            sensorsToDisable.camera=True
        else:
            sensorsToDisable.camera=False
        if sensor6.get()=="1":
            sensorsToDisable.logical_camera=True
        else:
            sensorsToDisable.logical_camera=False
        newSensorBO.sensors_to_disable=sensorsToDisable
        newSensorBOCond=Condition()
        if condition.get()!="":
            newSensorBOCond.type=conditionTypes.index(condition.get())-1
            if condition.get()==conditionTypes[1]:
                newSensorBOCond.time_condition.seconds=float(time.get())
            elif condition.get()==conditionTypes[2]:
                newPart=Part()
                if partType.get()=="sensor":
                    newPart.type=newPart.SENSOR
                elif partType.get()=="pump":
                    newPart.type=newPart.PUMP
                elif partType.get()=="battery":
                    newPart.type=newPart.BATTERY
                else:
                    newPart.type=newPart.REGULATOR
                if partColor.get()=="red":
                    newPart.color=newPart.RED
                elif partColor.get()=="green":
                    newPart.color=newPart.GREEN
                elif partColor.get()=="blue":
                    newPart.color=newPart.BLUE
                elif partColor.get()=="orange":
                    newPart.color=newPart.ORANGE
                else:
                    newPart.color=newPart.PURPLE
                newSensorBOCond.part_place_condition.part=newPart
                newSensorBOCond.part_place_condition.agv=int(agv.get())
            elif condition.get()==conditionTypes[2]:
                newSensorBOCond.submission_condition.order_id=annID.get()
        newSensorBO.condition=newSensorBOCond
        sensorBlackouts.append(newSensorBO)

def saveOrders(wind, challengesFlag): # allows the while loop in main to stop so the challenges window stops when the user saves
    challengesFlag.set('0')
    wind.destroy()

def runChallengeWind(robotMalfunctions, usedIDs, faultyParts, droppedParts, sensorBlackouts,cancelFlag, pathIncrement,fileName, createdDir, challengesFlag, mainWind):
    challengeWind=tk.Toplevel()
    challengesFlag.set('1')
    challengeWind.title("Challenges")
    #challengeWind.geometry("850x600")
    challengeWind.attributes('-fullscreen', True)
    #robot malfunciton
    if len(usedIDs)>0:
        new_robot_malfunction=partial(newRobotMalfunction, robotMalfunctions, usedIDs)
        robotMalfunctionButton=tk.Button(challengeWind, text="Add robot malfunction", command=new_robot_malfunction)
        robotMalfunctionButton.pack(pady=10)
        #faluty part
        new_faulty_part=partial(newFaultyPart, faultyParts, usedIDs)
        faultyPartButton=tk.Button(challengeWind, text="Add faulty part", command=new_faulty_part)
        faultyPartButton.pack(pady=10)
    #dropped part
    new_dropped_part=partial(newDroppedPart, droppedParts)
    droppedPartButton=tk.Button(challengeWind, text="Add dropped part", command=new_dropped_part)
    droppedPartButton.pack(pady=10)
    #sensor blackout
    if len(usedIDs)>0:
        new_sensor_blackout=partial(newSensorBlackout, sensorBlackouts, usedIDs)
        sensorBlackoutButton=tk.Button(challengeWind, text="Add sensor blackout", command=new_sensor_blackout)
        sensorBlackoutButton.pack(pady=10)
    #save and cancel buttons
    save_challenges=partial(saveOrders, mainWind, challengesFlag)
    saveChallengeButton=tk.Button(challengeWind, text="Save and return to menu", command=save_challenges)
    saveChallengeButton.pack(pady=20)
    challengeWind.mainloop()
