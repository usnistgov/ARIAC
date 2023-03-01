import tkinter as tk
from tkinter import ttk
from functools import partial
from ariac_msgs.msg import *
from ariac_gui.timeFunctions import validateTime
allPartTypes=["sensor", "pump", "regulator", "battery"]
allPartColors=['green', 'red', 'purple','blue','orange']
robotTypes=["ceiling_robot","floor_robot"]
agvOptions=["1","2","3","4"]
sensBOCategories=["time-based","during kitting", "during assembly","after kitting", "after assembly"]
conditionTypes=['time','partPlace','submission']
behaviorOptions=["antagonistic", "indifferent","helpful"]

def saveRobotMalfunction(allChallengeWidgetsArr, floorRobot, ceilRobot, duration, condition, time, partType, partColor, agv, annID, robotMalfunctions, challengeCounter):
    for widget in allChallengeWidgetsArr:
        widget.grid_forget()
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
    newRobotMalfCondition.type=conditionTypes.index(condition.get())
    if condition.get()==conditionTypes[0]:
        newRobotMalfCondition.time_condition.seconds=float(time.get())
    elif condition.get()==conditionTypes[1]:
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
    currVal=int(challengeCounter.get())
    challengeCounter.set(currVal+1)

def saveFaultyPart(currentOrderID, q1,q2,q3,q4,faultyParts, allChallengeWidgetsArr, challengeCounter):
    for widget in allChallengeWidgetsArr:
        widget.grid_forget()
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
    currVal=int(challengeCounter.get())
    challengeCounter.set(currVal+1)

def saveDroppedPart(robotType, partType, partColor, dropAfterNum, dropAfterTime, droppedParts, allChallengeWidgetsArr, challengeCounter):
    for widget in allChallengeWidgetsArr:
        widget.grid_forget()
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
    currVal=int(challengeCounter.get())
    challengeCounter.set(currVal+1)

def saveSensorBlackout(duration, sensor1, sensor2,sensor3,sensor4,sensor5, sensor6, condition, time, partType, partColor, agv, annID, sensorBlackouts, allChallengeWidgetsArr, challengeCounter):
    for widget in allChallengeWidgetsArr:
        widget.grid_forget()
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
        newSensorBOCond.type=conditionTypes.index(condition.get())
        if condition.get()==conditionTypes[0]:
            newSensorBOCond.time_condition.seconds=float(time.get())
        elif condition.get()==conditionTypes[1]:
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
    currVal=int(challengeCounter.get())
    challengeCounter.set(currVal+1)

def saveHumanChallenge(chosenBehavior, condition,time, partType, partColor, agv, annID, humanChallenges, allChallengeWidgetsArr, challengeCounter):
    for widget in allChallengeWidgetsArr:
        widget.grid_forget()
    newHumanChallenge=HumanChallenge()
    newHumanCond=Condition()
    if condition.get()!="":
        newHumanCond.type=conditionTypes.index(condition.get())
        if condition.get()==conditionTypes[0]:
            newHumanCond.time_condition.seconds=float(time.get())
        elif condition.get()==conditionTypes[1]:
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
            newHumanCond.part_place_condition.part=newPart
            newHumanCond.part_place_condition.agv=int(agv.get())
        elif condition.get()==conditionTypes[2]:
            newHumanCond.submission_condition.order_id=annID.get()
    newHumanChallenge.condition=newHumanCond
    newHumanChallenge.behavior=behaviorOptions.index(chosenBehavior.get())
    humanChallenges.append(newHumanChallenge)
    currVal=int(challengeCounter.get())
    challengeCounter.set(currVal+1)

def showCorrectMenu(condition, conditionMenu, time, timeLabel, timeEntry, agv, agvLabel, agvMenu, partType, partTypeLabel, partTypeMenu, partColor, partColorLabel, partColorMenu, annID, annIDLabel, annIDMenu,tempIDs,presentChallengeWidgets,a,b,c):
    if condition.get()=="":
        time.set('')
        agv.set('')
        partType.set('')
        partColor.set('')
        annID.set('')
        timeLabel.grid_forget()
        timeEntry.grid_forget()
        agvLabel.grid_forget()
        agvMenu.grid_forget()
        partTypeLabel.grid_forget()
        partTypeMenu.grid_forget()
        partColorLabel.grid_forget()
        partColorMenu.grid_forget()
        annIDLabel.grid_forget()
        annIDMenu.grid_forget()
    elif condition.get()==conditionTypes[0]:
        timeLabel.grid(column=2, row=len(presentChallengeWidgets)+1)
        timeEntry.grid(column=2, row=len(presentChallengeWidgets)+2)
        time.set('0')
        agv.set('')
        partType.set('')
        partColor.set('')
        annID.set('')
        agvLabel.grid_forget()
        agvMenu.grid_forget()
        partTypeLabel.grid_forget()
        partTypeMenu.grid_forget()
        partColorLabel.grid_forget()
        partColorMenu.grid_forget()
        annIDLabel.grid_forget()
        annIDMenu.grid_forget()
        presentChallengeWidgets.append(timeLabel)
        presentChallengeWidgets.append(timeEntry)
    elif condition.get()==conditionTypes[1]:
        agvLabel.grid(column=2, row=len(presentChallengeWidgets)+1)
        agvMenu.grid(column=2, row=len(presentChallengeWidgets)+2)
        agv.set(agvOptions[0])
        partTypeLabel.grid(column=2, row=len(presentChallengeWidgets)+3)
        partTypeMenu.grid(column=2, row=len(presentChallengeWidgets)+4)
        partColorLabel.grid(column=2, row=len(presentChallengeWidgets)+5)
        partColorMenu.grid(column=2, row=len(presentChallengeWidgets)+6)
        partType.set(allPartTypes[0])
        partColor.set(allPartColors[0])
        time.set('')
        timeLabel.grid_forget()
        timeEntry.grid_forget()
        annID.set('')
        annIDLabel.grid_forget()
        annIDMenu.grid_forget()
        presentChallengeWidgets.append(agvLabel)
        presentChallengeWidgets.append(agvMenu)
        presentChallengeWidgets.append(partTypeLabel)
        presentChallengeWidgets.append(partTypeMenu)
        presentChallengeWidgets.append(partColorLabel)
        presentChallengeWidgets.append(partColorMenu)
    else:
        annIDLabel.grid(column=2, row=len(presentChallengeWidgets)+1)
        annIDMenu.grid(column=2, row=len(presentChallengeWidgets)+2)
        annID.set(tempIDs[0])
        time.set('')
        agv.set('')
        partType.set('')
        partColor.set('')
        timeLabel.grid_forget()
        timeEntry.grid_forget()
        agvLabel.grid_forget()
        agvMenu.grid_forget()
        partTypeLabel.grid_forget()
        partTypeMenu.grid_forget()
        partColorLabel.grid_forget()
        partColorMenu.grid_forget()
        presentChallengeWidgets.append(annIDLabel)
        presentChallengeWidgets.append(annIDMenu)

def robotMalfunctionMenu(allChallengeWidgetsArr,presentChallengeWidgets, rmVals, chCondVals, conditionVal):
    conditionVal[0].set(conditionTypes[0])
    for widget in presentChallengeWidgets:
        widget.grid_forget()
    for val in rmVals:
        val.set('0')
    presentChallengeWidgets.clear()
    chCondVals[0].set('0')
    for index in range(4): #how many widgets there are for robot malfunction
        allChallengeWidgetsArr[index].grid(column=2, row=1+index)
        presentChallengeWidgets.append(allChallengeWidgetsArr[index])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-4].grid(column=2, row=5) # condition menu
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-3].grid(column=2, row=6)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-4])
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-3])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-2].grid(column=2, row=7) #time entrys
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-1].grid(column=2, row=8)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-2])
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-1])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-8].grid(column=2,row=16)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-8])

def faultyPartMenu(allChallengeWidgetsArr,presentChallengeWidgets,fpVals,usedIds):
    for widget in presentChallengeWidgets:
        widget.grid_forget()
    fpVals[0].set(usedIds[0])
    for i in range(1, 5):
        fpVals[i].set('0')
    presentChallengeWidgets.clear()
    for index in range(6): #how many widgets there are for faulty part
        allChallengeWidgetsArr[index+4].grid(column=2, row=1+index)
        presentChallengeWidgets.append(allChallengeWidgetsArr[index+4])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-7].grid(column=2,row=7)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-7])

def droppedPartMenu(allChallengeWidgetsArr,presentChallengeWidgets, dpVals):
    for widget in presentChallengeWidgets:
        widget.grid_forget()
    dpVals[0].set(robotTypes[0])
    dpVals[1].set(allPartTypes[0])
    dpVals[2].set(allPartColors[0])
    dpVals[3].set('0')
    dpVals[4].set('0')
    presentChallengeWidgets.clear()
    for index in range(10): #how many widgets there are for dropped part
        allChallengeWidgetsArr[index+10].grid(column=2, row=1+index)
        presentChallengeWidgets.append(allChallengeWidgetsArr[index+10])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-6].grid(column=2,row=11)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-6])

def sensorBlackoutMenu(allChallengeWidgetsArr, presentChallengeWidgets,sbVals, chCondVals, conditionVal):
    conditionVal[0].set(conditionTypes[0])
    for widget in presentChallengeWidgets:
        widget.grid_forget()
    presentChallengeWidgets.clear()
    sbVals[0].set(sensBOCategories[0])
    for i in range(1,8):
        sbVals[i].set('0')
    chCondVals[0].set('0')
    for index in range(8): #how many widgets there are for dropped part
        allChallengeWidgetsArr[index+22].grid(column=2, row=1+index)
        presentChallengeWidgets.append(allChallengeWidgetsArr[index+22])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-4].grid(column=2, row=11) #condition menu
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-3].grid(column=2, row=12)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-4])
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-3])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-2].grid(column=2, row=13) #time entrys
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-1].grid(column=2, row=14)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-2])
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-1])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-5].grid(column=2,row=24)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-5])

def humanChallengeMenu(allChallengeWidgetsArr, presentChallengeWidgets, huVals,chCondVals, conditionVal):
    conditionVal[0].set(conditionTypes[0])
    for widget in presentChallengeWidgets:
        widget.grid_forget()
    presentChallengeWidgets.clear()
    huVals[0].set(behaviorOptions[0])
    chCondVals[0].set('0')
    for index in range(2):
        allChallengeWidgetsArr[index+30].grid(column=2, row=1+index)
        presentChallengeWidgets.append(allChallengeWidgetsArr[index+30])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-4].grid(column=2, row=3) #condition menu
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-3].grid(column=2, row=4)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-4])
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-3])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-2].grid(column=2, row=5) #time entrys
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-1].grid(column=2, row=6)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-2])
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-1])
    allChallengeWidgetsArr[len(allChallengeWidgetsArr)-9].grid(column=2,row=24)
    presentChallengeWidgets.append(allChallengeWidgetsArr[len(allChallengeWidgetsArr)-9])


def activateFaultyPart(usedIds, faultyPartButton,a,b,c):
    if len(usedIds)>0:
        faultyPartButton.config(state=tk.NORMAL)

def chooseChallenge(challengeFrame,allChallengeWidgetsArr,presentChallengeWidgets,rmVals,fpVals, dpVals, sbVals, huVals, chCondVals,usedIds, partOrdCounter, conditionVal):
    new_robot_malfunction=partial(robotMalfunctionMenu, allChallengeWidgetsArr,presentChallengeWidgets,rmVals, chCondVals, conditionVal)
    newRobotMalfunctionButton=tk.Button(challengeFrame, text="Add new robot malfunction", command=new_robot_malfunction)
    newRobotMalfunctionButton.grid(column=1)
    new_faulty_part=partial(faultyPartMenu, allChallengeWidgetsArr, presentChallengeWidgets,fpVals, usedIds)
    newFaultyPartButton=tk.Button(challengeFrame, text="Add new faulty part", command=new_faulty_part, state=tk.DISABLED)
    newFaultyPartButton.grid(column=1)
    new_dropped_part=partial(droppedPartMenu, allChallengeWidgetsArr, presentChallengeWidgets,dpVals)
    newDroppedPartButton=tk.Button(challengeFrame, text="Add new dropped part", command=new_dropped_part)
    newDroppedPartButton.grid(column=1)
    new_sensor_blackout=partial(sensorBlackoutMenu, allChallengeWidgetsArr, presentChallengeWidgets,sbVals, chCondVals, conditionVal)
    newSensorBlackoutButton=tk.Button(challengeFrame, text="Add new sensor blackout", command=new_sensor_blackout)
    newSensorBlackoutButton.grid(column=1)
    new_human_challenge=partial(humanChallengeMenu,allChallengeWidgetsArr, presentChallengeWidgets, huVals,chCondVals, conditionVal)
    newHumanChallengeButton=tk.Button(challengeFrame, text="Add new human challenge", command=new_human_challenge)
    newHumanChallengeButton.grid(column=1)
    activate_faulty_part=partial(activateFaultyPart, usedIds, newFaultyPartButton)
    partOrdCounter.trace('w', activate_faulty_part)


def updateConditionMenu(orderMSGS, conditionMenu, condition, annIDMenu, annID, usedIDs,orderIDMenu,currentOrderID,a,b,c):
    if len(orderMSGS)>0:
        annID.set(usedIDs[0])
        conMen=conditionMenu['menu']
        conMen.delete(0, 'end')
        for option in conditionTypes:
            conMen.add_command(label=option, command=lambda option=option: condition.set(option))
        annIDMen=annIDMenu['menu']
        annIDMen.delete(0,'end')
        for id in usedIDs:
            annIDMen.add_command(label=id, command=lambda id=id: annID.set(id))
        currentOrderID.set(usedIDs[0])
        ordIDMen=orderIDMenu['menu']
        ordIDMen.delete(0,'end')
        for id in usedIDs:
            ordIDMen.add_command(label=id, command=lambda id=id: currentOrderID.set(id))

def allChallengeWidgets(challengesFrame,allChallengeWidgetsArr,presentChallengeWidgets,robotMalfunctions,faultyParts, droppedParts, sensorBlackouts,humanChallenges,rmVals,fpVals, dpVals, sbVals, huVals, chCondVals, challengeCounter, partOrdCounter, orderMSGS,usedIds, conditionVal):
    #robot malfunction
    rmDuration=tk.StringVar()
    rmDuration.set("0")
    rmDurationLabel=tk.Label(challengesFrame, text="Enter the duration of the robot malfunction")
    rmDurationLabel.grid_forget()
    rmDurationEntry=tk.Entry(challengesFrame, textvariable=rmDuration)
    rmDurationEntry.grid_forget()
    floorRobot=tk.StringVar()
    floorRobot.set("0")
    ceilRobot=tk.StringVar()
    ceilRobot.set("0")
    floorRobotCB=tk.Checkbutton(challengesFrame, text="Floor robot", variable=floorRobot, onvalue="1", offvalue="0", height=1, width=20)
    floorRobotCB.grid_forget()
    ceilRobotCB=tk.Checkbutton(challengesFrame, text="Ceiling robot", variable=ceilRobot, onvalue="1", offvalue="0", height=1, width=20)
    ceilRobotCB.grid_forget()
    allChallengeWidgetsArr.append(rmDurationLabel)
    allChallengeWidgetsArr.append(rmDurationEntry)
    allChallengeWidgetsArr.append(floorRobotCB)
    allChallengeWidgetsArr.append(ceilRobotCB)
    rmVals.append(rmDuration)
    rmVals.append(floorRobot)
    rmVals.append(ceilRobot)
    #faulty Part
    currentOrderID=tk.StringVar()
    currentOrderID.set("")
    orderIDLabel=tk.Label(challengesFrame, text="Select the order ID for the faulty part")
    orderIDLabel.grid_forget()
    orderIDMenu=tk.OptionMenu(challengesFrame, currentOrderID, *allPartTypes)
    orderIDMenu.grid_forget()
    q1=tk.StringVar()
    q1.set('0')
    q2=tk.StringVar()
    q2.set('0')
    q3=tk.StringVar()
    q3.set('0')
    q4=tk.StringVar()
    q4.set('0')
    q1CB=tk.Checkbutton(challengesFrame, text="Quadrant 1", variable=q1, onvalue="1", offvalue="0", height=1, width=20)
    q1CB.grid_forget()
    q2CB=tk.Checkbutton(challengesFrame, text="Quadrant 2", variable=q2, onvalue="1", offvalue="0", height=1, width=20)
    q2CB.grid_forget()
    q3CB=tk.Checkbutton(challengesFrame, text="Quadrant 3", variable=q3, onvalue="1", offvalue="0", height=1, width=20)
    q3CB.grid_forget()
    q4CB=tk.Checkbutton(challengesFrame, text="Quadrant 4", variable=q4, onvalue="1", offvalue="0", height=1, width=20)
    q4CB.grid_forget()
    allChallengeWidgetsArr.append(orderIDLabel)
    allChallengeWidgetsArr.append(orderIDMenu)
    allChallengeWidgetsArr.append(q1CB)
    allChallengeWidgetsArr.append(q2CB)
    allChallengeWidgetsArr.append(q3CB)
    allChallengeWidgetsArr.append(q4CB)
    fpVals.append(currentOrderID)
    fpVals.append(q1)
    fpVals.append(q2)
    fpVals.append(q3)
    fpVals.append(q4)
    #dropped part
    robotType=tk.StringVar()
    robotType.set(robotTypes[0])
    robotTypeLabel=tk.Label(challengesFrame, text="Select the robot type for the dropped part")
    robotTypeLabel.grid_forget()
    robotTypeMenu=tk.OptionMenu(challengesFrame, robotType, *robotTypes)
    robotTypeMenu.grid_forget()
    partType=tk.StringVar()
    partType.set(allPartTypes[0])
    partTypeLabel=tk.Label(challengesFrame, text="Select the type of part")
    partTypeLabel.grid_forget()
    partTypeMenu=tk.OptionMenu(challengesFrame, partType, *allPartTypes)
    partTypeMenu.grid_forget()
    partColor=tk.StringVar()
    partColor.set(allPartColors[0])
    partColorLabel=tk.Label(challengesFrame, text="Select the color of the part")
    partColorLabel.grid_forget()
    partColorMenu=tk.OptionMenu(challengesFrame, partColor, *allPartColors)
    partColorMenu.grid_forget()
    dropAfterNum=tk.StringVar()
    dropAfterNum.set("0")
    dropAfterNumLabel=tk.Label(challengesFrame, text="Set the number to drop the part after")
    dropAfterNumLabel.grid_forget()
    dropAfterNumEntry=tk.Entry(challengesFrame, textvariable=dropAfterNum)
    dropAfterNumEntry.grid_forget()
    dropAfterTime=tk.StringVar()
    dropAfterTime.set('0')
    dropAfterTimeLabel=tk.Label(challengesFrame,text="Set the time to drop the part after")
    dropAfterTimeLabel.grid_forget()
    dropAfterTimeEntry=tk.Entry(challengesFrame, textvariable=dropAfterTime)
    dropAfterTimeEntry.grid_forget()
    allChallengeWidgetsArr.append(robotTypeLabel)
    allChallengeWidgetsArr.append(robotTypeMenu)
    allChallengeWidgetsArr.append(partTypeLabel)
    allChallengeWidgetsArr.append(partTypeMenu)
    allChallengeWidgetsArr.append(partColorLabel)
    allChallengeWidgetsArr.append(partColorMenu)
    allChallengeWidgetsArr.append(dropAfterNumLabel)
    allChallengeWidgetsArr.append(dropAfterNumEntry)
    allChallengeWidgetsArr.append(dropAfterTimeLabel) 
    allChallengeWidgetsArr.append(dropAfterTimeEntry)
    dpVals.append(robotType)
    dpVals.append(partType)
    dpVals.append(partColor)
    dpVals.append(dropAfterNum)
    dpVals.append(dropAfterTime)
    #sensor blackout
    category=tk.StringVar()
    category.set(sensBOCategories[0])
    categoryLabel=tk.Label(challengesFrame, text="Choose the category for the sensor blackout")
    categoryLabel.grid_forget()
    categoryMenu=tk.OptionMenu(challengesFrame, category, *sensBOCategories)
    categoryMenu.grid_forget()
    sbDuration=tk.StringVar()
    sbDuration.set('0')
    sbDurationLabel=tk.Label(challengesFrame, text="Enter the duration for the sensor blackout")
    sbDurationLabel.grid_forget()
    sbDurationEntry=tk.Entry(challengesFrame, textvariable=sbDuration)
    sbDurationEntry.grid_forget()
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
    sensor1CB=tk.Checkbutton(challengesFrame, text="break beam", variable=sensor1, onvalue="1", offvalue="0", height=1, width=20)
    sensor1CB.grid_forget()
    sensor2CB=tk.Checkbutton(challengesFrame, text="proximity", variable=sensor2, onvalue="1", offvalue="0", height=1, width=20)
    sensor2CB.grid_forget()
    sensor3CB=tk.Checkbutton(challengesFrame, text="laser profiler", variable=sensor3, onvalue="1", offvalue="0", height=1, width=20)
    sensor3CB.grid_forget()
    sensor4CB=tk.Checkbutton(challengesFrame, text="lidar", variable=sensor4, onvalue="1", offvalue="0", height=1, width=20)
    sensor4CB.grid_forget()
    sensor5CB=tk.Checkbutton(challengesFrame, text="camera", variable=sensor5, onvalue="1", offvalue="0", height=1, width=20)
    sensor5CB.grid_forget()
    sensor6CB=tk.Checkbutton(challengesFrame, text="logical camera", variable=sensor6, onvalue="1", offvalue="0", height=1, width=20)
    sensor6CB.grid_forget()
    allChallengeWidgetsArr.append(categoryLabel)
    allChallengeWidgetsArr.append(categoryMenu)
    allChallengeWidgetsArr.append(sbDurationLabel)
    allChallengeWidgetsArr.append(sbDurationEntry)
    allChallengeWidgetsArr.append(sensor1CB)
    allChallengeWidgetsArr.append(sensor2CB)
    allChallengeWidgetsArr.append(sensor3CB)
    allChallengeWidgetsArr.append(sensor4CB)
    allChallengeWidgetsArr.append(sensor5CB)
    allChallengeWidgetsArr.append(sensor6CB)
    sbVals.append(category)
    sbVals.append(sbDuration)
    sbVals.append(sensor1)
    sbVals.append(sensor2)
    sbVals.append(sensor3)
    sbVals.append(sensor4)
    sbVals.append(sensor5)
    sbVals.append(sensor6)
    #human challenge
    chosenBehavior=tk.StringVar()
    chosenBehavior.set(behaviorOptions[0])
    humanLabel=tk.Label(challengesFrame, text="Select the behavior for the humans")
    humanLabel.grid_forget()
    humanMenu=tk.OptionMenu(challengesFrame, chosenBehavior, *behaviorOptions)
    humanMenu.grid_forget()
    allChallengeWidgetsArr.append(humanLabel)
    allChallengeWidgetsArr.append(humanMenu)
    huVals.append(chosenBehavior)
    #condition
    condition=tk.StringVar()
    condition.set(conditionTypes[0])
    conditionLabel=tk.Label(challengesFrame, text="Select a condition for the order")
    conditionLabel.grid_forget()
    conditionMenu=tk.OptionMenu(challengesFrame, condition, *conditionTypes[:-1])
    conditionMenu.grid_forget()
    conTime=tk.StringVar()
    conTime.set('0')
    conTimeLabel=tk.Label(challengesFrame, text="Enter the time")
    conTimeLabel.grid_forget()
    conTimeEntry=tk.Entry(challengesFrame, textvariable=conTime)
    conTimeEntry.grid_forget()
    conAgv=tk.StringVar()
    conAgv.set("")
    conAgvLabel=tk.Label(challengesFrame, text="Choose the agv")
    conAgvLabel.grid_forget()
    conAgvMenu=tk.OptionMenu(challengesFrame, conAgv, *agvOptions)
    conAgvMenu.grid_forget()
    conPartType=tk.StringVar()
    conPartType.set("")
    conPartTypeLabel=tk.Label(challengesFrame, text="Select the type of part")
    conPartTypeLabel.grid_forget()
    conPartTypeMenu=tk.OptionMenu(challengesFrame, conPartType, *allPartTypes)
    conPartTypeMenu.grid_forget()
    conPartColor=tk.StringVar()
    conPartColor.set("")
    conPartColorLabel=tk.Label(challengesFrame, text="Select the color of the part")
    conPartColorLabel.grid_forget()
    conPartColorMenu=tk.OptionMenu(challengesFrame, conPartColor, *allPartColors)
    conPartColorMenu.grid_forget()
    annID=tk.StringVar()
    annID.set("")
    annIDLabel=tk.Label(challengesFrame, text="Select the order ID")
    annIDLabel.grid_forget()
    annIDMenu=tk.OptionMenu(challengesFrame, annID, *allPartColors)#Filled with dummy values when usedIds is empty
    annIDMenu.grid_forget()
    chCondVals.append(conTime)
    update_con_menus=partial(showCorrectMenu,condition, conditionMenu, conTime, conTimeLabel, conTimeEntry, conAgv, conAgvLabel, conAgvMenu, conPartType, conPartTypeLabel, conPartTypeMenu, conPartColor, conPartColorLabel, conPartColorMenu, annID, annIDLabel, annIDMenu,usedIds,presentChallengeWidgets)
    condition.trace('w', update_con_menus)
    updateAnnIDMenu=partial(updateConditionMenu,orderMSGS, conditionMenu, condition, annIDMenu, annID, usedIds, orderIDMenu,currentOrderID)
    partOrdCounter.trace('w', updateAnnIDMenu)
    conditionVal.append(condition)
    #Value validations
    validate_con_time=partial(validateTime, conTime)
    conTime.trace('w', validate_con_time)
    validate_rm_duration=partial(validateTime, rmDuration)
    rmDuration.trace('w', validate_rm_duration)
    validate_sb_duration=partial(validateTime, sbDuration)
    sbDuration.trace('w', validate_sb_duration)
    #Save buttons
    save_robot_breakdown=partial(saveRobotMalfunction,allChallengeWidgetsArr, floorRobot, ceilRobot, rmDuration, condition, conTime, conPartType, conPartColor, conAgv, annID, robotMalfunctions, challengeCounter)
    rmSaveButton=tk.Button(challengesFrame, text="Save robot malfunction", command=save_robot_breakdown)
    rmSaveButton.grid_forget()
    save_faulty_part=partial(saveFaultyPart,currentOrderID, q1,q2,q3,q4,faultyParts, allChallengeWidgetsArr, challengeCounter)
    saveFaultyPartButton=tk.Button(challengesFrame, text="Save faulty part", command=save_faulty_part)
    saveFaultyPartButton.grid_forget()
    save_dropped_part=partial(saveDroppedPart,robotType, partType, partColor, dropAfterNum, dropAfterTime, droppedParts, allChallengeWidgetsArr, challengeCounter)
    saveDroppedPartButton=tk.Button(challengesFrame, text="Save dropped part", command=save_dropped_part)
    saveDroppedPartButton.grid_forget()
    save_sensor_blackout=partial(saveSensorBlackout, sbDuration, sensor1, sensor2,sensor3,sensor4,sensor5, sensor6, condition, conTime, conPartType, conPartColor, conAgv, annID, sensorBlackouts, allChallengeWidgetsArr, challengeCounter)
    saveSensorBlackoutButton=tk.Button(challengesFrame, text="Save sensor blackout", command=save_sensor_blackout)
    saveSensorBlackoutButton.grid_forget()
    save_human_challenge=partial(saveHumanChallenge,chosenBehavior, condition,conTime, conPartType, conPartColor, conAgv, annID, humanChallenges, allChallengeWidgetsArr, challengeCounter)
    saveHumanChallengeButton=tk.Button(challengesFrame, text="Save human challenge", command=save_human_challenge)
    saveHumanChallengeButton.grid_forget()
    allChallengeWidgetsArr.append(conAgvLabel)
    allChallengeWidgetsArr.append(conAgvMenu)
    allChallengeWidgetsArr.append(conPartTypeLabel)
    allChallengeWidgetsArr.append(conPartTypeMenu)
    allChallengeWidgetsArr.append(conPartColorLabel)
    allChallengeWidgetsArr.append(conPartColorMenu)
    allChallengeWidgetsArr.append(annIDLabel)
    allChallengeWidgetsArr.append(annIDMenu)
    allChallengeWidgetsArr.append(saveHumanChallengeButton)
    allChallengeWidgetsArr.append(rmSaveButton)
    allChallengeWidgetsArr.append(saveFaultyPartButton)
    allChallengeWidgetsArr.append(saveDroppedPartButton)
    allChallengeWidgetsArr.append(saveSensorBlackoutButton)
    allChallengeWidgetsArr.append(conditionLabel)
    allChallengeWidgetsArr.append(conditionMenu)
    allChallengeWidgetsArr.append(conTimeLabel)
    allChallengeWidgetsArr.append(conTimeEntry)