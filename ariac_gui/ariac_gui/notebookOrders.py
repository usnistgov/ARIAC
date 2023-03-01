import random
import string
import tkinter as tk
from functools import partial
from ariac_msgs.msg import *
orderTypes=["kitting", "assembly", "combined"]
quadrants=["1","2","3","4"]
agvOptions=["1","2","3","4"]
kittingDestinations=["warehouse", "assembly_front","assembly_back","kitting"]
assemblyStations=["as1","as2","as3","as4"]
kittingTrayIDs=[]
for i in range(10):
    kittingTrayIDs.append(str(i))
currentQuadMenu=[]
orderCategories=["time-based","during kitting", "during assembly","after kitting", "after assembly"]
taskPresentFlag=[]
allProdTypes=["sensor", "pump", "regulator", "battery"]
allProdColors=['green', 'red', 'purple','blue','orange']
conditionTypes=['time','partPlace','submission']
def partAvailable(agv1Parts, agv2Parts, agv3Parts, agv4Parts, type, color):
    flag=False
    for part in agv1Parts:
        if type==part.pType and color==part.color:
            flag=True
    for part in agv2Parts:
        if type==part.pType and color==part.color:
            flag=True
    for part in agv3Parts:
        if type==part.pType and color==part.color:
            flag=True
    for part in agv4Parts:
        if type==part.pType and color==part.color:
            flag=True
    return flag

def checkForParts(agv1Parts, agv2Parts, agv3Parts, agv4Parts, warningLabel, type, color,a,b,c):
    if partAvailable(agv1Parts, agv2Parts, agv3Parts, agv4Parts, type.get(), color.get()):
        warningLabel.config(text="")
    else:
        warningLabel.config(text="Warning: The part have currently selected is not available from an AGV.")

def showNewOrderMenu(orderWidgetsArr, orderValsArr, usedIDs):
    #orderValsArr[0].set(orderCategories[0])
    orderValsArr[1].set(orderTypes[0])
    orderValsArr[2].set('0')
    orderValsArr[3].set(conditionTypes[0])
    orderValsArr[4].set('0')
    orderValsArr[5].set("")
    orderValsArr[6].set("")
    orderValsArr[7].set("")
    orderValsArr[8].set("")
    orderValsArr[9].set(agvOptions[0])
    orderValsArr[10].set(kittingTrayIDs[0])
    orderValsArr[11].set(kittingDestinations[0])
    orderValsArr[12].set("")
    orderValsArr[13].set(generateOrderId(usedIDs))
    '''orderWidgetsArr[0].grid(column=2, row=0)
    orderWidgetsArr[1].grid(column=2, row=1)'''
    orderWidgetsArr[2].grid(column=2, row=2)
    orderWidgetsArr[3].grid(column=2, row=3)
    orderWidgetsArr[4].grid(column=2, row=4)
    orderWidgetsArr[5].grid(column=2, row=5)
    orderWidgetsArr[6].grid(column=2, row=6)
    #orderWidgetsArr[7].grid(column=2, row=7)
    #orderWidgetsArr[8].grid(column=2, row=8)
    orderWidgetsArr[17].grid(column=2, row=14)
    orderWidgetsArr[18].grid(column=2, row=15)
    orderWidgetsArr[19].grid(column=2, row=16)
    orderWidgetsArr[20].grid(column=2, row=17)
    orderWidgetsArr[21].grid(column=2, row=18)
    orderWidgetsArr[22].grid(column=2, row=19)
    #orderWidgetsArr[23].grid(column=2, row=20)
    #orderWidgetsArr[24].grid(column=2, row=21)
    orderWidgetsArr[25].grid(column=2, row=22)
    orderWidgetsArr[26].grid(column=2, row=23)
    orderWidgetsArr[27].grid(column=2, row=24)

def backOrder(orderWidgetsArr, orderValsArr, usedIDs):
    for widget in orderWidgetsArr:
        widget.grid_forget()
    for val in orderValsArr:
        val.set("")
    tempIDs=[]
    for id in usedIDs[:-1]:
        tempIDs.append(id)
    usedIDs.clear()
    for id in tempIDs:
        usedIDs.append(id)

def backKitProd(kitWidgetsArr, kitValsArr):
    for widget in kitWidgetsArr:
        widget.grid_forget()
    for val in kitValsArr:
        val.set("")

def backAssemblyProd(assemblyWidgetsArr, assemblyValsArr):
    for widget in assemblyWidgetsArr:
        widget.grid_forget()
    for val in assemblyValsArr:
        val.set("")

def showCorrectMenu(orderValsArr, orderWidgetsArr,tempIDs,a,b,c):
    if orderValsArr[3].get()=="":
        orderValsArr[4].set('')#time
        orderValsArr[5].set('')#agv
        orderValsArr[6].set('')#partType
        orderValsArr[7].set('')#partColor
        orderValsArr[8].set('')#annId
        orderWidgetsArr[7].grid_forget()#timeLabel
        orderWidgetsArr[8].grid_forget()#timeEntry
        orderWidgetsArr[9].grid_forget()#agvLabel
        orderWidgetsArr[10].grid_forget()#agvMenu
        orderWidgetsArr[11].grid_forget()#partTypeLabel
        orderWidgetsArr[12].grid_forget()#partTypeMenu
        orderWidgetsArr[13].grid_forget()#partColorLabel
        orderWidgetsArr[14].grid_forget()#partColorMenu
        orderWidgetsArr[15].grid_forget()#annIDLabel
        orderWidgetsArr[16].grid_forget()#annIDMenu
    elif orderValsArr[3].get()==conditionTypes[0]:
        orderWidgetsArr[7].grid(column=2, row=7)
        orderWidgetsArr[8].grid(column=2, row=8)
        orderValsArr[4].set('0')
        orderValsArr[5].set('')
        orderValsArr[6].set('')
        orderValsArr[7].set('')
        orderValsArr[8].set('')
        orderWidgetsArr[9].grid_forget()
        orderWidgetsArr[10].grid_forget()
        orderWidgetsArr[11].grid_forget()
        orderWidgetsArr[12].grid_forget()
        orderWidgetsArr[13].grid_forget()
        orderWidgetsArr[14].grid_forget()
        orderWidgetsArr[15].grid_forget()
        orderWidgetsArr[16].grid_forget()
    elif orderValsArr[3].get()==conditionTypes[1]:
        orderWidgetsArr[9].grid(column=2, row=7)
        orderWidgetsArr[10].grid(column=2, row=8)
        orderValsArr[5].set(agvOptions[0])
        orderWidgetsArr[11].grid(column=2, row=9)
        orderWidgetsArr[12].grid(column=2, row=10)
        orderWidgetsArr[13].grid(column=2, row=11)
        orderWidgetsArr[14].grid(column=2, row=12)
        orderValsArr[6].set(allProdTypes[0])
        orderValsArr[7].set(allProdColors[0])
        orderValsArr[4].set('')
        orderWidgetsArr[7].grid_forget()
        orderWidgetsArr[8].grid_forget()
        orderValsArr[8].set('')
        orderWidgetsArr[15].grid_forget()
        orderWidgetsArr[16].grid_forget()
    else:
        orderWidgetsArr[15].grid(column=2, row=7)
        orderWidgetsArr[16].grid(column=2, row=8)
        orderValsArr[8].set(tempIDs[0])
        orderValsArr[4].set('')
        orderValsArr[5].set('')
        orderValsArr[6].set('')
        orderValsArr[7].set('')
        orderWidgetsArr[7].grid_forget()
        orderWidgetsArr[8].grid_forget()
        orderWidgetsArr[9].grid_forget()
        orderWidgetsArr[10].grid_forget()
        orderWidgetsArr[11].grid_forget()
        orderWidgetsArr[12].grid_forget()
        orderWidgetsArr[13].grid_forget()
        orderWidgetsArr[14].grid_forget()

def typeOfProdSelect(orderType, kitValsArr, kitWidgetsArr, assemblyValsArr, assemblyWidgetsArr):
    '''Runs the correct function based on the order type'''
    if orderType.get()=="kitting":
        addKittingProduct(kitValsArr, kitWidgetsArr)
    else:
        addAssembProduct(assemblyValsArr, assemblyWidgetsArr)

def addKittingProduct(kitValsArr, kitWidgetsArr):
    kitValsArr[0].set(allProdTypes[0])
    kitValsArr[1].set(allProdColors[0])
    kitValsArr[2].set(quadrants[0])
    for i in range(len(kitWidgetsArr)):
        kitWidgetsArr[i].grid(column=3, row=i)

def addAssembProduct(assemblyValsArr, assemblyWidgetsArr):
    assemblyValsArr[0].set(allProdTypes[0])
    assemblyValsArr[1].set(allProdColors[0])
    for i in range(2,len(assemblyValsArr)):
        assemblyValsArr[i].set('0')
    for i in range(len(assemblyWidgetsArr)):
        assemblyWidgetsArr[i].grid(column=3, row=i)

def saveKittingProd(kitValsArr, kitWidgetsArr, kittingParts):
    for widget in kitWidgetsArr:
        widget.grid_forget()
    newKittingPart=KittingPart()
    newPart=Part()
    if kitValsArr[0].get()=="sensor":
        newPart.type=newPart.SENSOR
    elif kitValsArr[0].get()=="pump":
        newPart.type=newPart.PUMP
    elif kitValsArr[0].get()=="battery":
        newPart.type=newPart.BATTERY
    else:
        newPart.type=newPart.REGULATOR
    if kitValsArr[1].get()=="red":
        newPart.color=newPart.RED
    elif kitValsArr[1].get()=="green":
        newPart.color=newPart.GREEN
    elif kitValsArr[1].get()=="blue":
        newPart.color=newPart.BLUE
    elif kitValsArr[1].get()=="orange":
        newPart.color=newPart.ORANGE
    else:
        newPart.color=newPart.PURPLE
    newKittingPart.part=newPart
    newKittingPart.quadrant=int(kitValsArr[2].get())
    kittingParts.append(newKittingPart)

def saveAssemblyProd(assemblyValsArr, assemblyWidgetsArr, assemblyParts):
    for widget in assemblyWidgetsArr:
        widget.grid_forget()
    newAssembPart=AssemblyPart()
    newPart=Part()
    if assemblyValsArr[0].get()=="sensor":
        newPart.type=newPart.SENSOR
    elif assemblyValsArr[0].get()=="pump":
        newPart.type=newPart.PUMP
    elif assemblyValsArr[0].get()=="battery":
        newPart.type=newPart.BATTERY
    else:
        newPart.type=newPart.REGULATOR
    if assemblyValsArr[1].get()=="red":
        newPart.color=newPart.RED
    elif assemblyValsArr[1].get()=="green":
        newPart.color=newPart.GREEN
    elif assemblyValsArr[1].get()=="blue":
        newPart.color=newPart.BLUE
    elif assemblyValsArr[1].get()=="orange":
        newPart.color=newPart.ORANGE
    else:
        newPart.color=newPart.PURPLE
    newAssembPart.part=newPart
    assemblyParts.append(newAssembPart)
    
def updateTaskOptions(orderType, kitTrayId, taskAgvMenu,kitTrayIdLabel, kitTrayIdMenu, kittingDestination, kittingDestinationLabel, kittingDestinationMenu, assemblyStation, assemblyStationLabel, assemblyStationMenu,a,b,c):
    '''Shows the correct options for different types of orders'''
    if orderType.get()=="kitting" and len(taskPresentFlag)>0:
        taskPresentFlag.clear()
        kitTrayId.set(kittingTrayIDs[0])
        kittingDestination.set(kittingDestinations[0])
        assemblyStation.set("")
        kitTrayIdLabel.grid(column=2, row=16)
        kitTrayIdMenu.grid(column=2, row=17)
        kittingDestinationLabel.grid(column=2, row=18)
        kittingDestinationMenu.grid(column=2, row=17)
        assemblyStationLabel.grid_forget()
        assemblyStationMenu.grid_forget()
    elif (orderType.get()=="assembly" or orderType.get()=="combined") and len(taskPresentFlag)==0:
        taskPresentFlag.append(0)
        kitTrayId.set("")
        kittingDestination.set("")
        assemblyStation.set(assemblyStations[0])
        kitTrayIdLabel.grid_forget()
        kitTrayIdMenu.grid_forget()
        kittingDestinationLabel.grid_forget()
        kittingDestinationMenu.grid_forget()
        assemblyStationLabel.grid(column=2, row=16)
        assemblyStationMenu.grid(column=2, row=17)

def generateOrderId(usedId):
    '''Generates a unique id for each order'''
    newId=''.join(random.choices(string.ascii_uppercase+string.digits,k=8))
    if newId in usedId:
        while newId in usedId:
            newId=''.join(random.choices(string.ascii_uppercase+string.digits,k=8))
    usedId.append(newId)
    return newId

def saveOrder(orderWidgetsArr, orderValsArr, kittingParts, assemblyParts, orderMSGS, orderConditions, partOrdCounter):
    currVal=int(partOrdCounter.get())
    for widget in orderWidgetsArr:
        widget.grid_forget()
    tempKittingParts=[]
    tempAssemblyParts=[]
    for part in kittingParts:
        tempKittingParts.append(part)
    for part in assemblyParts:
        tempAssemblyParts.append(part)
    newOrder=Order()
    newOrder.id=orderValsArr[13].get()
    newOrder.type=orderTypes.index(orderValsArr[1].get())
    if orderValsArr[2].get()=="0":
        newOrder.priority=False
    else:
        newOrder.priority=True
    if orderValsArr[1].get()=="kitting":
        newKittingTask=KittingTask()
        newKittingTask.agv_number=int(orderValsArr[9].get())
        newKittingTask.tray_id=int(orderValsArr[10].get())
        if orderValsArr[11].get()=="warehouse":
            newKittingTask.destination=newKittingTask.WAREHOUSE
        elif orderValsArr[11].get()=="kitting":
            newKittingTask.destination=newKittingTask.KITTING
        elif orderValsArr[11].get()=="assembly_front":
            newKittingTask.destination=newKittingTask.ASSEMBLY_FRONT
        else:
            newKittingTask.destination=newKittingTask.ASSEMBLY_BACK
        newKittingTask.parts=tempKittingParts
        newOrder.kitting_task=newKittingTask
    elif orderValsArr[1].get()=="assembly":
        agvNumList=[]
        agvNumList.append(int(orderValsArr[9].get()))
        newAssemblyTask=AssemblyTask()
        newAssemblyTask.agv_numbers=agvNumList
        newAssemblyTask.station=assemblyStations.index(orderValsArr[12].get())+1
        newAssemblyTask.parts=tempAssemblyParts
        newOrder.assembly_task=newAssemblyTask
    else:
        agvNumList=[]
        agvNumList.append(int(orderValsArr[9].get()))
        newCombinedTask=CombinedTask()
        newCombinedTask.station=assemblyStations.index(orderValsArr[12].get())+1
        newCombinedTask.parts=tempAssemblyParts
        newOrder.combined_task=newCombinedTask
    orderMSGS.append(newOrder)
    orderCondition=Condition()
    orderCondition.type=conditionTypes.index(orderValsArr[3].get())
    if orderValsArr[3].get()==conditionTypes[0]:
        orderCondition.time_condition.seconds=float(orderValsArr[4].get())
    elif orderValsArr[3].get()==conditionTypes[1]:
        newPart=Part()
        if orderValsArr[6].get()=="sensor":
            newPart.type=newPart.SENSOR
        elif orderValsArr[6].get()=="pump":
            newPart.type=newPart.PUMP
        elif orderValsArr[6].get()=="battery":
            newPart.type=newPart.BATTERY
        else:
            newPart.type=newPart.REGULATOR
        if orderValsArr[7].get()=="red":
            newPart.color=newPart.RED
        elif orderValsArr[7].get()=="green":
            newPart.color=newPart.GREEN
        elif orderValsArr[7].get()=="blue":
            newPart.color=newPart.BLUE
        elif orderValsArr[7].get()=="orange":
            newPart.color=newPart.ORANGE
        else:
            newPart.color=newPart.PURPLE
        orderCondition.part_place_condition.part=newPart
        orderCondition.part_place_condition.agv=int(orderValsArr[5].get())
    elif orderValsArr[3].get()==conditionTypes[2]:
        orderCondition.submission_condition.order_id=orderValsArr[8].get()
    orderConditions.append(orderCondition)
    kittingParts.clear()
    assemblyParts.clear()
    partOrdCounter.set(str(currVal+1))

def kittingProdWidgets(orderFrame, kittingParts, kitValsArr, kitWidgetsArr,agv1Parts, agv2Parts, agv3Parts, agv4Parts):
    if partAvailable(agv1Parts, agv2Parts, agv3Parts, agv4Parts, allProdTypes[0], allProdColors[0]):
        message="Warning: The part have currently selected is not available from an AGV."
    else:
        message=""
    kittingWarningLabel=tk.Label(orderFrame,text=message)
    kittingWarningLabel.grid_forget()
    kitWidgetsArr.append(kittingWarningLabel)
    prodType=tk.StringVar()
    prodType.set(allProdTypes[0])
    prodTypeLabel=tk.Label(orderFrame, text="Select the type of product for the kitting task")
    prodTypeLabel.grid_forget()
    prodTypeMenu=tk.OptionMenu(orderFrame, prodType, *allProdTypes)
    prodTypeMenu.grid_forget()
    kitValsArr.append(prodType)
    kitWidgetsArr.append(prodTypeLabel)
    kitWidgetsArr.append(prodTypeMenu)
    #product color
    prodColor=tk.StringVar()
    prodColor.set(allProdColors[0])
    prodColorLabel=tk.Label(orderFrame, text="Select the color of the product for the kitting task")
    prodColorLabel.grid_forget()
    prodColorMenu=tk.OptionMenu(orderFrame, prodColor, *allProdColors)
    prodColorMenu.grid_forget()
    kitValsArr.append(prodColor)
    kitWidgetsArr.append(prodColorLabel)
    kitWidgetsArr.append(prodColorMenu)
    #product quadrant
    prodQuad=tk.StringVar()
    prodQuad.set(quadrants[0])
    prodQuadLabel=tk.Label(orderFrame, text="Select the quadrant for the product")
    prodQuadLabel.grid_forget()
    prodQuadMenu=tk.OptionMenu(orderFrame, prodQuad, *quadrants)
    prodQuadMenu.grid_forget()
    kitValsArr.append(prodQuad)
    kitWidgetsArr.append(prodQuadLabel)
    kitWidgetsArr.append(prodQuadMenu)
    #save  and back buttons
    save_kit_prod=partial(saveKittingProd, kitValsArr, kitWidgetsArr, kittingParts)
    saveKitButton=tk.Button(orderFrame, text="Save product", command=save_kit_prod)
    saveKitButton.grid_forget()
    kitWidgetsArr.append(saveKitButton)
    back_kit_prod=partial(backKitProd, kitWidgetsArr, kitValsArr)
    backKitButton=tk.Button(orderFrame, text="Back", command=back_kit_prod)
    backKitButton.grid_forget()
    kitWidgetsArr.append(backKitButton)
    updateKitWarning=partial(checkForParts,agv1Parts, agv2Parts, agv3Parts, agv4Parts, kittingWarningLabel, prodType, prodColor)
    prodType.trace('w', updateKitWarning)
    prodColor.trace('w', updateKitWarning)


def assemblyProdWidgets(orderFrame, assemblyParts, assemblyValsArr, assemblyWidgetsArr,agv1Parts, agv2Parts, agv3Parts, agv4Parts):
    if partAvailable(agv1Parts, agv2Parts, agv3Parts, agv4Parts, allProdTypes[0], allProdColors[0]):
        message="Warning: The part have currently selected is not available from an AGV."
    else:
        message=""
    assemWarningLabel=tk.Label(orderFrame,text=message)
    assemWarningLabel.grid_forget()
    assemblyWidgetsArr.append(assemWarningLabel)
    prodType=tk.StringVar()
    prodType.set(allProdTypes[0])
    prodTypeLabel=tk.Label(orderFrame, text="Select the type of product for the assembly task")
    prodTypeLabel.grid_forget()
    prodTypeMenu=tk.OptionMenu(orderFrame, prodType, *allProdTypes)
    prodTypeMenu.grid_forget()
    assemblyValsArr.append(prodType)
    assemblyWidgetsArr.append(prodTypeLabel)
    assemblyWidgetsArr.append(prodTypeMenu)
    #product color
    prodColor=tk.StringVar()
    prodColor.set(allProdColors[0])
    prodColorLabel=tk.Label(orderFrame, text="Select the color of the product for the assembly task")
    prodColorLabel.grid_forget()
    prodColorMenu=tk.OptionMenu(orderFrame, prodColor, *allProdColors)
    prodColorMenu.grid_forget()
    assemblyValsArr.append(prodColor)
    assemblyWidgetsArr.append(prodColorLabel)
    assemblyWidgetsArr.append(prodColorMenu)
    #save and back buttons
    save_assemb_prod=partial(saveAssemblyProd, assemblyValsArr, assemblyWidgetsArr, assemblyParts)
    saveAssembButton=tk.Button(orderFrame, text="Save product", command=save_assemb_prod)
    saveAssembButton.grid_forget()
    assemblyWidgetsArr.append(saveAssembButton)
    back_assemb_prod=partial(backAssemblyProd, assemblyWidgetsArr, assemblyValsArr)
    backAssembButton=tk.Button(orderFrame, text="Back", command=back_assemb_prod)
    backAssembButton.grid_forget()
    assemblyWidgetsArr.append(backAssembButton)
    updateAssembWarning=partial(checkForParts,agv1Parts, agv2Parts, agv3Parts, agv4Parts, assemWarningLabel, prodType, prodColor)
    prodType.trace('w', updateAssembWarning)
    prodColor.trace('w', updateAssembWarning)

def updateConditionMenus(usedIDs, annID, conditionMenu, condition, annIDMenu,a,b,c):
    if len(usedIDs):
        annID.set(usedIDs[0])
        conMen=conditionMenu['menu']
        conMen.delete(0, 'end')
        for option in conditionTypes:
            conMen.add_command(label=option, command=lambda option=option: condition.set(option))
        annIDMen=annIDMenu['menu']
        annIDMen.delete(0,'end')
        for id in usedIDs:
            annIDMen.add_command(label=id, command=lambda id=id: annID.set(id))

def orderWidgets(orderFrame, orderMSGS,orderConditions, usedIDs, kittingParts, assemblyParts, partOrdCounter,agv1Parts, agv2Parts, agv3Parts, agv4Parts):
    kitWidgetsArr=[]
    kitValsArr=[]
    assemblyWidgetsArr=[]
    assemblyValsArr=[]
    #generate the order id
    orderWidgetsArr=[]
    orderValsArr=[]
    orderCategory=tk.StringVar()
    orderCategory.set(orderCategories[0])
    orderCategoryLabel=tk.Label(orderFrame, text="Select the category of the order")
    orderCategoryLabel.grid_forget()
    orderCategoryMenu=tk.OptionMenu(orderFrame, orderCategory, *orderCategories)
    orderCategoryMenu.grid_forget()
    orderValsArr.append(orderCategory)
    orderWidgetsArr.append(orderCategoryLabel)
    orderWidgetsArr.append(orderCategoryMenu)
    #order type
    orderType=tk.StringVar()
    orderType.set(orderTypes[0])
    orderTypeSelectionLabel=tk.Label(orderFrame, text="Select the type of order")
    orderTypeSelectionLabel.grid_forget()
    orderTypeSelectionMenu=tk.OptionMenu(orderFrame, orderType, *orderTypes)
    orderTypeSelectionMenu.grid_forget()
    orderValsArr.append(orderType)
    orderWidgetsArr.append(orderTypeSelectionLabel)
    orderWidgetsArr.append(orderTypeSelectionMenu)
    #Priority
    orderPriority=tk.StringVar()
    orderPriority.set('0')
    orderPriorityCheckBox=tk.Checkbutton(orderFrame, text="Priority", variable=orderPriority, onvalue="1", offvalue="0", height=1, width=20)
    orderPriorityCheckBox.grid_forget()
    orderValsArr.append(orderPriority)
    orderWidgetsArr.append(orderPriorityCheckBox)
    #announcement
    condition=tk.StringVar()
    condition.set(conditionTypes[0])
    conditionLabel=tk.Label(orderFrame, text="Select a condition for the order")
    conditionLabel.grid_forget()
    conditionMenu=tk.OptionMenu(orderFrame, condition, *conditionTypes[:-1])
    conditionMenu.grid_forget()
    orderValsArr.append(condition)
    orderWidgetsArr.append(conditionLabel)
    orderWidgetsArr.append(conditionMenu)
    time=tk.StringVar()
    time.set('0')
    timeLabel=tk.Label(orderFrame, text="Enter the time")
    timeLabel.grid_forget()
    timeEntry=tk.Entry(orderFrame, textvariable=time)
    timeEntry.grid_forget()
    orderValsArr.append(time)
    orderWidgetsArr.append(timeLabel)
    orderWidgetsArr.append(timeEntry)
    agv=tk.StringVar()
    agv.set("")
    agvLabel=tk.Label(orderFrame, text="Choose the agv")
    agvLabel.grid_forget()
    agvMenu=tk.OptionMenu(orderFrame, agv, *agvOptions)
    agvMenu.grid_forget()
    orderValsArr.append(agv)
    orderWidgetsArr.append(agvLabel)
    orderWidgetsArr.append(agvMenu)
    partType=tk.StringVar()
    partType.set("")
    partTypeLabel=tk.Label(orderFrame, text="Select the type of part")
    partTypeLabel.grid_forget()
    partTypeMenu=tk.OptionMenu(orderFrame, partType, *allProdTypes)
    partTypeMenu.grid_forget()
    orderValsArr.append(partType)
    orderWidgetsArr.append(partTypeLabel)
    orderWidgetsArr.append(partTypeMenu)
    partColor=tk.StringVar()
    partColor.set("")
    partColorLabel=tk.Label(orderFrame, text="Select the color of the part")
    partColorLabel.grid_forget()
    partColorMenu=tk.OptionMenu(orderFrame, partColor, *allProdColors)
    partColorMenu.grid_forget()
    orderValsArr.append(partColor)
    orderWidgetsArr.append(partColorLabel)
    orderWidgetsArr.append(partColorMenu)
    annID=tk.StringVar()
    annID.set("")
    annIDLabel=tk.Label(orderFrame, text="Select the order ID")
    annIDLabel.grid_forget()
    annIDMenu=tk.OptionMenu(orderFrame, annID, *allProdColors)
    annIDMenu.grid_forget()
    orderValsArr.append(annID)
    orderWidgetsArr.append(annIDLabel)
    orderWidgetsArr.append(annIDMenu)
    #Task options
    bufferLabel=tk.Label(orderFrame, text="")
    bufferLabel.grid(pady=5)
    taskAGV=tk.StringVar()
    taskAGV.set(agvOptions[0])
    taskAGVLabel=tk.Label(orderFrame, text="Select the agv for the task")
    taskAGVLabel.grid_forget()
    taskAgvMenu=tk.OptionMenu(orderFrame, taskAGV, *agvOptions)
    taskAgvMenu.grid_forget()
    orderValsArr.append(taskAGV)
    orderWidgetsArr.append(taskAGVLabel)
    orderWidgetsArr.append(taskAgvMenu)
    kitTrayId=tk.StringVar()
    kitTrayId.set(kittingTrayIDs[0])
    kitTrayIdLabel=tk.Label(orderFrame, text="Select the tray ID for the kitting task")
    kitTrayIdLabel.grid_forget()
    kitTrayIdMenu=tk.OptionMenu(orderFrame, kitTrayId, *kittingTrayIDs)
    kitTrayIdMenu.grid_forget()
    orderValsArr.append(kitTrayId)
    orderWidgetsArr.append(kitTrayIdLabel)
    orderWidgetsArr.append(kitTrayIdMenu)
    kittingDestination=tk.StringVar()
    kittingDestination.set(kittingDestinations[0])
    kittingDestinationLabel=tk.Label(orderFrame, text="Select the destination for kitting")
    kittingDestinationLabel.grid_forget()
    kittingDestinationMenu=tk.OptionMenu(orderFrame, kittingDestination, *kittingDestinations)
    kittingDestinationMenu.grid_forget()
    orderValsArr.append(kittingDestination)
    orderWidgetsArr.append(kittingDestinationLabel)
    orderWidgetsArr.append(kittingDestinationMenu)
    assemblyStation=tk.StringVar()
    assemblyStation.set("")
    assemblyStationLabel=tk.Label(orderFrame, text="Select the station for assembly")
    assemblyStationLabel.grid_forget()
    assemblyStationMenu=tk.OptionMenu(orderFrame, assemblyStation, *assemblyStations)
    assemblyStationMenu.grid_forget()
    orderValsArr.append(assemblyStation)
    orderWidgetsArr.append(assemblyStationLabel)
    orderWidgetsArr.append(assemblyStationMenu)
    orderID=tk.StringVar()
    orderValsArr.append(orderID)
    #Build the menus for kitting and assembly products
    kittingProdWidgets(orderFrame, kittingParts, kitValsArr, kitWidgetsArr,agv1Parts, agv2Parts, agv3Parts, agv4Parts)
    assemblyProdWidgets(orderFrame, assemblyParts, assemblyValsArr, assemblyWidgetsArr,agv1Parts, agv2Parts, agv3Parts, agv4Parts)
    #Add order button
    show_new_order_menu=partial(showNewOrderMenu,orderWidgetsArr, orderValsArr,usedIDs)
    addOrderButton=tk.Button(orderFrame, text="Add order", command=show_new_order_menu)
    addOrderButton.grid(column=1, row=5)
    #add product button
    type_of_prod_select=partial(typeOfProdSelect, orderType, kitValsArr, kitWidgetsArr, assemblyValsArr, assemblyWidgetsArr)
    addProdButton=tk.Button(orderFrame, text="Add product", command=type_of_prod_select)
    addProdButton.grid_forget()
    orderWidgetsArr.append(addProdButton)
    #save and back buttons
    save_order=partial(saveOrder, orderWidgetsArr, orderValsArr, kittingParts, assemblyParts, orderMSGS, orderConditions, partOrdCounter)
    saveOrdButton=tk.Button(orderFrame, text="Save order", command=save_order)
    saveOrdButton.grid_forget()
    orderWidgetsArr.append(saveOrdButton)
    back_order=partial(backOrder, orderWidgetsArr, orderValsArr, usedIDs)
    orderBackButton=tk.Button(orderFrame, text="Back", command=back_order)
    orderBackButton.grid_forget()
    orderWidgetsArr.append(orderBackButton)
    #update menu functions
    update_task_options=partial(updateTaskOptions, orderType, kitTrayId, taskAgvMenu,kitTrayIdLabel, kitTrayIdMenu, kittingDestination, kittingDestinationLabel, kittingDestinationMenu, assemblyStation, assemblyStationLabel, assemblyStationMenu)
    orderType.trace('w', update_task_options)
    show_correct_cond_menu=partial(showCorrectMenu,orderValsArr, orderWidgetsArr,usedIDs)
    condition.trace('w', show_correct_cond_menu)
    update_condition_menus=partial(updateConditionMenus,usedIDs, annID, conditionMenu, condition, annIDMenu)
    partOrdCounter.trace('w', update_condition_menus)