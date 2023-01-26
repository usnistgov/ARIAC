import random
import string
import tkinter as tk
from ariac_gui.checkCancel import *
from functools import partial
from ariac_gui.validationFunctions import *
from ariac_gui.newClasses import *
from ariac_gui.timeFunctions import *
from ariac_gui.msgNames import *
from ariac_msgs.msg import *
from geometry_msgs.msg import *
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
def typeOfProdSelect(kittingParts, assemblyParts, orderType):
    '''Runs the correct function based on the order type'''
    if orderType.get()=="kitting":
        addKittingProduct(kittingParts)
    else:
        addAssembProduct(assemblyParts)
    

def updateTaskOptions(orderType, kitTrayId, taskAgvMenu,kitTrayIdLabel, kitTrayIdMenu, kittingDestination, kittingDestinationLabel, kittingDestinationMenu, assemblyStation, assemblyStationLabel, assemblyStationMenu,a,b,c):
    '''Shows the correct options for different types of orders'''
    if orderType.get()=="kitting" and len(taskPresentFlag)>0:
        taskPresentFlag.clear()
        kitTrayId.set(kittingTrayIDs[0])
        kittingDestination.set(kittingDestinations[0])
        assemblyStation.set("")
        kitTrayIdLabel.pack(after=taskAgvMenu)
        kitTrayIdMenu.pack(after=kitTrayIdLabel)
        kittingDestinationLabel.pack(after=kitTrayIdMenu)
        kittingDestinationMenu.pack(after=kittingDestinationLabel)
        assemblyStationLabel.pack_forget()
        assemblyStationMenu.pack_forget()
    elif orderType.get()!="kitting" and len(taskPresentFlag)==0:
        taskPresentFlag.append(0)
        kitTrayId.set("")
        kittingDestination.set("")
        assemblyStation.set(assemblyStations[0])
        kitTrayIdLabel.pack_forget()
        kitTrayIdMenu.pack_forget()
        kittingDestinationLabel.pack_forget()
        kittingDestinationMenu.pack_forget()
        assemblyStationLabel.pack(after=taskAgvMenu)
        assemblyStationMenu.pack(after=assemblyStationLabel)

def generateOrderId(usedId):
    '''Generates a unique id for each order'''
    newId=''.join(random.choices(string.ascii_uppercase+string.digits,k=8))
    if newId in usedId:
        while newId in usedId:
            newId=''.join(random.choices(string.ascii_uppercase+string.digits,k=8))
    usedId.append(newId)
    return newId

def addKittingProduct(kittingParts):
    '''Adds a product to a kitting order'''
    kitProdWind=tk.Toplevel()
    kitProdWind.attributes('-fullscreen', True)
    #type of product
    prodType=tk.StringVar()
    prodType.set(allProdTypes[0])
    prodTypeLabel=tk.Label(kitProdWind, text="Select the type of product for the kitting task")
    prodTypeLabel.pack()
    prodTypeMenu=tk.OptionMenu(kitProdWind, prodType, *allProdTypes)
    prodTypeMenu.pack()
    #product color
    prodColor=tk.StringVar()
    prodColor.set(allProdColors[0])
    prodColorLabel=tk.Label(kitProdWind, text="Select the color of the product for the kitting task")
    prodColorLabel.pack()
    prodColorMenu=tk.OptionMenu(kitProdWind, prodColor, *allProdColors)
    prodColorMenu.pack()
    #product quadrant
    prodQuad=tk.StringVar()
    prodQuad.set(quadrants[0])
    prodQuadLabel=tk.Label(kitProdWind, text="Select the quadrant for the product")
    prodQuadLabel.pack()
    prodQuadMenu=tk.OptionMenu(kitProdWind, prodQuad, *quadrants)
    prodQuadMenu.pack()
    #save and cancel buttons
    kitProdCancelFlag=tk.StringVar()
    kitProdCancelFlag.set("0")
    saveKitProdButton=tk.Button(kitProdWind, text="Save and Exit", command=kitProdWind.destroy)
    saveKitProdButton.pack(pady=20)
    cancel_new_kit_prod=partial(cancel_func, kitProdWind, kitProdCancelFlag)
    cancelNewKitProdButton=tk.Button(kitProdWind, text="Cancel", command=cancel_new_kit_prod)
    cancelNewKitProdButton.pack(pady=20)
    kitProdWind.mainloop()
    if kitProdCancelFlag.get()=="0":
        newKittingPart=KittingPart()
        newPart=Part()
        if prodType.get()=="sensor":
            newPart.type=newPart.SENSOR
        elif prodType.get()=="pump":
            newPart.type=newPart.PUMP
        elif prodType.get()=="battery":
            newPart.type=newPart.BATTERY
        else:
            newPart.type=newPart.REGULATOR
        if prodColor.get()=="red":
            newPart.color=newPart.RED
        elif prodColor.get()=="green":
            newPart.color=newPart.GREEN
        elif prodColor.get()=="blue":
            newPart.color=newPart.BLUE
        elif prodColor.get()=="orange":
            newPart.color=newPart.ORANGE
        else:
            newPart.color=newPart.PURPLE
        newKittingPart.part=newPart
        newKittingPart.quadrant=int(prodQuad.get())
        kittingParts.append(newKittingPart)

def addAssembProduct(assemblyParts):
    '''Adds a product to an assembly or combined order'''
    assembProdWind=tk.Toplevel()
    assembProdWind.attributes('-fullscreen', True)
    #product type
    prodType=tk.StringVar()
    prodType.set(allProdTypes[0])
    prodTypeLabel=tk.Label(assembProdWind, text="Select the type of product for the assembly task")
    prodTypeLabel.pack()
    prodTypeMenu=tk.OptionMenu(assembProdWind, prodType, *allProdTypes)
    prodTypeMenu.pack()
    #product color
    prodColor=tk.StringVar()
    prodColor.set(allProdColors[0])
    prodColorLabel=tk.Label(assembProdWind, text="Select the color of the product for the assembly task")
    prodColorLabel.pack()
    prodColorMenu=tk.OptionMenu(assembProdWind, prodColor, *allProdColors)
    prodColorMenu.pack()
    #XYZ and RPY variable declarations
    x_val = tk.StringVar()
    x_val.set('0')
    y_val = tk.StringVar()
    y_val.set('0')
    z_val = tk.StringVar()
    z_val.set('0')
    r_val = tk.StringVar()
    r_val.set('0')
    p_val = tk.StringVar()
    p_val.set('0')
    y_rpy_val = tk.StringVar()
    y_rpy_val.set('0')
    #XYZ and RPY entry boxes
    x_val_label = tk.Label(assembProdWind, text="Enter the x value")
    x_val_label.pack()
    x_val_entry = tk.Entry(assembProdWind, textvariable=x_val)
    x_val_entry.pack()
    y_val_label = tk.Label(assembProdWind, text="Enter the y value")
    y_val_label.pack()
    y_val_entry = tk.Entry(assembProdWind, textvariable=y_val)
    y_val_entry.pack()
    z_val_label = tk.Label(assembProdWind, text="Enter the z value")
    z_val_label.pack()
    z_val_entry = tk.Entry(assembProdWind, textvariable=z_val)
    z_val_entry.pack()
    r_val_label = tk.Label(assembProdWind, text="Enter the r value")
    r_val_label.pack()
    r_val_entry = tk.Entry(assembProdWind, textvariable=r_val)
    r_val_entry.pack()
    p_val_label = tk.Label(assembProdWind, text="Enter the p value")
    p_val_label.pack()
    p_val_entry = tk.Entry(assembProdWind, textvariable=p_val)
    p_val_entry.pack()
    y_rpy_val_label = tk.Label(assembProdWind, text="Enter the y (rpy) value")
    y_rpy_val_label.pack()
    y_rpy_val_entry = tk.Entry(assembProdWind, textvariable=y_rpy_val)
    y_rpy_val_entry.pack()
    #assembly direction declarations
    x_dir=tk.StringVar()
    x_dir.set("0")
    y_dir=tk.StringVar()
    y_dir.set("0")
    z_dir=tk.StringVar()
    z_dir.set("0")
    #assembly direction entry boxes
    x_dir_label = tk.Label(assembProdWind, text="Enter the x value for the assembly direction")
    x_dir_label.pack()
    x_dir_entry = tk.Entry(assembProdWind, textvariable=x_dir)
    x_dir_entry.pack()
    y_dir_label = tk.Label(assembProdWind, text="Enter the y value for the assembly direction")
    y_dir_label.pack()
    y_dir_entry = tk.Entry(assembProdWind, textvariable=y_dir)
    y_dir_entry.pack()
    z_dir_label = tk.Label(assembProdWind, text="Enter the z value for the assembly direction")
    z_dir_label.pack()
    z_dir_entry = tk.Entry(assembProdWind, textvariable=z_dir)
    z_dir_entry.pack()
    #save and cancel buttons
    assembProdCancelFlag=tk.StringVar()
    assembProdCancelFlag.set("0")
    saveAssembProdButton=tk.Button(assembProdWind, text="Save and Exit", command=assembProdWind.destroy)
    saveAssembProdButton.pack(pady=20)
    cancel_new_assemb_prod=partial(cancel_func, assembProdWind, assembProdCancelFlag)
    cancelNewAssembProdButton=tk.Button(assembProdWind, text="Cancel", command=cancel_new_assemb_prod)
    cancelNewAssembProdButton.pack(pady=20)
    assembProdWind.mainloop()
    if assembProdCancelFlag.get()=="0":
        newAssembPart=AssemblyPart()
        newPart=Part()
        if prodType.get()=="sensor":
            newPart.type=newPart.SENSOR
        elif prodType.get()=="pump":
            newPart.type=newPart.PUMP
        elif prodType.get()=="battery":
            newPart.type=newPart.BATTERY
        else:
            newPart.type=newPart.REGULATOR
        if prodColor.get()=="red":
            newPart.color=newPart.RED
        elif prodColor.get()=="green":
            newPart.color=newPart.GREEN
        elif prodColor.get()=="blue":
            newPart.color=newPart.BLUE
        elif prodColor.get()=="orange":
            newPart.color=newPart.ORANGE
        else:
            newPart.color=newPart.PURPLE
        newAssembPart.part=newPart
        newDirection=Vector3()
        newDirection.x =float(x_dir.get())
        newDirection.y=float(y_dir.get())
        newDirection.z=float(z_dir.get())
        newAssembPart.install_direction=newDirection
        newPoint=Point()
        newPoint.x=float(x_val.get())
        newPoint.y=float(y_val.get())
        newPoint.z=float(z_val.get())
        assemblyParts.append(newAssembPart)

def updateQuadMenu(orderNum, orderQuadrant, orderQuadMenu, orderPriorityCheckBox, orderQuadLabel, a,b,c):
    '''Updates the quadrant menu'''
    if orderNum.get()!=" " and len(currentQuadMenu)==0:
        orderQuadrant.set('0')
        orderQuadMenu.pack(before=orderPriorityCheckBox)
        orderQuadLabel.pack(before=orderQuadMenu)
        currentQuadMenu.append(0)
    elif orderNum.get()==" ":
        orderQuadrant.set(' ')
        orderQuadMenu.pack_forget()
        orderQuadLabel.pack_forget()
        for i in currentQuadMenu:
            currentQuadMenu.remove(i)

def showAGVMenu(agvShow,agvShowCB, agvMenu, agvLabel, agv,quadLabel, quadMenu, quad, a,b,c):
    if agvShow.get()=="1":
        agvLabel.pack(after=agvShowCB)
        agvMenu.pack(after=agvLabel)
        quadLabel.pack(after=agvMenu)
        quadMenu.pack(after=quadLabel)
        agv.set(agvOptions[0])
        quad.set(quadrants[0])
    else:
        agvLabel.pack_forget()
        agvMenu.pack_forget()
        quadLabel.pack_forget()
        quadMenu.pack_forget()
        agv.set('')
        quad.set('')

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
        partType.set(allProdTypes[0])
        partColor.set(allProdColors[0])
    else:
        partTypeLabel.pack_forget()
        partTypeMenu.pack_forget()
        partColorLabel.pack_forget()
        partColorMenu.pack_forget()
        partType.set("")
        partColor.set("")

def showAnnIDMenu(annIDShow, annIDShowCB, annIDLabel, annIDMenu, annID, tempIDs, a,b,c):
    if annIDShow.get()=="1":
        annIDLabel.pack(after=annIDShowCB)
        annIDMenu.pack(after=annIDLabel)
        annID.set(tempIDs[0])
    else:
        annIDLabel.pack_forget()
        annIDMenu.pack_forget()
        annID.set("")

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
        partType.set(allProdTypes[0])
        partColor.set(allProdColors[0])
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

def addNewOrder(orderMSGS,orderConditions, orderCounter, usedIDs, mainWind):
    '''Window for adding a new order'''
    taskPresentFlag.clear()
    orderCounter.append(0)
    tempIDs=[]
    for id in usedIDs:
        tempIDs.append(id)
    orderID=generateOrderId(usedIDs)
    kittingParts=[]
    assemblyParts=[]
    newOrderWind=tk.Toplevel()
    #newOrderWind.geometry("850x800")
    newOrderWind.attributes('-fullscreen', True)
    #orderCategory
    orderCategory=tk.StringVar()
    orderCategory.set(orderCategories[0])
    orderCategoryLabel=tk.Label(newOrderWind, text="Select the category of the order")
    orderCategoryLabel.pack()
    orderCategoryMenu=tk.OptionMenu(newOrderWind, orderCategory, *orderCategories)
    orderCategoryMenu.pack()
    #order type
    orderType=tk.StringVar()
    orderType.set(orderTypes[0])
    orderTypeSelectionLabel=tk.Label(newOrderWind, text="Select the type of order")
    orderTypeSelectionLabel.pack()
    orderTypeSelectionMenu=tk.OptionMenu(newOrderWind, orderType, *orderTypes)
    orderTypeSelectionMenu.pack()
    #Priority
    orderPriority=tk.StringVar()
    orderPriority.set('0')
    orderPriorityCheckBox=tk.Checkbutton(newOrderWind, text="Priority", variable=orderPriority, onvalue="1", offvalue="0", height=1, width=20)
    orderPriorityCheckBox.pack()
    #announcement
    condition=tk.StringVar()
    condition.set(conditionTypes[0])
    conditionLabel=tk.Label(newOrderWind, text="Select a condition for the order")
    conditionLabel.pack()
    if len(tempIDs)>0:
        conditionMenu=tk.OptionMenu(newOrderWind, condition, *conditionTypes)
    else:
        conditionMenu=tk.OptionMenu(newOrderWind, condition, *conditionTypes[:-1])
    conditionMenu.pack()
    time=tk.StringVar()
    time.set('0')
    timeLabel=tk.Label(newOrderWind, text="Enter the time")
    timeLabel.pack()
    timeEntry=tk.Entry(newOrderWind, textvariable=time)
    timeEntry.pack()
    agv=tk.StringVar()
    agv.set("")
    agvLabel=tk.Label(newOrderWind, text="Choose the agv")
    agvLabel.pack_forget()
    agvMenu=tk.OptionMenu(newOrderWind, agv, *agvOptions)
    agvMenu.pack_forget()
    partType=tk.StringVar()
    partType.set("")
    partTypeLabel=tk.Label(newOrderWind, text="Select the type of part")
    partTypeLabel.pack_forget()
    partTypeMenu=tk.OptionMenu(newOrderWind, partType, *allProdTypes)
    partTypeMenu.pack_forget()
    partColor=tk.StringVar()
    partColor.set("")
    partColorLabel=tk.Label(newOrderWind, text="Select the color of the part")
    partColorLabel.pack_forget()
    partColorMenu=tk.OptionMenu(newOrderWind, partColor, *allProdColors)
    partColorMenu.pack_forget()
    if len(tempIDs)>0:
        annID=tk.StringVar()
        annID.set("")
        annIDLabel=tk.Label(newOrderWind, text="Select the order ID")
        annIDLabel.pack_forget()
        annIDMenu=tk.OptionMenu(newOrderWind, annID, *tempIDs)
        annIDMenu.pack_forget()
    else:
        annID=tk.StringVar()
        annID.set("")
        annIDLabel=tk.Label(newOrderWind, text="Select the order ID")
        annIDLabel.pack_forget()
        annIDMenu=tk.OptionMenu(newOrderWind, annID, *allProdTypes) #Dummy values for first iteration because tempIDs is empty for the first order
        annIDMenu.pack_forget()
    #Task options
    bufferLabel=tk.Label(newOrderWind, text="")
    bufferLabel.pack(pady=5)
    taskAGV=tk.StringVar()
    taskAGV.set(agvOptions[0])
    taskAGVLabel=tk.Label(newOrderWind, text="Select the agv for the task")
    taskAGVLabel.pack()
    taskAgvMenu=tk.OptionMenu(newOrderWind, taskAGV, *agvOptions)
    taskAgvMenu.pack()
    kitTrayId=tk.StringVar()
    kitTrayId.set(kittingTrayIDs[0])
    kitTrayIdLabel=tk.Label(newOrderWind, text="Select the tray ID for the kitting task")
    kitTrayIdLabel.pack()
    kitTrayIdMenu=tk.OptionMenu(newOrderWind, kitTrayId, *kittingTrayIDs)
    kitTrayIdMenu.pack()
    kittingDestination=tk.StringVar()
    kittingDestination.set(kittingDestinations[0])
    kittingDestinationLabel=tk.Label(newOrderWind, text="Select the destination for kitting")
    kittingDestinationLabel.pack()
    kittingDestinationMenu=tk.OptionMenu(newOrderWind, kittingDestination, *kittingDestinations)
    kittingDestinationMenu.pack()
    assemblyStation=tk.StringVar()
    assemblyStation.set("")
    assemblyStationLabel=tk.Label(newOrderWind, text="Select the station for assembly")
    assemblyStationLabel.pack_forget()
    assemblyStationMenu=tk.OptionMenu(newOrderWind, assemblyStation, *assemblyStations)
    assemblyStationMenu.pack_forget()
    #add product button
    type_of_prod_select=partial(typeOfProdSelect, kittingParts, assemblyParts, orderType)
    addProdButton=tk.Button(newOrderWind, text="Add product", command=type_of_prod_select)
    addProdButton.pack()
    #save and cancel buttons
    saveOrdButton=tk.Button(newOrderWind, text="Save and Exit", command=mainWind.destroy)
    saveOrdButton.pack()
    ordCancelFlag=tk.StringVar()
    ordCancelFlag.set('0')
    cancel_new_ord_part=partial(cancel_func, newOrderWind, ordCancelFlag)
    cancelNewOrdButton=tk.Button(newOrderWind, text="Cancel", command=cancel_new_ord_part)
    cancelNewOrdButton.pack(pady=20)
    #update menu functions
    update_task_options=partial(updateTaskOptions, orderType, kitTrayId, taskAgvMenu,kitTrayIdLabel, kitTrayIdMenu, kittingDestination, kittingDestinationLabel, kittingDestinationMenu, assemblyStation, assemblyStationLabel, assemblyStationMenu)
    orderType.trace('w', update_task_options)
    updateConditionMenu=partial(showCorrectMenu,condition, conditionMenu, time, timeLabel, timeEntry, agv, agvLabel, agvMenu, partType, partTypeLabel, partTypeMenu, partColor, partColorLabel, partColorMenu, annID, annIDLabel, annIDMenu,tempIDs)
    condition.trace('w', updateConditionMenu)
    newOrderWind.mainloop()
    if ordCancelFlag.get()=="1":
        orderCounter.remove(0)
    else:
        newOrder=Order()
        newOrder.id=orderID
        newOrder.type=orderTypes.index(orderType.get())
        if orderPriority.get()=="0":
            newOrder.priority=False
        else:
            newOrder.priority=True
        if orderType.get()=="kitting":
            newKittingTask=KittingTask()
            newKittingTask.agv_number=int(taskAGV.get())
            newKittingTask.tray_id=int(kitTrayId.get())
            if kittingDestination.get()=="warehouse":
                newKittingTask.destination=newKittingTask.WAREHOUSE
            elif kittingDestination.get()=="kitting":
                newKittingTask.destination=newKittingTask.KITTING
            elif kittingDestination.get()=="assembly_front":
                newKittingTask.destination=newKittingTask.ASSEMBLY_FRONT
            else:
                newKittingTask.destination=newKittingTask.ASSENBLY_BACK
            newKittingTask.parts=kittingParts
            newOrder.kitting_task=newKittingTask
        elif orderType.get()=="assembly":
            agvNumList=[]
            agvNumList.append(int(taskAGV.get()))
            newAssemblyTask=AssemblyTask()
            newAssemblyTask.agv_numbers=agvNumList
            newAssemblyTask.station=assemblyStations.index(assemblyStation.get())+1
            newAssemblyTask.parts=assemblyParts
            newOrder.assembly_task=newAssemblyTask
        else:
            agvNumList=[]
            agvNumList.append(int(taskAGV.get()))
            newCombinedTask=CombinedTask()
            newCombinedTask.station=assemblyStations.index(assemblyStation.get())+1
            newCombinedTask.parts=assemblyParts
            newOrder.combined_task=newCombinedTask
        orderMSGS.append(newOrder)
        orderCondition=Condition()
        orderCondition.type=conditionTypes.index(condition.get())
        if condition.get()==conditionTypes[0]:
            orderCondition.time_condition.seconds=float(time.get())
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
            orderCondition.part_place_condition.part=newPart
            orderCondition.part_place_condition.agv=int(agv.get())
        elif condition.get()==conditionTypes[2]:
            orderCondition.submission_condition.order_id=annID.get()
        orderConditions.append(orderCondition)

def saveOrders(wind, ordersFlag): # allows the while loop in main to stop so the orders window stops when the user saves
    ordersFlag.set('0')
    wind.destroy()

def runOrdersWind(orderMSGS, orderConditions, orderCounter, usedIDs, ordersFlag, mainWind):
    ordersWind=tk.Toplevel()
    ordersFlag.set('1')
    ordersWind.title("Orders")
    #ordersWind.geometry("850x600")
    ordersWind.attributes('-fullscreen', True)
    new_order_func=partial(addNewOrder, orderMSGS, orderConditions, orderCounter, usedIDs, mainWind)
    newOrderButton=tk.Button(ordersWind, text="New Order", command=new_order_func)
    newOrderButton.pack()
    currentOrdersVal="Current Orders:\n"
    if len(orderMSGS)==0:
        currentOrdersVal+="NONE"
    else:
        c=0
        for order in orderMSGS:
            currentOrdersVal+="\nOrder "+str(c)+":\n"
            currentOrdersVal+="ID: "+order.id+"  Type: "+getOrderType(order.type)+"\n"
            currentOrdersVal+="Number of parts: "
            if order.type==0:
                currentOrdersVal+=str(len(order.kitting_task.parts))
            elif order.type==1:
                currentOrdersVal+=str(len(order.assembly_task.parts))
            else:
                currentOrdersVal+=str(len(order.combined_task.parts))
            currentOrdersVal+="\n"
            c+=1
    currentOrdersLabel=tk.Label(ordersWind, text=currentOrdersVal)
    currentOrdersLabel.pack()
    #save and cancel buttons
    save_orders=partial(saveOrders, mainWind, ordersFlag)
    saveOrdersButton=tk.Button(ordersWind, text="Save and return to main menu", command=save_orders)
    saveOrdersButton.pack(pady=20)
    ordersWind.mainloop()