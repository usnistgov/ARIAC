import random
import string
import tkinter as tk
from functools import partial
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
def showNewOrderMenu(orderWidgetsArr, orderValsArr):
    orderValsArr[0].set(orderCategories[0])
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
    orderWidgetsArr[0].grid(column=2, row=0)
    orderWidgetsArr[1].grid(column=2, row=1)
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
    orderWidgetsArr[23].grid(column=2, row=20)
    #orderWidgetsArr[24].grid(column=2, row=21)
    orderWidgetsArr[25].grid(column=2, row=22)
    orderWidgetsArr[26].grid(column=2, row=23)
    

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

def saveKittingProd(kitValsArr, kitWidgetsArr):
    for widget in kitWidgetsArr:
        widget.grid_forget()

def saveAssemblyProd(assemblyValsArr, assemblyWidgetsArr):
    for widget in assemblyWidgetsArr:
        widget.grid_forget()
    
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
    elif orderType.get()!="kitting" and len(taskPresentFlag)==0:
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

def saveOrder(orderWidgetsArr):
    for widget in orderWidgetsArr:
        widget.grid_forget()

def kittingProdWidgets(orderFrame, kittingParts, kitValsArr, kitWidgetsArr):
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
    #save button
    save_kit_prod=partial(saveKittingProd, kitValsArr, kitWidgetsArr)
    saveKitButton=tk.Button(orderFrame, text="Save product", command=save_kit_prod)
    saveKitButton.grid_forget()
    kitWidgetsArr.append(saveKitButton)

def assemblyProdWidgets(orderFrame, assemblyParts, assemblyValsArr, assemblyWidgetsArr):
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
    assemblyValsArr.append(x_val)
    assemblyValsArr.append(y_val)
    assemblyValsArr.append(z_val)
    assemblyValsArr.append(r_val)
    assemblyValsArr.append(p_val)
    assemblyValsArr.append(y_rpy_val)
    #XYZ and RPY entry boxes
    x_val_label = tk.Label(orderFrame, text="Enter the x value")
    x_val_label.grid_forget()
    x_val_entry = tk.Entry(orderFrame, textvariable=x_val)
    x_val_entry.grid_forget()
    y_val_label = tk.Label(orderFrame, text="Enter the y value")
    y_val_label.grid_forget()
    y_val_entry = tk.Entry(orderFrame, textvariable=y_val)
    y_val_entry.grid_forget()
    z_val_label = tk.Label(orderFrame, text="Enter the z value")
    z_val_label.grid_forget()
    z_val_entry = tk.Entry(orderFrame, textvariable=z_val)
    z_val_entry.grid_forget()
    r_val_label = tk.Label(orderFrame, text="Enter the r value")
    r_val_label.grid_forget()
    r_val_entry = tk.Entry(orderFrame, textvariable=r_val)
    r_val_entry.grid_forget()
    p_val_label = tk.Label(orderFrame, text="Enter the p value")
    p_val_label.grid_forget()
    p_val_entry = tk.Entry(orderFrame, textvariable=p_val)
    p_val_entry.grid_forget()
    y_rpy_val_label = tk.Label(orderFrame, text="Enter the y (rpy) value")
    y_rpy_val_label.grid_forget()
    y_rpy_val_entry = tk.Entry(orderFrame, textvariable=y_rpy_val)
    y_rpy_val_entry.grid_forget()
    assemblyWidgetsArr.append(x_val_label)
    assemblyWidgetsArr.append(x_val_entry)
    assemblyWidgetsArr.append(y_val_label)
    assemblyWidgetsArr.append(y_val_entry)
    assemblyWidgetsArr.append(z_val_label)
    assemblyWidgetsArr.append(z_val_entry)
    assemblyWidgetsArr.append(r_val_label)
    assemblyWidgetsArr.append(r_val_entry)
    assemblyWidgetsArr.append(p_val_label)
    assemblyWidgetsArr.append(p_val_entry)
    assemblyWidgetsArr.append(y_rpy_val_label)
    assemblyWidgetsArr.append(y_rpy_val_entry)
    #assembly direction declarations
    x_dir=tk.StringVar()
    x_dir.set("0")
    y_dir=tk.StringVar()
    y_dir.set("0")
    z_dir=tk.StringVar()
    z_dir.set("0")
    assemblyValsArr.append(x_dir)
    assemblyValsArr.append(y_dir)
    assemblyValsArr.append(z_dir)
    #assembly direction entry boxes
    x_dir_label = tk.Label(orderFrame, text="Enter the x value for the assembly direction")
    x_dir_label.grid_forget()
    x_dir_entry = tk.Entry(orderFrame, textvariable=x_dir)
    x_dir_entry.grid_forget()
    y_dir_label = tk.Label(orderFrame, text="Enter the y value for the assembly direction")
    y_dir_label.grid_forget()
    y_dir_entry = tk.Entry(orderFrame, textvariable=y_dir)
    y_dir_entry.grid_forget()
    z_dir_label = tk.Label(orderFrame, text="Enter the z value for the assembly direction")
    z_dir_label.grid_forget()
    z_dir_entry = tk.Entry(orderFrame, textvariable=z_dir)
    z_dir_entry.grid_forget()
    assemblyWidgetsArr.append(x_dir_label)
    assemblyWidgetsArr.append(x_dir_entry)
    assemblyWidgetsArr.append(y_dir_label)
    assemblyWidgetsArr.append(y_dir_entry)
    assemblyWidgetsArr.append(z_dir_label)
    assemblyWidgetsArr.append(z_dir_entry)
    #save button
    save_assemb_prod=partial(saveKittingProd, assemblyValsArr, assemblyWidgetsArr)
    saveAssembButton=tk.Button(orderFrame, text="Save product", command=save_assemb_prod)
    saveAssembButton.grid_forget()
    assemblyWidgetsArr.append(saveAssembButton)

def orderWidgets(orderFrame, orderMSGS,orderConditions, usedIDs, kittingParts, assemblyParts):
    kitWidgetsArr=[]
    kitValsArr=[]
    assemblyWidgetsArr=[]
    assemblyValsArr=[]
    #generate the order id
    tempIDs=[]
    orderWidgetsArr=[]
    orderValsArr=[]
    for id in usedIDs:
        tempIDs.append(id)
    tempIDs.append(" ")
    orderID=generateOrderId(usedIDs)
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
    conditionMenu=tk.OptionMenu(orderFrame, condition, *conditionTypes)
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
    annIDMenu=tk.OptionMenu(orderFrame, annID, *tempIDs)
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
    #Build the menus for kitting and assembly products
    kittingProdWidgets(orderFrame, kittingParts, kitValsArr, kitWidgetsArr)
    assemblyProdWidgets(orderFrame, assemblyParts, assemblyValsArr, assemblyWidgetsArr)
    #Add order button
    show_new_order_menu=partial(showNewOrderMenu,orderWidgetsArr, orderValsArr)
    addOrderButton=tk.Button(orderFrame, text="Add order", command=show_new_order_menu)
    addOrderButton.grid(column=1, row=5)
    #add product button
    type_of_prod_select=partial(typeOfProdSelect, orderType, kitValsArr, kitWidgetsArr, assemblyValsArr, assemblyWidgetsArr)
    addProdButton=tk.Button(orderFrame, text="Add product", command=type_of_prod_select)
    addProdButton.grid_forget()
    orderWidgetsArr.append(addProdButton)
    #save and cancel buttons
    save_order=partial(saveOrder, orderWidgetsArr)
    saveOrdButton=tk.Button(orderFrame, text="Save order", command=save_order)
    saveOrdButton.grid_forget()
    orderWidgetsArr.append(saveOrdButton)
    #update menu functions
    update_task_options=partial(updateTaskOptions, orderType, kitTrayId, taskAgvMenu,kitTrayIdLabel, kitTrayIdMenu, kittingDestination, kittingDestinationLabel, kittingDestinationMenu, assemblyStation, assemblyStationLabel, assemblyStationMenu)
    orderType.trace('w', update_task_options)
    updateConditionMenu=partial(showCorrectMenu,orderValsArr, orderWidgetsArr,tempIDs)
    condition.trace('w', updateConditionMenu)