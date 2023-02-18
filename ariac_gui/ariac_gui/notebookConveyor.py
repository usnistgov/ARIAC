import tkinter as tk
from functools import partial
acceptedNum = "0123456789.-"  # for requiring number input for offset value
partTypes=["sensor", "pump", "regulator", "battery"]
partColors=['green', 'red', 'purple','blue','orange']
convOrders=["random", "sequential"]
def validateOffset(offsetVal, a,b,c):
    '''Validates the value that the user inputs for the offset'''
    perFlag=0
    negFlag=0
    tempStr=offsetVal.get()
    for i in tempStr:
        if i not in acceptedNum:
            tempStr=tempStr.replace(i, "")
    if tempStr.count('.')>0:
        for i in range(len(tempStr)):
            if tempStr[i]=='.' and perFlag==0:
                perFlag=1
            elif tempStr[i]=='.':
                tempStr=tempStr[:i]+tempStr[i+1:]
                break
    if tempStr.count('-')>0:
        for i in range(len(tempStr)):
            if tempStr[i]=='-' and negFlag==0:
                negFlag=1
            elif tempStr[i]=='-':
                tempStr=tempStr[:i]+tempStr[i+1:]
                break
    if tempStr!="":
        if float(tempStr)>1:
            tempStr="1"
        elif float(tempStr)<-1:
            tempStr="-1"
    offsetVal.set(tempStr)

def showAndHideConvButton(addConvButton, val, convOptionFlag, saveNewConvButton,a,b,c):
    if val.get()=="":
        addConvButton.pack(side = tk.BOTTOM)
        saveNewConvButton.pack_forget()
        convOptionFlag.set('0')
    elif convOptionFlag.get()=="0":
        saveNewConvButton.pack(side = tk.BOTTOM)
        addConvButton.pack_forget()
        convOptionFlag.set('1')

def saveConv(convValsArr, convWidgetsArr,convFlag, convSettingsWidgets):
    switchConvMenu(convValsArr, convWidgetsArr,convFlag, convSettingsWidgets)

def switchConvMenu(convValsArr, convWidgetsArr,convFlag, convSettingsWidgets):
    if convFlag.get()=="0":
        convValsArr[0].set(partTypes[0])
        convValsArr[1].set(partColors[0])
        convValsArr[2].set('0')
        convValsArr[3].set('0')
        convValsArr[4].set('0')
        for widget in convWidgetsArr:
            widget.pack()
        for widget in convSettingsWidgets:
            widget.pack_forget()
        convFlag.set('1')
    else:
        for val in convValsArr:
            val.set('')
        for widget in convWidgetsArr:
            widget.pack_forget()
        for widget in convSettingsWidgets:
            widget.pack()
        convFlag.set('0')

def convWidgets(convFrame):
    convFlag=tk.StringVar()
    convFlag.set('0')
    convOptionFlag=tk.StringVar()
    convOptionFlag.set('0')
    convValsArr=[]
    convWidgetsArr=[]
    convSettingsWidgets=[]
    #part type
    convPartType=tk.StringVar()
    convPartType.set(partTypes[0])
    convPartTypeSelectLabel=tk.Label(convFrame, text="Select the type of part")
    convPartTypeSelectLabel.pack_forget()
    convPartTypeSelectMenu=tk.OptionMenu(convFrame, convPartType, *partTypes)
    convPartTypeSelectMenu.pack_forget()
    convValsArr.append(convPartType)
    convWidgetsArr.append(convPartTypeSelectLabel)
    convWidgetsArr.append(convPartTypeSelectMenu)
    #part color selection
    convPartColor=tk.StringVar()
    convPartColor.set(partColors[0])
    convPartColorSelectLabel=tk.Label(convFrame, text="Select the color of the part")
    convPartColorSelectLabel.pack_forget()
    convPartColorSelectMenu=tk.OptionMenu(convFrame, convPartColor, *partColors)
    convPartColorSelectMenu.pack_forget()
    convValsArr.append(convPartColor)
    convWidgetsArr.append(convPartColorSelectLabel)
    convWidgetsArr.append(convPartColorSelectMenu)
    #number
    numberParts=tk.StringVar()
    numberParts.set('0')
    numberLabel=tk.Label(convFrame, text="Enter the number of parts")
    numberLabel.pack_forget()
    numberEntry=tk.Entry(convFrame, textvariable=numberParts)
    numberEntry.pack_forget()
    convValsArr.append(numberParts)
    convWidgetsArr.append(numberLabel)
    convWidgetsArr.append(numberEntry)
    #offset
    offsetParts=tk.StringVar()
    offsetParts.set('0')
    offsetLabel=tk.Label(convFrame, text="Enter the offset for the part")
    offsetLabel.pack_forget()
    offsetEntry=tk.Entry(convFrame, textvariable=offsetParts)
    offsetEntry.pack_forget()
    convValsArr.append(offsetParts)
    convWidgetsArr.append(offsetLabel)
    convWidgetsArr.append(offsetEntry)
    #rotation entry
    partRotation=tk.StringVar()
    partRotation.set('0')
    partRotationLabel=tk.Label(convFrame, text="Enter the rotation of the part")
    partRotationLabel.pack_forget()
    partRotationEntry=tk.Entry(convFrame, textvariable=partRotation)
    partRotationEntry.pack_forget()
    convValsArr.append(partRotation)
    convWidgetsArr.append(partRotationLabel)
    convWidgetsArr.append(partRotationEntry)

    #conveyor belt settings
    spawnRate=tk.StringVar()
    spawnRate.set('0')
    convOrder=tk.StringVar()
    convOrder.set(convOrders[0])
    convActive=tk.StringVar()
    convActive.set('0')
    conveyorBeltLabel=tk.Label(convFrame, text="Conveyor Belt Settings")
    conveyorBeltLabel.pack()
    activeCheck=tk.Checkbutton(convFrame, text="Active", variable=convActive, onvalue="1", offvalue="0", height=3, width=20)
    activeCheck.pack()
    spawnRateEntryLabel=tk.Label(convFrame, text="Enter the spawn rate for the conveyor belt")
    spawnRateEntryLabel.pack()
    spawnRateEntry=tk.Entry(convFrame, textvariable=spawnRate)
    spawnRateEntry.pack()
    convOrderLabel=tk.Label(convFrame, text="Order of the conveyor belt")
    convOrderLabel.pack()
    convOrderMenu=tk.OptionMenu(convFrame, convOrder, *convOrders)
    convOrderMenu.pack()
    convSettingsWidgets.append(conveyorBeltLabel)
    convSettingsWidgets.append(activeCheck)
    convSettingsWidgets.append(spawnRateEntryLabel)
    convSettingsWidgets.append(spawnRateEntry)
    convSettingsWidgets.append(convOrderLabel)
    convSettingsWidgets.append(convOrderMenu)

    #save conveyor button
    save_and_exit_conv=partial(saveConv, convValsArr, convWidgetsArr,convFlag, convSettingsWidgets)
    saveConvButton=tk.Button(convFrame, text="Save and Exit", command=save_and_exit_conv)
    saveConvButton.pack_forget()
    #add conv button
    show_add_button=partial(switchConvMenu,convValsArr, convWidgetsArr,convFlag, convSettingsWidgets)
    addConvButton=tk.Button(convFrame,text="Add conveyor part", command=show_add_button)
    addConvButton.pack(side=tk.BOTTOM)
    switch_buttons=partial(showAndHideConvButton, addConvButton, convValsArr[0], convOptionFlag, saveConvButton)
    convValsArr[0].trace('w', switch_buttons)
    #entry validation
    validate_offset=partial(validateOffset, offsetParts)
    offsetParts.trace('w', validate_offset)