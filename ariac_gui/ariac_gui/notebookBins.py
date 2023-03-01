import tkinter as tk
from functools import partial
from ariac_gui.newClasses import Bin
from ariac_gui.validationFunctions import validateRotationValue
partTypes=["sensor", "pump", "regulator", "battery"]
partColors=['green', 'red', 'purple','blue','orange']
allBins=[]
checkBoxes=[]
for i in range(8):
    allBins.append('bin'+str(i+1))

def runSlotChecks(addBinWind, currentBin,slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots,saveNewBinButton,binOptionFlag, a,b,c):
    '''runs the slotChecks function for the correct bin'''
    if binOptionFlag.get()=="1":
        if currentBin.get()=="bin1":
            slotChecks(bin1Slots, addBinWind, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,saveNewBinButton)
        elif currentBin.get()=="bin2":
            slotChecks(bin2Slots, addBinWind, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,saveNewBinButton)
        elif currentBin.get()=="bin3":
            slotChecks(bin3Slots, addBinWind, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,saveNewBinButton)
        elif currentBin.get()=="bin4":
            slotChecks(bin4Slots, addBinWind, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,saveNewBinButton)
        elif currentBin.get()=="bin5":
            slotChecks(bin5Slots, addBinWind, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,saveNewBinButton)
        elif currentBin.get()=="bin6":
            slotChecks(bin6Slots, addBinWind, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,saveNewBinButton)
        elif currentBin.get()=="bin7":
            slotChecks(bin7Slots, addBinWind, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,saveNewBinButton)
        elif currentBin.get()=="bin8":
            slotChecks(bin8Slots, addBinWind, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,saveNewBinButton)

def slotChecks(arr, addBinWind, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, presentChecks,saveNewBinButton):
    '''puts the correct checkboxes in the window. This guarantees that slots are not repeated for bins'''
    firstFlag=0
    for i in presentChecks:
        i.pack_forget()
    slot1.set("0")
    slot2.set("0")
    slot3.set("0")
    slot4.set("0")
    slot5.set("0")
    slot6.set("0")
    slot7.set("0")
    slot8.set("0")
    slot9.set("0")
    if "1" in arr:
        slot1Check=tk.Checkbutton(addBinWind, text="Slot 1", variable=slot1, onvalue="1", offvalue="0", height=1, width=20)
        slot1Check.pack(before=saveNewBinButton)
        presentChecks.append(slot1Check)
        firstFlag=1
    if "2" in arr:
        slot2Check=tk.Checkbutton(addBinWind, text="Slot 2", variable=slot2, onvalue="1", offvalue="0", height=1, width=20)
        if firstFlag==1:
            slot2Check.pack(after=presentChecks[len(presentChecks)-1])
        else:
            slot2Check.pack(before=saveNewBinButton)
            firstFlag=1
        presentChecks.append(slot2Check)
    if "3" in arr:
        slot3Check=tk.Checkbutton(addBinWind, text="Slot 3", variable=slot3, onvalue="1", offvalue="0", height=1, width=20)
        if firstFlag==1:
            slot3Check.pack(after=presentChecks[len(presentChecks)-1])
        else:
            slot3Check.pack(before=saveNewBinButton)
            firstFlag=1
        presentChecks.append(slot3Check)
    if "4" in arr:
        slot4Check=tk.Checkbutton(addBinWind, text="Slot 4", variable=slot4, onvalue="1", offvalue="0", height=1, width=20)
        if firstFlag==1:
            slot4Check.pack(after=presentChecks[len(presentChecks)-1])
        else:
            slot4Check.pack(before=saveNewBinButton)
            firstFlag=1
        presentChecks.append(slot4Check)
    if "5" in arr:
        slot5Check=tk.Checkbutton(addBinWind, text="Slot 5", variable=slot5, onvalue="1", offvalue="0", height=1, width=20)
        if firstFlag==1:
            slot5Check.pack(after=presentChecks[len(presentChecks)-1])
        else:
            slot5Check.pack(before=saveNewBinButton)
            firstFlag=1
        presentChecks.append(slot5Check)
    if "6" in arr:
        slot6Check=tk.Checkbutton(addBinWind, text="Slot 6", variable=slot6, onvalue="1", offvalue="0", height=1, width=20)
        if firstFlag==1:
            slot6Check.pack(after=presentChecks[len(presentChecks)-1])
        else:
            slot6Check.pack(before=saveNewBinButton)
            firstFlag=1
        presentChecks.append(slot6Check)
    if "7" in arr:
        slot7Check=tk.Checkbutton(addBinWind, text="Slot 7", variable=slot7, onvalue="1", offvalue="0", height=1, width=20)
        if firstFlag==1:
            slot7Check.pack(after=presentChecks[len(presentChecks)-1])
        else:
            slot7Check.pack(before=saveNewBinButton)
            firstFlag=1
        presentChecks.append(slot7Check)
    if "8" in arr:
        slot8Check=tk.Checkbutton(addBinWind, text="Slot 8", variable=slot8, onvalue="1", offvalue="0", height=1, width=20)
        if firstFlag==1:
            slot8Check.pack(after=presentChecks[len(presentChecks)-1])
        else:
            slot8Check.pack(before=saveNewBinButton)
            firstFlag=1
        presentChecks.append(slot8Check)
    if "9" in arr:
        slot9Check=tk.Checkbutton(addBinWind, text="Slot 9", variable=slot9, onvalue="1", offvalue="0", height=1, width=20)
        if firstFlag==1:
            slot9Check.pack(after=presentChecks[len(presentChecks)-1])
        else:
            slot9Check.pack(before=saveNewBinButton)
            firstFlag=1
        presentChecks.append(slot9Check)

def updateAvailableSlots(currentBin, bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9,window, mainWind):
    '''Removes slots that have been selected by the user'''
    allSlots=[slot1.get(),slot2.get(),slot3.get(),slot4.get(),slot5.get(),slot6.get(),slot7.get(),slot8.get(),slot9.get()]
    counter=1
    if currentBin.get()=="bin1":
        for i in allSlots:
            if i=="1":
                bin1Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin2":
        for i in allSlots:
            if i=="1":
                bin2Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin3":
        for i in allSlots:
            if i=="1":
                bin3Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin4":
        for i in allSlots:
            if i=="1":
                bin4Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin5":
        for i in allSlots:
            if i=="1":
                bin5Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin6":
        for i in allSlots:
            if i=="1":
                bin6Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin7":
        for i in allSlots:
            if i=="1":
                bin7Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin8":
        for i in allSlots:
            if i=="1":
                bin8Slots.remove(str(counter))
            counter+=1
    else:
        print("Error")
    mainWind.destroy()

def switchBinMenu(binWidgetsArr, binValsArr,binFlag):
    if binFlag.get()=="0":
        binValsArr[0].set('bin1')
        binValsArr[1].set(partTypes[0])
        binValsArr[2].set(partColors[0])
        for i in range(3, 12):
            binValsArr[i].set('0')
        binValsArr[12].set('0')
        binValsArr[13].set('0')
        for widget in binWidgetsArr:
            widget.pack()
        binFlag.set('1')
    else:
        for val in binValsArr:
            val.set('')
        for widget in binWidgetsArr:
            widget.pack_forget()
        for cb in checkBoxes:
            cb.pack_forget()
        checkBoxes.clear()
        binFlag.set('0')

def showAndHideBinButton(addBinButton, val, binOptionFlag,bin1Slots, binFrame, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, saveNewBinButton,backButton,a,b,c):
    if val.get()=="":
        addBinButton.pack(side = tk.BOTTOM)
        saveNewBinButton.pack_forget()
        backButton.pack_forget()
        binOptionFlag.set('0')
    elif binOptionFlag.get()=="0":
        saveNewBinButton.pack(side = tk.BOTTOM)
        backButton.pack(side=tk.BOTTOM)
        slotChecks(bin1Slots, binFrame, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, checkBoxes, saveNewBinButton)
        addBinButton.pack_forget()
        binOptionFlag.set('1')

def saveBin(currentBin, bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9,binWidgetsArr, binValsArr,binFlag, bins, partOrdCounter):
    allSlots=[slot1.get(),slot2.get(),slot3.get(),slot4.get(),slot5.get(),slot6.get(),slot7.get(),slot8.get(),slot9.get()]
    counter=1
    if currentBin.get()=="bin1":
        for i in allSlots:
            if i=="1":
                bin1Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin2":
        for i in allSlots:
            if i=="1":
                bin2Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin3":
        for i in allSlots:
            if i=="1":
                bin3Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin4":
        for i in allSlots:
            if i=="1":
                bin4Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin5":
        for i in allSlots:
            if i=="1":
                bin5Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin6":
        for i in allSlots:
            if i=="1":
                bin6Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin7":
        for i in allSlots:
            if i=="1":
                bin7Slots.remove(str(counter))
            counter+=1
    elif currentBin.get()=="bin8":
        for i in allSlots:
            if i=="1":
                bin8Slots.remove(str(counter))
            counter+=1
    else:
        print("Error")
    selectedSlots=[]
    allSlots=[slot1.get(),slot2.get(),slot3.get(),slot4.get(),slot5.get(),slot6.get(),slot7.get(),slot8.get(),slot9.get()]
    counter=1
    for i in allSlots:
        if i=="1":
            selectedSlots.append(str(counter))
        counter+=1
    slotsString=",".join(selectedSlots)
    rotationVal=binValsArr[12].get()
    if 'pi' in binValsArr[12].get():
        rotationVal=rotationVal.replace('\'',"")
        rotationVal='\''+rotationVal+'\''
    bins.append(Bin(binValsArr[0].get(),binValsArr[1].get(), binValsArr[2].get(),"["+slotsString+"]",rotationVal, binValsArr[13].get()))
    switchBinMenu(binWidgetsArr, binValsArr,binFlag)
    currVal=int(partOrdCounter.get())
    partOrdCounter.set(currVal+1)

def backBin(binWidgetsArr, binValsArr, binFlag):
    switchBinMenu(binWidgetsArr, binValsArr,binFlag)

def binWidgets(binFrame,bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots, bins, partOrdCounter):
    binWidgetsArr=[]
    binValsArr=[]
    binFlag=tk.StringVar()
    binFlag.set('0')
    binOptionFlag=tk.StringVar()
    binOptionFlag.set('0')
    binID=tk.StringVar()
    binID.set(allBins[0])
    binSelectLabel=tk.Label(binFrame, text="Select the bin")
    binSelectLabel.pack_forget()
    binSelectMenu=tk.OptionMenu(binFrame, binID, *allBins)
    binSelectMenu.pack_forget()
    binValsArr.append(binID)
    binWidgetsArr.append(binSelectLabel)
    binWidgetsArr.append(binSelectMenu)
    #part type
    partType=tk.StringVar()
    partType.set(partTypes[0])
    partTypeLabel=tk.Label(binFrame, text="Select the type of part")
    partTypeLabel.pack_forget()
    partTypeMenu=tk.OptionMenu(binFrame, partType, *partTypes)
    partTypeMenu.pack_forget()
    binValsArr.append(partType)
    binWidgetsArr.append(partTypeLabel)
    binWidgetsArr.append(partTypeMenu)
    #part color
    partColor=tk.StringVar()
    partColor.set(partColors[0])
    partColorSelectLabel=tk.Label(binFrame, text="Select the color of the part")
    partColorSelectLabel.pack_forget()
    partColorSelectMenu=tk.OptionMenu(binFrame, partColor, *partColors)
    partColorSelectMenu.pack_forget()
    binValsArr.append(partColor)
    binWidgetsArr.append(partColorSelectLabel)
    binWidgetsArr.append(partColorSelectMenu)
    #slots
    slot1=tk.StringVar()
    slot2=tk.StringVar()
    slot3=tk.StringVar()
    slot4=tk.StringVar()
    slot5=tk.StringVar()
    slot6=tk.StringVar()
    slot7=tk.StringVar()
    slot8=tk.StringVar()
    slot9=tk.StringVar()
    slot1.set("0")
    slot2.set("0")
    slot3.set("0")
    slot4.set("0")
    slot5.set("0")
    slot6.set("0")
    slot7.set("0")
    slot8.set("0")
    slot9.set("0")
    binValsArr.append(slot1)
    binValsArr.append(slot2)
    binValsArr.append(slot3)
    binValsArr.append(slot4)
    binValsArr.append(slot5)
    binValsArr.append(slot6)
    binValsArr.append(slot7)
    binValsArr.append(slot8)
    binValsArr.append(slot9)
    #rotation
    partRotation=tk.StringVar()
    partRotation.set('0')
    partRotationLabel=tk.Label(binFrame, text="Enter the rotation of the part")
    partRotationLabel.pack_forget()
    partRotationEntry=tk.Entry(binFrame, textvariable=partRotation)
    partRotationEntry.pack_forget()
    binValsArr.append(partRotation)
    binWidgetsArr.append(partRotationLabel)
    binWidgetsArr.append(partRotationEntry)
    #flipped
    flippedFlag=tk.StringVar()
    flippedFlag.set('0')
    flippedCheck=tk.Checkbutton(binFrame, text="Flipped", variable=flippedFlag, onvalue="1", offvalue="0", height=5, width=20)
    flippedCheck.pack_forget()
    binValsArr.append(flippedFlag)
    binWidgetsArr.append(flippedCheck)
    save_new_bin=partial(saveBin,binID, bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9,binWidgetsArr, binValsArr,binFlag,bins, partOrdCounter)
    saveNewBinButton=tk.Button(binFrame, text="Save bin", command=save_new_bin)
    saveNewBinButton.pack_forget()
    back_bin=partial(backBin,binWidgetsArr, binValsArr,binFlag)
    backBinButton=tk.Button(binFrame, text="Back", command=back_bin)
    backBinButton.pack_forget()
    show_add_button=partial(switchBinMenu,binWidgetsArr, binValsArr,binFlag)
    addBinButton=tk.Button(binFrame,text="Add bin", command=show_add_button)
    addBinButton.pack(side=tk.BOTTOM)
    switch_buttons=partial(showAndHideBinButton,addBinButton, binValsArr[0], binOptionFlag,bin1Slots, binFrame, slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, saveNewBinButton, backBinButton)
    binValsArr[0].trace('w', switch_buttons)
    update_checkboxes=partial(runSlotChecks,binFrame, binID,slot1,slot2,slot3,slot4,slot5,slot6,slot7,slot8,slot9, checkBoxes,bin1Slots,bin2Slots,bin3Slots,bin4Slots,bin5Slots,bin6Slots,bin7Slots,bin8Slots,saveNewBinButton,binOptionFlag)
    binID.trace('w', update_checkboxes)
    validate_rotation=partial(validateRotationValue,partRotation, saveNewBinButton)
    partRotation.trace('w', validate_rotation)


def writeBinsToFile(name, binsList, saveFileName):
    '''Writes a given bin to the file'''
    with open(saveFileName, "a") as o:
        o.write("    "+name+":\n")
        for i in binsList:
            if i.binName==name:
                o.write("      - type: \'"+i.type+"\'\n")
                o.write("        color: \'"+i.color+"\'\n")
                o.write("        slots: "+i.slots+"\n")
                try:
                    val=float(i.rotation)
                except:
                    val=1
                if val!=0:
                    o.write("        rotation: "+i.rotation+"\n")
                if i.flipped=="1":
                    o.write("        flipped: true\n")
