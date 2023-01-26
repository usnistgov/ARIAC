import tkinter as tk
from functools import partial
from ariac_gui.checkCancel import *
acceptedNum = "0123456789"  # for requiring positive numbers for time
CHECKBOXHEIGHT=1
def updateTimeInputBox(val, box, a,b,c):
    '''disables and enables the time box if the user selects no time limit'''
    if val.get()=="1":
        box.configure(state="disabled")
    else:
        box.configure(state="normal")

def validateTime(val,a,b,c):
    '''Validates the time from the user'''
    tempStr=val.get()
    for i in tempStr:
        if i not in acceptedNum:
            tempStr=tempStr.replace(i, "")
    if tempStr!="":
        numVal=int(tempStr)
        if numVal<0:
            tempStr="0"
    val.set(tempStr)

def guiTimeWindow(timeList, mainWind):
    timeWind=tk.Toplevel()
    timeWind.title("Time limit")
    #timeWind.geometry("850x600")
    timeWind.attributes('-fullscreen', True)
    timeInstructions=tk.Label(timeWind, text="Enter the time limit you would like for the simulation")
    timeInstructions.pack(pady=100)
    timeVal=tk.StringVar()
    timeVal.set(timeList[0])
    noTimeVal=tk.StringVar()
    noTimeVal.set(timeList[1])
    noTimeLim=tk.Checkbutton(timeWind, text="No time limit", variable=noTimeVal, onvalue="1", offvalue="0", height=CHECKBOXHEIGHT, width=20)
    noTimeLim.pack()
    if noTimeVal.get()=="1":
        getTime=tk.Entry(timeWind, textvariable=timeVal, state="disabled")
    else:
        getTime=tk.Entry(timeWind, textvariable=timeVal, state="normal")
    getTime.pack()
    saveTimeButton=tk.Button(timeWind, text="Save and return to main menu", command=mainWind.destroy)
    saveTimeButton.pack(pady=20)
    updateGetTime=partial(updateTimeInputBox, noTimeVal, getTime)
    validateTimeInput=partial(validateTime, timeVal)
    noTimeVal.trace('w', updateGetTime)
    timeVal.trace('w', validateTimeInput)
    timeWind.mainloop()
    timeList.clear()
    timeList.append(timeVal.get())
    timeList.append(noTimeVal.get())