import tkinter as tk
acceptedNum = "0123456789."  # for requiring number input
acceptedRotation="0123456789/.pi"
def require_num_rotation(val):
    """Makes sure a tkinter stringvar is numerical and has no more than one decimal point"""
    perFlag=0
    tempStr=val
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
    return tempStr


def validateRotationValue(rotationValue, button, a, b, c):
    temp_r=rotationValue.get()
    temp_r=temp_r.lower()
    for i in temp_r:
        if i not in acceptedRotation:
            temp_r=temp_r.replace(i,"")
    rotationValue.set(temp_r)
    if temp_r.count("/")>0:
        tempSplit=temp_r.split("/")
        if "pi" in tempSplit[0]:
            rotationValue.set("pi/"+require_num_rotation(tempSplit[1]))
            try:
                float(tempSplit[1])
                button.config(state=tk.NORMAL)
            except:
                button.config(state=tk.DISABLED)
        else:
            rotationValue.set(tempSplit[0]+"/"+require_num_rotation(tempSplit[1]))
            try:
                float(tempSplit[0])
                float(tempSplit[1])
                button.config(state=tk.NORMAL)
            except:
                button.config(state=tk.DISABLED)
    else:
        if temp_r=="pi":
            button.config(state=tk.NORMAL)
        else:
            try:
                float(require_num_rotation(temp_r))
                rotationValue.set(require_num_rotation(temp_r))
                button.config(state=tk.NORMAL)
            except:
                button.config(state=tk.DISABLED)
        
        
def add_quotes(strVar):
    """Formats the rpy values correctly"""
    tempStr=strVar.get()
    tempStr=tempStr.lower()
    tempStr=tempStr.replace("\"", "")
    tempStr=tempStr.replace("\'", "")
    if tempStr[0]!="\'":
        tempStr="\'"+tempStr
    if tempStr[len(tempStr)-1]!="\'":
        tempStr=tempStr+"\'"
    strVar.set(tempStr)

def require_num(val, a, b , c):
    """Makes sure a tkinter stringvar is numerical and has no more than one decimal point"""
    perFlag=0
    tempStr=val.get()
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
    val.set(tempStr)