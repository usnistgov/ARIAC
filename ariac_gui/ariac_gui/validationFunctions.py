import tkinter as tk
acceptedNum = "0123456789."  # for requiring number input

def validateRotationValue(rotationValue, button, a, b, c):
    """Validates all input for the rpy entries. Activates the save and exit button accordingly"""
    temp_r=rotationValue.get()
    temp_r=temp_r.replace(".","",1)
    pi_flag_r=0
    if temp_r.count("/")==1:
        temp_r_split=temp_r.split("/")
        if temp_r_split[0]=="pi" and temp_r_split[1].isnumeric():
            pi_flag_r=1
    if (pi_flag_r==1 or temp_r.isnumeric()) and button['state']==tk.DISABLED:
        button.config(state=tk.NORMAL)
    elif button['state']==tk.NORMAL:
        button.config(state=tk.DISABLED)
        
def add_quotes(strVar):
    """Formats the rpy values correctly"""
    tempStr=strVar.get()
    tempStr=tempStr.lower()
    if 'pi' in tempStr:
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