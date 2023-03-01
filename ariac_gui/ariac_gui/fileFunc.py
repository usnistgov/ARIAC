from tkinter import filedialog
import platform
import os
from os import path
import tkinter as tk
if platform.system()=="Windows": #allows paths as inputs for linux
    invalidFileChar = " /~`,;\"\'\\!@#$%^&*()+=[]"  # characters not allowed in file names for windows
else:
    invalidFileChar = " `,;\"\'\\!@#$%^&*()+=[]"  # characters not allowed in file names for linux

def make_file(wind, fileNameVar):
    """Used for making a file for the user"""
    file=filedialog.asksaveasfile(defaultextension=".yaml", filetypes=[("YAML file", ".yaml")])
    if file:
        if str(os.path.abspath(file.name))!='':
            fileNameVar.set(str(os.path.abspath(file.name)))
            file.close
            wind.destroy()


def correct_file_name(tempFileName, a, b , c):  # deletes any invalid characters in file name
    """This function removes any characters which can not be used in the file name. It does so as the user is typing. Not needed anymore"""
    tempStr = tempFileName.get()
    for char in invalidFileChar:
        if char in tempStr:
            tempStr = tempStr.replace(char, '')
    tempFileName.set(tempStr)

def get_file_name_next(fileName, invalidFlag, nameLabels, getFileName, reqFlag, existFlag):  # checks to see if the file name the user selects exists or is empty
    """Reads the file name and puts a message on the window if invalid characters are found
    the file is empty, or if the file inputted already exists"""
    inv_char_found = []
    output_inv = ''
    c=1
    for i in fileName.get():
        if i in invalidFileChar:
            inv_char_found.append(i)
    if len(inv_char_found)>0:
        output_inv+="\""+inv_char_found[0]+"\""
        for i in inv_char_found[1:]:
            c+=1
            if c==len(inv_char_found):
                output_inv+=", and \""+i+"\""
            else:
                output_inv+=", \""+i+"\""
    if len(inv_char_found)!=0 and invalidFlag.get()=='0':
        for label in nameLabels:
            label.destroy()
        nameLabels.clear()
        invalid_char_label = tk.Label(getFileName, text="The name entered contains invalid characters: "+output_inv)
        invalid_char_label.pack()
        nameLabels.append(invalid_char_label)
        invalidFlag.set('1')
        reqFlag.set('0')
        existFlag.set('0')
        inv_char_found.clear()
        output_inv = ''
        c=0
    elif fileName.get() == "" and reqFlag.get() == "0":
        for label in nameLabels:
            label.destroy()
        nameLabels.clear()
        req_label = tk.Label(getFileName, text="This field is required. Please create a file")
        req_label.pack()
        nameLabels.append(req_label)
        reqFlag.set('1')
        invalidFlag.set('0')
        existFlag.set('0')
    elif (path.exists(fileName.get()) or path.exists(fileName.get() + '.yaml')) and existFlag.get() == '0':
        for label in nameLabels:
            label.destroy()
        nameLabels.clear()
        exist_label = tk.Label(getFileName,
                               text="A file with this name already exists. Please enter another file name.")
        exist_label.pack()
        nameLabels.append(exist_label)
        existFlag.set('1')
        invalidFlag.set('0')
        reqFlag.set('0')
    elif fileName.get() != '' and not (path.exists(fileName.get()) or path.exists(fileName.get() + '.yaml')) and invalidFlag.get()!='1':
        getFileName.destroy()
    invalidFlag.set('0')