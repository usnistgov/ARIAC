from os import chdir, path
import os
import tkinter as tk
def check_cancel(cancel_flag, pathIncrement, fileName, createdDir):  # deletes the file if the user cancels from inside the program
    """Checks if the program is canceled. If it is, the program removes the file and quits. If directories have been created,
    those are removed as well"""
    if cancel_flag == '1':
        for dir in pathIncrement:
            chdir(dir)
        if path.exists(fileName.get()):
            os.remove(fileName.get())
        elif path.exists(fileName.get() + '.yaml'):
            os.remove(fileName.get() + '.yaml')
        chdir('../')
        createdDir.reverse()
        for dir in createdDir:
            if len(os.listdir(dir))!=0:
                break
            os.rmdir(dir)
            chdir('../')
        quit()

def cancel_wind(window, cancelFlag):  # cancels at any point in the program
    """Used to chancel a window"""
    cancelFlag.set('1')
    window.destroy()

def cancel_func(wind, flag):
    """Sets flag to 1 and destroys a window for when the user wants to cancel and exit"""
    wind.destroy()
    flag.set("1")

def exitAndFlag(window, parFlag):
    """Exits the given window and activates the given flag"""
    parFlag.set('1')
    window.destroy()

def skip_wind(flag, window):  # function for skipping a window
    """Sets a given flag and destroys a windos for skip buttons"""
    flag.set('1')
    window.destroy()