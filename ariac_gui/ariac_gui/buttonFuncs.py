import tkinter as tk
def activateButton(button, parFlag, c, d, e):
    """Depending on the status of the flag, it will activate a deactivated button"""
    if parFlag.get() == '1':
        button.config(state=tk.NORMAL)

def deactivateButton(button, parFlag, c, d, e):
    """Depending on the status of the flag, it will deactivate an activated button"""
    if parFlag.get() =='1':
        button.config(state=tk.DISABLED)