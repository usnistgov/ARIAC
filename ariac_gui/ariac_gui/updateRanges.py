kAgv1List = ['[ks1]', '[as1]', '[as2]']  # all possible locations for agv1 in the kitting format
kAgv2List = ['[ks2]', '[as1]', '[as2]']  # all possible locations for agv2 in the kitting format
kAgv3List = ['[ks3]', '[as3]', '[as4]']  # all possible locations for agv3 in the kitting format
kAgv4List = ['[ks4]', '[as3]', '[as4]']  # all possible locations for agv4 in the kitting format
breakdownAll = ['agv1', 'agv2', 'agv3', 'agv4', 'as1', 'as2', 'as3', 'as4']
breakdownAGVs = ['agv1', 'agv2', 'agv3', 'agv4']
def update_dest(dropdownMenu, agvSelection, currentStation, a, b, c):  # switches the options present based off of the agv selected
    """Updates the possible stations for agvs"""
    menu = dropdownMenu['menu']
    menu.delete(0, 'end')
    if agvSelection.get() == 'agv1':
        currentStation.set(kAgv1List[0])
        for dest in kAgv1List:
            menu.add_command(label=dest, command=lambda dest=dest: currentStation.set(dest))
    elif agvSelection.get() == 'agv2':
        currentStation.set(kAgv2List[0])
        for dest in kAgv2List:
            menu.add_command(label=dest, command=lambda dest=dest: currentStation.set(dest))
    elif agvSelection.get() == 'agv3':
        currentStation.set(kAgv3List[0])
        for dest in kAgv3List:
            menu.add_command(label=dest, command=lambda dest=dest: currentStation.set(dest))
    else:
        currentStation.set(kAgv4List[0])
        for dest in kAgv4List:
            menu.add_command(label=dest, command=lambda dest=dest: currentStation.set(dest))


def update_bd_locations(dropdown, numProds, robot_type, location, a, b, c):
    """Updates the locations of the robot on breakdown"""
    print("test")
    menu = dropdown['menu']
    menu.delete(0, 'end')
    if numProds.get()!='4'and robot_type.get()=='assembly_robot':
        location.set(breakdownAGVs[0])
        for i in breakdownAGVs:
            menu.add_command(label=i, command=lambda i=i: location.set(i))
    else:
        location.set(breakdownAll[0])
        for i in breakdownAll:
            menu.add_command(label=i, command=lambda i=i: location.set(i))



def update_id_range(dropdownMenu, currentProduct, currentID, binProds, a, b, c):  # updates the ids for faulty products
    """Updates the range of possible id's based on bins"""
    menu = dropdownMenu['menu']
    menu.delete(0, 'end')
    bin_prods_ind = 0
    for binProd in binProds:
        if binProd.pType == currentProduct.get():
            break
        bin_prods_ind += 1
    currentID.set('1')
    for num in range(int(binProds[bin_prods_ind].pNum)):
        temp_num = str(num + 1)
        menu.add_command(label=temp_num, command=lambda temp_num=temp_num: currentID.set(temp_num))


def update_val_label(label, func, c, d, e):  # for having the current number for the slider
    """Live label for the current value of a slider. Not being used"""
    label.configure(text="Current value = "+func())