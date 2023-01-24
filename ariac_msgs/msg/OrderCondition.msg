uint8 KITTING=0
uint8 ASSEMBLY=1
uint8 COMBINED=2

string id
uint8 type
bool priority
ariac_msgs/KittingTask kitting_task 
ariac_msgs/AssemblyTask assembly_task
ariac_msgs/CombinedTask combined_task
ariac_msgs/Condition condition