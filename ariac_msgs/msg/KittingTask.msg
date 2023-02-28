uint8 KITTING=0
uint8 ASSEMBLY_FRONT=1
uint8 ASSEMBLY_BACK=2
uint8 WAREHOUSE=3

uint8 agv_number
int8 tray_id
uint8 destination # KITTING, ASSEMBLY_FRONT, ASSEMBLY_BACK, WAREHOUSE
ariac_msgs/KittingPart[] parts