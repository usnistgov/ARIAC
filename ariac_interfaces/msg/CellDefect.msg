# CellDefect.msg

uint8 DENT=1
uint8 BULGE=2
uint8 SCRATCH=3

uint8 defect_type # one of the defect types above

# Location of the defect in cylinderical coordinates 
#  relative to the cell base (r is implied as the radius of the cell)

float64 theta # azimuthal angle of the defect centroid
float64 z # height of the defect`centroid