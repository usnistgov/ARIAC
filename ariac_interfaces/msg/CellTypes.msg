# CellTypes.msg

uint8 NONE=0 
uint8 LI_ION=1 # Lithium Ion cells
uint8 NIMH=2  # Nickel-Metal Hydride

float64 LI_ION_NOMINAL_VOLTAGE = 3.6 # Volts
float64 NIMH_NOMINAL_VOLTAGE = 1.2 # Volts

float64 CELL_VOLTAGE_TOLERANCE = 0.2 # ± Volts
float64 KIT_VOLTAGE_TOLERANCE = 0.15 # ± Volts