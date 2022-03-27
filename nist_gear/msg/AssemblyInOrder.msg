# Shipment message
# This structure contains the information of an assembly shipment.

# type of shipment
string shipment_type

# station used to do assembly
string station_id

# collection of products at station_id
ProductInOrder[] products
