# Order message
# This structure contains the information of an order.

# ID of the order
string order_id

# Collection of kitting shipments
KittingInOrder[] kitting_shipments
# Collection of assembly shipments
AssemblyInOrder[] assembly_shipments
