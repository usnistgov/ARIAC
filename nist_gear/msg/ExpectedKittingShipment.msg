# # Shipment message
# # This structure contains the information of shipment.

# # The type of shipment
# string shipment_type

# MovableTrayInOrder movable_tray
# # Informs which AGV the shipment must be delivered to
# # Possible values are "agv1", "agv2",  "agv3", "agv4", or "any"
# string agv_id

# # Informs which station the shipment must be delivered to
# # Possible values are "AS1", "AS2", "AS3", "AS4", "AS5", "AS6", or "any"
# string station_id

# # Collection of products for this kitting shipment
# Product[] products
# # Shipment message
# # This structure contains the information of shipment.

# # # The type of shipment
# # string shipment_type

# # TrayContents tray_content

# # # Informs which station the shipment must be delivered to
# # # Possible values are "AS1", "AS2", "AS3", "AS4", "AS5", "AS6", or "any"
# # string assembly_station

# expected type for this shipment, e.g., order_0_kitting_shipment_0
string shipment_type
# movable tray and products expected for this shipment
TrayContents tray_content
# expected assembly station for this shipment, e.g., as1
string assembly_station