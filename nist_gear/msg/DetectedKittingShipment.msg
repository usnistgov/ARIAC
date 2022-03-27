# # Shipment contents message
# # This structure contains the information about a shipment being submitted

# # The ID of the shipping_box or tray the shipment was detected in
# string agv_id

# # kitting or assembly station where the kit is built
# string station_id

# # model of the tray where the parts are placed
# string movable_tray_model

# # Collection of products
# DetectedProduct[] products
# The type of shipment
string shipment_type

TrayContents tray_content

# Informs which station the shipment must be delivered to
# Possible values are "AS1", "AS2", "AS3", "AS4", "AS5", "AS6", or "any"
string assembly_station