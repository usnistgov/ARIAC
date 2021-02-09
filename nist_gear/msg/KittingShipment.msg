# Shipment message
# This structure contains the information of shipment.

# The type of shipment
string shipment_type

# ID used to say which AGV the shipment must be delivered to
# Possible values are "agv1", "agv2", or "any"
string agv_id

# ID used to say which station the shipment must be delivered to
# Possible values are "AS1", "AS2", "AS3", "AS4", "AS5", "AS6", or "any"
string station_id

# Collection of products
Product[] products
