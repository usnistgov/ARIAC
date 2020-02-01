# Shipment message
# This structure contains the information of shipment.

# The type of shipment
string shipment_type

# ID used to say which AGV the shipment must be delivered to
# Possible values are "agv1", "agv2", or "any"
string agv_id

# Collection of products
Product[] products
