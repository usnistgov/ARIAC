class PartsClass:  # for organizing the data for all products
    """Holds the information for a product"""
    def __init__(self, p_type, color, quadrant, rotation):
        self.pType = p_type
        self.color=color
        self.quadrant=quadrant
        self.rotation=rotation
class Bin:  # for holding information about present bins
    """Holds information for the bin"""
    def __init__(self, name, type, color, slots, rotation, flipped):
        self.binName=name
        self.type=type
        self.color=color
        self.slots=slots
        self.rotation=rotation
        self.flipped=flipped

class PartConv:  # for holding information about parts for the conveyor belt
    """Holds data about parts for the conveyor belt"""
    def __init__(self, type, color, number, offset, rotation):
        self.type=type
        self.color=color
        self.number=number
        self.offset=offset
        self.rotation=rotation