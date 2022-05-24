#!/usr/bin/env python

import rospy

from test_competitor.competitor import Competitor

# This code works with the trial configurations:
# assembly_as1.yaml
# assembly_as2.yaml
# assembly_as3.yaml
# assembly_as4.yaml

if __name__ == "__main__":
    rospy.init_node("test_node")

    competitor = Competitor()
    competitor.start_competition()
    rospy.sleep(5.0)
    competitor.submit_assembly_shipment("as1", "order_0_assembly_shipment_0")
