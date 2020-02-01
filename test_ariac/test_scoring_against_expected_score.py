#!/usr/bin/env python

from __future__ import print_function

import sys
import time

import rospy
import rostest
from test_example_node import ExampleNodeTester


class ScoringTester(ExampleNodeTester):

    def test(self):
        expectedScore = float(sys.argv[1])
        rospy.loginfo('Using expected score of: ' + str(expectedScore))
        commands = []
        if len(sys.argv) > 2:
            for arg in sys.argv[2:]:
                if arg.startswith('--'):
                    break
                commands.append(arg)
        self.assertNotEqual([], commands)
        self.prepare_tester()

        # Starting the competition will cause products from the order to be spawned on shipping_box_0
        self._test_start_comp()

        for command in commands:
            opcode, arg = command.split(":")
            if "wait" == opcode:
                print('Waiting for ' + arg + ' seconds')
                time.sleep(float(arg))
            elif "submit_agv1" == opcode:
                print('Submitting agv1 shipment ' + arg)
                self._test_submit_shipment(shipment_type=arg, agv_num=1)
            elif "submit_agv2" == opcode:
                print('Submitting agv2 shipment ' + arg)
                self._test_submit_shipment(shipment_type=arg, agv_num=2)
            elif "collide_arms" == opcode:
                print('Commanding arms to collide')
                self.comp_class.send_arm1_to_state([0, 0, 0, 0, 0, 0, -1.18])
                self.comp_class.send_arm2_to_state([0, 0, 0, 0, 0, 0, 1.18])
            else:
                raise ValueError("unknown command: " + repr(command))

        self.assertEqual(self.current_comp_score, expectedScore)


if __name__ == '__main__':
    rospy.init_node('test_scoring_against_expected_score', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(20.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_scoring_against_expected_score', ScoringTester, sys.argv)
