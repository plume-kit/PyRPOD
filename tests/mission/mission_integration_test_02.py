# Andy Torres
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-16-24

# ========================
# PyRPOD: tests/mission/mission_integration_test_02.py
# ========================
# A brief test case to calculate the 6DOF performance of thruster working groups

import unittest, os, sys
from pyrpod.vehicle import LogisticsModule

class ThrusterGroupingChecks(unittest.TestCase):
    def test_performance_per_thruster_group(self):

        # Define LM mass distrubtion properties.
        m = 0.45*30000 # lb converted to kg
        h = 14 # m
        r = 4.0/2.0 # m

        # # Instantiate LogisticModule object.
        # lm = LogisticsModule.LogisticsModule(m, h, r)

        # # Load in thruster configuration data from text file
        # lm.add_thruster_config('../data/tcd/TCD2.txt')

        # # Draco/Hypergolic thrusters
        # lm.add_thruster_performance(400, 300)
        
        # # Assign thruster groups and calculate their performance.
        # # (creates unwanted files of data so test is not run for now.)
        # lm.assign_thruster_groups()
        # lm.check_thruster_groups()


if __name__ == '__main__':
    unittest.main()