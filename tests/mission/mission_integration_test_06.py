# Andy Torres
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-16-24

# ========================
# PyRPOD: tests/mission/mission_integration_test_06.py
# ========================
# Test case to graph a thrust vs time or distance required given design requirements
# and create a flight envelope to establish thrust requirements.

# Given Reuirements
# 1. Change in velocity (dV and /or dw)
# 2. System mass properties
# 3. Time or distance limits

# Desired outputs
# TODO: these are bad. need to think about them more.
# 1. Graph ISP vs Fuel reuired.
# 2. Graph Thrust vs Distance required.
# 3. Use time and distance limits to create flight envelope data.
# 4. Add data points for relevant thruster technologies.
# 5. Given a delta V and W requirement, comparison of thrust. isp, and mass flow rate. (M3) (Fuel Usage)

import test_header
import unittest, os, sys
from pyrpod.vehicle import LogisticsModule
from pyrpod.mission import MissionPlanner, MissionEnvironment

class DeltaMassChecks(unittest.TestCase):
    def test_delta_m_plots(self):

        # set case directory
        case_dir = '../case/mission/flight_envelopes/'

        # Instantiate LogisticModule object.
        lm = LogisticsModule.LogisticsModule(case_dir)

        # Define LM mass distrubtion properties.
        m = 0.45*30000 # lb converted to kg
        h = 14 # m
        r = 4.0/2.0 # m
        lm.set_inertial_props(m, h, r)

        # Load in thruster configuration data from text file
        lm.set_thruster_config()

        # Draco/Hypergolic thrusters
        lm.add_thruster_performance(400, 300)
        lm.assign_thruster_groups()

        me = MissionEnvironment.MissionEnvironment(case_dir)
        mp = MissionPlanner.MissionPlanner(me)
        mp.set_lm(lm)
        mp.flight_eval.read_flight_plan(lm)

        # mp.plot_delta_mass(1885)
        
if __name__ == '__main__':
    unittest.main()