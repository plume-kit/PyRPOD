# Andy Torres
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-16-24

# ========================
# PyRPOD: tests/mission/mission_integration_test_07.py
# ========================
# Test case to contour the burn plot graph across various thrust and ISP values. (NEEDS TLC)

import test_header
import unittest, os, sys
from pyrpod.vehicle import LogisticsModule
from pyrpod.mission import MissionPlanner, MissionEnvironment

class BurnTimeContourChecks(unittest.TestCase):
    def test_burn_time_contour_plots(self):

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

        # Read in flight data and plot burntime for a given Δv requirement.
        # Graph is contoured according to various ISP values.
        me = MissionEnvironment.MissionEnvironment(case_dir)
        mp = MissionPlanner.MissionPlanner(me)
        mp.set_lm(lm)
        mp.flight_eval.read_flight_plan(lm)
        # mp.plot_burn_time_contour(1194)

if __name__ == '__main__':
    unittest.main()