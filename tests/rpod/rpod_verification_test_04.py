# Nicholas A. Palumbo
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 3-21-24

# ========================
# PyRPOD: tests/rpod_verification_test_04.py
# ========================
# Visualizes STLs after decoupling the TCD and verifies that the plume strikes line up with the thrusters.

import test_header
import unittest, os, sys

from pyrpod.vehicle import LogisticsModule, TargetVehicle, VisitingVehicle
from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy
from pyrpod.mission import MissionEnvironment

class LoadSTLModels(unittest.TestCase):
    def test_decoupled_tcd(self):

        # Set case directory.
        case_dir = '../case/tcd_decoupling/'

        # Instantiate JetFiringHistory object.
        jfh = JetFiringHistory.JetFiringHistory(case_dir)
        jfh.read_jfh()

        # Instantiate TargetVehicle object.
        tv = TargetVehicle.TargetVehicle(case_dir)
        # Load Target Vehicle.
        tv.set_stl()

        # Instantiate VisitingVehicle object.
        vv = VisitingVehicle.VisitingVehicle(case_dir)
        # Load Visiting Vehicle.
        vv.set_stl()
        # Load in thruster configuration file.
        vv.set_thruster_config()
        # Load in cluster configuration file.
        vv.set_cluster_config()
        # Load in thruster data file
        vv.set_thruster_metrics()

        # Set mission environment.
        me = MissionEnvironment.MissionEnvironment(case_dir)

        # Instantiate RPOD object.
        plume_strike_study = PlumeStrikeEstimationStudy.PlumeStrikeEstimationStudy(me)
        # Initiate RPOD study.
        plume_strike_study.study_init(jfh, tv, vv)
        # Load STLs in Paraview
        plume_strike_study.graph_jfh()
        # Run plume strike analysis.
        plume_strike_study.jfh_plume_strikes()

if __name__ == '__main__':
    unittest.main()