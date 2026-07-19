# Nicholas Palumbo
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 04-21-24

# ========================
# PyRPOD: tests/rpod/rpod_verification_test_05.py
# ========================
# Test case for visualizing the flow field using a series of TVs.

import test_header
import unittest, os, sys
from pyrpod import JetFiringHistory, TargetVehicle, VisitingVehicle, RPOD

class LoadPlume(unittest.TestCase):
    def test_plume_field(self):

        # Set case directory.
        case_dir = '../case/flow_visualization/'

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

        # Instantiate RPOD object.
        rpod = RPOD.RPOD(case_dir)
        # Initialize iteration counter to be used in graph_jfh and jfh_plume_strikes file naming
        rpod.count = 1
        # Initiate RPOD study.
        rpod.study_init(jfh, tv, vv)
        # Load STLs in Paraview
        rpod.graph_jfh()
        # Run plume strike analysis.
        rpod.jfh_plume_strikes()


if __name__ == '__main__':
    unittest.main()