# Juan P. Roldan
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-16-24

# ========================
# PyRPOD: tests/rpod/rpod_verification_test_03.py
# ========================
# Test case for testing plume gas kinetic models in jfh firings **with multiple thrusters per firing**.

import test_header
import unittest, os, sys

from pyrpod.vehicle import LogisticsModule, TargetVehicle, VisitingVehicle
from pyrpod.rpod import RPOD, JetFiringHistory
from pyrpod.mission import MissionEnvironment


class LoadJFHChecks(unittest.TestCase):
    def test_plume_constraints(self):

        # Path to directory holding data assets and results for a specific RPOD study.
        case_dir = '../case/rpod/multi_thrusters_square/'

        # Load JFH data.
        jfh = JetFiringHistory.JetFiringHistory(case_dir)
        jfh.read_jfh()

        tv = TargetVehicle.TargetVehicle(case_dir)
        tv.set_stl()

        vv = VisitingVehicle.VisitingVehicle(case_dir)
        vv.set_stl()
        vv.set_thruster_config()
        vv.set_thruster_metrics()

        me = MissionEnvironment.MissionEnvironment(case_dir)

        rpod = RPOD.RPOD(me)
        rpod.study_init(jfh, tv, vv)

        rpod.graph_jfh()
        rpod.jfh_plume_strikes()

if __name__ == '__main__':
    unittest.main()