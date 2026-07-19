# Andy Torres
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 12-14-23

# ========================
# PyRPOD: test/test_case_13.py
# ========================
# Test case for converting STL data to VTK data.
# This is accomplished by checking for the proper data format of VTK files.

import test_header
import unittest, os, sys
from pyrpod import JetFiringHistory, TargetVehicle, VisitingVehicle, RPOD

class LoadJFHChecks(unittest.TestCase):
    def test_jfh_reader(self):

        # Path to directory holding data assets and results for a specific RPOD study.
        case_dir = '../case/base_case/'

        # Load JFH data.
        jfh = JetFiringHistory.JetFiringHistory(case_dir)
        jfh.read_jfh()

        tv = TargetVehicle.TargetVehicle(case_dir)
        tv.set_stl()

        vv = VisitingVehicle.VisitingVehicle(case_dir)
        vv.set_stl()
        vv.set_thruster_config()

        rpod = RPOD.RPOD(case_dir)
        rpod.study_init(jfh, tv, vv)
        rpod.graph_jfh()
        rpod.jfh_plume_strikes()

if __name__ == '__main__':
    unittest.main()