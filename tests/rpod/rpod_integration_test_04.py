import logging
logging.basicConfig(filename='rpod_integration_test_04.log', level=logging.INFO, format='%(message)s')

# Andy Torres, Nicholas A. Palumbo
# Last Changed: 03-16-24

# ========================
# PyRPOD: tests/rpod/rpod_verification_test_01.py
# ========================
# Test case for producing hollow cube data.

import test_header
import unittest, os, sys
import pandas as pd

from pyrpod.vehicle import LogisticsModule, TargetVehicle, VisitingVehicle
from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy
from pyrpod.mission import MissionEnvironment

class HollowCubeChecks(unittest.TestCase):
    def test_hollow_cube(self):

    # 1. Set Up    
        # Path to directory holding data assets and results for a specific RPOD study.
        case_dir = '../case/rpod/hollow_cube/'

        # Load JFH data.
        jfh = JetFiringHistory.JetFiringHistory(case_dir)
        jfh.read_jfh()

        # Load Target Vehicle.
        tv = TargetVehicle.TargetVehicle(case_dir)
        tv.set_stl()

        # Load Visiting Vehicle.
        vv = VisitingVehicle.VisitingVehicle(case_dir)
        vv.set_stl()
        vv.set_thruster_config()
        # vv.set_thruster_metrics()

        # Set mission environment.
        me = MissionEnvironment.MissionEnvironment(case_dir)

        # Instantiate RPOD object.
        rpod = PlumeStrikeEstimationStudy.RPOD(me)
        rpod.study_init(jfh, tv, vv)

    # 2. Execute
        # Run plume strike analysis
        rpod.graph_jfh()
        strikes = rpod.jfh_plume_strikes()

    # 3. Assert
        # Read in expected strikes from text file.
        file_path = 'rpod/rpod_int_test_04_expected_strikes.log'
        expected_strikes = {}
        with open(file_path, 'r') as file:
            file_content = file.readlines()

            cur_firing = ''

            for line in file_content:
                # Make a an array of data to orgnaize strike data by firing. 
                if 'n_firing' in line:
                    cur_firing = line.split()[1]
                    expected_strikes[cur_firing] = []
                # Append index values of the current cell to 'n_firing' array.
                else: 
                    expected_strikes[cur_firing].append(int(line))

        # # Development statements used to write comparison entries in expected_strikes
        for n_firing in strikes.keys():
        #     logging.info('n_firing ' + str(n_firing))
        #     for i in range(len(strikes[n_firing]['strikes'])):
        #         if strikes[n_firing]['strikes'][i] > 0:
        #             # string = 'strikes[' + str(i) + '] = ' + str(strikes[n_firing]['cum_strikes'][i])
        #             # logging.info(string)

        #             logging.info(str(i))

            # Sums the total amount of cell strikes per firing. Saves data to a dictionary.
            # string = '\''+str(n_firing)+'\': ' + ' ' +str(strikes[n_firing]['strikes'].sum()) +','
            # logging.info(string)

            # Assert that each firing is striking the expected cells by comparing index values. 
            for i in range(len(strikes[n_firing]['strikes'])):
                if strikes[n_firing]['strikes'][i] > 0:
                    self.assertIn(i, expected_strikes[n_firing])

if __name__ == '__main__':
    unittest.main()