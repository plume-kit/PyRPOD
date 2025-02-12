# Andy Torres
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-16-24

# ========================
# PyRPOD: tests/rpod/rpod_integration_test_03.py
# ========================
# Test case to analyze Keep Out Zone Impingement. (WIP)

import logging
logging.basicConfig(filename='rpod_integration_test_03.log', level=logging.INFO, format='%(message)s')


import test_header
import unittest, os, sys

from pyrpod.vehicle import LogisticsModule, TargetVehicle
from pyrpod.rpod import RPOD, JetFiringHistory

class KeepOutZoneChecks(unittest.TestCase):
    def test_keep_out_zone(self):

    # 1. Set Up
        # Path to directory holding data assets and results for a specific RPOD study.
        case_dir = '../case/rpod/koz/'

        # Instantiate JetFiringHistory object.
        jfh = JetFiringHistory.JetFiringHistory(case_dir)

        # Load Target Vehicle.
        tv = TargetVehicle.TargetVehicle(case_dir)
        # print(tv.config.items)

        tv.set_stl()

        # Instantiate LogisticModule object.
        lm = LogisticsModule.LogisticsModule(case_dir)

        # Load in thruster configuration file.
        lm.set_thruster_config()

        # Instantiate RPOD object.
        rpod = RPOD.RPOD(case_dir)
        rpod.study_init(jfh, tv, lm)

        # Read in JFH.
        jfh.read_jfh()

    # 2. Execute
        # Conduct RPOD analysis
        rpod.graph_jfh()
        strikes = rpod.jfh_plume_strikes()

    # 3. Assert
        # Read in expected strikes from text file.
        expected_strikes = {
                            '1':  147.0,
                            '2':  118.0,
                            '3':  92.0,
                            '4':  67.0,
                            '5':  45.0,
                            '6':  32.0,
                            '7':  21.0,
                            '8':  10.0,
                            '9':  5.0,
                            '10':  1.0
        }

        expected_cum_strikes = {
                            '1':  147.0,
                            '2':  265.0,
                            '3':  357.0,
                            '4':  424.0,
                            '5':  469.0,
                            '6':  501.0,
                            '7':  522.0,
                            '8':  532.0,
                            '9':  537.0,
                            '10':  538.0
        }

        file_path = 'rpod/rpod_int_test_03_expected_strikes.log'
        expected_strike_ids = {}
        with open(file_path, 'r') as file:
            file_content = file.readlines()

            cur_firing = ''

            for line in file_content:
                # Make a an array of data to orgnaize strike data by firing. 
                if 'n_firing' in line:
                    cur_firing = line.split()[1]
                    expected_strike_ids[cur_firing] = []
                # Append index values of the current cell to 'n_firing' array.
                else: 
                    expected_strike_ids[cur_firing].append(int(line))

        # # Development statements used to write comparison entries in expected_strikes
        for n_firing in strikes.keys():
            # logging.info('n_firing ' + str(n_firing))
        #     for i in range(len(strikes[n_firing]['strikes'])):
        #         if strikes[n_firing]['strikes'][i] > 0:
        #             string = 'strikes[' + str(i) + '] = ' + str(strikes[n_firing]['cum_strikes'][i])
        #             # logging.info(string)

        #             # logging.info(str(i))

            # Sums the total amount of cell strikes per firing. Saves data to a dictionary.
            string = '\''+str(n_firing)+'\': ' + ' ' +str(strikes[n_firing]['cum_strikes'].sum()) +','
            # logging.info(string)

            # Assert that each firing is striking the expected cells by comparing index values. 
            for i in range(len(strikes[n_firing]['strikes'])):
                if strikes[n_firing]['strikes'][i] > 0:
                    self.assertIn(i, expected_strike_ids[n_firing])

            # Number of strikes for a given time step.
            n_strikes = strikes[n_firing]['strikes'].sum()
            n_cum_strikes = strikes[n_firing]['cum_strikes'].sum()
            # logging.info('n_strikes ' + str(n_strikes))

            # Assert that it matches the expected value.
            # print(type(expected_strikes[n_firing]), expected_strikes[n_firing])
            self.assertEqual(n_strikes, expected_strikes[n_firing])
            self.assertEqual(n_cum_strikes, expected_cum_strikes[n_firing])

        return

if __name__ == '__main__':
    unittest.main()