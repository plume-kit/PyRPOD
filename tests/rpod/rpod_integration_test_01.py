import logging
logging.basicConfig(filename='rpod_integration_test_01.log', level=logging.INFO, format='%(message)s')

# Andy Torres, Nicholas Palumbo
# Last Changed: 11-17-24

# ========================
# PyRPOD: tests/rpod/rpod_integration_test_01.py
# ========================
# This test asserts the expected number of cell strikes on a flat plate STL
# for a notional trajectory meant to reperesent a "sweep" above it. This
# is also established as the base case for RPOD plume impingement analysis.
# The test uses JFH data to assert expected strike counts across 20 distinct firings.

import test_header
import unittest, os, sys
from pyrpod.vehicle import LogisticsModule, TargetVehicle
from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy
from pyrpod.mission import MissionEnvironment

class BaseCaseChecks(unittest.TestCase):
    def test_base_case(self):

    # 1. Set Up
        # Path to directory holding data assets and results for a specific RPOD study.
        case_dir = '../case/rpod/base_case/'

        # Instantiate JetFiringHistory object.
        jfh = JetFiringHistory.JetFiringHistory(case_dir)

        # Load Target Vehicle.
        tv = TargetVehicle.TargetVehicle(case_dir)
        tv.set_stl()

        # Instantiate LogisticModule object.
        lm = LogisticsModule.LogisticsModule(case_dir)

        # Load in thruster configuration file.
        lm.set_thruster_config()

        # Set mission environment.
        me = MissionEnvironment.MissionEnvironment(case_dir)

        # Instantiate RPOD object.
        plume_strike_study = PlumeStrikeEstimationStudy.PlumeStrikeEstimationStudy(me)
        plume_strike_study.study_init(jfh, tv, lm)

        # Read in JFH.
        jfh.read_jfh()

    # 2. Execute
        # Conduct RPOD analysis
        plume_strike_study.graph_jfh()
        strikes = plume_strike_study.jfh_plume_strikes()

    # 3. Assert
        # Assert expected strike values for each firing in the JFH.
        expected_strikes = {
                            '1':  282.0,
                            '2':  279.0,
                            '3':  285.0,
                            '4':  282.0,
                            '5':  280.0,
                            '6':  284.0,
                            '7':  281.0,
                            '8':  280.0,
                            '9':  282.0,
                            '10':  280.0,
                            '11':  280.0,
                            '12':  277.0,
                            '13':  283.0,
                            '14':  282.0,
                            '15':  279.0,
                            '16':  285.0,
                            '17':  282.0,
                            '18':  280.0,
                            '19':  284.0,
                            '20':  282.0
        }

        # Assert expected cumulative strike values for each firing in the JFH.
        expected_cum_strikes = {
                            '1':  282.0,
                            '2':  561.0,
                            '3':  846.0,
                            '4':  1128.0,
                            '5':  1408.0,
                            '6':  1692.0,
                            '7':  1973.0,
                            '8':  2253.0,
                            '9':  2535.0,
                            '10':  2815.0,
                            '11':  3095.0,
                            '12':  3372.0,
                            '13':  3655.0,
                            '14':  3937.0,
                            '15':  4216.0,
                            '16':  4501.0,
                            '17':  4783.0,
                            '18':  5063.0,
                            '19':  5347.0,
                            '20':  5629.0
        }

        # Read in expected strikes from text file.
        file_path = 'rpod/rpod_int_test_01_expected_strikes.log'
        expected_strike_ids = {}
        with open(file_path, 'r') as file:
            file_content = file.readlines()

            cur_firing = ''

            for line in file_content:
                # Make a an array of data to orgnaize strike data by firing.
                if 'n_firing' in line:
                    cur_firing = str(line.split()[1])
                    expected_strike_ids[cur_firing] = []
                else:
                    expected_strike_ids[cur_firing].append(int(line))

        for n_firing in strikes.keys():
            # Development statements used to write comparison entries in expected_strikes
            # logging.info('n_firing ' + str(n_firing))
            for i in range(len(strikes[n_firing]['strikes'])):
                if strikes[n_firing]['strikes'][i] > 0:
                    # string = 'strikes[' + str(i) + '] = ' + str(strikes[n_firing]['cum_strikes'][i])
                    # logging.info(string)

                    # logging.info(str(i))
                    self.assertIn(i, expected_strike_ids[n_firing])
            # Development statements used to write comparison entries in expected_strikes
            string = '\''+str(n_firing)+'\': ' + ' ' +str(strikes[n_firing]['cum_strikes'].sum()) +','
            logging.info(string)

            # Number of strikes for a given time step.
            n_strikes = strikes[n_firing]['strikes'].sum()
            n_cum_strikes = strikes[n_firing]['cum_strikes'].sum()

            # Assert that it matches the expected value.
            self.assertEqual(n_strikes, expected_strikes[n_firing])
            self.assertEqual(n_cum_strikes, expected_cum_strikes[n_firing])

if __name__ == '__main__':
    unittest.main()