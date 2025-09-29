import logging
logging.basicConfig(filename='rpod_integration_test_02.log', level=logging.INFO, format='%(message)s')

# Andy Torres, Nicholas Palumbo
# Last Changed: 11-10-24

# ========================
# PyRPOD: tests/rpod/rpod_integration_test_02.py
# ========================
# This test asserts the expected number of cell strikes on a flat plate STL
# as a notional VV approaches it via a direct trajectory solved using 1D physics. This 1D
# trajectory reperesents a VV firing its adverse thrusters to slow down in preperation for docking.
# The test uses JFH data to assert expected strike counts across 15 distinct firings.

import test_header
import unittest, os, sys

from pyrpod.vehicle import LogisticsModule, TargetVehicle
from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy
from pyrpod.mission import MissionEnvironment

class OneDimTransApproachChecks(unittest.TestCase):
    def test_1d_approach(self):

    # 1. Set Up
        # Path to directory holding data assets and results for a specific RPOD study.
        case_dir = '../case/rpod/1d_approach/'

        # Instantiate JetFiringHistory object.
        jfh = JetFiringHistory.JetFiringHistory(case_dir)

        # Load Target Vehicle.
        tv = TargetVehicle.TargetVehicle(case_dir)
        tv.set_stl()

        # Instantiate LogisticModule object.
        lm = LogisticsModule.LogisticsModule(case_dir)

        # Define LM mass distribution properties.
        m = 14000 # kg
        h = 11 # m
        r = 2 # m
        lm.set_inertial_props(m, h, r)

        # Load in thruster configuration file.
        lm.set_thruster_config()
        # Load in thruster data file
        lm.set_thruster_metrics()
        # Use TCD to group DOF
        lm.assign_thruster_groups()

        # Set mission environment.
        me = MissionEnvironment.MissionEnvironment(case_dir)

        # Instantiate RPOD object.
        plume_strike_study = PlumeStrikeEstimationStudy.PlumeStrikeEstimationStudy(me)
        plume_strike_study.study_init(jfh, tv, lm)

        # Produce JFH using 1D physics
        # r_o = 40 # initial distance (m)
        # v_o = 0.2 # Initial velocity (m/s)
        # v_ida = 0.03 # Docking velocity (m/s)
        # rpod.print_jfh_1d_approach(v_ida, v_o, r_o)

        # Read in JFH.
        jfh.read_jfh()

    # 2. Execute
        # Conduct RPOD analysis
        plume_strike_study.graph_jfh()
        strikes = plume_strike_study.jfh_plume_strikes()

        # logging.info(len(strikes['1']['cum_strikes']))  

    # 3. Assert
        # Assert expected strike values for each firing in the JFH.
        expected_strikes = {
            '1':  1492.0,
            '2':  1368.0,
            '3':  1138.0,
            '4':  864.0,
            '5':  632.0,
            '6':  428.0,
            '7':  276.0,
            '8':  150.0,
            '9':  64.0,
            '10':  16.0
        }

        # Assert expected cumulative strike values for each firing in the JFH.
        expected_cum_strikes = {
            '1':  1492.0,
            '2':  2860.0,
            '3':  3998.0,
            '4':  4862.0,
            '5':  5494.0,
            '6':  5922.0,
            '7':  6198.0,
            '8':  6348.0,
            '9':  6412.0,
            '10':  6428.0
        }

        # Read in expected strikes from text file.
        file_path = 'rpod/rpod_int_test_02_expected_strikes.log'
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
                    # # logging.info(string)

                    # logging.info(str(i))

                    # assert that the tracking of each face strike is enough.
                    self.assertIn(i, expected_strike_ids[n_firing])

            # Development statements used to write comparison entries in expected_strikes
            # string = '\''+str(n_firing)+'\': ' + ' ' +str(strikes[n_firing]['strikes'].sum()) +','
            # logging.info(string)

            # Number of strikes for a given time step.
            n_strikes = strikes[n_firing]['strikes'].sum()
            n_cum_strikes = strikes[n_firing]['cum_strikes'].sum()

            # logging.info('n_firing ' + str(n_firing))

            # Assert that it matches the expected value.
            self.assertEqual(n_strikes, expected_strikes[n_firing])
            self.assertEqual(n_cum_strikes, expected_cum_strikes[n_firing])

if __name__ == '__main__':
    unittest.main()