# Juan P. Roldan
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-28-24

# ========================
# PyRPOD: test/mdao/mdao_unit_test_03.py
# ========================
# A test case to create array of cant angle swept thruster configurations.
# The sweep assumes the given thrusters angle symmetrically. 
# This means that opposite pitch thrusters angle opposite but equally to each other
# and yaw thrusters angle opposite but equally to each other
# !!currently, the pitch and yaw thrusters are canted in unison!!
# pitch thrusters are not angled such that they can produce a yaw
# yaw thrusters are not angled such that they can introduce a pitch

# TODO: Re-factor code to save data in a relevant object. Also add files to save to.

import test_header
import unittest, os, sys
import numpy as np

from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy
from pyrpod.vehicle import TargetVehicle, VisitingVehicle
from pyrpod.mdao import SweepConfig


class CoordinateSweepCheck(unittest.TestCase):
    def test_cant_sweep(self):

        # # number of deceleration thrusters to evenly distribute
        # # !!!IMPORTANT!!! also ensure to update jfh, fire all thrusters one wants to visualize!!!
        # nthrusters = 7

        # # define the LM's radius
        # r = 2 # m

        # # creating an example configuration

        # # identity matrix as the default for all thrusters
        # # techically not needed, since all the thrusters are set to -x group
        # # these thrusters DCMs are standardized within SweepConfig
        # dcm = np.eye(3)

        # # creating example thruster groups, to confirm thrusters that are for decel
        # thruster_groups = {
        #     '+x': [],
        #     'neg_x': [],
        #     '+y': [],
        #     '-y': [],
        #     '+z': [],
        #     '-z': [],
        #     '+pitch': [],
        #     '-pitch': [],
        #     '+yaw' : [],
        #     '-yaw' : []
        # }

        # config = {}

        # # name each thruster, as corresponding to different packs
        # # evenly distribute the exit coordinate of each thruster
        # # append each thruster to deceleration group
        # for i in range(1, nthrusters+1):
        #     name = 'P' + str(i) + 'T1'
        #     exit = [0, r*np.cos((i-1)*(2 * np.pi / nthrusters)), r*np.sin((i-1)*(2 * np.pi / nthrusters))]
        #     config[name] = {'name': [name], 'type': ['001'], 'exit': [exit], 'dcm': dcm}
        #     thruster_groups['neg_x'].append(name)

        # # define step sizes for each angle
        # dcant = 10 # deg

        # # create SweepAngles object
        # angle_sweep = SweepConfig.SweepDecelAngles(config, thruster_groups)
        
        # # call sweep_long_thruster on the configuration and print the DCM's
        # config_swept_array = angle_sweep.sweep_decel_thrusters_all(dcant)

        # for i, config in enumerate(config_swept_array):
        #     # Path to directory holding data assets and results for a specific RPOD study.
        #     case_dir = '../case/mdao/cant_sweep/'

        #     # Load JFH data.
        #     jfh = JetFiringHistory.JetFiringHistory(case_dir)
        #     jfh.read_jfh()

        #     tv = TargetVehicle.TargetVehicle(case_dir)
        #     tv.set_stl()

        #     vv = VisitingVehicle.VisitingVehicle(case_dir)
        #     vv.set_stl()
        #     vv.set_thruster_config(config)

        #     rpod = RPOD.RPOD(case_dir)
        #     rpod.study_init(jfh, tv, vv)

        #     rpod.visualize_sweep(i)
        return

if __name__ == '__main__':
    unittest.main()