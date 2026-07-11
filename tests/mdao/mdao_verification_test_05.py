# Juan P. Roldan
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 04-04-24


# ========================
# PyRPOD: test/mdao/mdao_verification_test_02.py
# ========================
# 

import unittest

from pyrpod.vehicle import TargetVehicle, LogisticsModule
from pyrpod.mdao import TradeStudy

class MDAOTest(unittest.TestCase):
    def test_mdao(self):

    # # 1. Set Up
    #     # Load in Fixed LM and thruster configuation for trade study.
    #     case_dir = '../case/mdao/trade_study/'

    #     # Load Target Vehicle.
    #     tv = TargetVehicle.TargetVehicle(case_dir)
    #     tv.set_stl()

    #     # Instantiate LogisticModule object.
    #     lm = LogisticsModule.LogisticsModule(case_dir)

    #     # Define LM mass distribution properties.
    #     m = 14000 # kg
    #     h = 11 # m
    #     r = 2 # m
    #     lm.set_inertial_props(m, h, r)

    #     # Load in thruster configuration.
    #     lm.set_thruster_config()
    #     lm.set_thruster_metrics()
    #     lm.assign_thruster_groups()

    #     # Define LM Docking conditions
    #     v_ida = 0.03 # m/s (target velocity for safe docking)
    #     tv.set_v_ida(v_ida)
    #     r_o = 20 # m (initial distance at start of initial burn)
    #     tv.set_r_o(r_o)
 
    #     # Determine design variables to vary over. 
    #     # axial_overshoot = [0, 25, 50, 75, 100] # m/s (WIP, replace with physical values)
    #     # axial_overshoot = lm.calc_overshoot_v_range(v_ida, r_o)
    #     # axial_thruster_pos = [0, 2.75, 5.5, 8.25, 11] # m (ignoring solar panel since decel thrusters)
    #     surface_cant_angles = [0, 15, 30, 45, 60] # degrees

    #     sweep_vars = {
    #         'axial_overshoot': 1,
    #         # 'axial_thruster_pos': axial_thruster_pos,
    #         'surface_cant_angles': surface_cant_angles
    #     }

    # # 2. Excecute
    #     # Produce data for trade study by running docking analysis according to relevant design variable sweeps.
    #     study = TradeStudy.TradeStudy(case_dir)
    #     results = study.run_surface_cant_sweep(sweep_vars, lm, tv)

    #     # Post process results and perform trade studies analysis.
    #     # design_metrics = ['fuel_usage', 'plume', 'maneuver', 'safety']
    #     # ideal_configs = study.process_results(design_metrics, results)

    # # 3. Assert
    #     # TBD. This will be developed at the very end to lock in desired results given 

        return

if __name__ == '__main__':
    unittest.main()
