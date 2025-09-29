# Nicholas A. Palumbo
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 4-9-24

# ========================
# PyRPOD: tests/mdao/mdao_verification_test_01.py
# ========================
# Axial positioning optimizer


import openmdao.api as om

from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy
from pyrpod.vehicle import TargetVehicle, LogisticsModule
from pyrpod.mdao import SweepConfig
from pyrpod.mission import MissionPlanner

class EvaluateImpingement(om.ExplicitComponent):

    """
        Minimizes the max heat flux load for a 1D approach by increasing the axial
        positioning of the thruster packs.
    """

    def initialize(self):

        # Path to directory holding data assets and results for a specific RPOD study.
        self.case_dir = '../case/axial_optimization/'

        # Instantiate TargetVehicle object.
        self.tv = TargetVehicle.TargetVehicle(self.case_dir)
        # Load Target Vehicle.
        self.tv.set_stl()

        # Instantiate LogisticModule object.
        self.lm = LogisticsModule.LogisticsModule(self.case_dir)
        # Load Visiting Vehicle.
        self.lm.set_stl()
        # Define LM mass distribution properties.
        self.m_dock = 14000 # kg
        self.h = 8.5 # m
        self.r = 1.65 # m
        self.lm.set_inertial_props(self.m_dock, self.h, self.r)
        # Load in thruster configuration file.
        self.lm.set_thruster_config()
        # Load in cluster configuration file.
        self.lm.set_cluster_config()
        # Load in thruster data file
        self.lm.set_thruster_metrics()
        # Use TCD to group DoF
        self.lm.assign_thruster_groups()

        # Instantiate RPOD object.
        self.rpod = PlumeStrikeEstimationStudy.RPOD(self.case_dir)

        self.mp = MissionPlanner.MissionPlanner(self.case_dir)
        self.mp.read_flight_plan()

        # Initialize iteration counter to be used in graph_jfh and jfh_plume_strikes file naming
        self.rpod.count = 1
        

    def setup(self):

        self.add_input('x', val=0.1)

        self.add_output('f_x', val=0)
    

    def setup_partials(self):

        self.declare_partials('*', '*', method='fd')


    def compute(self, inputs, outputs):
        
        # Load design variable
        x = inputs['x']
        
        # TEMP printing out the design variables steps
        x_print = "%.6f" % float(x)
        print('The current axial position being evaluated is', x_print, 'm.')


        # Update the nozzle's exit position in the thruster_data dictionary attribute
        self.lm.change_cluster_config(-x)


        # Resetting the inertial properties
        self.lm.set_inertial_props(self.m_dock, self.h, self.r)

        # Instantiate JetFiringHistory object.
        self.jfh = JetFiringHistory.JetFiringHistory(self.case_dir)
        # Read in the current JFH
        self.jfh.read_jfh()

        # Initiate RPOD study.
        self.rpod.study_init(self.jfh, self.tv, self.lm)
        
        # Produce JFH using 1D physics
        v_o = 2.1 # Initial velocity (m/s)
        v_ida = 0.03 # Docking velocity (m/s)
        
        # New function to edit self.jfh.JFH
        # 3rd input is cant angle
        self.rpod.calc_jfh_1d_approach(v_ida, v_o, 0)

        # Load STLs in Paraview
        self.rpod.graph_jfh()
        # Run plume strike analysis
        self.rpod.jfh_plume_strikes()

        # Initialization
        max_cum_heat_flux_load = 0
        # print('self.rpod.cellData["heat_flux_load"][0] is', self.rpod.cellData["heat_flux_load"][0])

        # Find the max_cum_heat_flux_load for a given approach
        for i in range(len(self.rpod.cellData["cum_heat_flux_load"])):
            if self.rpod.cellData["cum_heat_flux_load"][i] > max_cum_heat_flux_load:
                max_cum_heat_flux_load = self.rpod.cellData["cum_heat_flux_load"][i]
        max_cum_heat_flux_load_print = "%.6f" % max_cum_heat_flux_load
        print('The corresponding max cumulative heat flux load is', max_cum_heat_flux_load, 'J/m^2.')
        # print('len(self.rpod.cellData["heat_flux_load"]) is', len(self.rpod.cellData["heat_flux_load"]))
        
        # Set JFH to be used in propellant mass calculations.
        self.mp.set_jfh(self.jfh)
        # Assigns inertial properties to an attribute of mp, vv, to be used in propellant mass calculations.
        self.mp.set_lm(self.lm)
        # Calculate the propellant mass required for all maneuvers
        self.mp.calc_total_delta_mass()

        # TEMP print statements for propellant expenditure
        # print('The total propellant expended over the flight plan is', self.mp.dm_total, 'kg')
        dm_jfh_total_print = "%.6f" % self.mp.dm_jfh_total
        print('The corresponding JFH propellant expenditure is', dm_jfh_total_print, 'kg.')

        # Evaluate impingement
        outputs['f_x'] = max_cum_heat_flux_load

        # Increment the iteration counter to be used in graph_jfh and jfh_plume_strikes file naming
        self.rpod.count += 1







# This has been having issues, disregard for now (4/2/24)
if __name__ == '__main__':
    model = om.Group()
    model.add_subsystem('impingement_comp', EvaluateImpingement())

    prob = om.Problem(model)
    prob.setup()

    prob.set_val('impingement_comp.x', 0)
    prob.run_model()
    print(prob['impingement_comp.f_x'])

    prob.set_val('impingement_comp.x', 11.04-0.306)
    prob.run_model()
    print(prob['impingement_comp.f_x'])