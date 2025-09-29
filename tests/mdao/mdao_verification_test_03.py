# Nicholas A. Palumbo
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 4-9-24

# ========================
# PyRPOD: tests/mdao/mdao_verification_test_03.py
# ========================
# Axial + Cant Evaluator


import openmdao.api as om
from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy
from pyrpod.vehicle import TargetVehicle, LogisticsModule
from pyrpod.mdao import SweepConfig
from pyrpod.mission import MissionPlanner


class EvaluateImpingement(om.ExplicitComponent):

    """
        Evaluates the performance of a RCS configuration given the axial positioning and
        cant angle of the deceleration thrusters, outputting parameters such as max
        cumulative heat flux load and JFH propellant expenditure for a 1D approach.

        NOTE: The first two stl and vtu groups saved will be
            for the min and max cant angle respectively.
    """

    def initialize(self):

        # Path to directory holding data assets and results for a specific RPOD study.
        self.case_dir = '../case/cant_optimization/'

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

        self.add_input('x', val=0)
        self.add_input('y', val=0)

        self.add_output('dt')

        self.add_output('pressure')
        self.add_output('shear stress')
        self.add_output('heat flux rate')
        self.add_output('heat flux load')
        self.add_output('cumulative heat flux load')
        self.add_output('propellant')

        self.add_output('scaled_pressure')
        self.add_output('scaled_shear_stress')
        self.add_output('scaled_heat_flux_rate')
        self.add_output('scaled_heat_flux_load')
        self.add_output('scaled_cum_heat_flux_load')
        self.add_output('scaled_propellant')

        self.add_output('scaled_pressure_sum')
        self.add_output('scaled_shear_stress_sum')
        self.add_output('scaled_heat_flux_rate_sum')
        self.add_output('scaled_heat_flux_load_sum')
        self.add_output('scaled_cum_heat_flux_load_sum')


    def setup_partials(self):

        self.declare_partials('*', '*', method='fd')


    def compute(self, inputs, outputs):
        
        # Load design variables.
        x = inputs['x']
        y = inputs['y']
        
        # TEMP printing out the design variables steps
        x_print = "%.3f" % float(x)
        y_print = "%.3f" % float(y)
        print('--- Cant angle:', x_print, 'deg --- Axial position:', y_print, 'm ---')


        # Update the thrusters cant angle in the thruster_data dictionary attribute
        # Instantiate SweepAngles object.
        self.sc = SweepConfig.SweepDecelAngles(self.lm.thruster_data, self.lm.rcs_groups)
        # Load LogisticsModule object to reference rcs_groups and thruster_data in SweepConfig.py
        self.sc.set_lm(self.lm)
        # Create the new tcf
        self.new_tcf = self.sc.one_cant_decel_thrusters_all(self.lm.thruster_data, float(x))
        
        # Update VisitingVehicle's thruster_data attribute.
        self.lm.set_thruster_config(self.new_tcf)


        # Update the nozzle's exit position in the thruster_data dictionary attribute
        self.lm.change_cluster_config(-y)


        # Resetting the inertial properties.
        self.lm.set_inertial_props(self.m_dock, self.h, self.r)

        # Instantiate JetFiringHistory object.
        self.jfh = JetFiringHistory.JetFiringHistory(self.case_dir)
        # Read in the current JFH.
        self.jfh.read_jfh()

        # Initiate RPOD study.
        self.rpod.study_init(self.jfh, self.tv, self.lm)
        
        # Produce JFH using 1D physics.
        v_o = 2.1 # Initial velocity (m/s)
        v_ida = 0.03 # Docking velocity (m/s)
        
        # New function to edit self.jfh.JFH
        # Lets no longer pass in r_0 and instead start from zero and find
            # the distance required to slow down
        self.rpod.calc_jfh_1d_approach(v_ida, v_o, float(x))
        outputs['dt'] = self.rpod.dt
        # Load STLs in Paraview.
        self.rpod.graph_jfh()
        # Run plume strike analysis.
        self.rpod.jfh_plume_strikes()



        # Initialization.
        max_pressure = 0
        max_shear_stress = 0
        max_heat_flux_rate = 0
        max_heat_flux_load = 0
        max_cum_heat_flux_load = 0

        # print('self.rpod.cellData["heat_flux_load"][0] is', self.rpod.cellData["heat_flux_load"][0])

        # print('len(self.rpod.cellData) is', len(self.rpod.cellData))
        
        # print('len(self.rpod.cellData["max_pressures"]) is', len(self.rpod.cellData["max_pressures"]))
        # print('len(self.rpod.cellData["max_shear_force"]) is', len(self.rpod.cellData["max_shear_force"]))
        # print('len(self.rpod.cellData["heat_flux_rate"]) is', len(self.rpod.cellData["heat_flux_rate"]))
        # print('len(self.rpod.cellData["heat_flux_load"]) is', len(self.rpod.cellData["heat_flux_load"]))
        # print('len(self.rpod.cellData["cum_heat_flux_load"]) is', len(self.rpod.cellData["cum_heat_flux_load"]))

        # Find the max plume parameters for a given approach.
        for i in range(len(self.rpod.cellData["cum_heat_flux_load"])):
            if self.rpod.cellData["pressures"][i] > max_pressure:
                max_pressure = self.rpod.cellData["pressures"][i]
            if self.rpod.cellData["shear_stress"][i] > max_shear_stress:
                max_shear_stress = self.rpod.cellData["shear_stress"][i]
            if self.rpod.cellData["heat_flux_rate"][i] > max_heat_flux_rate:
                max_heat_flux_rate = self.rpod.cellData["heat_flux_rate"][i]
            if self.rpod.cellData["heat_flux_load"][i] > max_heat_flux_load:
                max_heat_flux_load = self.rpod.cellData["heat_flux_load"][i]
            if self.rpod.cellData["cum_heat_flux_load"][i] > max_cum_heat_flux_load:
                max_cum_heat_flux_load = self.rpod.cellData["cum_heat_flux_load"][i]

        # TEMP print statements for plume parameters
        # max_pressures_print = "%.2f" % max_pressures
        # max_shear_print = "%.2f" % max_shear
        # max_heat_flux_rate_print = "%.2f" % max_heat_flux_rate
        # max_heat_flux_load_print = "%.2f" % max_heat_flux_load
        # max_cum_heat_flux_load_print = "%.2f" % max_cum_heat_flux_load
        # print('The max pressure is', max_pressures_print, 'Pa.')
        # print('The max shear stress is', max_shear_print, 'Pa.')
        # print('The max heat flux rate is', max_heat_flux_rate_print, 'W/m^2.')
        # print('The max heat flux load is', max_heat_flux_load_print, 'J/m^2.')
        # print('The max cumulative heat flux load is', max_cum_heat_flux_load_print, 'J/m^2.')
        


        # Set JFH to be used in propellant mass calculations.
        self.mp.set_jfh(self.jfh)
        # Assigns inertial properties to an attribute of mp, vv, to be used in propellant mass calculations.
        self.mp.set_lm(self.lm)

        # Define a boolean to decide whether flight plan propellant usage is calculated
            # with an additional 10% of translational propellant per maneuver for pitch|yaw rotations
        self.mp.rotational_maneuvers = True

        # Calculate the propellant mass required for all maneuvers.
        self.mp.calc_total_delta_mass()

        # TEMP print statements for propellant expenditure
        # print('The total propellant expended over the flight plan is', self.mp.dm_total, 'kg')
        # dm_jfh_total_print = "%.2f" % self.mp.dm_jfh_total
        # print('The JFH propellant expenditure is', dm_jfh_total_print, 'kg.')


        # Evaluate impingement.
        outputs['pressure'] = max_pressure
        outputs['shear stress'] = max_shear_stress
        outputs['heat flux rate'] = max_heat_flux_rate
        outputs['heat flux load'] = max_heat_flux_load
        outputs['cumulative heat flux load'] = max_cum_heat_flux_load
        # Define the propellant expenditure.
        outputs['propellant'] = self.mp.dm_total


        # Increment the iteration counter to be used in graph_jfh and jfh_plume_strikes file naming.
        self.rpod.count += 1
        # print('self.rpod.count is', self.rpod.count)






# This has been having issues, disregard for now (4/3/24)
if __name__ == '__main__':
    model = om.Group()
    model.add_subsystem('impingement_comp', EvaluateImpingement())

    prob = om.Problem(model)
    prob.setup()

    prob.set_val('impingement_comp.x', 60)
    prob.run_model()
    print(prob['impingement_comp.f_x'])

    print('impingement_comp.x.upper is', 'impingement_comp.x.upper')

    prob.set_val('impingement_comp.x.upper', )
    prob.run_model()
    print(prob['impingement_comp.f_x'])