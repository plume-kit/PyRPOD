from pyrpod.vehicle.LogisticsModule import LogisticsModule
from pyrpod.orbital.HohmannTransfer import HohmannTransfer

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import configparser
import math

class MissionPlanner:
    """
        Class responsible for initial performance analysis of space flight missions.

        Caculated metrics (outputs) include propellant usage,
        trajectory character, and performance with respect to factors of safety.

        Data Inputs inlcude (redundant? better said in user guide?)
        1. LogisticsModule (LM) object with properly defined inertial properties and candidate RCS configurations.
        2. Proposed flight plan including required chenges in velocity and supporting requirements.


        Attributes
        ----------

        vv : LogisticsModule
            Visiting vehicle of interest. Includes complete RCS configuration and surface mesh data.


        Methods
        -------
        set_lm(LogisticsModule)
            Simple setter method to set VV/LM used in analysis.
        
        set_jfh(JetFiringHistory)
            Simple setter method to set JFH used in propellant usage calculations.
        
        set_current_6dof_state(v = [0, 0, 0], w = [0,0,0])
            Sets VV current inertial state. Can be done manually or read from flight plan.

        set_desired_6dof_state(v = [0, 0, 0], w = [0,0,0])
            Sets VV desired inertial state. Can be done manually or read from flight plan.

        calc_burn_time(dv, isp, T)
            Calculates burn time when given change in velocity (dv), specific impulse (isp), and thrust (T)

        plot_burn_time(dv)
            Plots burn time for a given dv and isp value. Varries thrust according the inputs.

        plot_burn_time_contour(dv)
            Plots burn time for a given dv by varrying thrust values. Graph is contoured using ISP values.

        plot_burn_time_flight_plan()
            Plots burn time for all dv maneuvers in the specified flight plan.

        calc_delta_mass(dv, isp)
            Calculates propellant usage using expressions derived from the ideal rocket equation.

        plot_delta_mass(dv)
            Plots propellant usage for a given dv requirements by varying ISP according to user inputs.

        plot_delta_mass_contour()
            Co-Plots propellant usage for all dv maneuvers in the specified flight plan.

        calc_trans_performance(motion, dv)
            Calculates RCS performance according to thruster working groups for a direction of 3DOF motion .

        calc_6dof_performance()
            Calculates performance for translation and rotational maneuvers.

        read_flight_plan()
            Reads in VV flight as specified using CSV format.

        calc_flight_performance()
            Calculates 6DOF performance for all firings specified in the flight plan.

        plot_thrust_envelope()
            Plots operational envelope relating burn time to thrust required for all firings in the flight plan.

        calc_delta_v(dt, v_e, m_dot)
            Calculates change in velocity from the Tsiolkovsky equation given change in time (dt), exhaust velocity (v_e), and mass flow rate (m_dot).

        calc_delta_omega_rotation(dm, group)
            Calculates the angular velocity by discretizing the propellant expenditure
            and iteratively solving the rotational form of Newton's Second Law with updated moments of inertia.

        calc_delta_mass_rotation(dw, group, forward_propagation)
            Calculates propellant usage for a rotational maneuver by discretizing the desired angular velocity
            and iteratively solving the rotational form of Newton's Second Law with updated moments of inertia.

        calc_delta_mass_v_e(dv, v_e)
            Calculates propellant usage using expressions derived from the ideal rocket equation.

        calc_thrust_sum(group)
            Calculates thrust sum for a thruster group.

        calc_m_dot_sum(group)
            Calculates mass flow rate sum for a thruster group.

        calc_v_e(group)
            Calculates mean exhaust velocity for a thruster group.

        calc_total_delta_mass(LogisticsModule)
            Sums propellant expenditure.
    """
    def __init__(self, case_dir):
        """
            Designates assets for RPOD analysis.

            Parameters
            ----------
            LogisticsModule : LogisticsModule
                LM Object containing surface mesh and thruster configuration data.

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        """
        # TODO: Add variables for trade study analysis. Maybe?
        self.case_dir = case_dir
        config = configparser.ConfigParser()
        config.read(self.case_dir + "config.ini")
        self.config = config
        # print(self.config)

    # def set_lm(self, LogisticsModule):
        """
            Simple setter method to set VV/LM used in analysis.

            NOTE: This begs the question: What's up with LM vs VV. Do we need both classes?
            If so, how do we handle inheritance between them? Previous efforts have broken
            the code. This is due to "hacky/minimal" effort. A follow up attempt would
            require research into how Python handles inheritance including "container classes".

            Parameters
            ----------
            LogisticsModule : LogisticsModule
                LogisticsModule Object containing inertial properties.

            Returns
            -------
            None
        """
        self.vv = LogisticsModule
    
    # def set_jfh(self, JetFiringHistory):
        """
            Simple setter method to set JFH used in propellant usage calculations.

            Parameters
            ----------
            JetFiringHistory : JetFiringHistory
                JFH Object containing specifics of each firing.

            Returns
            -------
            None
        """
        self.jfh = JetFiringHistory

    # def set_current_6dof_state(self, v = [0, 0, 0], w = [0,0,0]):
        """
            Sets current inertial state for the VV. Can be done manually or read from flight plan.

            Parameters
            ----------
            v : 3 element list
                Contains vector components of translational velocity.

            w : 3 element list
                Contains vector components of rotational velocity.

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        """
        self.v_current = np.array(v)
        self.w_current = np.array(w)
        return

    # def set_desired_6dof_state(self, v = [0, 0, 0], w = [0,0,0]):
        """
            Sets desired inertial state for the VV. Can be done manually or read from flight plan.

            Parameters
            ----------
            v : 3 element list
                Contains vector components of translational velocity.

            w : 3 element list
                Contains vector components of rotational velocity.

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        """
        self.v_desired = np.array(v)
        self.w_desired = np.array(w)
        return

    # def calc_burn_time(self, dv, isp, T):
    #     """
    #         Calculates burn time when given change in velocity (dv), specific impulse (isp), and thrust (T)

    #         TODO: Create, return, and test t_burn (return variable).

    #         Parameters
    #         ----------
    #         dv : float
    #             Specified change in velocity value.

    #         isp : float
    #             Specified specific impulse value.

    #         T : float
    #             Specified thrust value.

    #         Returns
    #         -------
    #         t_burn : float
    #             Required burn time is seconds.
    #     """
    #     g_0=9.81
    #     m_f=self.vv.mass
    #     a = (dv)/(isp*g_0)
    #     K=(isp*g_0*m_f*(np.exp(a) - 1))
    #     return K / T

    # def plot_burn_time(self, dv):
    #     """
    #         Plots burn time for a given dv and isp value. Varies thrust according to user inputs.

    #         TODO: Add ISP value as a parameter. Remove isp_vals, add isp as a parameter to the function.
    #         Test code.

    #         TODO: Integrate with establsihed configuration file framework.

    #         Parameters
    #         ----------
    #         dv : float
    #             Specified change in velocity value.

    #         isp : float
    #             Specified specific impulse value.

    #         Returns
    #         -------
    #         Method doesn't currently return anything. Simply sets class members as needed.
    #         Does the method need to return a status message? or pass similar data?

    #     """
    #     isp_vals = [50, 200, 300, 400, 500]
    #     thrust_range = np.linspace(50, 600, 5000)
    #     burn_time = []

    #     isp = 200
    #     for thrust in thrust_range:
    #         burn_time.append(abs(self.calc_burn_time(dv, isp, thrust)))
    #     burn_time = np.array(burn_time)

    #     fig, ax = plt.subplots()


    #     ax.plot(thrust_range, burn_time)
    #     ax.set(xlabel='Thrust (s)', ylabel='Burn-time (s)',
    #         title='Thrust vs Burn-time Required (' + str(abs(dv)) + ')')
    #     ax.grid()
    #     ax.legend()
    #     # plt.xscale("log")
    #     # plt.yscale("log")
    #     fig.savefig("test.png")

    # def plot_burn_time_contour(self, dv):
    #     """
    #         Plots burn time for a given dv by varrying thrust values. Graph is contoured using ISP values.

    #         TODO: Add isp_vals as a parameter. Integrate with configuration file framework. Test Code.

    #         Parameters
    #         ----------
    #         dv : float
    #             Specified change in velocity value.

    #         Returns
    #         -------
    #         Method doesn't currently return anything. Simply sets class members as needed.
    #         Does the method need to return a status message? or pass similar data?
    #     """
    #     isp_vals = [300]
    #     thrust_range = np.linspace(1, 1000, 5000)

    #     fig, ax = plt.subplots()

    #     for isp in isp_vals:
    #         burn_time = []
    #         for thrust in thrust_range:
    #             burn_time.append(abs(self.calc_burn_time(dv, isp, thrust)))
    #         burn_time = np.array(burn_time) / (3600*24)
    #         ax.plot(thrust_range, burn_time, label = 'ISP = (' + str(abs(isp)) + ' s)')

    #     ax.set(xlabel='Thrust (N)', ylabel='Burn-time (days)',
    #         title='Thrust vs Burn-time Required (Δv = ' + str(abs(dv)) + ' m/s)')
    #     ax.grid()
    #     ax.legend()
    #     plt.xscale("log")
    #     # plt.yscale("log")
    #     fig.savefig("test.png")
    #     return

    # def plot_burn_time_flight_plan(self):
    #     """
    #         Plots burn time for all dv maneuvers in the specified flight plan.

    #         Parameters
    #         ----------
    #         None

    #         Returns
    #         -------
    #         Method doesn't currently return anything. Simply sets class members as needed.
    #         Does the method need to return a status message? or pass similar data?
    #     """
    #     isp = 300
    #     thrust_range = np.linspace(10, 100, 5000)

    #     fig, ax = plt.subplots()

    #     dv = self.flight_plan.iterrows()

    #     for v in dv:
    #         # print(type(v[1]))
    #         dv = v[1][1]
    #         print()
    #         burn_time = []
    #         for thrust in thrust_range:
    #             burn_time.append(abs(self.calc_burn_time(dv, isp, thrust)))
    #         burn_time = np.array(burn_time) / (3600*24)
    #         ax.plot(thrust_range, burn_time, label = 'Δv = (' + str(abs(dv)) + ' m/s)')

    #     ax.set(xlabel='Thrust (N)', ylabel='Burn-time (days)',
    #         title='Burn-time Required vs Thrust (ISP = ' + str(abs(300)) + ' s)')
    #     ax.grid()
    #     ax.legend()
    #     # plt.xscale("log")
    #     # plt.yscale("log")
    #     fig.savefig("test.png")

    #     return

    # def calc_delta_mass(self, dv, isp):
        """
            Calculates propellant usage using expressions derived from the ideal rocket equation.

            Parameters
            ----------
            dv : float
                Speficied change in velocity value.

            isp : float
                Speficied specific impulse value.

            Returns
            -------
            dm : float
                Change in mass calculated using the ideal rocket equation.
        """
        g_0 = 9.81
        a = (dv/(isp*g_0))
        m_f = self.vv.mass
        dm = m_f * (np.exp(a) - 1)
        self.vv.mass += dm
        return dm

    # def plot_delta_mass(self, dv):
    #     """
    #         Plots propellant usage for a given dv requirements by varying ISP according to user inputs.

    #         Parameters
    #         ----------
    #         dv : float
    #             Speficied change in velocity value.

    #         Returns
    #         -------
    #         Method doesn't currently return anything. Simply sets class members as needed.
    #         Does the method need to return a status message? or pass similar data?
    #     """
    #     isp_range = np.linspace(100, 600, 5000)
    #     delta_mass = []

    #     for isp in isp_range:
    #         delta_mass.append(abs(self.calc_delta_mass(dv, isp)))
    #     delta_mass = np.array(delta_mass)
    #     # for i, isp, in enumerate(isp_range):
    #     #     print(isp_range[i], delta_mass[i])

    #     thrust_tech = {
    #         'electro thermal': [50, 185],
    #         # 'hall-effect': [800, 1950],
    #         'cold-warm-gas': [30, 110],
    #         'mono-bi-propellants': [160, 310]
    #     }

    #     fig, ax = plt.subplots()
    #     for tech in thrust_tech:
    #         # print(tech)

    #         y_vals = np.array([delta_mass.max(), delta_mass.mean(), delta_mass.min()])
    #         isp_val = thrust_tech[tech][1]
    #         isp_line = np.array([isp_val, isp_val, isp_val])

    #         ax.plot(isp_line, y_vals, label=tech)

    #     ax.plot(isp_range, delta_mass)
    #     ax.set(xlabel='ISP (s)', ylabel='mass (kg)',
    #         title='Max ISP vs Propellant Mass Required (' + str(abs(dv)) + ' m/s)')
    #     ax.grid()
    #     ax.legend()
    #     # plt.xscale("log")
    #     # plt.yscale("log")
    #     fig.savefig("test.png")

    # def plot_delta_mass_contour(self):
        """
            Co-Plots propellant usage for all dv maneuvers in the specified flight plan.

            TODO: Add isp_range as a parameter. Integrate with configuration file framework. Test Code.

            Parameters
            ----------
            None

            Returns
            -------
            None
        """
        #creat plotting object.
        fig, ax = plt.subplots()

        delta_mass_min = 10e9
        delta_mass_max = 0

        # Step through all planned firings in the flight plan
        for firing in self.flight_plan.iterrows():
            # save delta v requirement to a local variable.
            dv = firing[1][1]

            # Calculate change in mass for a given range of ISP values.
            isp_range = np.linspace(50, 400, 5000)
            delta_mass = []

            for isp in isp_range:
                delta_mass.append(abs(self.calc_delta_mass(dv, isp)))
            delta_mass = np.array(delta_mass)

            # Save absolute min and max data for plotting.
            if delta_mass.max() > delta_mass_max:
                delta_mass_max = delta_mass.max()

            if delta_mass.min() < delta_mass_min:
                delta_mass_min = delta_mass.min()

            # Plot data.
            ax.plot(isp_range, delta_mass, label='( Δv =' + str(abs(dv)) + ' m/s)')

        # thrust_tech = {
        #     # 'electro thermal': [50, 185],
        #     # 'hall-effect': [800, 1950],
        #     'cold-warm-gas': [30, 110],
        #     'mono-bi-propellants': [160, 310]
        # }

        # for tech in thrust_tech:
        #     print(tech)

        #     y_vals = np.array([delta_mass_max, 0.5*(delta_mass_max + delta_mass_min), delta_mass_min])
        #     isp_val = thrust_tech[tech][1]
        #     isp_line = np.array([isp_val, isp_val, isp_val])

        #     ax.plot(isp_line, y_vals, label=tech, linestyle='dotted')

        # Set plot display parameters.
        ax.set(xlabel='Specific Impulse (s)', ylabel='Propellant Mass Required (kg)',
            title='Propellant Mass Required vs Specific Impulse')
        ax.grid()
        ax.legend()
        # plt.xscale("log")
        # plt.yscale("log")
        fig.tight_layout(pad=1.8)

        # Save to file
        fig.savefig("test.png")
        return

    # def calc_trans_performance(self, motion, dv):
    #     """
    #         Calculates RCS performance according to thruster working groups for a direction of motion.

    #         This method assumes constant mass, which needs to be addressed.

    #         Needs better name?

    #         Parameters
    #         ----------
    #         dv : float
    #             Speficied change in velocity value.

    #         motion : str
    #             Directionality of motion. Used to select active thrusters.

    #         Returns
    #         -------
    #         time : float
    #             Burn time elapsed.

    #         distance : float
    #             Distance covered during burn time.

    #         propellant_used : float
    #             Propellant used during burn time.
    #     """
    #     # Calculate RCS performance according to thrusters grouped to be in the direction.
    #     # WIP: Initial code executes simple 1DOF calculations
    #     # print(type(self.vv))
    #     # print(self.vv)
    #     if self.vv.rcs_groups == None:
    #         # print("WARNING: Thruster Grouping File not Set")
    #         return

    #     n_thrusters = len(self.vv.rcs_groups[motion])
    #     total_thrust = n_thrusters * self.vv.thrust
    #     acceleration = total_thrust / self.vv.mass
    #     # print(acceleration)
    #     time = abs(dv) / acceleration
    #     distance = 0.5 * abs(dv) * time
    #     m_dot = total_thrust / self.vv.isp
    #     propellant_used = m_dot * time

    #     # Print info to screen (TODO: write this to a data structure)
    #     p = 2 # how many decimals places to print
    #     # print('Total thrust produced', round(total_thrust, p), 'N')
    #     # print('Resulting accelration', round(acceleration, p), 'm / s ^ 2')
    #     # print('Time required', round(time, p), 's')
    #     # print('Distance Covered', round(distance, p), 'm')
    #     # print('Total propellant used', round(propellant_used, p), 'kg')

    #     return time, distance, propellant_used

    # def calc_6dof_performance(self):
    #     """
    #         Wrapper method used to calculate performance for translation and rotational maneuvers.

    #         Parameters
    #         ----------
    #         None

    #         Returns
    #         -------
    #         None
    #     """
    #     # Wrapper function that sets up data for 6DOF performance
    #     dv = self.v_desired - self.v_current
    #     dw = self.w_desired - self.w_current

    #     # print('Required changes in 6DOF state')
    #     # print('dv', dv, 'm/s, dw', dw, 'm/s')
    #     # print()

    #     # Calculate performance for translation maneuvers
    #     # and assess directionality as needed
    #     translations = ['x', 'y', 'z']
    #     for i, v in enumerate(dv):
    #         if v ==0:
    #             pass
    #         elif v > 0:
    #             motion = '+' + translations[i]
    #             self.calc_trans_performance(motion, v)
    #         else:
    #             motion = '-' + translations[i]
    #             self.calc_trans_performance(motion, v)
    #     # print()

    #     # # Calculate performance for rotational maneuvers
    #     # # and assess directionality as needed
    #     # rotations = ['pitch', 'roll', 'yaw']
    #     # for i, v in enumerate(dv):
    #     #     if v ==0:
    #     #         pass
    #     #     elif v > 0:
    #     #         motion = '+' + rotations[i]
    #     #         self.calc_rot_performance(motion)
    #     #     else:
    #     #         motion = '-' + rotations[i]
    #     #         self.calc_rot_performance(motion)
    #     return

    # def read_flight_plan(self):
        """
            Reads in VV flight as specified using CSV format.

            NOTE: Method assumes that self.case_dir and self.config are instantiated
            correctly. Potential defensive programming statements?

            Parameters
            ----------
            None

            Returns
            -------
            None
        """
        # Reads and parses through flight plan CSV file.
        try:
            path_to_file = self.case_dir + 'jfh/' + self.config['jfh']['flight_plan']
        except KeyError:
            # print("WARNING: flight plan not set")
            self.flight_plan = None
            return
        self.flight_plan = pd.read_csv(path_to_file)
        # print(self.flight_plan)

        return

    def calc_flight_performance(self):
        """
            Calculates 6DOF performance for all firings specified in the flight plan.

            Parameters
            ----------
            None

            Returns
            -------
            None
        """
        for firing in self.flight_plan.iterrows():

                    # Convert firing data to numpy arra for easier data manipulation.
                    firing_array = np.array(firing[1])

                    # save firing ID
                    nth_firing = np.array(firing[1].iloc[0])
                    # print('Firing number', nth_firing)

                    # calculate required change in translational velcoity
                    v1 = firing_array[4:7]
                    v0 = firing_array[1:4]
                    dv = v1 - v0

                    # calculate required change in translational velcoity
                    w1 = firing_array[10:13]
                    w0 = firing_array[7:10]
                    dw = w1 - w0
                    # print(nth_firing, dv, dw)

                    self.set_current_6dof_state(v0, w0)
                    self.set_desired_6dof_state(v1, w1)

                    self.calc_6dof_performance()
                    # print('======================================')
        return

    # def plot_thrust_envelope(self):
    #     """
    #         Plots operational envelope relating burn time to thrust required for all firings in the flight plan.

    #         Parameters
    #         ----------
    #         None

    #         Returns
    #         -------
    #         None
    #     """
    #     # print(self.vv)
    #     # print(self.flight_plan)

    #     for firing in self.flight_plan.iterrows():
    #         # Parse flight plan data.
    #         # print(firing)
    #         firing_array = np.array(firing[1])
    #         # print(firing_array)

    #         # calculate required change in translational velcoity
    #         v1 = firing_array[4:7]
    #         v0 = firing_array[1:4]
    #         dv = v1 - v0

    #         # Create lists to hold data for plotting
    #         time_req = []
    #         distance_req = []

    #         # Create range of thrust values to claculate.
    #         thrust_vals = np.linspace(10, 1000, 100)

    #         for thrust in thrust_vals:

    #             self.vv.add_thruster_performance(thrust, 100)
    #             time, distance, propellant_used = self.calc_trans_performance('+x', dv)
                
    #             time_req.append(time)
    #             distance_req.append(distance)

    #         fig, ax = plt.subplots()
    #         ax.plot(time_req, thrust_vals)


    #         ax.set(xlabel='time (s)', ylabel='thrust (N)',
    #             title='Thrust vs Time Required (' + str(abs(dv[0])) + ')')
            
    #         ax.grid()
    #         plt.xscale("log")
    #         # plt.yscale("log")


    #         fig.savefig("test" + str(firing[0]) + ".png")
    #         plt.show()
    #     return
    
    # def calc_delta_v(self, dt, v_e, m_dot, m_current):
    #     """
    #         Calculates change in velocity from the Tsiolkovsky equation given change in time (dt), exhaust velocity (v_e), and mass flow rate (m_dot).

    #         Parameters
    #         ----------
    #         dt : float
    #             Specified change in time value.

    #         v_e : float
    #             Specified exhaust velocity.

    #         m_dot : float
    #             Specified mass flow rate.

    #         m_current : float
    #             Specified mass.

    #         Returns
    #         -------
    #         dv : float
    #             Change in velocity in meters per second
    #     """
    #     dv = v_e*np.log(((m_dot*dt)/m_current)+1)
    #     return dv
    
    # def calc_delta_omega_rotation(self, dm, group):
    #     """
    #         Calculates the angular velocity by discretizing the propellant expenditure
    #         and iteratively solving the rotational form of Newton's Second Law with updated moments of inertia.

    #         Parameters
    #         ----------
    #         dm : float
    #             Specified propellant usage value.

    #         group : string
    #             Thruster group from flight plan.

    #         Returns
    #         -------
    #         dw : float
    #             Change in angular velocity.
    #     """
    #     if group == 'pos_pitch' or group == 'pos_yaw':
    #         self.vv.set_inertial_props(self.vv.mass, self.vv.height, self.vv.radius)
    #         t_firing = dm / (self.calc_m_dot_sum(group) / 2)
    #         dw = (t_firing*self.vv.radius*(self.calc_thrust_sum(group) / 2)) / self.vv.I_y
    #     if group == 'roll':
    #         print('ERROR: functionality not added for a roll rotation')
        
    #     return dw

    # def calc_delta_mass_rotation(self, dw, group, forward_propagation):
    #     """
    #         Calculates propellant usage for a rotational maneuver by discretizing the desired angular velocity
    #         and iteratively solving the rotational form of Newton's Second Law with updated moments of inertia.

    #         TODO: add a 180 degree roll to the flight plan pre approach to match
    #         Orion Rendezvous, Proximity Operations, and Docking Design and Analysis by Souza

    #         Parameters
    #         ----------
    #         dw : float
    #             Specified change in angular velocity (rad / s) value.

    #         group : string
    #             Thruster group from flight plan.

    #         forward_propagation : boolean
    #             Specified direction of propagation.

    #         Returns
    #         -------
    #         dm : float
    #             Change in mass.
    #     """
    #     if forward_propagation == False:
    #         if group == 'pos_pitch' or group == 'pos_yaw':
    #             self.vv.set_inertial_props(self.vv.mass, self.vv.height, self.vv.radius)
    #             t_firing = (self.vv.I_y*dw)/(self.vv.radius*(self.calc_thrust_sum(group) / 2)) # I_y is pitch/yaw
    #             dm = (self.calc_m_dot_sum(group) / 2)*t_firing
    #             self.vv.mass += dm
    #         if group == 'roll':
    #             print('ERROR: functionality not added for a roll rotation')
        

    #     if forward_propagation == True:
    #         if group == 'pos_pitch' or group == 'pos_yaw':
    #             self.vv.set_inertial_props(self.vv.mass, self.vv.height, self.vv.radius)
    #             # M = I*alpha -> Fr = I*omega/t
    #             t_firing = (self.vv.I_y*dw)/(self.vv.radius*self.calc_thrust_sum(group)) # I_y is pitch/yaw
    #             dm = self.calc_m_dot_sum(group)*t_firing
    #             self.vv.mass -= dm
    #         if group == 'roll':
    #             print('ERROR: functionality not added for a roll rotation')
    #     return dm

    # def calc_delta_mass_v_e(self, dv, v_e, forward_propagation):
    #     """
    #         Calculates propellant usage using expressions derived from the ideal rocket equation.

    #         Parameters
    #         ----------
    #         dv : float
    #             Specified change in velocity value.

    #         v_e : float
    #             Specified exhaust velocity value.

    #         forward_propagation : boolean
    #             Specified direction of propagation.

    #         Returns
    #         -------
    #         dm : float
    #             Change in mass.
    #     """
    #     m_current = self.vv.mass
    #     # print('m_current is', m_current)
    #     dm = m_current*(np.exp(dv/v_e)-1)

    #     if forward_propagation == False:
    #         self.vv.mass += dm

    #     if forward_propagation == True:
    #         self.vv.mass -= dm

    #     return dm

    # def calc_thrust_sum(self, group):
    #     """
    #         Calculates thrust sum for a thruster group.

    #         Parameters
    #         ----------
    #         group : string
    #             Thruster group from flight plan.

    #         Returns
    #         -------
    #         Thrust sum.
    #     """
    #     # print('self.vv.thruster_data is', self.vv.thruster_data)
    #     thrust_sum = 0
    #     if group == 'pos_x':
    #         for thruster_name in self.vv.rcs_groups[group]:
    #             thruster_type = self.vv.thruster_data[thruster_name]['type'][0]
    #             thrust = self.vv.thruster_metrics[thruster_type]['F']
    #             # print('thrust is', thrust)
    #             thrust_sum += thrust


    #     if group == 'neg_x' or group == 'pos_pitch':
    #         for thruster_name in self.vv.rcs_groups[group]:
    #             thruster_type = self.vv.thruster_data[thruster_name]['type'][0]
    #             # print('thruster_type is', thruster_type)
    #             thrust = np.cos(self.cant) * self.vv.thruster_metrics[thruster_type]['F']
    #             # print('thrust is', thrust)
    #             thrust_sum += thrust
    #     return thrust_sum

    # def calc_m_dot_sum(self, group):
    #     """
    #         Calculates mass flow rate sum for a thruster group.

    #         Parameters
    #         ----------
    #         group : string
    #             Thruster group from flight plan.

    #         Returns
    #         -------
    #         Mass flow rate sum.
    #     """
    #     m_dot_sum = 0
    #     for thruster_name in self.vv.rcs_groups[group]:
    #         thruster_type = self.vv.thruster_data[thruster_name]['type'][0]
    #         m_dot = self.vv.thruster_metrics[thruster_type]['mdot']
    #         m_dot_sum += m_dot
    #     return m_dot_sum

    # def calc_v_e(self, group):
    #     """
    #         Calculates mean exhaust velocity for a thruster group.

    #         Parameters
    #         ----------
    #         group : string
    #             Thruster group from flight plan.

    #         Returns
    #         -------
    #         Exhaust velocity.
    #     """
    #     thrust_sum = 0
    #     m_dot_sum = 0
    #     thrust_sum = self.calc_thrust_sum(group)
    #     m_dot_sum = self.calc_m_dot_sum(group)
    #     v_e = thrust_sum / m_dot_sum
    #     return v_e

    # def calc_total_delta_mass(self):
    #     """
    #         Sums total propellant expenditure.
    #         Starts with calculating the propellant expenditure for the JFH twice (approach which back propagates with a starting mass of 14,000 kg,
    #         and departure which forward propagates with a starting mass of 8,600 kg.), and saves mass before approach and mass after departure to
    #         be used as initial mass values for propellant expenditure calculations for maneuvers defined in the flight plan.

    #         Parameters
    #         ----------
    #         None
            
    #         Returns
    #         -------
    #         Total change in mass.
    #     """
    #     # Initialization
    #     self.dm_total = 0
    #     dm_jfh_total = 0
    #     payload_mass = 5400
    #     # Docking mass and post delivery mass
    #     initial_masses = [self.vv.mass, self.vv.mass - payload_mass]

    #     # Loop to find LM mass before approach and after departure
    #     for m in range(len(initial_masses)):

    #         self.vv.mass = initial_masses[m]

    #         # Make sure JFH is defined and has at least one firing.
    #         if self.jfh.JFH != None and len(self.jfh.JFH) > 0:
    #             # Read the JFH and add propellant expended for each firing to a sum
    #             for f in range(len(self.jfh.JFH)):
    #                 dm = 0
    #                 # Backpropagate with a vv.mass of 14,000 kg to find the vv.mass pre-approach
    #                 if m == 0:
    #                     forward_propagation = False
    #                 # Forward propagate with a vv.mass of 8,600 kg to find the vv.mass post-departure
    #                 if m == 1:
    #                     forward_propagation = True

    #                 # The JFH only contains firings done by the neg_x group
    #                 thruster_type = self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]
    #                 m_dot_sum = self.calc_m_dot_sum('neg_x')
    #                 v_e = self.calc_v_e('neg_x')
    #                 dt = float(self.jfh.JFH[0]['t'])

    #                 # Change in x velocity (axial)
    #                 dv_x = self.calc_delta_v(dt, v_e, m_dot_sum, initial_masses[m])

    #                 # Calculate fuel usage
    #                 if dv_x > 0:
    #                     # The change in mass will be the same for approach and departure regardless of the LM's mass
    #                     # because it is the same thruster group firing for the same amount of time
    #                     dm = self.calc_delta_mass_v_e(dv_x, v_e, forward_propagation)
    #                     # If backpropagating then add the propellant mass expended
    #                     if m == 0:
    #                         initial_masses[m] += dm
    #                         # If the last firing in the JFH has been accounted for, then m_approach has been found
    #                         if f == len(self.jfh.JFH) - 1:
    #                             m_approach = initial_masses[m]
    #                             # print('self.dm_total is', self.dm_total)
    #                             # print('dm is', dm)
    #                             self.dm_jfh_total = self.dm_total + dm
    #                             # print('dm_jfh_total is', dm_jfh_total)
    #                     # If forward propagating, then subtract the propellant mass expended
    #                     if m == 1:
    #                         initial_masses[m] -= dm
    #                         # If the last firing in the JFH has been accounted for, then m_departure has been found
    #                         if f == len(self.jfh.JFH) - 1:
    #                             m_departure = initial_masses[m]

    #                 self.dm_total += dm

    #     # print('JFH prop usage is', self.dm_total)



    #     # Saving the flight plan into a Pandas dataframe
    #     try:
    #         dataframe = pd.read_csv(self.case_dir + 'jfh/' + self.config['jfh']['flight_plan'])
    #     except KeyError:
    #         # print("WARNING: flight plan not set")
    #         return

    #     dataframe.columns = dataframe.columns.str.replace(' ', '')

    #     firings_list = dataframe.to_dict(orient='records')

    #     keys_list = dataframe.keys().tolist()

    #     # Back propagate from pre approach
    #     # Then forward propagate from post departure, accounting for a 180 degree pitch maneuver and disposal
    #     # A loop to create the flight plan "order of operations" (ooo) here from the flight_plan

    #     flight_plan_order_of_operations = []
    #     for o in range(len(firings_list)):
    #         flight_plan_order_of_operations.append(firings_list[o]['ooo'])

    #     for i in flight_plan_order_of_operations:

    #         # Starting flight plan back propagation
    #         if i == flight_plan_order_of_operations[0]:
    #             self.vv.mass = m_approach
    #             forward_propagation = False

    #         # Starting flight plan forward propagation
    #         # Subtracting 2 because there are the last two maneuvers are forward propagated for both flight plans
    #         if i == flight_plan_order_of_operations[len(flight_plan_order_of_operations) - 2]:
    #             # print('Initial separation mass is ', self.vv.mass, '\n')
    #             self.vv.mass = m_departure
    #             forward_propagation = True
            
    #         dm = 0

    #         # Re-naming to avoid indexing multiple times in the method.
    #         firing = firings_list[i]

    #         # Read in and calculate required inertial state changes.

    #         # The size of the inertial_state list is found by subtracting 7 from the length of the keys_list
    #         # since we dont need an inertial state for 'firing' or 'ooo' (2) and only need one per DOF excluding axial (5)
    #         inertial_state = np.zeros(len(keys_list) - 7)
            
    #         # Determinants for double count logic
    #         next = 2
    #         next_step = 1

    #         # Loop to populate inertial_state with values from the flight plan dataframe
    #         for k in range(len(inertial_state)):
    #             # Any index in inertial_state < len(inertial_state) - 5 has just one delta-v
    #             if k < len(inertial_state) - 5:
    #                 inertial_state[k] = firing[keys_list[next]]
    #                 next += 1
    #             # Any index in inertial_state > len(inertial_state) - 6 has two delta-vs to sum
    #             if k > (len(inertial_state) - 6):
    #                 inertial_state[k] = firing[keys_list[len(inertial_state) - 4 + next_step]] + firing[keys_list[len(inertial_state) - 4 + 1 + next_step]]
    #                 next_step += 2


    #         # Simplified to all positive because expenditure magnitude is independent of direction
    #         # DLT groups
    #         if len(inertial_state) == 8:
    #             groups = ['mae', 'me', 'ae', 'pos_y', 'pos_z', 'pos_roll', 'pos_pitch', 'pos_yaw']
    #         # BLT groups
    #         if len(inertial_state) == 6:
    #             groups = ['pos_x', 'pos_y', 'pos_z', 'pos_roll', 'pos_pitch', 'pos_yaw']
            
    #         # print('inertial_state is', inertial_state)

    #         # Calculate fuel usage for each change in inertial state.
    #         for i, state in enumerate(inertial_state):
    #             if state > 0:
    #                 # Subtracting 3 because the last 3 inertial state array values are rotational
    #                 # Any index in inertial_state < len(inertial_state) - 3 is a translational velocity
    #                 if i < len(inertial_state) - 3:
    #                     # print('groups[i] is', groups[i])
    #                     v_e = self.calc_v_e(groups[i])
    #                     dm = self.calc_delta_mass_v_e(state, v_e, forward_propagation)
    #                     # print('dm_translation is', dm)




    #                     if self.rotational_maneuvers == True:
    #                         # Make 10% of the propellant usage for a given translational maneuver be directed towards rotations
    #                         # print('-------- dm / 10 is', (dm / 10))
    #                         # Disregarding minimum duty cycle, discretize the dm
    #                         discretizing_resolution = 0.0001 # kg / s
    #                         dw_sum_rot = 0 # rad / s
    #                         num_iters = round(((dm / 10) / np.cos(self.cant)) / (discretizing_resolution))
    #                         for i in range(num_iters):
    #                             dw_rot = self.calc_delta_omega_rotation(discretizing_resolution, 'pos_pitch')
    #                             dw_sum_rot += dw_rot
    #                         # print('dw_sum_rot is', dw_sum_rot)
                            
    #                         discretizing_resolution = 0.0001 # rad / s
    #                         dm_sum_rot = 0
    #                         # Currently the rotation calculation is hardcoded for the pitch maneuver, which is why two is subtracted
    #                         num_iters = round(dw_sum_rot / discretizing_resolution)
                            
    #                         for j in range(num_iters):
    #                             dm_rot = self.calc_delta_mass_rotation(discretizing_resolution, 'pos_pitch', forward_propagation)
    #                             dm_sum_rot += dm_rot
    #                         dm += dm_sum_rot
    #                         # print('dm_sum_rot is', dm_sum_rot)
    #                         # print('dm_rotation + dm_translation is', dm)




    #                 # Any index in inertial_state > (len(inertial_state) - 4) is an angular velocity
    #                 if i > (len(inertial_state) - 4):
    #                     # print('self.vv.mass before is', self.vv.mass)
    #                     discretizing_resolution = 0.0001 # rad / s
    #                     dm_sum_rot = 0
    #                     # Currently the rotation calculation is hardcoded for the pitch maneuver, which is why two is subtracted
    #                     num_iters = round(inertial_state[len(inertial_state) - 2] / discretizing_resolution)
                        
    #                     for j in range(num_iters):
    #                         # if j == 1:
    #                         #     print('self.vv.I_y is', self.vv.I_y)
    #                         dm_rot = self.calc_delta_mass_rotation(discretizing_resolution, groups[i], forward_propagation)
    #                         dm_sum_rot += dm_rot
    #                     # print('dm_sum_rot is', dm_sum_rot)
    #                     # print('self.vv.mass after is', self.vv.mass)
    #                     dm += dm_sum_rot

    #         # print('dm is', dm)
    #         self.dm_total += dm

    #     # print('self.dm_total is', self.dm_total)


    #     # # Create results directory if it doesn't already exist.
    #     # results_dir = self.case_dir + 'results'
    #     # if not os.path.isdir(results_dir):
    #     #     #print("results dir doesn't exist")
    #     #     os.mkdir(results_dir)

    #     # # Save results to rudimentary log file.
    #     # with open(self.case_dir + 'results/prop_usage.txt', 'w') as f:
    #     #     self.dm_total = round(self.dm_total, 3)
    #     #     message = "The total propellant expended over the flight plan is " +  str(self.dm_total) + " kg"
    #     #     f.write(message)

    #     return

    # def init_hohmann_transfers(self):
    #     self.hohmann_transfers = []

    # def add_hohmann_transfer(self, h1_km: float, h2_km: float, leg_id: str = None):
    #     """
    #     Computes and stores the delta-v and time of flight for a Hohmann transfer leg.

    #     Parameters:
    #         h1_km (float): Starting orbit altitude [km]
    #         h2_km (float): Ending orbit altitude [km]

    #         leg_id (str, optional): Descriptive name or identifier for the transfer leg
    #     """
    #     transfer = HohmannTransfer(h1_km, h2_km, leg_id)
    #     result = transfer._compute_transfer()
    #     if leg_id:
    #         print(leg_id)
    #         print(result)
    #         result['leg_id'] = leg_id
    #     self.hohmann_transfers.append(result)

    # def summarize_hohmann_transfers(self):
    #     """
    #     Prints a summary of all stored Hohmann transfers.
    #     """
    #     for i, t in enumerate(self.hohmann_transfers):
    #         print(f"Leg {i+1} ({t.get('leg_id', 'unnamed')}):")
    #         print(f"  Δv1 = {t['dv1']:.3f} km/s, Δv2 = {t['dv2']:.3f} km/s")
    #         print(f"  Total Δv = {t['dv_total']:.3f} km/s")
    #         print(f"  Time of Flight = {t['tof'] / 60:.2f} hours\n")


