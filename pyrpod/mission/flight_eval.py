import pandas as pd
import numpy as np

class FlightEvaluator:
    def __init__(self, case_dir, config):
        self.case_dir = case_dir
        self.config = config
        self.maneuvers = []

    def load_plan(self):
        # Parse CSV or other source
        pass

    def execute(self, mission_planner):
        # Loop over maneuvers and update planner
        pass


    def read_flight_plan(self, vv):
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

        self.vv = vv

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


    def set_current_6dof_state(self, v = [0, 0, 0], w = [0,0,0]):
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

    def set_desired_6dof_state(self, v = [0, 0, 0], w = [0,0,0]):
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

    def get_deltas(self):
        return self.v_desired - self.v_current, self.w_desired - self.w_current


    def calc_trans_performance(self, motion, dv):
        """
            Calculates RCS performance according to thruster working groups for a direction of motion.

            This method assumes constant mass, which needs to be addressed.

            Needs better name?

            Parameters
            ----------
            dv : float
                Speficied change in velocity value.

            motion : str
                Directionality of motion. Used to select active thrusters.

            Returns
            -------
            time : float
                Burn time elapsed.

            distance : float
                Distance covered during burn time.

            propellant_used : float
                Propellant used during burn time.
        """
        # Calculate RCS performance according to thrusters grouped to be in the direction.
        # WIP: Initial code executes simple 1DOF calculations
        # print(type(self.vv))
        # print(self.vv)
        if self.vv.rcs_groups == None:
            # print("WARNING: Thruster Grouping File not Set")
            return

        n_thrusters = len(self.vv.rcs_groups[motion])
        total_thrust = n_thrusters * self.vv.thrust
        acceleration = total_thrust / self.vv.mass
        # print(acceleration)
        time = abs(dv) / acceleration
        distance = 0.5 * abs(dv) * time
        m_dot = total_thrust / self.vv.isp
        propellant_used = m_dot * time

        # Print info to screen (TODO: write this to a data structure)
        p = 2 # how many decimals places to print
        # print('Total thrust produced', round(total_thrust, p), 'N')
        # print('Resulting accelration', round(acceleration, p), 'm / s ^ 2')
        # print('Time required', round(time, p), 's')
        # print('Distance Covered', round(distance, p), 'm')
        # print('Total propellant used', round(propellant_used, p), 'kg')

        return time, distance, propellant_used

    def calc_6dof_performance(self):
        """
            Wrapper method used to calculate performance for translation and rotational maneuvers.

            Parameters
            ----------
            None

            Returns
            -------
            None
        """
        # Wrapper function that sets up data for 6DOF performance
        dv = self.v_desired - self.v_current
        dw = self.w_desired - self.w_current

        # print('Required changes in 6DOF state')
        # print('dv', dv, 'm/s, dw', dw, 'm/s')
        # print()

        # Calculate performance for translation maneuvers
        # and assess directionality as needed
        translations = ['x', 'y', 'z']
        for i, v in enumerate(dv):
            if v ==0:
                pass
            elif v > 0:
                motion = '+' + translations[i]
                self.calc_trans_performance(motion, v)
            else:
                motion = '-' + translations[i]
                self.calc_trans_performance(motion, v)
        # print()

        # # Calculate performance for rotational maneuvers
        # # and assess directionality as needed
        # rotations = ['pitch', 'roll', 'yaw']
        # for i, v in enumerate(dv):
        #     if v ==0:
        #         pass
        #     elif v > 0:
        #         motion = '+' + rotations[i]
        #         self.calc_rot_performance(motion)
        #     else:
        #         motion = '-' + rotations[i]
        #         self.calc_rot_performance(motion)
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
