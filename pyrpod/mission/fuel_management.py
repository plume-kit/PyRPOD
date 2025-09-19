from pyrpod.mission.SubModule import SubModule
import numpy as np
import pandas as pd
from pyrpod.logging_utils import get_logger

logger = get_logger(__name__)

class FuelManager(SubModule):
    # def __init__(self, isp, mass):
    #     self.isp = isp
    #     self.mass = mass

    def compute_burn_time(self, dv, thrust):
        # Simplified rocket equation stub
        pass

    def compute_delta_mass(self, dv):
        # Return delta-mass from ideal rocket equation
        pass

    def calc_burn_time(self, dv, isp, T):
        """
            Calculates burn time when given change in velocity (dv), specific impulse (isp), and thrust (T)

            TODO: Create, return, and test t_burn (return variable).

            Parameters
            ----------
            dv : float
                Specified change in velocity value.

            isp : float
                Specified specific impulse value.

            T : float
                Specified thrust value.

            Returns
            -------
            t_burn : float
                Required burn time is seconds.
        """
        g_0=9.81
        m_f=self.vv.mass
        a = (dv)/(isp*g_0)
        K=(isp*g_0*m_f*(np.exp(a) - 1))
        return K / T

    def calc_delta_mass(self, dv, isp):
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

    def calc_delta_v(self, dt, v_e, m_dot, m_current):
        """
            Calculates change in velocity from the Tsiolkovsky equation given change in time (dt), exhaust velocity (v_e), and mass flow rate (m_dot).

            Parameters
            ----------
            dt : float
                Specified change in time value.

            v_e : float
                Specified exhaust velocity.

            m_dot : float
                Specified mass flow rate.

            m_current : float
                Specified mass.

            Returns
            -------
            dv : float
                Change in velocity in meters per second
        """
        dv = v_e*np.log(((m_dot*dt)/m_current)+1)
        return dv
    
    def calc_delta_omega_rotation(self, dm, group):
        """
            Calculates the angular velocity by discretizing the propellant expenditure
            and iteratively solving the rotational form of Newton's Second Law with updated moments of inertia.

            Parameters
            ----------
            dm : float
                Specified propellant usage value.

            group : string
                Thruster group from flight plan.

            Returns
            -------
            dw : float
                Change in angular velocity.
        """
        if group == 'pos_pitch' or group == 'pos_yaw':
            self.vv.set_inertial_props(self.vv.mass, self.vv.height, self.vv.radius)
            t_firing = dm / (self.calc_m_dot_sum(group) / 2)
            dw = (t_firing*self.vv.radius*(self.calc_thrust_sum(group) / 2)) / self.vv.I_y
        if group == 'roll':
            logger.error('ERROR: functionality not added for a roll rotation')
        
        return dw

    def calc_delta_mass_rotation(self, dw, group, forward_propagation):
        """
            Calculates propellant usage for a rotational maneuver by discretizing the desired angular velocity
            and iteratively solving the rotational form of Newton's Second Law with updated moments of inertia.

            TODO: add a 180 degree roll to the flight plan pre approach to match
            Orion Rendezvous, Proximity Operations, and Docking Design and Analysis by Souza

            Parameters
            ----------
            dw : float
                Specified change in angular velocity (rad / s) value.

            group : string
                Thruster group from flight plan.

            forward_propagation : boolean
                Specified direction of propagation.

            Returns
            -------
            dm : float
                Change in mass.
        """
        if forward_propagation == False:
            if group == 'pos_pitch' or group == 'pos_yaw':
                self.vv.set_inertial_props(self.vv.mass, self.vv.height, self.vv.radius)
                t_firing = (self.vv.I_y*dw)/(self.vv.radius*(self.calc_thrust_sum(group) / 2)) # I_y is pitch/yaw
                dm = (self.calc_m_dot_sum(group) / 2)*t_firing
                self.vv.mass += dm
            if group == 'roll':
                logger.error('ERROR: functionality not added for a roll rotation')
        

        if forward_propagation == True:
            if group == 'pos_pitch' or group == 'pos_yaw':
                self.vv.set_inertial_props(self.vv.mass, self.vv.height, self.vv.radius)
                # M = I*alpha -> Fr = I*omega/t
                t_firing = (self.vv.I_y*dw)/(self.vv.radius*self.calc_thrust_sum(group)) # I_y is pitch/yaw
                dm = self.calc_m_dot_sum(group)*t_firing
                self.vv.mass -= dm
            if group == 'roll':
                logger.error('ERROR: functionality not added for a roll rotation')
        return dm

    def calc_delta_mass_v_e(self, dv, v_e, forward_propagation):
        """
            Calculates propellant usage using expressions derived from the ideal rocket equation.

            Parameters
            ----------
            dv : float
                Specified change in velocity value.

            v_e : float
                Specified exhaust velocity value.

            forward_propagation : boolean
                Specified direction of propagation.

            Returns
            -------
            dm : float
                Change in mass.
        """
        m_current = self.vv.mass
        # print('m_current is', m_current)
        dm = m_current*(np.exp(dv/v_e)-1)

        if forward_propagation == False:
            self.vv.mass += dm

        if forward_propagation == True:
            self.vv.mass -= dm

        return dm

    def calc_total_delta_mass(self):
        """
            Sums total propellant expenditure.
            Starts with calculating the propellant expenditure for the JFH twice (approach which back propagates with a starting mass of 14,000 kg,
            and departure which forward propagates with a starting mass of 8,600 kg.), and saves mass before approach and mass after departure to
            be used as initial mass values for propellant expenditure calculations for maneuvers defined in the flight plan.

            Parameters
            ----------
            None
            
            Returns
            -------
            Total change in mass.
        """
        # Initialization
        self.dm_total = 0
        dm_jfh_total = 0
        payload_mass = 5400
        # Docking mass and post delivery mass
        initial_masses = [self.vv.mass, self.vv.mass - payload_mass]

        # Loop to find LM mass before approach and after departure
        for m in range(len(initial_masses)):

            self.vv.mass = initial_masses[m]

            # Make sure JFH is defined and has at least one firing.
            if self.jfh.JFH != None and len(self.jfh.JFH) > 0:
                # Read the JFH and add propellant expended for each firing to a sum
                for f in range(len(self.jfh.JFH)):
                    dm = 0
                    # Backpropagate with a vv.mass of 14,000 kg to find the vv.mass pre-approach
                    if m == 0:
                        forward_propagation = False
                    # Forward propagate with a vv.mass of 8,600 kg to find the vv.mass post-departure
                    if m == 1:
                        forward_propagation = True

                    # The JFH only contains firings done by the neg_x group
                    thruster_type = self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]
                    m_dot_sum = self.calc_m_dot_sum('neg_x')
                    v_e = self.calc_v_e('neg_x')
                    dt = float(self.jfh.JFH[0]['t'])

                    # Change in x velocity (axial)
                    dv_x = self.calc_delta_v(dt, v_e, m_dot_sum, initial_masses[m])

                    # Calculate fuel usage
                    if dv_x > 0:
                        # The change in mass will be the same for approach and departure regardless of the LM's mass
                        # because it is the same thruster group firing for the same amount of time
                        dm = self.calc_delta_mass_v_e(dv_x, v_e, forward_propagation)
                        # If backpropagating then add the propellant mass expended
                        if m == 0:
                            initial_masses[m] += dm
                            # If the last firing in the JFH has been accounted for, then m_approach has been found
                            if f == len(self.jfh.JFH) - 1:
                                m_approach = initial_masses[m]
                                # print('self.dm_total is', self.dm_total)
                                # print('dm is', dm)
                                self.dm_jfh_total = self.dm_total + dm
                                # print('dm_jfh_total is', dm_jfh_total)
                        # If forward propagating, then subtract the propellant mass expended
                        if m == 1:
                            initial_masses[m] -= dm
                            # If the last firing in the JFH has been accounted for, then m_departure has been found
                            if f == len(self.jfh.JFH) - 1:
                                m_departure = initial_masses[m]

                    self.dm_total += dm

        # print('JFH prop usage is', self.dm_total)



        # Saving the flight plan into a Pandas dataframe
        try:
            dataframe = pd.read_csv(self.case_dir + 'jfh/' + self.config['jfh']['flight_plan'])
        except KeyError:
            # print("WARNING: flight plan not set")
            return

        dataframe.columns = dataframe.columns.str.replace(' ', '')

        firings_list = dataframe.to_dict(orient='records')

        keys_list = dataframe.keys().tolist()

        # Back propagate from pre approach
        # Then forward propagate from post departure, accounting for a 180 degree pitch maneuver and disposal
        # A loop to create the flight plan "order of operations" (ooo) here from the flight_plan

        flight_plan_order_of_operations = []
        for o in range(len(firings_list)):
            flight_plan_order_of_operations.append(firings_list[o]['ooo'])

        for i in flight_plan_order_of_operations:

            # Starting flight plan back propagation
            if i == flight_plan_order_of_operations[0]:
                self.vv.mass = m_approach
                forward_propagation = False

            # Starting flight plan forward propagation
            # Subtracting 2 because there are the last two maneuvers are forward propagated for both flight plans
            if i == flight_plan_order_of_operations[len(flight_plan_order_of_operations) - 2]:
                # print('Initial separation mass is ', self.vv.mass, '\n')
                self.vv.mass = m_departure
                forward_propagation = True
            
            dm = 0

            # Re-naming to avoid indexing multiple times in the method.
            firing = firings_list[i]

            # Read in and calculate required inertial state changes.

            # The size of the inertial_state list is found by subtracting 7 from the length of the keys_list
            # since we dont need an inertial state for 'firing' or 'ooo' (2) and only need one per DOF excluding axial (5)
            inertial_state = np.zeros(len(keys_list) - 7)
            
            # Determinants for double count logic
            next = 2
            next_step = 1

            # Loop to populate inertial_state with values from the flight plan dataframe
            for k in range(len(inertial_state)):
                # Any index in inertial_state < len(inertial_state) - 5 has just one delta-v
                if k < len(inertial_state) - 5:
                    inertial_state[k] = firing[keys_list[next]]
                    next += 1
                # Any index in inertial_state > len(inertial_state) - 6 has two delta-vs to sum
                if k > (len(inertial_state) - 6):
                    inertial_state[k] = firing[keys_list[len(inertial_state) - 4 + next_step]] + firing[keys_list[len(inertial_state) - 4 + 1 + next_step]]
                    next_step += 2


            # Simplified to all positive because expenditure magnitude is independent of direction
            # DLT groups
            if len(inertial_state) == 8:
                groups = ['mae', 'me', 'ae', 'pos_y', 'pos_z', 'pos_roll', 'pos_pitch', 'pos_yaw']
            # BLT groups
            if len(inertial_state) == 6:
                groups = ['pos_x', 'pos_y', 'pos_z', 'pos_roll', 'pos_pitch', 'pos_yaw']
            
            # print('inertial_state is', inertial_state)

            # Calculate fuel usage for each change in inertial state.
            for i, state in enumerate(inertial_state):
                if state > 0:
                    # Subtracting 3 because the last 3 inertial state array values are rotational
                    # Any index in inertial_state < len(inertial_state) - 3 is a translational velocity
                    if i < len(inertial_state) - 3:
                        # print('groups[i] is', groups[i])
                        v_e = self.calc_v_e(groups[i])
                        dm = self.calc_delta_mass_v_e(state, v_e, forward_propagation)
                        # print('dm_translation is', dm)




                        if self.rotational_maneuvers == True:
                            # Make 10% of the propellant usage for a given translational maneuver be directed towards rotations
                            # print('-------- dm / 10 is', (dm / 10))
                            # Disregarding minimum duty cycle, discretize the dm
                            discretizing_resolution = 0.0001 # kg / s
                            dw_sum_rot = 0 # rad / s
                            num_iters = round(((dm / 10) / np.cos(self.cant)) / (discretizing_resolution))
                            for i in range(num_iters):
                                dw_rot = self.calc_delta_omega_rotation(discretizing_resolution, 'pos_pitch')
                                dw_sum_rot += dw_rot
                            # print('dw_sum_rot is', dw_sum_rot)
                            
                            discretizing_resolution = 0.0001 # rad / s
                            dm_sum_rot = 0
                            # Currently the rotation calculation is hardcoded for the pitch maneuver, which is why two is subtracted
                            num_iters = round(dw_sum_rot / discretizing_resolution)
                            
                            for j in range(num_iters):
                                dm_rot = self.calc_delta_mass_rotation(discretizing_resolution, 'pos_pitch', forward_propagation)
                                dm_sum_rot += dm_rot
                            dm += dm_sum_rot
                            # print('dm_sum_rot is', dm_sum_rot)
                            # print('dm_rotation + dm_translation is', dm)




                    # Any index in inertial_state > (len(inertial_state) - 4) is an angular velocity
                    if i > (len(inertial_state) - 4):
                        # print('self.vv.mass before is', self.vv.mass)
                        discretizing_resolution = 0.0001 # rad / s
                        dm_sum_rot = 0
                        # Currently the rotation calculation is hardcoded for the pitch maneuver, which is why two is subtracted
                        num_iters = round(inertial_state[len(inertial_state) - 2] / discretizing_resolution)
                        
                        for j in range(num_iters):
                            # if j == 1:
                            #     print('self.vv.I_y is', self.vv.I_y)
                            dm_rot = self.calc_delta_mass_rotation(discretizing_resolution, groups[i], forward_propagation)
                            dm_sum_rot += dm_rot
                        # print('dm_sum_rot is', dm_sum_rot)
                        # print('self.vv.mass after is', self.vv.mass)
                        dm += dm_sum_rot

            # print('dm is', dm)
            self.dm_total += dm

        # print('self.dm_total is', self.dm_total)


        # # Create results directory if it doesn't already exist.
        # results_dir = self.case_dir + 'results'
        # if not os.path.isdir(results_dir):
        #     #print("results dir doesn't exist")
        #     os.mkdir(results_dir)

        # # Save results to rudimentary log file.
        # with open(self.case_dir + 'results/prop_usage.txt', 'w') as f:
        #     self.dm_total = round(self.dm_total, 3)
        #     message = "The total propellant expended over the flight plan is " +  str(self.dm_total) + " kg"
        #     f.write(message)

        return
