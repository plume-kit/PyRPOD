class ThrusterGrouping:
    def __init__(self, rcs_config):
        self.rcs_groups = rcs_config

    def sum_thrust(self, group):
        # Aggregate thrust from a group
        pass

    def sum_mass_flow(self, group):
        # Aggregate m_dot from a group
        pass

    def mean_exhaust_velocity(self, group):
        # Compute effective exhaust velocity
        pass

    def calc_thrust_sum(self, group):
        """
            Calculates thrust sum for a thruster group.

            Parameters
            ----------
            group : string
                Thruster group from flight plan.

            Returns
            -------
            Thrust sum.
        """
        # print('self.vv.thruster_data is', self.vv.thruster_data)
        thrust_sum = 0
        if group == 'pos_x':
            for thruster_name in self.vv.rcs_groups[group]:
                thruster_type = self.vv.thruster_data[thruster_name]['type'][0]
                thrust = self.vv.thruster_metrics[thruster_type]['F']
                # print('thrust is', thrust)
                thrust_sum += thrust


        if group == 'neg_x' or group == 'pos_pitch':
            for thruster_name in self.vv.rcs_groups[group]:
                thruster_type = self.vv.thruster_data[thruster_name]['type'][0]
                # print('thruster_type is', thruster_type)
                thrust = np.cos(self.cant) * self.vv.thruster_metrics[thruster_type]['F']
                # print('thrust is', thrust)
                thrust_sum += thrust
        return thrust_sum

    def calc_m_dot_sum(self, group):
        """
            Calculates mass flow rate sum for a thruster group.

            Parameters
            ----------
            group : string
                Thruster group from flight plan.

            Returns
            -------
            Mass flow rate sum.
        """
        m_dot_sum = 0
        for thruster_name in self.vv.rcs_groups[group]:
            thruster_type = self.vv.thruster_data[thruster_name]['type'][0]
            m_dot = self.vv.thruster_metrics[thruster_type]['mdot']
            m_dot_sum += m_dot
        return m_dot_sum

    def calc_v_e(self, group):
        """
            Calculates mean exhaust velocity for a thruster group.

            Parameters
            ----------
            group : string
                Thruster group from flight plan.

            Returns
            -------
            Exhaust velocity.
        """
        thrust_sum = 0
        m_dot_sum = 0
        thrust_sum = self.calc_thrust_sum(group)
        m_dot_sum = self.calc_m_dot_sum(group)
        v_e = thrust_sum / m_dot_sum
        return v_e
