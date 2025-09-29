import os
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy
from pyrpod.mdao import SweepConfig
import configparser

class TradeStudy():
    def __init__(self, case_dir):
        self.case_dir = case_dir
        config = configparser.ConfigParser()
        config.read(self.case_dir + "config.ini")
        self.config = config

    def init_trade_study(self, lm, tv):
        """
        Organizes data needed to kick off an RPOD trade study.

        Mainly done by properly configuring an RPOD object 

        """
        # Save variable name for readability.        
        case_dir = self.case_dir

        # Instantiate JetFiringHistory object.
        jfh = JetFiringHistory.JetFiringHistory(case_dir)

        # Instantiate RPOD object.
        rpod = PlumeStrikeEstimationStudy.RPOD(case_dir)
        rpod.study_init(jfh, tv, lm)
        self.rpod = rpod

    def init_trade_study_case(self):
        """
        Resets JFH data according to current case key.

        Case key is a unique identified for a specific configuration within the trade study. 
        """

        # Instantiate JetFiringHistory object.
        jfh = JetFiringHistory.JetFiringHistory(self.case_dir)
        jfh.config['jfh']['jfh'] = self.rpod.get_case_key() + ".A"
        jfh.read_jfh()

        self.rpod.jfh = jfh

    def print_mission_report(self):

        case_key = self.rpod.get_case_key()

        max_v0 = self.max_v0

        fuel_mass = self.rpod.fuel_mass

        max_pressure = self.rpod.max_pressure
        max_shear = self.rpod.max_shear
        max_heat_rate = self.rpod.max_heat_rate
        max_heat_load = self.rpod.max_heat_load
        max_cum_heat_load = self.rpod.max_cum_heat_load

        # Check if the file exists
        report_path = self.case_dir + 'results/MissionReport.csv'
        file_exists = os.path.isfile(report_path)

        # Open CSV file in append mode
        with open(report_path, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)

            # Write header if the file is newly created
            if not file_exists:
                csv_writer.writerow(['CaseKey', 'MaxV0', 'FuelMass', 'MaxPressure', 'MaxShear', 'MaxHeatRate', 'MaxHeatLoad', 'MaxCumulativeHeatLoad'])

            # Write data row
            csv_writer.writerow([
                case_key,
                max_v0,
                fuel_mass,
                max_pressure,
                max_shear,
                max_heat_rate,
                max_heat_load,
                max_cum_heat_load
            ])

    def graph_mission_report(self, report_results):
        """
        """
        # Extract the first column (assuming it's the parameter you want to plot against)
        first_column = report_results.columns[0]
        # Convert DataFrame columns to NumPy arrays for indexing
        x_values = report_results[first_column].to_numpy()
        print(report_results)
        # Plot each parameter against the first parameter
        for column in report_results.columns[1:8]:
            y_values = report_results[column].to_numpy()
            plt.figure()  # Create a new figure for each plot
            plt.plot(x_values,y_values, label=column)
            plt.xlabel(first_column)
            plt.ylabel(column)
            plt.title(f'{column} vs {first_column}')
            plt.legend()
            plt.grid(True)

        # Show all plots
        plt.show()


    def interpret_mission_report(self):
        """
        """
        report_path = self.case_dir + 'results/MissionReport.csv'
        report_results = pd.read_csv(report_path)

        max_pressure = float(self.rpod.config['tv']['normal_pressure'])
        max_shear = float(self.rpod.config['tv']['shear_pressure'])
        max_heat_rate = float(self.rpod.config['tv']['heat_flux'])
        # max_heat_load = float(self.rpod.config['tv'][]) no such constraint
        max_cum_heat_load = float(self.rpod.config['tv']['heat_flux_load'])

        plume_status = []
        plume_failure_mode = []

        for i, row in report_results.iterrows():
            if row['MaxPressure'] > max_pressure:
                plume_status.append('fail')
                plume_failure_mode.append('pressure')
            elif row['MaxShear'] > max_shear:
                plume_status.append('fail')
                plume_failure_mode.append('shear')
            elif row['MaxHeatRate'] > max_heat_rate:
                plume_status.append('fail')
                plume_failure_mode.append('heat_flux')
            # elif report_results['MaxHeatLoad'] > max_heat_load:  no such constraint
            #     plume_status.append('fail')
            elif row['MaxCumulativeHeatLoad'] > max_cum_heat_load:
                plume_status.append('fail')
                plume_failure_mode.append('cumulative_heat_flux_load')
            else:
                plume_status.append('pass')
                plume_failure_mode.append('none')

        report_results['PlumeStatus'] = plume_status
        report_results['PlumeFailureMode'] = plume_failure_mode
        self.graph_mission_report(report_results)

    def run_axial_overshoot_sweep(self, sweep_vars, lm, tv):
        """
        Simple variable sweep study that assesses RCS performance for a given set of axial overshoot velocity values.
        """

        # Organize variables to sweet over.
        axial_overshoot = sweep_vars['axial_overshoot']
        self.max_v0 = np.max(axial_overshoot)

        # Link elements for RPOD analysis.
        self.init_trade_study(lm, tv)

        # Create results directory if necessary.
        results_dir = self.case_dir + 'results'
        if not os.path.isdir(results_dir):
            os.mkdir(results_dir)       

        # Loop through over shoot velocities to test.
        for i, v_o in enumerate(axial_overshoot):

            # print(i, v_o)
            # # Set unique case identifier within trade study.
            self.rpod.set_case_key(i, 0)

            # Create JFH for a given velocity.
            self.rpod.print_jfh_1d_approach_n_fire(
                                    tv.v_ida,
                                    v_o,
                                    tv.r_o,
                                    n_firings = 100,
                                    trade_study = True
                                )

            # Reset JFH according to specific case.
            self.init_trade_study_case()   
            self.rpod.graph_jfh(trade_study= True)
            self.rpod.jfh_plume_strikes(trade_study = True)

            self.print_mission_report()
        self.interpret_mission_report()           
    

            # self.rpod.jfh_plume_strikes(trade_study = True)
    
    def run_surface_cant_sweep(self, sweep_vars, lm, tv):
        """
        """

        # Organize variables to sweet over.
        surface_cant_angles = sweep_vars['surface_cant_angles']
        v_o = sweep_vars['axial_overshoot']
        self.max_v0 = np.max(v_o)

        # Link elements for RPOD analysis.
        self.init_trade_study(lm, tv)

        angle_sweep = SweepConfig.SweepDecelAngles(lm.thruster_data, lm.rcs_groups)

        # Create results directory if necessary.
        results_dir = self.case_dir + 'results'
        if not os.path.isdir(results_dir):
            os.mkdir(results_dir)       

        # Loop through over shoot velocities to test.
        for i, cant in enumerate(surface_cant_angles):

            cant = np.deg2rad(cant)
            lm.decel_cant = cant

            new_tcd = angle_sweep.cant_decel_thrusters(cant)   
            lm.set_thruster_config(new_tcd)

            # print(i, v_o)
            # # Set unique case identifier within trade study.
            self.rpod.set_case_key(0, i)

            # Create JFH for a given velocity.
            self.rpod.print_jfh_1d_approach_n_fire(
                                    tv.v_ida,
                                    v_o,
                                    tv.r_o,
                                    n_firings = 100,
                                    trade_study = True
                                )

            # Reset JFH according to specific case.
            self.init_trade_study_case()              
    
            self.rpod.graph_jfh(trade_study= True)
            self.rpod.jfh_plume_strikes(trade_study = True)

            self.print_mission_report()
        self.interpret_mission_report()

    def run_multi_var_sweep(self, sweep_vars, lm, tv):
        """
        """
        # Organize variables to sweet over.
        axial_overshoot = sweep_vars['axial_overshoot']
        surface_cant_angles = sweep_vars['surface_cant_angles']
        self.max_v0 = np.max(axial_overshoot)

        # Link elements for RPOD analysis.
        self.init_trade_study(lm, tv)

        angle_sweep = SweepConfig.SweepDecelAngles(lm.thruster_data, lm.rcs_groups)

        # Create results directory if necessary.
        results_dir = self.case_dir + 'results'
        if not os.path.isdir(results_dir):
            os.mkdir(results_dir)       

        # Loop through over shoot velocities to test.
        for i, v_o in enumerate(axial_overshoot):
            for j, cant in enumerate(surface_cant_angles):

                lm.decel_cant = cant

                new_tcd = angle_sweep.cant_decel_thrusters(cant)   
                lm.set_thruster_config(new_tcd)

                # print(i, v_o)
                # # Set unique case identifier within trade study.
                self.rpod.set_case_key(i, j)

            # Create JFH for a given velocity.
                self.rpod.print_jfh_1d_approach_n_fire(
                                    tv.v_ida,
                                    v_o,
                                    tv.r_o,
                                    n_firings = 10,
                                    trade_study = True
                                )

                # Reset JFH according to specific case.
                self.init_trade_study_case()              
        
                self.rpod.graph_jfh(trade_study= True)
                self.rpod.jfh_plume_strikes(trade_study = True)

                self.print_mission_report()
        self.interpret_mission_report() 