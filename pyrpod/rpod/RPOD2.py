import numpy as np
import os
import math

from stl import mesh

from pyrpod.vehicle.LogisticsModule import LogisticsModule
from pyrpod.mission.MissionPlanner import MissionPlanner
from pyrpod.plume.RarefiedPlumeGasKinetics import SimplifiedGasKinetics

from pyrpod.util.io.file_print import print_1d_JFH

from tqdm import tqdm
from queue import Queue

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    if vec1 == vec2:
        x = [1, 0, 0]
        y = [0, 1, 0]
        z = [0, 0, 1]
        id_matrix = np.array([x, y, z])
        return id_matrix

    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2)) 
    return rotation_matrix

class RPOD (MissionPlanner):
    """
        Class responsible for analyzing RPOD performance of visiting vehicles.

        Caculated metrics (outputs) include propellant usage, plume impingement,
        trajectory character, and performance with respect to factors of safety.

        Data Inputs inlcude (redundant? better said in user guide?)
        1. LogisticsModule (LM) object with properly defined RCS configuration
        2. Jet Firing History including LM location and orientation with repsect to the Gateway.
        3. Selected plume models for impingement analysis.
        4. Surface mesh data for target and visiting vehicle.

        Attributes
        ----------

        vv : LogisticsModule
            Visiting vehicle of interest. Includes complete RCS configuration and surface mesh data.

        jfh : JetFiringHistory
            Includes VV location and orientation with respect to the TV.

        plume_model : PlumeModel
            Contains the relevant governing equations selected for analysis.

        Methods
        -------
        study_init(self, JetFiringHistory, Target, Vehicle)
            Designates assets for RPOD analysis.
        
        graph_init_config(self)
            Creates visualization data for initiial configuration of RPOD analysis.

        graph_jfh_thruster_check(self)
            Creates visualization data for initiial configuration of RPOD analysis.
        
        graph_clusters(self, firing, vv_orientation)
            Creates visualization data for the cluster.
        
        graph_jfh(self)
            Creates visualization data for the trajectory of the proposed RPOD analysis.

        update_window_queue(self, window_queue, cur_window, firing_time, window_size)
            Takes the most recent window of time size, and adds the new firing time to the sum, and the window_queue.
            If the new window is larger than the allowed window_size, then earleist firing times are removed
            from the queue and subtracted from the cur_window sum, until the sum fits within the window size.
            A counter for how many firing times are removed and subtracted is recorded.

        update_parameter_queue(self, param_queue, param_window_sum, get_counter)
            Takes the current parameter_queue, and removes the earliest tracked parameters from the front of the queue.
            This occurs "get_counter" times. Each time a parameter is popped from the queue, the sum is also updated,
            as to not track the removed parameter (ie. subtract the value)

        jfh_plume_strikes(self)
            Calculates number of plume strikes according to data provided for RPOD analysis.
            Method does not take any parameters but assumes that study assets are correctly configured.
            These assets include one JetFiringHistory, one TargetVehicle, and one VisitingVehicle.
            A Simple plume model is used. It does not calculate plume physics, only strikes. which
            are determined with a user defined "plume cone" geometry. Simple vector mathematics is
            used to determine if an VTK surface elements is struck by the "plume cone".
        
        print_jfh_1d_approach(v_ida, v_o, r_o)
            Method creates JFH data for axial approach using simpified physics calculations.
    """
    # def __init__(self):
    #     print("Initialized Approach Visualizer")
    def study_init(self, JetFiringHistory, Target, Vehicle):
        """
            Designates assets for RPOD analysis.

            Parameters
            ----------
            JetFiringHistory : JetFiringHistory
                Object thruster firing history. It includes VV position, orientation, and IDs for active thrusters.

            Target : TargetVehicle
                Object containing surface mesh and thruster configurations for the Visiting Vehicle.

            Vehicle : VisitingVehicle
                Object containing surface mesh and surfave properties for the Target Vehicle.

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        """
        self.jfh = JetFiringHistory
        self.target = Target
        self.vv = Vehicle

    def graph_init_config(self):
        """
            Creates visualization data for initiial configuration of RPOD analysis.

            NOTE: Method does not take any parameters. It assumes that self.case_dir
            and self.config are instatiated correctly. Potential defensive programming statements?

            TODO: Needs to be re-factored to save VTK data in proper case directory.

            Returns
            -------
            Method doesn't currently return anything. Simply produces data as needed.
            Does the method need to return a status message? or pass similar data?
        """

        #TODO: Re-factor using 
        # Save first coordinate in the JFH
        vv_initial_firing = self.jfh.JFH[0]
        # print(type(vv_initial_firing['xyz']))

        # Translate VV STL to first coordinate
        self.vv.mesh.translate(vv_initial_firing['xyz'])

        # Combine target and VV STLs into one "Mesh" object.
        combined = mesh.Mesh(np.concatenate(
            [self.target.mesh.data, self.vv.mesh.data]
            ))

        figure = plt.figure()
        # axes = mplot3d.Axes3D(figure)
        axes = figure.add_subplot(projection = '3d')
        axes.add_collection3d(mplot3d.art3d.Poly3DCollection(combined.vectors))
        # axes.quiver(X, Y, Z, U, V, W, color = (0,0,0), length=1, normalize=True)
        lim = 100
        axes.set_xlim([-1*lim, lim])
        axes.set_ylim([-1*lim, lim])
        axes.set_zlim([-1*lim, lim])
        axes.set_xlabel('X')
        axes.set_ylabel('Y')
        axes.set_zlabel('Z')
        # figure.suptitle(str(i))
        plt.show()


    def graph_jfh_thruster_check(self):
        """
            Creates visualization data for initiial configuration of RPOD analysis.

            NOTE: Method does not take any parameters. It assumes that self.case_dir
            and self.config are instatiated correctly. Potential defensive programming statements?

            TODO: Needs to be re-factored to save VTK data in proper case directory.

            Returns
            -------
            Method doesn't currently return anything. Simply produces data as needed.
            Does the method need to return a status message? or pass similar data?
        """

        # Link JFH numbering of thrusters to thruster names.  
        link = {}
        i = 1
        for thruster in self.vv.thruster_data:
            link[str(i)] = self.vv.thruster_data[thruster]['name']
            i = i + 1

        # Loop through each firing in the JFH.
        for firing in range(len(self.jfh.JFH)):

            # Save active thrusters for current firing. 
            thrusters = self.jfh.JFH[firing]['thrusters']
            
            # Load and graph STL of visting vehicle.  
            VVmesh = mesh.Mesh.from_file('../stl/cylinder.stl')

            figure = plt.figure()
            # axes = mplot3d.Axes3D(figure)
            axes = figure.add_subplot(projection = '3d')
            axes.add_collection3d(mplot3d.art3d.Poly3DCollection(VVmesh.vectors))
 
            # Load and graph STLs of active thrusters. 
            for thruster in thrusters:
                # Load plume STL in initial configuration. 
                plumeMesh = mesh.Mesh.from_file('../stl/mold_funnel.stl')
                plumeMesh.translate([0, 0, -50])
                plumeMesh.rotate([1, 0, 0], math.radians(180)) 
                plumeMesh.points = 0.05 * plumeMesh.points

                # Transform according to DCM and exit vector for current thruster in VTDF
                print(link[str(thruster)][0])
                thruster_id = link[str(thruster)][0]
                thruster_orientation = np.matrix(
                    self.vv.thruster_data[thruster_id]['dcm']
                )

                plumeMesh.rotate_using_matrix(thruster_orientation.transpose())
                plumeMesh.translate(self.vv.thruster_data[thruster_id]['exit'][0])


                print(self.vv.thruster_data[thruster_id]['dcm'])

                # Add surface to graph. 
                surface = mplot3d.art3d.Poly3DCollection(plumeMesh.vectors)
                surface.set_facecolor('orange')
                axes.add_collection3d(surface)

            lim = 7
            axes.set_xlim([-1*lim - 3, lim - 3])
            axes.set_ylim([-1*lim, lim])
            axes.set_zlim([-1*lim, lim])
            axes.set_xlabel('X')
            axes.set_ylabel('Y')
            axes.set_zlabel('Z')
            shift=0
            axes.view_init(azim=0, elev=2*shift)


            print()

            if firing < 10:
                index = '00' + str(firing)
            elif firing < 100:
                index = '0' + str(firing)
            else:
                index = str(i)
            plt.savefig('img/frame' + str(index) + '.png')

    def graph_clusters(self, firing, vv_orientation):
        """
            Creates visualization data for the cluster.
            Parameters
            ----------
            firing : int
                Loop iterable over the length of the number of thrusters firing in the JFH.
            
            vv_orientation : np.array
                DCM from the JFH.
            Returns
            -------
            active_clusters : mesh
                Cluster of the current thruster firing.
        """
        active_clusters = None
        clusters_list = []
        for number in range(len(self.vv.cluster_data)):
            cluster_name = 'P' + str(number + 1)
            # print(cluster_name)
            clusters_list.append(cluster_name)

        # print('clusters_list is', clusters_list)
        # Load and graph STLs of active clusters. 
        for cluster in clusters_list:

            # Load plume STL in initial configuration. 
            clusterMesh = mesh.Mesh.from_file(self.case_dir + 'stl/' + self.config['vv']['stl_cluster'])

            # Transform cluster

            # First, according to DCM of current cluster in CCF
            cluster_orientation = np.array(
                self.vv.cluster_data[cluster]['dcm']
            )
            clusterMesh.rotate_using_matrix(cluster_orientation.transpose())

            # Second, according to DCM of VV in JFH
            clusterMesh.rotate_using_matrix(vv_orientation.transpose())

            # Third, according to position vector of the VV in JFH
            clusterMesh.translate(self.jfh.JFH[firing]['xyz'])

            # Fourth, according to position of current cluster in CCF
            clusterMesh.translate(self.vv.cluster_data[cluster]['exit'][0])
            # print(self.vv.cluster_data[cluster]['exit'][0])

            if active_clusters == None:
                active_clusters = clusterMesh
            else:
                active_clusters = mesh.Mesh(
                    np.concatenate([active_clusters.data, clusterMesh.data])
                )
        return active_clusters

    def graph_jfh(self, trade_study = False): 
        """
            Creates visualization data for the trajectory of the proposed RPOD analysis.

            This method does NOT calculate plume strikes.

            This utilities allows engineers to visualize the trajectory in the JFH before running
            the full simulation and wasting computation time.
            Returns
            -------
            Method doesn't currently return anything. Simply produces data as needed.
            Does the method need to return a status message? or pass similar data?
        """
        # Link JFH numbering of thrusters to thruster names.  
        link = {}
        i = 1
        for thruster in self.vv.thruster_data:
            link[str(i)] = self.vv.thruster_data[thruster]['name']
            i = i + 1
        
        # Create results directory if it doesn't already exist.
        results_dir = self.case_dir + 'results'
        if not os.path.isdir(results_dir):
            # print("results dir doesn't exist")
            os.mkdir(results_dir)


        if not trade_study:
            results_dir = results_dir + "/jfh"
            if not os.path.isdir(results_dir):
                #print("results dir doesn't exist")
                os.mkdir(results_dir)

        if trade_study:
            v_o = ['vo_0', 'vo_1', 'vo_2', 'vo_3', 'vo_4']
            cants = ['cant_0', 'cant_1', 'cant_2', 'cant_3', 'cant_4']
            for v in v_o:
                for cant in cants:
                    results_dir_case = results_dir + "/" + v + '_' + cant
                    if not os.path.isdir(results_dir_case):
                        #print("results dir doesn't exist")
                        os.mkdir(results_dir_case)

        # Save STL surface of target vehicle to local variable.
        target = self.target.mesh

        # Loop through each firing in the JFH.
        for firing in range(len(self.jfh.JFH)):
            # print('firing =', firing+1)

            # Save active thrusters for current firing. 
            thrusters = self.jfh.JFH[firing]['thrusters']
            # print("thrusters", thrusters)
             
            # Load, transform, and, graph STLs of visiting vehicle.  
            VVmesh = mesh.Mesh.from_file(self.case_dir + 'stl/' + self.config['vv']['stl_lm'])
            vv_orientation = np.array(self.jfh.JFH[firing]['dcm'])
            # print(vv_orientation.transpose())
            VVmesh.rotate_using_matrix(vv_orientation.transpose())
            VVmesh.translate(self.jfh.JFH[firing]['xyz'])

            active_cones = None

            # Load and graph STLs of active clusters.
            if self.vv.use_clusters == True:
                active_clusters = self.graph_clusters(firing, vv_orientation)

            # Load and graph STLs of active thrusters. 
            for thruster in thrusters:


                # Save thruster id using indexed thruster value.
                # Could naming/code be more clear?
                # print('thruster num', thruster, 'thruster id', link[str(thruster)][0])
                thruster_id = link[str(thruster)][0]

                # Load plume STL in initial configuration. 
                plumeMesh = mesh.Mesh.from_file(self.case_dir + 'stl/' + self.config['vv']['stl_thruster'])

                # Transform plume
                
                # First, according to DCM of current thruster id in TCF
                thruster_orientation = np.array(
                    self.vv.thruster_data[thruster_id]['dcm']
                )
                plumeMesh.rotate_using_matrix(thruster_orientation.transpose())

                # Second, according to DCM of VV in JFH
                plumeMesh.rotate_using_matrix(vv_orientation.transpose())

                # Third, according to position vector of the VV in JFH
                plumeMesh.translate(self.jfh.JFH[firing]['xyz'])
                
                # Fourth, according to position of current cluster in CCF
                if self.vv.use_clusters == True:
                    # thruster_id[0] = "P" and thruster_id[1] = "#", adding these gives the cluster identifier
                    plumeMesh.translate(self.vv.cluster_data[thruster_id[0] + thruster_id[1]]['exit'][0])

                # Fifth, according to exit vector of current thruster id in TCD
                plumeMesh.translate(self.vv.thruster_data[thruster_id]['exit'][0])

                # Takeaway: Do rotations before translating away from the rotation axes!   


                if active_cones == None:
                    active_cones = plumeMesh
                else:
                    active_cones = mesh.Mesh(
                        np.concatenate([active_cones.data, plumeMesh.data])
                    )

                # print('DCM: ', self.vv.thruster_data[thruster_id]['dcm'])
                # print('DCM: ', thruster_orientation[0], thruster_orientation[1], thruster_orientation[2])

            if self.vv.use_clusters != True:
                if not active_cones == None:
                    VVmesh = mesh.Mesh(
                        np.concatenate([VVmesh.data, active_cones.data])
                    )
            if self.vv.use_clusters == True:
                if not active_cones == None:
                    VVmesh = mesh.Mesh(
                        np.concatenate([VVmesh.data, active_cones.data, active_clusters.data])
                    )
            
            # print(self.vv.mesh)

            # print(self.case_dir + self.config['stl']['vv'])

            if trade_study == False:
                path_to_stl = self.case_dir + "results/jfh/firing-" + str(firing) + ".stl" 
            elif trade_study == True:
                path_to_stl = self.case_dir + "results/" + self.get_case_key() + "/jfh/firing-" + str(firing) + ".stl" 

            # path_to_stl = self.case_dir + "results/jfh/firing-" + str(firing) + ".stl"
            # self.vv.convert_stl_to_vtk(path_to_vtk, mesh =VVmesh)
            VVmesh.save(path_to_stl)

    def visualize_sweep(self, config_iter): 
        """
            Creates visualization data for the trajectory of the proposed RPOD analysis.

            This method is valid for SINGLE JFH firings, due to file naming conventions.
            Numbering of files is based on the iteration number of the current configuration provided.

            Method used for mdao_unit_test_02.py

            This utility allows engineers to visualize a configuration sweep before running
            the full simulation and wasting computational resources.
            Parameters
            ----------
            None

            Returns
            -------
            Method doesn't currently return anything. Simply produces data as needed.
            Does the method need to return a status message? or pass similar data?
        """
        # Link JFH numbering of thrusters to thruster names.  
        link = {}
        i = 1
        for thruster in self.vv.thruster_data:
            link[str(i)] = self.vv.thruster_data[thruster]['name']
            i = i + 1
        # print('link is', link)

        # Create results directory if it doesn't already exist.
        results_dir = self.case_dir + 'results'
        if not os.path.isdir(results_dir):
            # print("results dir doesn't exist")
            os.mkdir(results_dir)

        results_dir = results_dir + "/jfh"
        if not os.path.isdir(results_dir):
            # print("results dir doesn't exist")
            os.mkdir(results_dir)

        # Save STL surface of target vehicle to local variable.
        target = self.target.mesh

        # Loop through each firing in the JFH.
        for firing in range(len(self.jfh.JFH)):
            # print('firing =', firing+1)

            # Save active thrusters for current firing. 
            thrusters = self.jfh.JFH[firing]['thrusters']
            # print("thrusters is", thrusters)
             
            # Load, transform, and, graph STLs of visiting vehicle.  
            VVmesh = mesh.Mesh.from_file(self.case_dir + 'stl/' + self.config['vv']['stl_lm'])
            vv_orientation = np.array(self.jfh.JFH[firing]['dcm'])
            # print(vv_orientation.transpose())
            VVmesh.rotate_using_matrix(vv_orientation.transpose())
            VVmesh.translate(self.jfh.JFH[firing]['xyz'])

            active_cones = None

            # Load and graph STLs of active clusters.
            if self.vv.use_clusters == True:
                active_clusters = self.graph_clusters(firing, vv_orientation)

            # Load and graph STLs of active thrusters. 
            for thruster in thrusters:

                thruster_id = link[str(thruster)][0]

                # Save thruster id using indexed thruster value.
                # Could naming/code be more clear?
                # print('thruster num', thruster, 'thruster id', link[str(thruster)][0])

                # Load plume STL in initial configuration. 
                plumeMesh = mesh.Mesh.from_file(self.case_dir + 'stl/' + self.config['vv']['stl_thruster'])

                # Transform plume
                
                # First, according to DCM of current thruster id in TCF
                thruster_orientation = np.array(
                    self.vv.thruster_data[thruster_id]['dcm']
                )
                plumeMesh.rotate_using_matrix(thruster_orientation.transpose())

                # Second, according to DCM of VV in JFH
                plumeMesh.rotate_using_matrix(vv_orientation.transpose())

                # Third, according to position vector of the VV in JFH
                plumeMesh.translate(self.jfh.JFH[firing]['xyz'])
                
                # Fourth, according to position of current cluster in CCF
                if self.vv.use_clusters == True:
                    # thruster_id[0] = "P" and thruster_id[1] = "#", adding these gives the cluster identifier
                    plumeMesh.translate(self.vv.cluster_data[thruster_id[0] + thruster_id[1]]['exit'][0])

                # Fifth, according to exit vector of current thruster id in TCD
                plumeMesh.translate(self.vv.thruster_data[thruster_id]['exit'][0])

                # Takeaway: Do rotations before translating away from the rotation axes!   


                if active_cones == None:
                    active_cones = plumeMesh
                else:
                    active_cones = mesh.Mesh(
                        np.concatenate([active_cones.data, plumeMesh.data])
                    )

                # print('DCM: ', self.vv.thruster_data[thruster_id]['dcm'])
                # print('DCM: ', thruster_orientation[0], thruster_orientation[1], thruster_orientation[2])

            if self.vv.use_clusters != True:
                if not active_cones == None:
                    VVmesh = mesh.Mesh(
                        np.concatenate([VVmesh.data, active_cones.data])
                    )
            if self.vv.use_clusters == True:
                if not active_cones == None:
                    VVmesh = mesh.Mesh(
                        np.concatenate([VVmesh.data, active_cones.data, active_clusters.data])
                    )
            
            # print(self.vv.mesh)

            # print(self.case_dir + self.config['stl']['vv'])

            if self.count > 0:
                path_to_vtk = self.case_dir + "results/strikes/firing-" + str(self.count) + "-" + str(firing)
                path_to_stl = self.case_dir + "results/jfh/firing-" + str(self.count) + "-" + str(firing) + ".stl"
            else:
                path_to_vtk = self.case_dir + "results/jfh/firing-" + str(firing)
                path_to_stl = self.case_dir + "results/jfh/firing-" + str(firing) + ".stl"
            # self.vv.convert_stl_to_vtk(path_to_vtk, mesh =VVmesh)
            VVmesh.save(path_to_stl)

    # def update_window_queue(self, window_queue, cur_window, firing_time, window_size):
    #     """
    #         Takes the most recent window of time size, and adds the new firing time to the sum, and the window_queue.
    #         If the new window is larger than the allowed window_size, then earleist firing times are removed
    #         from the queue and subtracted from the cur_window sum, until the sum fits within the window size.
    #         A counter for how many firing times are removed and subtracted is recorded.

    #         Parameters
    #         ----------
    #         window_queue : Queue
    #                 Queue holding the tracked firing times
    #         cur_window : float
    #                 sum of the tracked firing times (s)
    #         firing_time : float
    #                 length of current firing (s)
    #         window_size : float
    #                 max length of a window of time to track (s)

    #         Returns
    #         -------
    #         Queue
    #             Stores tracked firing times after update
    #         float
    #             sum of tracked firing times after update
    #         int
    #             number of firings removed from the queue 
    #     """
    #     window_queue.put(firing_time)
    #     cur_window += firing_time
    #     get_counter = 0
    #     while cur_window > window_size:
    #         old_firing_time = window_queue.get()
    #         cur_window -= old_firing_time
    #         get_counter +=1
    #     return window_queue, cur_window, get_counter
    
    # def update_parameter_queue(self, param_queue, param_window_sum, get_counter):
    #     """
    #         Takes the current parameter_queue, and removes the earliest tracked parameters from the front of the queue.
    #         This occurs "get_counter" times. Each time a parameter is popped from the queue, the sum is also updated,
    #         as to not track the removed parameter (ie. subtract the value)

    #         Parameters
    #         ----------
    #         param_queue : Queue
    #                 Queue holding the tracked parameters per firing
    #         param_window_sum : float
    #                 sum of the parameters in all the tracked firings
    #         get_counter : int
    #                 number of times to remove a tracked parameter from the front of the queue

    #         Returns
    #         -------
    #         Queue
    #             stores tracked paramters per firing after update
    #         float
    #             sum of tracked parameter after update
    #     """
    #     for i in range(get_counter):
    #         old_param = param_queue.get()
    #         param_window_sum -= old_param
    #     return param_queue, param_window_sum

    # Helper functions for jfh_plume_strikes
    def create_results_dir(self):
        """
            Creates a results directory and sub-directories if they don't already exist.
        """
        sub_dirs = ['results', 'results/strikes', 'results/jfh']
        for sub_dir in sub_dirs:
            os.makedirs(os.path.join(self.case_dir, sub_dir), exist_ok=True)

    def set_strike_fields(self, target):
        # Initiate array containing cummulative strikes. 
        cum_strikes = np.zeros(len(target.vectors))

        # Initiate array containing max pressures induced on each element.
        max_pressures = np.zeros(len(target.vectors))

        # Initiate array containing max shears induced on each element.
        max_shears = np.zeros(len(target.vectors))

        # Initiate array containing cummulative heatflux.
        cum_heat_flux_load = np.zeros(len(target.vectors))

        return cum_strikes, max_pressures, max_shears, cum_heat_flux_load

    def extract_firing_data(self, firing):
        # Save active thrusters for current firing.
        thrusters = self.jfh.JFH[firing]['thrusters']
        # print("thrusters", thrusters)

        # Load visiting vehicle position and orientation
        vv_pos = self.jfh.JFH[firing]['xyz']

        vv_orientation = np.array(self.jfh.JFH[firing]['dcm']).transpose()

        return thrusters, vv_pos, vv_orientation


    def jfh_plume_strikes(self):
        """
        Calculates number of plume strikes according to data provided for RPOD analysis.
        This refactored function delegates tasks to smaller, more manageable helper functions.

        Returns
        -------
        Dictionary containing strike data for each firing.
        """
        thruster_mapping = self.create_thruster_mapping()
        self.create_results_dir()

        target = self.target.mesh
        target_normals = target.get_unit_normals()

        strike_fields = self.initialize_strike_fields(target)

        firing_data = {}
        for firing in range(len(self.jfh.JFH)):
            thrusters, vv_pos, vv_orientation = self.extract_firing_data(firing)
            
            # Process the current firing
            firing_data[str(firing + 1)] = self.process_single_firing(
                firing, target, target_normals, thrusters, vv_pos, vv_orientation, thruster_mapping, strike_fields
            )

        return firing_data

    def create_thruster_mapping(self):
        """Creates a mapping of thruster IDs to their names."""
        mapping = {}
        for i, thruster in enumerate(self.vv.thruster_data, start=1):
            mapping[str(i)] = self.vv.thruster_data[thruster]['name']
        return mapping

    def initialize_strike_fields(self, target):
        """Initializes fields for tracking strikes, pressures, and heat flux."""
        if self.config['pm']['kinetics'] != 'None':
            return self.set_strike_fields(target)
        return self.set_strike_fields(target)[:1]  # Only return cumulative strikes when kinetics is disabled

    def process_single_firing(self, firing, target, target_normals, thrusters, vv_pos, vv_orientation, thruster_mapping, strike_fields):
        """
        Processes a single firing, calculating strikes and optionally applying kinetics.

        Parameters
        ----------
        firing : int
            Index of the current firing.
        target : Mesh object
            The target vehicle's STL mesh.
        target_normals : ndarray
            Normals of the target mesh faces.
        thrusters : list
            Active thrusters for the current firing.
        vv_pos : ndarray
            Visiting vehicle position.
        vv_orientation : ndarray
            Visiting vehicle orientation matrix.
        thruster_mapping : dict
            Mapping of thruster indices to names.
        strike_fields : tuple
            Pre-initialized strike fields (arrays for strikes, pressures, etc.).

        Returns
        -------
        dict
            Strike data for the current firing.
        """
        strikes = np.zeros(len(target.vectors))
        firing_time = float(self.jfh.JFH[firing]['t'])

        for thruster in thrusters:
            self.thruster_id = thruster_mapping[str(thruster)][0]
            plume_normal, thruster_pos = self.calculate_thruster_geometry(self.thruster_id, vv_pos, vv_orientation)

            self.calculate_strike_data(
                target, target_normals, plume_normal, thruster_pos, strikes, strike_fields, firing_time
            )

        return self.format_firing_data(strikes, strike_fields)

    def calculate_thruster_geometry(self, thruster_id, vv_pos, vv_orientation):
        """Calculates the plume geometry for a given thruster."""
        thruster_orientation = np.array(self.vv.thruster_data[self.thruster_id]['dcm']).transpose()
        thruster_orientation = thruster_orientation.dot(vv_orientation)

        plume_normal = np.array(thruster_orientation[0])
        thruster_pos = vv_pos + np.array(self.vv.thruster_data[self.thruster_id]['exit'])[0]

        return plume_normal, thruster_pos

    def calculate_strike_data(self, target, target_normals, plume_normal, thruster_pos, strikes, strike_fields, firing_time):
        """Calculates strikes, pressures, and heat flux for each face on the target surface."""
        for i, face in enumerate(target.vectors):
            centroid = self.calculate_face_centroid(face)
            unit_distance, norm_distance = self.calculate_distance_vector(thruster_pos, centroid)
            
            if self.check_strike_conditions(target_normals[i], plume_normal, unit_distance, norm_distance):
                strikes[i] += 1
                strike_fields[0][i] += 1  # Update cumulative strikes

                if self.config['pm']['kinetics'] != 'None':  # Apply kinetics if enabled
                    self.apply_kinetics(
                        i, strike_fields, norm_distance, 
                        self.calculate_theta(plume_normal, unit_distance), firing_time
                    )

    def calculate_face_centroid(self, face):
        """Calculates the centroid of a triangular face."""
        face = np.array(face).transpose()
        return np.array([np.mean(face[0]), np.mean(face[1]), np.mean(face[2])])

    def calculate_distance_vector(self, thruster_pos, centroid):
        """Calculates the distance vector and its magnitude between the thruster and the face centroid."""
        distance = thruster_pos - centroid
        norm_distance = np.linalg.norm(distance)
        return distance / norm_distance, norm_distance

    def check_strike_conditions(self, surface_normal, plume_normal, unit_distance, norm_distance):
        """Checks if a face is struck by the plume based on distance, angle, and orientation."""
        theta = self.calculate_theta(plume_normal, unit_distance)

        within_distance = norm_distance < float(self.config['plume']['radius'])
        within_theta = theta < float(self.config['plume']['wedge_theta'])
        facing_thruster = np.dot(surface_normal, -plume_normal / np.linalg.norm(plume_normal)) < 0

        return within_distance and within_theta and facing_thruster

    def calculate_theta(self, plume_normal, unit_distance):
        """Calculates the angle between the plume centerline and the distance vector."""
        return np.pi - np.arccos(np.dot(unit_distance, plume_normal / np.linalg.norm(plume_normal)))

    def apply_kinetics(self, i, strike_fields, norm_distance, theta, firing_time):
        """Applies simplified gas kinetics to calculate pressures, shears, and heat flux."""
        simple_plume = SimplifiedGasKinetics(
            norm_distance, theta, 
            self.vv.thruster_metrics[self.vv.thruster_data[self.thruster_id]['type'][0]],
            float(self.config['tv']['surface_temp']), float(self.config['tv']['sigma'])
        )

        pressures, max_pressures, shear_stresses, max_shears, heat_flux, heat_flux_load, cum_heat_flux_load = strike_fields[1:]

        pressures[i] += simple_plume.get_pressure()
        max_pressures[i] = max(max_pressures[i], pressures[i])

        shear_stress = simple_plume.get_shear_pressure()
        shear_stresses[i] += abs(shear_stress)
        max_shears[i] = max(max_shears[i], shear_stresses[i])

        heat_flux_cur = simple_plume.get_heat_flux()
        heat_flux[i] += heat_flux_cur
        heat_flux_load[i] += heat_flux_cur * firing_time
        cum_heat_flux_load[i] += heat_flux_cur * firing_time

    def format_firing_data(self, strikes, strike_fields):
        """Formats the calculated data for output."""
        cell_data = {"strikes": strikes, "cum_strikes": strike_fields[0].copy()}

        if len(strike_fields) > 1:  # Add kinetic data if enabled
            cell_data.update({
                "pressures": strike_fields[1],
                "max_pressures": strike_fields[2],
                "shear_stress": strike_fields[3],
                "max_shears": strike_fields[4],
                "heat_flux_rate": strike_fields[5],
                "heat_flux_load": strike_fields[6],
                "cum_heat_flux_load": strike_fields[7]
            })

        return cell_data


    def calc_time_multiplier(self, v_ida, v_o, r_o):
        # Determine thruster configuration characterstics.
        # The JFH only contains firings done by the neg_x group
        m_dot_sum = self.calc_m_dot_sum('neg_x')
        # print('m_dot_sum is', m_dot_sum)
        MIB = self.vv.thruster_metrics[self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]]['MIB']
        # print('MIB is', MIB)
        F_thruster = self.vv.thruster_metrics[self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]]['F']
        F = F_thruster * np.cos(self.vv.decel_cant)
        n_thrusters = len(self.vv.rcs_groups['neg_x'])
        F = F * n_thrusters

        # Defining a multiplier reduce time steps and make running faster
        dt = (MIB / F_thruster)
        # print('dt is', dt)
        dm_firing = m_dot_sum * dt
        # print('dm_firing is', dm_firing)
        docking_mass = self.vv.mass
        # print('docking mass is', docking_mass)

        # Calculate required change in velocity.
        dv_req = v_o - v_ida
        v_e = self.calc_v_e('neg_x')

        # Calculate propellant used for docking and changes in mass.
        forward_propagation = False
        delta_mass_jfh = self.calc_delta_mass_v_e(dv_req, v_e, forward_propagation)
        # print('delta_mass_docking is',delta_mass_jfh)

        pre_approach_mass = self.vv.mass
        # print('pre-approach mass is', pre_approach_mass)

        # Instantiate data structure to hold JFH data + physics data.
        # Initializing position
        x = [r_o]
        y = [0]
        z = [0]

        # Initializing empty tracking lists
        dx = [0]
        t = [0]
        dv = [0]

        # Initializing inertial state
        dxdt = [v_o]

        # Initializing initial mass
        mass = [pre_approach_mass]

        # Initializing list to later sum propellant expenditure
        dm_total = [dm_firing]

        # Firing number
        n = [1]

        # Create dummy rotation matrices.
        x1 = [1, 0, 0]
        y1 = [1, 0, 0]

        rot = [np.array(rotation_matrix_from_vectors(x1, y1))]

        # Calculate JFH and 1D physics data for required firings.
        while (dv_req > 0):
            # print('dv_req', round(dv_req, 4), 'n firings', n[i])

            # Grab last value in the JFH arrays (initial conditions for current time step)
            # print('x, dx, dt, t, dxdt, mass, dm_total')
            # print(x[-1], dx[-1], dt_vals[-1], t[-1], dxdt[-1], mass[-1], dm_total[-1])

            # Update VV mass per firing
            mass_o = mass[-1]
            mass.append(mass_o - dm_firing)
            mass_f = mass[-1]
            # print('mass_f is', mass_f)

            # Calculate velocity change per firing.
            dv_firing = self.calc_delta_v(dt, v_e, m_dot_sum, mass_o)
            dv.append(dv_firing)
            # print('dv is', dv_firing)
            # print(round(dxdt[-1] - dv_firing, 2))
            # input()
            dxdt.append(dxdt[-1] - dv_firing)

            # Calculate distance traveled per firing
            # print(dxdt[-1], dxdt[-2]) # last and second to last element.
            v_avg = 0.5 * (dxdt[-1] + dxdt[-2])
            dx.append(v_avg * dt)
            x.append(x[-1] - v_avg*dt)
            y.append(0)
            z.append(0)

            # Calculate left over v_req (TERMINATES LOOP)
            dv_req -= dv_firing

            # Calculate mass expended up to this point.
            dm_total.append(dm_total[-1] + dm_firing)

            # Calculate current firing.
            n.append(n[-1]+1)

            # Add time data.
            t.append(t[-1] + dt)

            rot.append(np.array(rotation_matrix_from_vectors(x1, y1)))

        #     # constraint_file.close()

        return firing_data

        n_firings = len(t)
        time_multiplier = 10 / n_firings
        # print('n firings:', len(t), 'time multiplier:', time_multiplier)
        return (1/time_multiplier)

        # one_d_results = {
        #     'n_firings': n,
        #     'x': x,
        #     'dx': dx,
        #     't': t,
        #     'dv': dv,
        #     'v': dxdt,
        #     'mass': mass,
        #     'delta_mass': dm_total
        # }

        # print(one_d_results)

    def print_jfh_1d_approach_n_fire(self, v_ida, v_o, r_o, n_firings, trade_study = False):
       # Determine thruster configuration characterstics.
        # The JFH only contains firings done by the neg_x group
        m_dot_sum = self.calc_m_dot_sum('neg_x')
        # print('m_dot_sum is', m_dot_sum)
        MIB = self.vv.thruster_metrics[self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]]['MIB']
        # print('MIB is', MIB)
        F_thruster = self.vv.thruster_metrics[self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]]['F']
        F = F_thruster * np.cos(self.vv.decel_cant)
        n_thrusters = len(self.vv.rcs_groups['neg_x'])
        F = F * n_thrusters
        # print('F is', F)

        # Defining a multiplier reduce time steps and make running faster
        time_multiplier = self.calc_time_multiplier(v_ida, v_o, r_o)
        dt = (MIB / F_thruster) * time_multiplier
        # print('dt is', dt)
        dm_firing = m_dot_sum * dt
        # print('dm_firing is', dm_firing)
        docking_mass = self.vv.mass
        # print('docking mass is', docking_mass)

        # Calculate required change in velocity.
        dv_req = v_o - v_ida
        v_e = self.calc_v_e('neg_x')


        # Calculate propellant used for docking and changes in mass.
        forward_propagation = False
        delta_mass_jfh = self.calc_delta_mass_v_e(dv_req, v_e, forward_propagation)
        self.fuel_mass= delta_mass_jfh
        self.vv.mass = docking_mass
        # print('delta_mass_docking is',delta_mass_jfh)

        pre_approach_mass = self.vv.mass
        # print('pre-approach mass is', pre_approach_mass)

        # Instantiate data structure to hold JFH data + physics data.
        # Initializing position
        x = [r_o]
        y = [0]
        z = [0]

        # Initializing empty tracking lists
        dx = [0]
        t = [0]
        dv = [0]

        # Initializing inertial state
        dxdt = [v_o]

        # Initializing initial mass
        mass = [pre_approach_mass]

        # Initializing list to later sum propellant expenditure
        dm_total = [dm_firing]

        # Firing number
        n = [1]

        # Create dummy rotation matrices.
        x1 = [1, 0, 0]
        y1 = [1, 0, 0]

        rot = [np.array(rotation_matrix_from_vectors(x1, y1))]

        # Calculate JFH and 1D physics data for required firings.
        while (dv_req > 0):
            # print('dv_req', round(dv_req, 4), 'n firings', n[i])

            # Grab last value in the JFH arrays (initial conditions for current time step)
            # print('x, dx, dt, t, dxdt, mass, dm_total')
            # print(x[-1], dx[-1], dt_vals[-1], t[-1], dxdt[-1], mass[-1], dm_total[-1])

            # Update VV mass per firing
            mass_o = mass[-1]
            mass.append(mass_o - dm_firing)
            mass_f = mass[-1]
            # print('mass_f is', mass_f)

            # Calculate velocity change per firing.
            dv_firing = self.calc_delta_v(dt, v_e, m_dot_sum, mass_o)
            dv.append(dv_firing)
            # print('dv is', dv_firing)
            # print(round(dxdt[-1] - dv_firing, 2))
            # input()
            dxdt.append(dxdt[-1] - dv_firing)

            # Calculate distance traveled per firing
            # print(dxdt[-1], dxdt[-2]) # last and second to last element.
            v_avg = 0.5 * (dxdt[-1] + dxdt[-2])
            dx.append(v_avg * dt)

            curr_x  = x[-1] - v_avg*dt

            # if curr_x < 0:
            #     break

            x.append(curr_x)
            y.append(0)
            z.append(0)

            # Calculate left over v_req (TERMINATES LOOP)
            dv_req -= dv_firing

            # Calculate mass expended up to this point.
            dm_total.append(dm_total[-1] + dm_firing)

            # Calculate current firing.
            n.append(n[-1]+1)

            # Add time data.
            t.append(t[-1] + dt)

            rot.append(np.array(rotation_matrix_from_vectors(x1, y1)))

        # one_d_results = {
        #     'n_firings': n,
        #     'x': x,
        #     'dx': dx,
        #     't': t,
        #     'dv': dv,
        #     'v': dxdt,
        #     'mass': mass,
        #     'delta_mass': dm_total
        # }

        # print(one_d_results)
        # input()
        r = [x, y, z]

        if trade_study == False:
            jfh_path = self.case_dir + 'jfh/' + self.config['jfh']['jfh']
        elif trade_study == True:
            jfh_path = self.case_dir +'jfh/' + self.get_case_key() + '.A'

        # print(jfh_path)
        print_1d_JFH(t, r, rot, jfh_path) 


    def print_jfh_1d_approach(self, v_ida, v_o, r_o, trade_study = False):
        """
            Method creates JFH data for axial approach using simpified physics calculations.

            This approach models one continuous firing.

            Kinematics and mass changes are discretized according to the thruster's minimum firing time.

            Parameters
            ----------
            v_ida : float
                VisitingVehicle docking velocity (determined by international docking adapter)

            v_o : float
                VisitingVehicle incoming axial velocity.

            r_o : float
                Initial distance to docking port.

            Returns
            -------
            Method doesn't currently return anything. Simply prints data to a files as needed.
            Does the method need to return a status message? or pass similar data?

        """
        # Determine thruster configuration characterstics.
        # The JFH only contains firings done by the neg_x group
        m_dot_sum = self.calc_m_dot_sum('neg_x')
        # print('m_dot_sum is', m_dot_sum)
        MIB = self.vv.thruster_metrics[self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]]['MIB']
        # print('MIB is', MIB)
        F = self.vv.thruster_metrics[self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]]['F']
        F = F * np.cos(self.vv.decel_cant)
        n_thrusters = len(self.vv.rcs_groups['neg_x'])
        F = F * n_thrusters
        # print('F is', F)

        # Defining a multiplier reduce time steps and make running faster
        time_multiplier = 60
        dt = (MIB / F) * time_multiplier
        # print('dt is', dt)
        dm_firing = m_dot_sum * dt
        # print('dm_firing is', dm_firing)
        docking_mass = self.vv.mass
        # print('docking mass is', docking_mass)

        # Calculate required change in velocity.
        dv_req = v_o - v_ida
        v_e = self.calc_v_e('neg_x')
        forward_propagation = False


        # Calculate propellant used for docking and changes in mass.
        delta_mass_jfh = self.calc_delta_mass_v_e(dv_req, v_e, forward_propagation)
        # print('delta_mass_docking is',delta_mass_jfh)

        pre_approach_mass = self.vv.mass
        # print('pre-approach mass is', pre_approach_mass)

        # Instantiate data structure to hold JFH data + physics data.
        # Initializing position
        x = [r_o]
        y = [0]
        z = [0]

        # Initializing empty tracking lists
        dx = [0]
        t = [0]
        dv = [0]

        # Initializing inertial state
        dxdt = [v_o]

        # Initializing initial mass
        mass = [pre_approach_mass]

        # Initializing list to later sum propellant expenditure
        dm_total = [dm_firing]

        # Firing number
        n = [1]

        # Create dummy rotation matrices.
        x1 = [1, 0, 0]
        y1 = [1, 0, 0]

        rot = [np.array(rotation_matrix_from_vectors(x1, y1))]

        # Calculate JFH and 1D physics data for required firings.
        while (dv_req > 0):
            # print('dv_req', round(dv_req, 4), 'n firings', n[i])

            # Grab last value in the JFH arrays (initial conditions for current time step)
            # print('x, dx, dt, t, dxdt, mass, dm_total')
            # print(x[-1], dx[-1], dt_vals[-1], t[-1], dxdt[-1], mass[-1], dm_total[-1])

            # Update VV mass per firing
            mass_o = mass[-1]
            mass.append(mass_o - dm_firing)
            mass_f = mass[-1]
            # print('mass_f is', mass_f)

            # Calculate velocity change per firing.
            dv_firing = self.calc_delta_v(dt, v_e, m_dot_sum, mass_o)
            dv.append(dv_firing)
            # print('dv is', dv_firing)
            # print(round(dxdt[-1] - dv_firing, 2))
            # input()
            dxdt.append(dxdt[-1] - dv_firing)

            # Calculate distance traveled per firing
            # print(dxdt[-1], dxdt[-2]) # last and second to last element.
            v_avg = 0.5 * (dxdt[-1] + dxdt[-2])
            dx.append(v_avg * dt)
            x.append(x[-1] - v_avg*dt)
            y.append(0)
            z.append(0)

            # Calculate left over v_req (TERMINATES LOOP)
            dv_req -= dv_firing

            # Calculate mass expended up to this point.
            dm_total.append(dm_total[-1] + dm_firing)

            # Calculate current firing.
            n.append(n[-1]+1)

            # Add time data.
            t.append(t[-1] + dt)

            rot.append(np.array(rotation_matrix_from_vectors(x1, y1)))

        # one_d_results = {
        #     'n_firings': n,
        #     'x': x,
        #     'dx': dx,
        #     't': t,
        #     'dv': dv,
        #     'v': dxdt,
        #     'mass': mass,
        #     'delta_mass': dm_total
        # }

        # print(one_d_results)
        # input()
        r = [x, y, z]

        if trade_study == False:
            jfh_path = self.case_dir + 'jfh/' + self.config['jfh']['jfh']
        elif trade_study == True:
            jfh_path = self.case_dir +'jfh/' + self.get_case_key() + '.A'

        # print(jfh_path)
        print_1d_JFH(t, r, rot, jfh_path)

        # self.

        return

    def edit_1d_JFH(self, t_values, r,  rot):
        """
            Helper function to RPOD.calc_jfh_1d_approach() that is responsible for
            modifying the JFH attribute in memory with the values calculated.

            Parameters
            ----------
            t_values : np.array
                Array containing time step data for each firing in the JFH.

            r : np.array
                Array containing positional data for each firing in the JFH.

            rot : np.array
                Array containing rotational data for each firing in the JFH.

            Returns
            -------
            Method doesn't currently return anything. Simply prints data to a files as needed.
            Does the method need to return a status message? or pass similar data?

        """
        # Printing out the empty JFH and checking its size
        # print('self.jfh.JFH is', self.jfh.JFH)
        # print('len(self.jfh.JFH) is', len(self.jfh.JFH))

        # Scrap
        # print('self.jfh.JFH[0] is', self.jfh.JFH[0])
        # print('self.jfh.JFH[1] is', self.jfh.JFH[1])
        # self.jfh.JFH.append({'nt': '1', 'dt': '0.000', 't': '1', 'dcm': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], 'xyz': [-10.0, 0.0, 0.0], 'uf': 1.0, 'thrusters': [1, 2, 5, 6, 9, 10, 13, 14]})
        # print('self.jfh.JFH[0] is', self.jfh.JFH[0])

        # Checking the time step
        # print('t_values is', t_values)
        # print('len(t_values) is', len(t_values))

        # Changing the number of firings to be evaluated
        self.jfh.nt = len(t_values)
        # print('self.jfh.nt is', self.jfh.nt)
        # print('self.jfh.nt after is', self.jfh.nt)
        # print("self.jfh.JFH[0]['nt'] is", self.jfh.JFH[0]['nt'])

        # Confirming the time step is correct
        # print('str(t_values[1]) is', str(t_values[1]))

        # Verifying the format of the rot array
        # print('rot[0] is', rot[0])
        # print('[list(rot[0][0]), list(rot[0][1]), list(rot[0][2])] is', [list(rot[0][0]), list(rot[0][1]), list(rot[0][2])])
        # print('[list(rot[1][0]), list(rot[1][1]), list(rot[1][2])] is', [list(rot[1][0]), list(rot[1][1]), list(rot[1][2])])

        # Verifying the format of the r array
        # print('r is', r)
        # print('[r[0][0], r[1][0], r[2][0]] is', [r[0][0], r[1][0], r[2][0]])
        # print('[r[0][1], r[1][1], r[2][1]] is', [r[0][1], r[1][1], r[2][1]])

        for i in range(len(t_values)):
            # NOTE: 'thrusters' is currently hardcoded
            # NOTE: all the 'xyz' values are still negative
            # Start from the required distance to slow down and approach zero
            # print('-r[0][-1] + r[0][i] is', -r[0][-1] + r[0][i])

            
            # NOTE: The thrusters are currently hardcoded, but should be changed to the thrusters indicated in the neg_x group in the tgf
            self.jfh.JFH.append({'nt': str(i + 1), 'dt': str(t_values[i]), 't': str(t_values[1]), 'dcm': [list(rot[i][0]), list(rot[i][1]), list(rot[i][2])], 'xyz': [-r[0][-1] + r[0][i] - 0.5, -r[1][i], -r[2][i]], 'uf': 1.0, 'thrusters': [1, 2, 5, 6, 9, 10, 13, 14]})

        # Printing out the populated JFH and checking its size
        # print('self.jfh.JFH is', self.jfh.JFH)
        # print('len(self.jfh.JFH) is', len(self.jfh.JFH))

    def calc_jfh_1d_approach(self, v_ida, v_o, cant):
        """
            The x-position represents the distance required to reach a velocity of zero.    
        
            Method calculates JFH data for 1D approach using simpified physics calculations.

            This approach models one continuous firing.

            Kinematics and mass changes are discretized according to the thruster's minimum firing time.

            Parameters
            ----------
            v_ida : float
                VisitingVehicle docking velocity (determined by international docking adapter)

            v_o : float
                VisitingVehicle incoming axial velocity.

            cant : float
                Angling of all deceleration thrusters in degrees that 

            Returns
            -------
            Method doesn't currently return anything. Simply prints data to a files as needed.
            Does the method need to return a status message? or pass similar data?

        """
        # Determine thruster configuration characterstics.
        # The JFH only contains firings done by the neg_x group
        m_dot_sum = self.calc_m_dot_sum('neg_x')
        # print('m_dot_sum is', m_dot_sum)
        MIB = self.vv.thruster_metrics[self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]]['MIB']
        # print('MIB is', MIB)
        MissionPlanner.cant = np.radians(cant)
        F = np.cos(MissionPlanner.cant) * self.vv.thruster_metrics[self.vv.thruster_data[self.vv.rcs_groups['neg_x'][0]]['type'][0]]['F']
        # print('F is', F)



        # IMPORTANT IMPORTANT IMPORTANT IMPORTANT IMPORTANT
        # Defining a multiplier reduce time steps and make running faster
        time_multiplier = 100
        # Multiplying by cant to make time step independent of it, this cancels it out in the F term
        self.dt = (MIB / F) * time_multiplier * np.cos(MissionPlanner.cant)
        # print('dt is', dt)
        dm_firing = m_dot_sum * self.dt
        # print('dm_firing is', dm_firing)
        docking_mass = self.vv.mass
        # print('docking mass is', docking_mass)

        # Calculate required change in velocity.
        dv_req = v_o - v_ida
        v_e = self.calc_v_e('neg_x')
        forward_propagation = False

        # Calculate propellant used for docking and changes in mass.
        # Since this is already done by calc_total_delta_mass, the LM's mass
            # needs to be reset to docking_mass otherwise the propellant
            # expenditure for the JFH is counted twice
        delta_mass_jfh = self.calc_delta_mass_v_e(dv_req, v_e, forward_propagation)
        # print('delta_mass_docking is',delta_mass_jfh)
        self.vv.mass = docking_mass
        # print('self.vv.mass is', self.vv.mass)

        pre_approach_mass = self.vv.mass + delta_mass_jfh
        # print('pre-approach mass is', pre_approach_mass)

        # Instantiate data structure to hold JFH data + physics data.
        # Initializing position
        x = [0]
        y = [0]
        z = [0]

        # Initializing empty tracking lists
        dx = [0]
        t = [0]
        dv = [0]

        # Initializing inertial state
        dxdt = [v_o]

        # Initializing initial mass
        mass = [pre_approach_mass]

        # Initializing list to later sum propellant expenditure
        dm_total = [dm_firing]

        # Firing number
        n = [1]

        # Create dummy rotation matrices.
        x1 = [1, 0, 0]
        y1 = [1, 0, 0]

        rot = [np.array(rotation_matrix_from_vectors(x1, y1))]

        # Calculate JFH and 1D physics data for required firings.
        while (dv_req > 0):
            # print('dv_req', round(dv_req, 4), 'n firings', n[i])

            # Grab last value in the JFH arrays (initial conditions for current time step)
            # print('x, dx, dt, t, dxdt, mass, dm_total')
            # print(x[-1], dx[-1], dt_vals[-1], t[-1], dxdt[-1], mass[-1], dm_total[-1])

            # Update VV mass per firing
            mass_o = mass[-1]
            mass.append(mass_o - dm_firing)
            mass_f = mass[-1]
            # print('mass_f is', mass_f)

            # Calculate velocity change per firing.
            dv_firing = self.calc_delta_v(self.dt, v_e, m_dot_sum, mass_o)
            dv.append(dv_firing)
            # print('dv is', dv_firing)
            # print(round(dxdt[-1] - dv_firing, 2))
            # input()
            dxdt.append(dxdt[-1] - dv_firing)

            # Calculate distance traveled per firing
            # print(dxdt[-1], dxdt[-2]) # last and second to last element.
            v_avg = 0.5 * (dxdt[-1] + dxdt[-2])
            dx.append(v_avg * self.dt)
            x.append(x[-1] + v_avg * self.dt)
            y.append(0)
            z.append(0)

            # Calculate left over v_req (TERMINATES LOOP)
            dv_req -= dv_firing

            # Calculate mass expended up to this point.
            dm_total.append(dm_total[-1] + dm_firing)

            # Calculate current firing.
            n.append(n[-1] + 1)

            # Add time data.
            t.append(t[-1] + self.dt)

            rot.append(np.array(rotation_matrix_from_vectors(x1, y1)))

        # print('x is', x)

        # print('dm_total is', dm_total[-1])

        # one_d_results = {
        #     'n_firings': n,
        #     'x': x,
        #     'dx': dx,
        #     't': t,
        #     'dv': dv,
        #     'v': dxdt,
        #     'mass': mass,
        #     'delta_mass': dm_total
        # }

        # print(one_d_results)

        r = [x, y, z]

        jfh_path = self.case_dir + 'jfh/' + self.config['jfh']['jfh']
        # print(jfh_path)
        self.edit_1d_JFH(t, r, rot)

    def get_case_key(self):
        return self.case_key

    def set_case_key(self, v0_iter, cant_iter):

        self.case_key = 'vo_' + str(v0_iter) + '_cant_' + str(cant_iter)

        return

    def make_test_jfh():

        return