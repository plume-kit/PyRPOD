import pandas as pd

from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt
import numpy as np
import math
import os

from pyrpod.vehicle.Vehicle import Vehicle
from pyrpod.mdao import SweepConfig
from pyrpod.logging_utils import get_logger

logger = get_logger("pyrpod.vehicle.VisitingVehicle")

# Adapted from
# https://stackoverflow.com/questions/54616049/converting-a-rotation-matrix-to-euler-angles-and-back-special-case
def rot2eul(R):
    beta = -np.arcsin(R[2][0])
    alpha = np.arctan2(R[2][1]/np.cos(beta),R[2][2]/np.cos(beta))
    gamma = np.arctan2(R[1][0]/np.cos(beta),R[0][0]/np.cos(beta))
    return np.array((alpha, beta, gamma))

# Helper functions for constructer. 
def process_coordinates(str_coord):
    # Split str at spaces
    str_list = str_coord.split(' ')
    # Return as list of floats
    return [float(x) for x in str_list]

# Process definition of an individual thruster.
def process_thruster_def(str_thruster):
    columns = ['name', 'type', 'exit', 'dcm']
    # thruster = pd.DataFrame(columns = columns)
    # print(thruster.dtypes)
    thruster = {}
    # Remove new line char (last char) and split at any space char.    
    str_list = str_thruster[:-1].split(' ')
    # str_list = str_thruster.split(' ')
    # print(str_list)
    # Save name and type of thruster
    thruster["name"] = [str_list.pop(0)]
    thruster['type'] = [str_list.pop(0)]
    # print(thruster['name'])
    # Save coordinate for center of exit plane.
    coord = []
    for i in range(3):
        coord.append(float(str_list.pop(0)))
    thruster['exit'] = [coord]

    # Save direction cosine matrix of thruster relative to the vehicle    
    drm = []
    for i in range(3):
        row = []
        for j in range(3):
            row.append(float(str_list.pop(0)))
        drm.append(row)
    thruster['dcm'] = drm
    # thruster = pd.DataFrame(thruster)
    # print(thruster)
    # return pd.DataFrame(thruster)
    return thruster

# Wrapper function
def process_str_thrusters(str_thrusters):
    # dcm = direction cosine matrix
    columns = ['name', 'type', 'exit', 'dcm']
    thrusters_data = {}
    for thruster in str_thrusters:
        name = str(thruster.split(' ')[0])
        thrusters_data[name] = process_thruster_def(thruster)
        # print(process_thruster_def(thruster))
        # thrusters_data = pd.concat([thrusters_data,process_thruster_def(thruster)], ignore_index = True)
        # print(thrusters_data.dtypes)

    return thrusters_data

# Process definition of an individual cluster.
def process_cluster_def(str_cluster):
    columns = ['name', 'exit', 'dcm']
    cluster = {}
    # Remove new line char (last char) and split at any space char.    
    str_list = str_cluster[:-1].split(' ')
    # Save name of cluster
    cluster["name"] = [str_list.pop(0)]
    # Save coordinate for center of cluster.
    coord = []
    for i in range(3):
        coord.append(float(str_list.pop(0)))
    cluster['exit'] = [coord]

    # Save direction cosine matrix of cluster relative to the vehicle    
    drm = []
    for i in range(3):
        row = []
        for j in range(3):
            row.append(float(str_list.pop(0)))
        drm.append(row)
    cluster['dcm'] = drm
    return cluster

# Wrapper function
def process_str_clusters(str_clusters):
    # dcm = direction cosine matrix
    columns = ['name', 'exit', 'dcm']
    clusters_data = {}
    for cluster in str_clusters:
        name = str(cluster.split(' ')[0])
        clusters_data[name] = process_cluster_def(cluster)

    return clusters_data

class VisitingVehicle(Vehicle):
    """
        Class responsible for handling visiting vehicle data.

        Includes surface mesh and thruster configuration data.

        Attributes
        ----------
        num_thrusters : int
            Total number of thrusters in RCS configuration.

        thruster_units : str
            Units for thruster coordinates.

        cog : float
            Center of Gravity for the Visiting Vehicle.

        grapple : float
            Grappling coordinate for the Visiting Vehicle.

        thruster_data : dictionary
            Dictionary holding the main thruster configuration data.

        cluster_data : dictionary
            Dictionary holding the main cluster configuration data.

        jet_interactions : float
            Can be ignored for now.

        Methods
        -------
        set_stl()
            Reads in Vehicle surface mesh from STL file.
        
        set_thruster_config()
            Reads the thruster configuration file from the config.ini for the Visiting Vehicle and saves it as class members.

        change_cluster_config()
            Alters cluster configuration data using OpenMDAO inputs.


        set_cluster_config()
            Read in cluster configuration data from the provided file path.

        set_thruster_metrics()
            Reads the thruster data file to gather thruster-specific performance parameters for the configuration from a .csv file
            and saves it in a list of dictionaries. These dictionaries are then saved into each thruster in the configuration.    

        print_info()
            Simple method to format printing of vehicle info.

        initiate_plume_mesh()
            Helper method that reads in surface mesh for plume clone.

        transform_plume_mesh(thruster_id, plumeMesh)
            Transform plume mesh according to the specified thruster's DCM and exit coordinate.

        initiate_plume_normal(thruster_id)
            Collects plume normal vectors data for visualization.

        plot_vv_and_thruster(plumeMesh, thruster_id, normal, i)
            Plots Visiting Vehicle and plume cone for provided thruster id.

        check_thruster_configuration()
            Plots visiting vehicle and all thrusters in RCS configuration.
    """

    def print_info(self):
        """
            Simple method to format printing of vehicle info.

            Parameters
            ----------
            None

            Returns
            -------
            None
        """
        logger.info('number of thrusters: %s', self.num_thrusters)
        logger.info('thruster units: %s', self.thruster_units)
        logger.info('center of gravity: %s', self.cog)
        logger.info('grapple coordinate: %s', self.grapple)
        logger.info('number of dual jet interactions: %s', self.jet_interactions)
        return
    
    def set_stl(self):
        """
            Reads in Vehicle surface mesh from STL file.

            Parameters
            ----------
            None

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        """
        path_to_stl = self.case_dir + 'stl/' + self.config['vv']['stl_lm']
        self.mesh = mesh.Mesh.from_file(path_to_stl)
        self.path_to_stl = path_to_stl
        return

    def get_thruster_cant(self, thruster_name):
        """
            Finds the cant angle defined as angle from the LM surface tangent.
            Takes the thruster's DCM, undoes the frame transformation
            ie. the frame made by the surface tangent and the line from the LM's 
            axial surface to the exit coordinate, is rotated about x to match the universal YZ axes.
            Then the DCM is decomposed to grab the cant angling.

            Parameters
            ----------
            thruster_name : string
                name of the thruster of interest
            
            Returns
            -------
            float
                cant angle in rad
        """
        # find frame rotation (Tx)
        # taken directly from SweepConfig.SweepDecelAngles.calculate_frame_rot()
        exit_coords = self.thruster_data[thruster_name]['exit'][0]
        y = exit_coords[1]
        z = exit_coords[2]

        if y == 0 and z > 0:
            theta = np.pi/2
        elif y == 0 and z < 0:
            theta = -np.pi/2
        else:
            theta = np.arctan2(z, y)

        Tx = np.array([
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)]
        ])

        # Find the inverse of Tx
        inv_Tx = np.linalg.inv(Tx)

        # undo the frame rotation
        DCM = self.thruster_data[thruster_name]['dcm']
        Rz = np.dot(inv_Tx, DCM)

        # resulting matrix representes the rotation of DCM about z-axis
        # Rz -> cant angle
        cant = np.arccos(Rz[0][0])

        return cant

    def set_thruster_config(self, thruster_data=None):
        """
            Reads the thruster configuration file from the config.ini for the Visiting Vehicle and saves it as class members.

            If thruster data IS passed, simple overwrite self.thruster_data.
            This use is intended to occur only after a notional use of this method.
            (ie a method call without thruster_data, using the tcf file path instead.)

            Parameters
            ----------
            None

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        """
        if thruster_data is None:
            path_to_tcf = self.case_dir + 'tcd/' + self.config['tcd']['tcf']
        
            try:
                path_to_tcf = self.case_dir + 'tcd/' + self.config['tcd']['tcf']
            except KeyError:
                # print("WARNING: Thruster Configuration File not set")
                return
            # Simple program, reading text from a file.
            with open(path_to_tcf, 'r') as f:
                lines = f.readlines()

                # Parse through first few lines, save relevant information. 
                self.num_thrusters = int(lines.pop(0))
                self.thruster_units = lines.pop(0)[0] # dont want '\n'
                self.cog = process_coordinates(lines.pop(0))
                self.grapple = process_coordinates(lines.pop(0))

                # Save all strings containing thruster data in a list
                str_thrusters = []
                for i in range(self.num_thrusters):
                    str_thrusters.append(lines.pop(0))

                # Parse through strings and save data in a dictionary
                self.thruster_data = process_str_thrusters(str_thrusters)

                self.jet_interactions = lines.pop(0)
        
        else:
            self.thruster_data = thruster_data

        self.use_clusters = False
        
        return
    
    def change_cluster_config(self, x):
        """
            Alters cluster configuration data using OpenMDAO inputs.

            Parameters
            ----------
            x : array
                Axial position (along the x axis) of the nozzle exit with respect to the LM's docking adapter.
            
            Returns
            -------
            Method doesn't currently return anything.
        """
        # print('len(self.cluster_data) is', len(self.cluster_data))
        # print('float(x) is', float(x))
        for cluster in self.cluster_data:
            # print('cluster is', cluster)
            self.cluster_data[cluster]["exit"][0][0] = float(x)

    def set_cluster_config(self):
        """
            Read in cluster configuration data from the provided file path.
            Gathers cluster configuration data for the Visiting Vehicle from a .dat file
            and saves it as class members.
            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
        """

        path_to_ccf = self.case_dir + 'tcd/' + self.config['tcd']['ccf']

        # Simple program, reading text from a file.
        with open(path_to_ccf, 'r') as f:
            lines = f.readlines()

            # Parse through first few lines, save relevant information. 
            self.num_clusters = int(lines.pop(0))
            self.cluster_units = lines.pop(0)[0] # dont want '\n'

            # Save all strings containing cluster data in a list
            str_clusters = []
            for i in range(self.num_clusters):
                str_clusters.append(lines.pop(0))

            # Parse through strings and save data in a dictionary
            self.cluster_data = process_str_clusters(str_clusters)
        
        self.use_clusters = True
        
        return

    def set_thruster_metrics(self):
        """
            Reads the csv thruster data file to gather thruster-specific performance parameters for the configuration
            and saves it in a list of dictionaries. These dictionaries are then saved into each thruster in the configuration.

            Parameters
            ----------
            None

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        """

        # Read in path for thruster metric data.
        try:
            path_to_thruster_metrics = self.case_dir + 'tcd/' + self.config['tcd']['tdf']
        except KeyError:
            # print("WARNING: Thruster Metrics File Not Set")
            self.thruster_metrics = None
            return

        # specify columns to be read as strings.
        str_cols = ['#']
        dict_types = {x: 'str' for x in str_cols}

        # read csv into a pd dataframe
        thruster_metrics = pd.read_csv(path_to_thruster_metrics, dtype=dict_types)
        # print(thruster_characteristics)

        # convert the dataframe into a list of dictionaries
        thruster_metrics_list = thruster_metrics.to_dict(orient='records')

        self.thruster_metrics = {}

        for thruster in thruster_metrics_list:

            # Seperate thruster metrics to form new key value pairs.
            thruster_id = thruster['#']
            thruster_metrics = thruster.pop('#')

            # Save thruster metrics
            self.thruster_metrics[thruster_id] = thruster

        # print(self.thruster_metrics)

        return

    def initiate_plume_mesh(self):
        """
            Helper method that reads in surface mesh for plume clone.

            Parameters
            ----------
            None for now. Should/could include cone sizing parameters according to plume physics.
            This is easy. Simply produce a "unit cone" ahead of time, and scale the coordinates
            using numpy-stl. Cone half-angle can also be pre-programmed.

            Returns
            -------
            plumeMesh : mesh.Mesh
                Surface mesh constructed from STL file.
        """
        # TODO: use STL that is already oriented correctly.
        plumeMesh = mesh.Mesh.from_file('../data/stl/mold_funnel.stl')
        plumeMesh.translate([0, 0, -50])
        plumeMesh.rotate([1, 0, 0], math.radians(180))
        plumeMesh.points = 0.035 * plumeMesh.points
        return plumeMesh

    def transform_plume_mesh(self, thruster_id, plumeMesh):
        """
            Transform provided plume mesh according to specified thruster's DCM and exit coordinate.

            Parameters
            ----------
            thruster_id : str
                String to access thruster via a unique ID.

            plumeMesh : mesh.Mesh
                Surface mesh constructed from STL file in initial orientation.

            Returns
            -------
            plumeMesh : mesh.Mesh
                Surface mesh constructed from STL file in transformed orientation.

        """
        rot = np.array(self.thruster_data[thruster_id]['dcm'])
        plumeMesh.rotate_using_matrix(rot.T)
        plumeMesh.translate(self.thruster_data[thruster_id]['exit'][0])
        return plumeMesh

    def initiate_plume_normal(self, thruster_id):
        """
            Collects plume normal vectors data for visualization.

            Parameters
            ----------
            thruster_id : str
                String to access thruster via a unique ID.

            Returns
            -------
            [X,Y,Z,U,V,W] : 2D List
                2D list contains vector data for plume normal. This is janky but convenient for plotting.

        """

        X = []
        Y = []
        Z = []

        U = []
        V = []
        W = []

        # add position vectors to a list.
        position = self.thruster_data[thruster_id]['exit'][0]
        X.append(position[0])
        Y.append(position[1])
        Z.append(position[2])


        # add normal vectors to a list
        dcm = self.thruster_data[thruster_id]['dcm']
        U.append(dcm[0][2])
        V.append(dcm[1][2])
        W.append(dcm[2][2])


        return [X,Y,Z,U,V,W]

    def plot_vv_and_thruster(self, plumeMesh, thruster_id, normal, i):
        """
            Plots Visiting Vehicle and plume cone for provided thruster id.

            This is useful for a quick sanity check of STL file coordinates.

            Parameters
            ----------
            plumeMesh : mesh.Mesh
                Surface mesh constructed from STL file in transformed orientation.

            thruster_id : str
                String to access thruster via a unique ID.

            normal : 2D List
                2D list contains vector data for plume normal. This is janky but convenient for plotting.

            Returns
            -------
            i : int
                Integer is passed to the wrapper function for saving images with a sequential naming scheme.

        """

        # Set up nominal configuration for thruster
        VVmesh = self.mesh

        # graph vehicle and vectors.
        combined = mesh.Mesh(np.concatenate([VVmesh.data, plumeMesh.data]))

        # Instantiate data str to hold visual plots.
        figure = plt.figure()
        axes = figure.add_subplot(projection = '3d')
        axes.add_collection3d(mplot3d.art3d.Poly3DCollection(VVmesh.vectors))

        surface = mplot3d.art3d.Poly3DCollection(plumeMesh.vectors)
        surface.set_facecolor('orange')

        axes.add_collection3d(surface)
        axes.quiver(normal[0], normal[1], normal[2], normal[3], normal[4], normal[5], color = (0,0,0), length=4, normalize=True)

        lim = 7
        axes.set_xlim([-1*lim - 3, lim - 3])
        axes.set_ylim([-1*lim, lim])
        axes.set_zlim([-1*lim, lim])

        axes.set_xlabel('X')
        axes.set_ylabel('Y')
        axes.set_zlabel('Z')

        figure.suptitle(self.thruster_data[thruster_id]['name'][0])

        shift = 0

        if i < 4:
            axes.view_init(azim=0, elev=2*shift)
        elif i < 8:
            axes.view_init(azim=0, elev=2*shift)
        elif i < 12:
            axes.view_init(azim=0, elev=2*shift)
        else:
            axes.view_init(azim=0, elev=2*shift)

        if i < 10:
            index = '00' + str(i)
        elif i < 100:
            index = '0' + str(i)
        else:
            index = str(i)
        # screen_shot = vpl.screenshot_fig()
        # vpl.save_fig('img/frame' + str(index) + '.png')
        plt.savefig('img/frame' + str(index) + '.png')
        return i + 1

    def check_thruster_configuration(self):
        """
            Plots visiting vehicle and all thrusters in RCS configuration.

            Methods loads STL file of VV and turn on all thrusters to check locations + orientations.

            It is useful for a quick sanity check of the RCS configuration.
        """

        # Loop through each thruster, graphing normal vecotr and rotated plume cone.
        i = 0
        for thruster_id in self.thruster_data:

            # transform plume mesh to notional position.
            plumeMesh = self.initiate_plume_mesh()

            # transform plume mesh according to dcm data of current thruster.
            plumeMesh = self.transform_plume_mesh(thruster_id, plumeMesh)

            if not os.path.isdir('stl/tcd/'):
                os.system('mkdir stl/tcd')

            plumeMesh.save('stl/tcd/' + str(i) + '.stl')

            normal = self.initiate_plume_normal(thruster_id)

            i = self.plot_vv_and_thruster(plumeMesh, thruster_id, normal, i)

        return