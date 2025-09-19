import configparser
import numpy as np
import sympy as sp

from pyrpod.util.io.file_print import print_JFH
from pyrpod.logging_utils import get_logger

logger = get_logger("pyrpod.rpod.JetFiringHistory")

# Helper functions (move to util.py?)
def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix

def make_norm(vector_value_function):
    """Calculate vector norm/magnitude using the Pythagoream Theorem."""
    return sp.sqrt(sp.Pow(vector_value_function[0],2) + sp.Pow(vector_value_function[1],2))

class JetFiringHistory:
    """
        Class responsible for reading and parsing through text files
        that contain jet firing histories for a visiting vehicle.

        Attributes
        ----------
        config : ConfigParser
            Object responsible for reading data from the provided configuration file.

        case_dir : str
            Path to case directory. Used to store data and results for a specific scenario.

        JFH : list<dict>
            List containing information for each firing (stored as a dict) in the JFH.

        Methods
        -------
        read_JFH()
            Method reads in and parses through text file containing the JFH.
        
        graph_param_curve(self, t, r_of_t)
            Used to quickly prototype and visualize a proposed approach path.
            Calculates the unit tangent vector at a given timestep and 
            rotates the STL file accordingly. Data is plotted using matlab

        print_JFH_param_curve(self, jfh_path, t, r_of_t, align = False)
            Used to produce JFH data for a proposed approach path.
            Calculates the unit tangent vector at a given timestep and DCMs for
            STL file rotations. Data is then saved to a text file.
    """

    def __init__(self, case_dir):
        """
            Constructor simply sets case directory and parses the
            appopriate configuration file.

            Parameters
            ----------
            case_dir : str
                Path to case directory. Used to store data and results for a specific scenario.

            Returns
            -------
            jfh : JetFiringHistory
                Method reads in and parses through text file containing the JFH.
        """

        self.case_dir = case_dir
        config = configparser.ConfigParser()
        config.read(self.case_dir + "config.ini")
        self.config = config

    def read_jfh(self):
        """
            Method responsible for reading and parsing through JFH data.

            NOTE: Methods does not take any parameters. It assumes that self.case_dir
            and self.config are instantiated correctly. Potential defensive programming statements?

            Parameters
            ----------
            None

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?

        """

        try:
            path_to_jfh = self.case_dir + 'jfh/' + self.config['jfh']['jfh']
        except KeyError:
            # print("WARNING: Jet Firing History not set")
            self.JFH = None
            return

        with open(path_to_jfh, 'r') as f:
            lines = f.readlines()

            # Save number of firings in JFH. 
            try:
                self.nt = int(lines.pop(0).split(' ')[4])
            except IndexError:
                logger.warning("Supplied JFH file is empty: %s", path_to_jfh)
                self.JFH = None
                return

            # Throw away second line
            lines.pop(0)

            JFH = []

            for i in range(self.nt):
                
                # print(i)
                # Split current into a list row at every space character.
                curr_row = lines.pop(0).split(' ')
                # print(curr_row)

                
                # Remove empty strings from list. 
                while("" in curr_row):
                    curr_row.remove("")

                # Remove new line character. 
                curr_row[-1] = curr_row[-1].split('\n')[0]
                # print(curr_row)
                # Save all information in current row to a dictionary.
                time_step = {}
                
                # Save time data. 
                time_step['nt'] = curr_row.pop(0)
                time_step['dt'] = curr_row.pop(0)
                time_step['t'] = curr_row.pop(0)

                # Throw away column 4
                curr_row.pop(0)

                # Save direction cosine matrix of thruster relative to the vehicle    
                dcm = []
                for i in range(3):
                    row = []
                    for j in range(3):
                        row.append(float(curr_row.pop(0)))
                    dcm.append(row)
                time_step['dcm'] = dcm

                # Save position data 
                pos = []
                for i in range(3):
                    pos.append(float(curr_row.pop(0)))
                time_step['xyz'] = pos

                # Save uncertainty factor
                time_step['uf'] = float(curr_row.pop(0))

                # Save thruster data
                num_thrusters = int(float(curr_row.pop(0)))
                thrusters = []
                for i in range(num_thrusters):
                    thrusters.append(int(curr_row.pop(0)))

                time_step['thrusters'] = thrusters

                JFH.append(time_step)
            self.JFH = JFH
            f.close()
        return


    def graph_param_curve(self, t, r_of_t):
        ''' Used to quickly prototype and visualize a proposed approach path.
            Calculates the unit tangent vector at a given timestep and
            rotates the STL file accordingly. Data is plotted using matlab

            Current method is old and needs updating.

            Parameters
            ----------
            t : sp.symbol
                Time (t) is the independent variable used to evaulte the position vector equation.

            r_of_t : list<expressions?>
                List containing position vector expression for trajectory.
                X/Y/Z positions are de-coupled and only dependent on time.

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        '''

        t_values = np.linspace(0,2*np.pi,50)

        # Symbolic Calculations of tangent and normal unit vectors
        r = r_of_t
        rprime = [sp.diff(r[0], t), sp.diff(r[1], t), sp.diff(r[2], t)]
        tanvector = [rprime[0] / make_norm(rprime), rprime[1] / make_norm(rprime), rprime[2] / make_norm(rprime)]
        tanprime = [sp.diff(tanvector[0], t), sp.diff(tanvector[1], t), sp.diff(tanvector[2], t)]
        normalvector = [tanprime[0] / make_norm(tanprime), tanprime[1] / make_norm(tanprime), tanprime[1] / make_norm(tanprime)]
        tan_vector_functions = [sp.lambdify(t, tanvector[0]), sp.lambdify(t, tanvector[1]), sp.lambdify(t, tanvector[2])]
        normal_vector_functions = [sp.lambdify(t, normalvector[0]), sp.lambdify(t, normalvector[1]), sp.lambdify(t, normalvector[2])]
        value_functions = [sp.lambdify(t, r[0]), sp.lambdify(t, r[1]), sp.lambdify(t, r[2])]

        # Save data of evaluated position and velocity functions.
        x, y, z = [value_functions[0](t_values), value_functions[1](t_values), value_functions[2](t_values)]
        dx, dy, dz = [tan_vector_functions[0](t_values), tan_vector_functions[1](t_values), tan_vector_functions[2](t_values)]

        # draw the vectors along the curve and Graph STL.
        for i in range(len(t_values)):
            # Graph path
            # ax = plt.figure().add_subplot(projection='3d')
            from matplotlib import pyplot as plt
            from mpl_toolkits import mplot3d
            figure = plt.figure()
            ax = mplot3d.Axes3D(figure)
            ax.plot(x, y, z, label='position curve')
            ax.legend()
            normal_location = t_values[i]

            # Load, Transform, and Graph STL
            from stl import mesh
            VV = mesh.Mesh.from_file('../stl/cylinder.stl')
            VV.points = 0.2 * VV.points

            r = [x[i], y[i], z[i]]
            dr = [dx, dy, dz[i]]

            # Calculate require rotation matrix from initial orientation.
            x1 = [1, 0, 0]
            rot = np.matrix(rotation_matrix_from_vectors(x1, dr))

            VV.rotate_using_matrix(rot.transpose())
            VV.translate(r)

            ax.add_collection3d(
                mplot3d.art3d.Poly3DCollection(VV.vectors)
            )

            # print(
            # tan_vector_functions[0](normal_location),
            # tan_vector_functions[1](normal_location),
            # tan_vector_functions[2](normal_location)
            # )
            # print()
            length = 1
            ax.quiver(
                value_functions[0](normal_location),
                value_functions[1](normal_location),
                value_functions[2](normal_location),
                tan_vector_functions[0](normal_location),
                tan_vector_functions[1](normal_location),
                tan_vector_functions[2](normal_location),
                color='g',
                length = length
            )

            # ax.quiver(
            #     value_functions[0](normal_location),
            #     value_functions[1](normal_location),
            #     value_functions[2](normal_location),
            #     normal_vector_functions[0](normal_location),
            #     normal_vector_functions[1](normal_location),
            #     normal_vector_functions[2](normal_location),
            #     color='r'
            # )
            logger.info("graph_param_curve step %s/%s", i + 1, len(t_values))

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')

            if i < 10:
                index = '00' + str(i)
            elif i < 100:
                index = '0' + str(i)
            else:
                index = str(i)

            plt.savefig('img/observer-a-' + str(index) + '.png')

            plt.close()

    def print_JFH_param_curve(self, jfh_path, t, r_of_t, align = False):
        ''' Used to produce JFH data for a proposed approach path.
            Calculates the unit tangent vector at a given timestep and DCMs for
            STL file rotations. Data is then saved to a text file.


            Parameters
            ----------
            jfh_path : str
                Path to file for saving JFH data.

            t : sp.symbol
                Time (t) is the independent variable used to evaulte the position vector equation.

            r_of_t : list<expressions?>
                List containing position vector expression for trajectory.
                X/Y/Z positions are de-coupled and only dependent on time.

            aligh : Boolean
                Determines whether or not STL surface is rotated according to unit tangent vector.

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        '''

        # t_values = np.linspace(0,2*np.pi,100)
        t_values = np.linspace(0, 50, 20)

        # Symbolic Calculations of tangent and normal unit vectors
        r = r_of_t
        rprime = [sp.diff(r[0],t), sp.diff(r[1],t), sp.diff(r[2],t)]
        tanvector = [rprime[0]/make_norm(rprime), rprime[1]/make_norm(rprime), rprime[2]/make_norm(rprime)]
        tanprime = [sp.diff(tanvector[0],t), sp.diff(tanvector[1],t), sp.diff(tanvector[2],t)]
        normalvector = [tanprime[0]/make_norm(tanprime), tanprime[1]/make_norm(tanprime), tanprime[1]/make_norm(tanprime)]
        tan_vector_functions = [sp.lambdify(t, tanvector[0]),sp.lambdify(t, tanvector[1]), sp.lambdify(t, tanvector[2])]
        normal_vector_functions = [sp.lambdify(t, normalvector[0]),sp.lambdify(t, normalvector[1]), sp.lambdify(t, normalvector[2])]
        value_functions = [sp.lambdify(t, r[0]), sp.lambdify(t, r[1]), sp.lambdify(t, r[2])]

        # Save data of evaluated position and velocity functions.
        x, y, z = [value_functions[0](t_values), value_functions[1](t_values), value_functions[2](t_values)]
        dx, dy, dz = [tan_vector_functions[0](t_values), tan_vector_functions[1](t_values), tan_vector_functions[2](t_values)]

        # print(type(dx), type(dy), type(dz))

        # print(dx.size, dy.size, dz.size)

        # When derivatives reduce to constant value the lambda function will reutrn a float
        # instead of np.array. These if statements are here fill an array with that float value.
        # print(type(dx), dx)
        # print(type(dy), dy)
        if type(x) == int:
            x = np.full(t_values.size, x)

        if type(dx) == int or dx.size == 1:
            # print('dx is contant')
            # print(dx)
            dx = np.full(t_values.size, dx)
            # x = np.full(t_values.size, )

        if type(y) == int:
            y = np.full(t_values.size, y)
        if type(dy) == int or dy.size == 1:
            # print('dy is contant')
            # print(dy)
            dy = np.full(t_values.size, dy)

        if type(z) == int:
            z = np.full(t_values.size, z)
        if type(dz) == int or dz.size == 1:
            # print('dz is contant')
            # print(dz)
            dz = np.full(t_values.size, dz)

        # print(type(dx), type(dy), type(dz))
        # print(type(x), type(y), type(z))

        # Save rotation matrix for each time step
        rot = []
        if align:
            for i in range(len(t_values)):
                dr = [dx[i], dy[i], dz[i]]

                # Calculate required rotation matrix from initial orientation.
                x1 = [1, 0, 0]
                rot.append(np.matrix(rotation_matrix_from_vectors(x1, dr)))
        else:
            for i in range(len(t_values)):
                # Calculate required rotation matrix from initial orientation.
                y1 = [0, 0, -1]
                x1 = [1, 0, 0]
                rot.append(np.matrix(rotation_matrix_from_vectors(x1, y1)))

        r = [x, y, z]
        # dr = [dx, dy, dz]
        print_JFH(t_values, r,  rot, jfh_path)
