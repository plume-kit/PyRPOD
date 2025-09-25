"""Script contains helper functions for printing JFH data.
    As PyRPOD evolves this could become a utilities script.
    Alternatively these could be moved to JetFiringHistory.py
"""

import sys

def print_JFH(t_values, r,  rot, file_name):
    '''
        Helper function to RPOD.print_jfh_param_curve() that is responsible for
        printing the calculated JFH data to a text file.

        Parameters
        ----------
        t_values : np.array?
            Array containing time step data for each firing in the JFH.

        r : np.array?
            Array containing positional data for each firing in the JFH.

        rot : np.array?
            Array containing rotational data for each firing in the JFH.

        file_name : str
            File name for JFH txt file. It will be saved in configured case directory.

        Returns
        -------
        Method doesn't currently return anything. Simply saves data to files as needed.
        Does the method need to return a status message? or pass similar data?
    '''
    x = r[0]
    y = r[1]
    z = r[2]

    orig_stdout = sys.stdout
    with open(file_name, 'w') as f:

        sys.stdout = f

        print("offseted   ", str(len(t_values)), "      0")
        print("       0.000       0.000       0.000")
        for i in range(len(t_values)):

            print('     ', i+1, end=' ')
            
            p = 3
            
            # Print t values
            print(round(t_values[i], p), end = '    ')

            # Print dt values.
            if i == 0:
                print(round(t_values[i+1], p), end = '    ')
            else:
                print(round(t_values[i] - t_values[i -1], p), end = '    ')

            # Print unused column (#4)
            print("0.00", end = ' ')
            
            # Print elements of rotation matrix
            for j in range(3):
                for k in range(3):
                    print(round(rot[i][j][k], p), end = ' ')

            # Print position values. 
            print(round(x[i], p), round(y[i], p), round(z[i], p), end = '   ')

            # Print uncertainty factor
            print('1.000', end = ' ')

            # thruster = i%16+1
            thruster = 1
            print('1  ', thruster, end = ' ')

            # Print thrusters to be turned on. 
            # group = i % 4
            # for j in range(4):
            #     print((j+1)*group + 1, end =' ')
            # print(i % 16, end = ' ')

            print()
            # print(rot[i])
            # print()
    sys.stdout = orig_stdout

def print_test_JFH(t_values, r,  rot, file_name):

    '''
        Helper function to RPOD.print_jfh_param_curve() that is responsible for
        printing the calculated JFH data to a text file.

        NOTE: This is a test function and possibly broken. Keeping it for reference/possible future work.

        Parameters
        ----------
        t_values : np.array?
            Array containing time step data for each firing in the JFH.

        r : np.array?
            Array containing positional data for each firing in the JFH.

        rot : np.array?
            Array containing rotational data for each firing in the JFH.

        file_name : str
            File name for JFH txt file. It will be saved in configured case directory.

        Returns
        -------
        Method doesn't currently return anything. Simply saves data to files as needed.
        Does the method need to return a status message? or pass similar data?
    '''

    x = r[0]
    y = r[1]
    z = r[2]

    orig_stdout = sys.stdout
    with open(file_name, 'w') as f:

        sys.stdout = f

        print("offseted   ", str(len(t_values)), "      0")
        print("       0.000       0.000       0.000")
        for i in range(len(t_values)):

            print('     ', i+1, end=' ')
            
            p = 3
            
            # Print t values
            print(round(t_values[i], p), end = '    ')

            # Print dt values.
            if i == 0:
                print(round(t_values[i+1], p), end = '    ')
            else:
                print(round(t_values[i] - t_values[i -1], p), end = '    ')

            # Print unused column (#4)
            print("0.00", end = ' ')
            
            # Print elements of rotation matrix
            for j in range(3):
                for k in range(3):
                    print(round(rot[i][j][k], p), end = ' ')

            # Print position values. 
            print(round(x[i], p), round(y[i], p), round(z[i], p), end = '   ')

            # Print uncertainty factor
            print('1.000', end = ' ')

            thruster = i%16+1
            print('1  ', thruster, end = ' ')

            # Print thrusters to be turned on.
            # group = i % 4
            # for j in range(4):
            #     print((j+1)*group + 1, end =' ')
            # print(i % 16, end = ' ')

            print()
            # print(rot[i])
            # print()
    sys.stdout = orig_stdout

def print_1d_JFH(t_values, r,  rot, file_name):
    '''
        Helper function to RPOD.print_jfh_param_curve() that is responsible for
        printing the calculated JFH data to a text file.

        Parameters
        ----------
        t_values : np.array?
            Array containing time step data for each firing in the JFH.

        r : np.array?
            Array containing positional data for each firing in the JFH.

        rot : np.array?
            Array containing rotational data for each firing in the JFH.

        file_name : str
            File name for JFH txt file. It will be saved in configured case directory.

        Returns
        -------
        Method doesn't currently return anything. Simply saves data to files as needed.
        Does the method need to return a status message? or pass similar data?
    '''
    x = r[0]
    y = r[1]
    z = r[2]

    orig_stdout = sys.stdout
    with open(file_name, 'w') as f:

        sys.stdout = f

        print("offseted   ", str(len(t_values)), "      0")
        print("       0.000       0.000       0.000")
        for i in range(len(t_values)):

            print('     ', i+1, end=' ')

            p = 6

            # Print t values
            print('{:.3f}'.format(t_values[i]),  end = '    ')

            # Print dt values.
            if i == 0:
                # print('here', i)
                print(round(t_values[i+1], p), end = '    ')
            else:
                print(round(t_values[i] - t_values[i -1], p), end = '    ')

            # Print unused column (#4)
            print("0.00", end = ' ')

            # Print elements of rotation matrix
            for j in range(3):
                for k in range(3):
                    print('{:.6e}'.format(rot[i][j][k]),  end = '    ')

            # Print position values.
            print('{:.3f}'.format(-x[i]), '{:.3f}'.format(-y[i]), '{:.3f}'.format(-z[i]), end = '    ')

            # Print uncertainty factor
            print('1.000', end = ' ')

            # Print thrusters to be turned on.

            # thruster = i%16+1
            thruster = '1 2 3 4'
            print('4  ', thruster, end = '')

            # group = i % 4
            # for j in range(4):
            #     print((j+1)*group + 1, end =' ')
            # print(i % 16, end = ' ')

            print()
            # print(rot[i])
            # print()
    sys.stdout = orig_stdout
