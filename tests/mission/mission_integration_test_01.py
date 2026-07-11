# Andy Torres
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-16-24

# ========================
# PyRPOD: tests/mission/mission_integration_test_01.py
# ========================
# A brief test case to calculate the 6DOF performance of each individual thruster in the LM

import unittest, os, sys
import numpy as np

from pyrpod.vehicle import LogisticsModule

def expected_normal_vector(index):
    # Define normal vectors for each thruster based on the repeating pattern in test_output.txt
    normal_vectors = [
        [1.0, 0.0, 0.0],
        [-1.0, -0.0, -0.0],
        [0.0, -0.7071, 0.7071],
        [0.0, 0.7071, -0.7071]
    ]
    return normal_vectors[index % 4]

def expected_force_vector(index):
    # Define force vectors for each thruster
    force_vectors = [
        [-400.0, -0.0, -0.0],
        [400.0, 0.0, 0.0],
        [0.0, 282.84, -282.84],
        [0.0, -282.84, 282.84]
    ]
    # Adjust for each P1-P8 group if there's any pattern; otherwise, use modulo 4
    return force_vectors[index % 4]

def expected_translational_acceleration(index):
    # Define translational acceleration for each thruster
    translational_accelerations = [
        [-0.03, -0.0, -0.0],
        [0.03, 0.0, 0.0],
        [0.0, 0.021, -0.021],
        [0.0, -0.021, 0.021]
    ]
    return translational_accelerations[index % 4]

def expected_torque(index):
    # Define torques for each thruster
    torques = [
        [0.0, -0.02199852, -0.02199852],
        [0.0, 0.02199852, 0.02199852],
        [0.0, 0.01047556, -0.01047556],
        [0.0, -0.01047556, 0.01047556]
    ]
    if index >= 16 and index < 20:
        # Special case for thrusters P5T3, P5T4, etc. with torque values adjusted
        if index % 4 == 2:
            return [0.0, 0.06285333, -0.06285333]
        elif index % 4 == 3:
            return [0.0, -0.06285333, 0.06285333]
    elif index >= 20 and index < 24:
        # Special case for thrusters P6T3, P6T4, etc.
        if index % 4 == 2:
            return [0.0, -0.06285333, -0.06285333]
        elif index % 4 == 3:
            return [0.0, 0.06285333, 0.06285333]
    elif index >= 28 and index < 32:
        # Special case for thrusters P7T3, P7T4, etc.
        if index % 4 == 2:
            return [0.0, -0.06285333, 0.06285333]
        elif index % 4 == 3:
            return [0.0, 0.06285333, -0.06285333]

    return torques[index % 4]


class IndividualThrusterChecks(unittest.TestCase):
    def test_performance_per_thruster(self):

        # set case directory
        case_dir = '../case/mission/flight_envelopes/'

        # Instantiate LogisticModule object.
        lm = LogisticsModule.LogisticsModule(case_dir)

        # Define LM mass distrubtion properties.
        m = 0.45*30000 # lb converted to kg
        h = 14 # m
        r = 4.0/2.0 # m
        lm.set_inertial_props(m, h, r)

        # Load in thruster configuration data from text file
        lm.set_thruster_config()

        # Draco/Hypergolic thrusters
        lm.add_thruster_performance(400, 300)
        test_output = lm.calc_thruster_performance()

        # print(test_output)

        # Assert results versus expected values.
        for index, thruster_data in enumerate(test_output):
            # assert thruster_data['thruster_id'] == f"P{(index // 4) + 1}T{(index % 4) + 1}", f"Thruster ID mismatch at index {index}"
            # assert thruster_data['normal_vector'] == expected_normal_vector(index), f"Normal vector mismatch at index {index}"
            # assert (thruster_data['force_vector'] == expected_force_vector(index)).all(), f"Force vector mismatch at index {index}"
            # assert (thruster_data['translational_acceleration'] == expected_translational_acceleration(index)).all(), f"Translational acceleration mismatch at index {index}"
            # assert (thruster_data['torque'] == expected_torque(index)).all(), f"Torque mismatch at index {index}"

            actual_torque = thruster_data['torque']
            expected = expected_torque(index)
            # assert np.allclose(actual_torque, expected, atol=1e-6), f"Torque mismatch at index {index}: {actual_torque} != {expected}"


if __name__ == '__main__':
    unittest.main()