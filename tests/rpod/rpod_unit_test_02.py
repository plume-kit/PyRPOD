# Andy Torres
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-16-24

# ========================
# PyRPOD: tests/rpod/rpod_unit_test_02.py
# ========================
# Test case for reading in JFH data.


import unittest, os, sys
from pyrpod.rpod import JetFiringHistory

def assert_dictionary_content(data):
    for entry in data:
        # Assert that all required keys exist
        assert 'nt' in entry, "Key 'nt' is missing"
        assert 'dt' in entry, "Key 'dt' is missing"
        assert 't' in entry, "Key 't' is missing"
        assert 'dcm' in entry, "Key 'dcm' is missing"
        assert 'xyz' in entry, "Key 'xyz' is missing"
        assert 'uf' in entry, "Key 'uf' is missing"
        assert 'thrusters' in entry, "Key 'thrusters' is missing"

        # Assert the data types of values
        assert isinstance(entry['nt'], str), "'nt' should be a string"
        assert isinstance(entry['dt'], (float, str)), "'dt' should be a float or string"
        assert isinstance(entry['t'], (float, str)), "'t' should be a float or string"
        assert isinstance(entry['dcm'], list), "'dcm' should be a list"
        assert len(entry['dcm']) == 3, "'dcm' should have 3 rows"
        for row in entry['dcm']:
            assert len(row) == 3, "Each row in 'dcm' should have 3 elements"
            for value in row:
                assert isinstance(value, float), "Values in 'dcm' should be floats"

        assert isinstance(entry['xyz'], list), "'xyz' should be a list"
        assert len(entry['xyz']) == 3, "'xyz' should have 3 elements"
        for value in entry['xyz']:
            assert isinstance(value, float), "Values in 'xyz' should be floats"

        assert isinstance(entry['uf'], float), "'uf' should be a float"

        assert isinstance(entry['thrusters'], list), "'thrusters' should be a list"
        for value in entry['thrusters']:
            assert isinstance(value, int), "Values in 'thrusters' should be integers"

class LoadJFHChecks(unittest.TestCase):

    def test_jfh_reader(self):

        # Path to directory holding data assets and results for a specific RPOD study.
        case_dir = '../case/rpod/base_case/'

        # Load JFH data.
        jfh = JetFiringHistory.JetFiringHistory(case_dir)
        jfh.read_jfh()
        data = []
        for firing in range(len(jfh.JFH)):
            data.append(jfh.JFH[firing])

        assert_dictionary_content(data)

if __name__ == '__main__':
    unittest.main()