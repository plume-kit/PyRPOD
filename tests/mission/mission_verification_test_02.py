# Andy Torres
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 12-05-23


# ========================
# PyRPOD: test/mission/mission_verification_test_01.py
# ========================
# Write test case description.


import test_header
import unittest
from pyrpod.mission import MissionPlanner, MissionEnvironment

class MDAOTest(unittest.TestCase):
    def test_mission(self):
        case_dir = '../case/mission/flight_envelopes/'
        me = MissionEnvironment.MissionEnvironment(case_dir)
        planner = MissionPlanner.MissionPlanner(me)
        planner.orbital_transfer.init_hohmann_transfers()
        planner.orbital_transfer.add_hohmann_transfer(300, 20000, leg_id="LEO to MEO")
        planner.orbital_transfer.add_hohmann_transfer(20000, 35786, leg_id="MEO to GEO")
        planner.orbital_transfer.summarize_hohmann_transfers()


        return

if __name__ == '__main__':
    unittest.main()
