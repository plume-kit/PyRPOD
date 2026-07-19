# Andy Torres
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-16-24

# ========================
# PyRPOD: tests/plume/plume_verification_test_01.py
# ========================
# A test case to plot simple radial expansion profiles.
# TODO: Re-factor code to save data in a relevant object. Also add files to save to.

import unittest, os, sys
from pyrpod.plume import IsentropicExpansion

class IsentropicExpansionCheck(unittest.TestCase):
    def test_temp_vs_radial_expansion(self):

        #define flow and sonic properties.
        M1 = 1
        M2 = 25
        gamma = 5/3
        T_star = 500
        r_star = 1

        # Plot isentropic expansion curves.
        isen_plume = IsentropicExpansion.IsentropicExpansion()
        # isen_plume.plot_number_density_ratios_vs_radius(M1, M2, gamma, r_star)
        # isen_plume.plot_temp_ratios_vs_radius(M1, M2, gamma, r_star)

if __name__ == '__main__':
    unittest.main()
