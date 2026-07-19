# ========================
# PyRPOD: tests/plume/plume_integration_test_02.py
# ========================
# Asserts that PlumeStrikeEstimationStudy.jfh_plume_strikes():
#   1. keeps its default behavior serial and return-compatible (dict keyed
#      '1'..'N' with the documented per-firing arrays), and
#   2. produces identical per-firing and cumulative outputs when the optional
#      process-based parallel path is enabled, and
#   3. rejects invalid worker counts with a clear error.
#
# Two representative existing cases are used:
#   - case/rpod/1d_approach            (kinetics disabled)
#   - case/rpod/multi_thrusters_square (Simplified kinetics)
#
# Strike arrays (strikes, cum_strikes) and struck-face IDs must match
# exactly. Kinetics arrays are compared with np.allclose(rtol=1e-12,
# atol=1e-12): serial and parallel paths execute the same vectorized code on
# the same inputs (observed bit-identical here), but the tolerance guards
# against BLAS/SIMD reduction-order differences across platforms/processes.
#
# VTK output goes to each case's existing results/ directory, the same safe
# pattern the existing rpod integration tests rely on.

import unittest

import numpy as np

from pyrpod.mission import MissionEnvironment
from pyrpod.rpod import JetFiringHistory, PlumeStrikeEstimationStudy
from pyrpod.vehicle import TargetVehicle, VisitingVehicle

KINETICS_RTOL = 1e-12
KINETICS_ATOL = 1e-12
EXACT_KEYS = ("strikes", "cum_strikes")

GEOMETRY_KEYS = {"strikes", "cum_strikes"}
KINETICS_KEYS = GEOMETRY_KEYS | {
    "pressures", "max_pressures",
    "shear_stress", "max_shears",
    "heat_flux_rate", "heat_flux_load", "cum_heat_flux_load",
}


def make_study(case_dir):
    jfh = JetFiringHistory.JetFiringHistory(case_dir)
    jfh.read_jfh()

    tv = TargetVehicle.TargetVehicle(case_dir)
    tv.set_stl()

    vv = VisitingVehicle.VisitingVehicle(case_dir)
    vv.set_stl()
    vv.set_thruster_config()
    vv.set_thruster_metrics()

    me = MissionEnvironment.MissionEnvironment(case_dir)
    study = PlumeStrikeEstimationStudy.PlumeStrikeEstimationStudy(me)
    study.study_init(jfh, tv, vv)
    return study


class ParallelMatchesSerialChecks(unittest.TestCase):

    def assert_firing_data_equal(self, serial, parallel, case_dir):
        self.assertEqual(set(serial.keys()), set(parallel.keys()))
        for firing in serial:
            self.assertEqual(
                set(serial[firing].keys()), set(parallel[firing].keys()),
                msg=f"{case_dir} firing {firing}: cellData keys differ",
            )
            for key in serial[firing]:
                if key in EXACT_KEYS:
                    np.testing.assert_array_equal(
                        parallel[firing][key], serial[firing][key],
                        err_msg=f"{case_dir} firing {firing}: {key} differs",
                    )
                else:
                    self.assertTrue(
                        np.allclose(
                            parallel[firing][key], serial[firing][key],
                            rtol=KINETICS_RTOL, atol=KINETICS_ATOL,
                        ),
                        msg=f"{case_dir} firing {firing}: {key} differs",
                    )
            # Struck-face IDs must be identical.
            np.testing.assert_array_equal(
                np.nonzero(parallel[firing]["strikes"])[0],
                np.nonzero(serial[firing]["strikes"])[0],
                err_msg=f"{case_dir} firing {firing}: struck-face IDs differ",
            )

    def assert_return_structure(self, firing_data, n_firings, expected_keys):
        self.assertEqual(
            sorted(firing_data.keys(), key=int),
            [str(i + 1) for i in range(n_firings)],
        )
        for firing in firing_data:
            self.assertEqual(set(firing_data[firing].keys()), expected_keys)

    def run_case(self, case_dir, expected_keys):
        study = make_study(case_dir)
        n_firings = len(study.jfh.JFH)

        # Default call: no arguments, serial legacy behavior.
        serial = study.jfh_plume_strikes()
        self.assert_return_structure(serial, n_firings, expected_keys)

        # Parallel path with an explicit worker count.
        parallel = make_study(case_dir).jfh_plume_strikes(parallel=True, workers=2)
        self.assert_return_structure(parallel, n_firings, expected_keys)

        self.assert_firing_data_equal(serial, parallel, case_dir)

        # workers=1 must resolve to the serial path and identical output.
        one_worker = make_study(case_dir).jfh_plume_strikes(parallel=True, workers=1)
        self.assert_firing_data_equal(serial, one_worker, case_dir)

    def test_geometry_only_case(self):
        self.run_case('../case/rpod/1d_approach/', GEOMETRY_KEYS)

    def test_kinetics_multi_thruster_case(self):
        self.run_case('../case/rpod/multi_thrusters_square/', KINETICS_KEYS)

    def test_invalid_workers_rejected(self):
        study = make_study('../case/rpod/1d_approach/')
        with self.assertRaises(ValueError):
            study.jfh_plume_strikes(parallel=True, workers=0)
        with self.assertRaises(ValueError):
            study.jfh_plume_strikes(parallel=True, workers=-2)


if __name__ == '__main__':
    unittest.main()
