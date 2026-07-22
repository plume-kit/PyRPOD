# ========================
# PyRPOD: tests/rpod/rpod_verification_test_07.py
# ========================
# Cylinder-target sweep SMOKE test: exercises the strike pipeline on a
# CURVED, closed target (case/plume/plume_cylinder_sweep -- the shared
# high-res cylinder mesh data/stl/cylinder.stl: 14036 faces, radius 2 m,
# axis along X in [-7, 0], centroid (-3.5, 0, 0)) instead of a flat plate. The
# visiting vehicle (single argon thruster) is swept over 19 approach
# angles x 5 orbit radii (95 firings; jfh_cylinder_sweep.A) about the
# cylinder centroid, and the full PlumeStrikeEstimationStudy pipeline
# (jfh_plume_strikes) is run end-to-end.
#
# This confirms the case RUNS -- the pipeline loads the cylinder, sweeps
# every pose, and produces per-firing strikes -- NOT that the loads are
# physically meaningful (the orbit is a geometric smoke sweep, not a
# validated impingement scenario). Assertions therefore gate only on the
# pipeline completing with well-formed per-firing results and the sweep
# actually illuminating the cylinder.
#
# Run:  python -m pytest rpod/rpod_verification_test_07.py -s   (from tests/)

import sys
import time
import unittest
from pathlib import Path

import numpy as np

_TESTS_DIR = Path(__file__).resolve().parents[1]
if str(_TESTS_DIR.parent) not in sys.path:  # repo root for direct runs
    sys.path.insert(0, str(_TESTS_DIR.parent))

from pyrpod.mission import MissionEnvironment  # noqa: E402
from pyrpod.rpod import (  # noqa: E402
    JetFiringHistory,
    PlumeStrikeEstimationStudy,
)
from pyrpod.vehicle import TargetVehicle, VisitingVehicle  # noqa: E402

CASE_DIR = '../case/plume/plume_cylinder_sweep/'
N_ANGLES = 19
N_RADII = 5
N_FACES = 14036   # data/stl/cylinder.stl (high-res target mesh)


class CylinderSweepSmoke(unittest.TestCase):

    def test_cylinder_sweep_runs(self):
        # 1. Set up the case (minimal driver, as in rpod_integration_test_07);
        # the case config already selects the cylinder STL + sweep JFH.
        jfh = JetFiringHistory.JetFiringHistory(CASE_DIR)
        jfh.read_jfh()

        tv = TargetVehicle.TargetVehicle(CASE_DIR)
        tv.set_stl()

        vv = VisitingVehicle.VisitingVehicle(CASE_DIR)
        vv.set_thruster_config()
        vv.set_thruster_metrics()

        me = MissionEnvironment.MissionEnvironment(CASE_DIR)

        study = PlumeStrikeEstimationStudy.PlumeStrikeEstimationStudy(me)
        study.study_init(jfh, tv, vv)

        # 2. Geometry sanity: the 95-firing sweep on the cylinder mesh.
        n_firings = len(jfh.JFH)
        self.assertEqual(n_firings, N_ANGLES * N_RADII,
                         msg=f'unexpected sweep length {n_firings}; regenerate '
                             'with case/plume/plume_cylinder_sweep/jfh/'
                             'generate_cylinder_sweep_jfh.py')
        n_faces = len(tv.mesh.vectors)
        self.assertEqual(n_faces, N_FACES,
                         msg=f'unexpected target mesh ({n_faces} faces); '
                             'expected the cylinder_transformed.stl target')

        # 3. Execute the full pipeline end-to-end.
        t0 = time.perf_counter()
        firing_data = study.jfh_plume_strikes()
        dt = time.perf_counter() - t0

        # 4. Every firing produced well-formed per-face results.
        self.assertEqual(len(firing_data), n_firings)
        total_strikes = 0
        for key, cell in firing_data.items():
            strikes = np.asarray(cell['strikes'])
            self.assertEqual(strikes.shape, (n_faces,),
                             msg=f'firing {key}: bad strikes shape')
            # strikes are a 0/1 membership mask
            self.assertTrue(np.all((strikes == 0) | (strikes == 1)),
                            msg=f'firing {key}: strikes not binary')
            # cumulative strikes never fall below this firing's own strikes
            cum = np.asarray(cell['cum_strikes'])
            self.assertTrue(np.all(cum >= strikes),
                            msg=f'firing {key}: cum_strikes < strikes')
            total_strikes += int(strikes.sum())

        # 5. The sweep must actually illuminate the cylinder somewhere
        # (a geometry/aiming regression would strike zero faces everywhere).
        self.assertGreater(total_strikes, 0,
                           msg='no faces struck across the whole sweep -- '
                               'the orbit never illuminates the cylinder')

        peak = max(int(np.asarray(c['strikes']).sum())
                   for c in firing_data.values())
        print(f'[cylinder-sweep] {n_firings} firings x {n_faces} faces ran in '
              f'{dt:.2f} s; {total_strikes} total face-strikes, peak '
              f'{peak}/{n_faces} struck in a single firing')


if __name__ == '__main__':
    unittest.main()
