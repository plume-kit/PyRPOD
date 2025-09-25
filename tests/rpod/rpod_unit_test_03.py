# Andy Torres
# Embry-Riddle Aeronautical University
# Department of Aerospace Engineering
# Last Changed: 09-18-25

# ========================
# PyRPOD: tests/rpod/rpod_unit_test_03.py
# ========================
# Goal: Capture the exact file outputs of the three JFH printing helpers
# in pyrpod.util.io.file_print without making assertions yet. These files
# will serve as fixtures for future tests to lock current behavior.

import test_header  # adds project root to sys.path
import unittest, os
import numpy as np

from pyrpod.util.io import file_print as fp
from pyrpod.util.io.fs import ensure_dir


class CaptureJFHOutputs(unittest.TestCase):
	def setUp(self):
		# Output directory for captured files (kept within tests tree)
		self.out_dir = os.path.join(os.path.dirname(__file__), 'jfh_outputs')
		ensure_dir(self.out_dir)

		# Deterministic sample data used across all captures
		# Three firings, simple linear times, simple positions, and rotation matrices
		self.t_values = np.array([0.0, 1.5, 3.0])
		self.r = np.array([
			[1.2345, 2.3456, 3.4567],  # x
			[4.5678, 5.6789, 6.7890],  # y
			[7.8901, 8.9012, 9.0123],  # z
		])

		# For print_JFH and print_test_JFH, rot[i] is expected to have attribute .A
		base_rot = np.array([
			[1.0, 0.0, 0.0],
			[0.0, 1.0, 0.0],
			[0.0, 0.0, 1.0],
		])
		self.rot_with_A = [np.array(base_rot) for _ in range(3)]

		# For print_1d_JFH, rot[i][j][k] indexing is used with scientific formatting
		self.rot_array = [base_rot.copy() for _ in range(3)]

	def test_capture_print_JFH(self):
		out_file = os.path.join(self.out_dir, 'print_JFH_output.txt')
		fp.print_JFH(self.t_values, self.r, self.rot_with_A, out_file)
		# No asserts yet; file is created as ground-truth capture
		# Keep a tiny smoke check to ensure file write occurred successfully
		self.assertTrue(os.path.exists(out_file))

	def test_capture_print_test_JFH(self):
		out_file = os.path.join(self.out_dir, 'print_test_JFH_output.txt')
		fp.print_test_JFH(self.t_values, self.r, self.rot_with_A, out_file)
		self.assertTrue(os.path.exists(out_file))

	def test_capture_print_1d_JFH(self):
		out_file = os.path.join(self.out_dir, 'print_1d_JFH_output.txt')
		fp.print_1d_JFH(self.t_values, self.r, self.rot_array, out_file)
		self.assertTrue(os.path.exists(out_file))


if __name__ == '__main__':
	unittest.main()
