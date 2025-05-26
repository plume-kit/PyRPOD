# Andy Torres, Pearce Patterson
# University of Central Florida
# Department of Mechanical and Aerospace Engineering
# Last Changed: 03-28-24

# ========================
# PyRPOD: tests/rpod/rpod_unit_test_01.py
# ========================
# Test case for converting STL data to VTK data.
# This is accomplished by checking for the proper data format of VTK files.
import test_header
import unittest, os
import numpy as np
import meshio
from pyrpod.vehicle import Vehicle
from pyrpod.mission import MissionPlanner, MissionEnvironment

class STLtoVTKChecks(unittest.TestCase):
    def test_stl_to_vtk(self):
        # 1. Setup
        case_dir = '../case/rpod/stl_to_vtk/'
        vtk_file_path = os.path.join(case_dir, 'results', 'cylinder.vtu')

        # Load mission planner and vehicle object data for analysis
        me = MissionEnvironment.MissionEnvironment(case_dir)
        mp = MissionPlanner.MissionPlanner(me)
        v = Vehicle.Vehicle(case_dir)

        # 2. Execute
        v.set_stl()  # Read STL surface data
        v.convert_stl_to_vtk()  # Convert STL to VTK and save

        # Load cylinder STL and calculate cell and point counts
        cylinder_stl = v.mesh
        num_stl_cells = len(cylinder_stl.vectors)
        num_stl_points = sum(map(len, [cylinder_stl.v2, cylinder_stl.v1, cylinder_stl.v0]))

        # Ensure STL mesh is non-empty
        self.assertGreater(num_stl_cells, 0, "STL file contains no cells.")
        self.assertGreater(num_stl_points, 0, "STL file contains no points.")

        # Load VTK file and calculate cell and point counts
        self.assertTrue(os.path.exists(vtk_file_path), "VTK file was not created.")
        cylinder_vtu = meshio.read(vtk_file_path)
        num_vtu_points = len(cylinder_vtu.points)
        num_vtu_cells = len(cylinder_vtu.cells_dict.get('triangle', []))

        # Ensure VTK mesh is non-empty
        self.assertGreater(num_vtu_cells, 0, "VTK file contains no cells.")
        self.assertGreater(num_vtu_points, 0, "VTK file contains no points.")

        # 3. Assertions
        # Assert cell and point counts match between STL and VTK
        self.assertEqual(num_vtu_cells, num_stl_cells, "Cell counts do not match between STL and VTK.")
        self.assertEqual(num_vtu_points, num_stl_points, "Point counts do not match between STL and VTK.")

        # Verify file extensions
        # self.assertTrue(cylinder_stl.filename.endswith('.stl'), "Input file does not have .stl extension.")
        self.assertTrue(vtk_file_path.endswith('.vtu'), "Output file does not have .vtu extension.")

        # Validate mesh bounds
        stl_bounds = [np.min(cylinder_stl.vectors, axis=(0, 1)), np.max(cylinder_stl.vectors, axis=(0, 1))]
        vtu_bounds = [np.min(cylinder_vtu.points, axis=0), np.max(cylinder_vtu.points, axis=0)]
        self.assertTrue(np.allclose(stl_bounds, vtu_bounds, atol=1e-5), "Mesh bounds do not match between STL and VTK.")

        # Ensure VTK cells are of type 'triangle'
        self.assertIn('triangle', cylinder_vtu.cells_dict, "VTK cells are not of type 'triangle'.")

        # Check surface normals consistency if available
        if hasattr(cylinder_stl, 'normals') and 'Normals' in cylinder_vtu.point_data:
            stl_normals = np.linalg.norm(cylinder_stl.normals, axis=1)
            vtu_normals = np.linalg.norm(cylinder_vtu.point_data['Normals'], axis=1)
            self.assertTrue(np.allclose(stl_normals, vtu_normals, atol=1e-5), "Surface normals do not match between STL and VTK.")

if __name__ == '__main__':
    unittest.main()
