from stl import mesh
import numpy as np
import os
import configparser

from pyevtk.vtk import VtkTriangle, VtkQuad
from pyrpod.util.stl.stl import convert_stl_to_vtk

class Vehicle:
    """
        Class responsible for handling visiting vehicle data.

        Includes surface mesh and thruster configuration data.

        Attributes
        ----------
        config : ConfigParser
            Object responsible for reading data from the provided configuration file.

        case_dir : str
            Path to case directory. Used to store data and results for a specific scenario.

        Methods
        -------
        convert_stl_to_vtk(cellData, mesh)
            Converts STL mesh to a VTK file and attaches surface properties supplied in cellData.
    """
    def __init__(self, case_dir):
        self.case_dir = case_dir
        config = configparser.ConfigParser()
        config.read(self.case_dir + "config.ini")
        self.config = config

    def set_stl(self):
        """
            Reads in Vehicle surface mesh from STL file.

            Parameters
            ----------
            path_to_stl : str
                file location for Vehicle's surface mesh using an STL file.

            Returns
            -------
            Method doesn't currently return anything. Simply sets class members as needed.
            Does the method need to return a status message? or pass similar data?
        """
        path_to_stl = self.case_dir + 'stl/' + self.config['vv']['stl_lm']

        self.mesh = mesh.Mesh.from_file(path_to_stl)
        self.path_to_stl = path_to_stl
        return

    def convert_stl_to_vtk_strikes(self, path_to_vtk, cellData, mesh):
        """
            Converts STL mesh to a VTK file and attaches surface properties supplied in cellData.

            Parameters
            ----------
            cellData : dict<np.array>
                Dictionary storing arrays that contain all surface properties.

            Returns
            -------
            Method doesn't currently return anything. Simply saves data to files as needed.
            Does the method need to return a status message? or pass similar data?
        """

        # if self.mesh == None and mesh == None:
        #     print("mesh is not set. Please load using self.set_stl() method")
        #     return

        surface = self.mesh if mesh is None else mesh
        # Ensure output directory exists and derive base filename
        out_dir = self.case_dir + 'results/'
        convert_stl_to_vtk(surface, out_dir, filename=path_to_vtk, cellData=cellData)
        return


    def convert_stl_to_vtk(self):
        """
            Converts STL mesh to a VTK file and attaches surface properties supplied in cellData.

            Parameters
            ----------
            cellData : dict<np.array>
                Dictionary storing arrays that contain all surface properties.

            Returns
            -------
            Method doesn't currently return anything. Simply saves data to files as needed.
            Does the method need to return a status message? or pass similar data?
        """

        surface = self.mesh
        out_dir = self.case_dir + 'results/'
        # Default cell data
        cellData = {"strikes": np.zeros(len(surface.vectors))}
        # Derive filename from path_to_stl if available
        filename = None
        if hasattr(self, 'path_to_stl') and self.path_to_stl:
            filename = self.path_to_stl.split('/')[-1].split('.')[0]
        convert_stl_to_vtk(surface, out_dir, filename=filename, cellData=cellData)
        return