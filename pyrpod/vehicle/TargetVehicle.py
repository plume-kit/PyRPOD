from stl import mesh
from pyrpod.vehicle.Vehicle import Vehicle
from pyrpod.util.io.fs import resolve_asset_path

class TargetVehicle(Vehicle):
    """
        Class responsible for handling visiting vehicle data.

        Includes surface mesh and thruster configuration data.

        Attributes
        ----------
        Includes attributes defined in parent class "Vehicle".

        path_to_stl : str
            file location for Vehicle's surface mesh using an STL file.

        mesh : stl.mesh.Mesh
            Contains surface mesh using data read from STL file.

        Methods
        -------
        set_stl()
            Read in thruster configuration data from the provided file path.

        set_stl_elements()
            Read in thruster configuration data from the provided file path.

    """

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
        path_to_stl = resolve_asset_path(self.case_dir, 'stl', self.config['tv']['stl'])
        meshes = mesh.Mesh.from_multi_file(path_to_stl)
        self.mesh = next(meshes)
        #self.mesh = next(meshes)
        self.path_to_stl = path_to_stl
        return

    def set_stl_elements(self):
        """
            place holder method now. A strech goal could be to 
            load in a multi surface stl file which accounts for 
            different componoents of the Gateway to impinge upon.

        """
        print('')
        return

    def set_v_ida(self, v_ida):
        self.v_ida = v_ida

    def set_r_o(self, r_o):
        self.r_o = r_o