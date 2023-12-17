import OCC.Core.gp as gp
from OCC.Extend.DataExchange import read_iges_file, write_stl_file
from OCC.Core.gp import gp_Vec, gp_Trsf,gp_Pnt2d
from OCC.Extend.DataExchange import read_iges_file
from OCC.Core.TopoDS import topods
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_VERTEX
from OCC.Core.TopLoc import TopLoc_Location
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Display.SimpleGui import init_display

from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeSphere

from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.BRep import BRep_Builder

from OCC.Core.TopoDS import TopoDS_Iterator

import random

def convert_iges_to_stl(iges_path, stl_path):
    # Read IGES file
    shape = read_iges_file(iges_path)

    # Export the shape to STL file
    write_stl_file(shape, stl_path)

def apply_random_deformation(shape, max_deformation=0.1):
    # Create a transformation vector with random displacements
    deformation_vector = gp_Vec(random.uniform(-max_deformation, max_deformation),
                                random.uniform(-max_deformation, max_deformation),
                                random.uniform(-max_deformation, max_deformation))

    # Iterate over all vertices in the shape and apply the deformation
    explorer = TopExp_Explorer(shape, TopAbs_VERTEX)
    while explorer.More():
        vertex = topods.Vertex(explorer.Current())
        transformation =  gp_Trsf()
        transformation.SetTransformation(deformation_vector)

        
        explorer.Next()

    return shape

def display(shape):
    
    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(shape, update=True)
    start_display()

def print_all_brep_shapes(compound_shape):
    # Create a TopoDS_Iterator to iterate through the subshapes of the compound
    it = TopoDS_Iterator(compound_shape)

    # Iterate through the subshapes
    index = 1
    while it.More():
        subshape = it.Value()
        print(f"BRep Shape {index}: {subshape.ShapeType()}")

        it = TopoDS_Iterator(subshape)
        
        while it.More():
            subshape = it.Value()
            print(f"BRep Shape {index}: {subshape.ShapeType()}")
            it.Next()
            index += 1

            # Get the surface from the face
            surface_adaptor = BRepAdaptor_Surface(subshape)

            # Get the domain (parameter space) of the surface
            u_min, u_max, v_min, v_max = surface_adaptor.FirstUParameter(), surface_adaptor.LastUParameter(), surface_adaptor.FirstVParameter(), surface_adaptor.LastVParameter()

            # Generate random parameter values within the domain
            u = random.uniform(u_min, u_max)
            v = random.uniform(v_min, v_max)

            # Create a gp_Pnt2d with the random parameter values
            #point_2d = gp_Pnt2d(u, v)

            # Map the 2D point to the 3D point on the surface
            #location = TopLoc_Location()
            point_3d = surface_adaptor.Value(u, v)

            # Generate a random radius for the sphere
            radius = random.uniform(0.1, 1.0)  # Adjust the range as needed

            # Create a sphere with the random center and radius
            sphere = BRepPrimAPI_MakeSphere(point_3d, radius).Shape()
            builder = BRep_Builder()
            
            builder.Add(compound_shape, sphere)


        # You can further process or analyze the subshape if needed
        
        # Move to the next subshape
        print("reading compund object finished")


if __name__ == "__main__":
    # Replace these paths with the actual paths to your IGES and STL files
    iges_path = '/home/onur/geometry-read/european-rail-standarts/54E1.IGS'
    stl_path = '/home/onur/geometry-read/file.stl'

    shape = read_iges_file(iges_path)

    print_all_brep_shapes(shape)

    from OCC.Core.TopAbs import TopAbs_COMPOUND, TopAbs_SHAPE, TopAbs_FACE

    print(TopAbs_COMPOUND + TopAbs_SHAPE + TopAbs_FACE)  # Output: 0

    display(shape)

    #deformed_shape = apply_random_deformation(shape, max_deformation=0.1)

    try:
        convert_iges_to_stl(iges_path, stl_path)
        print(f"Conversion successful. STL file saved at: {stl_path}")
    except Exception as e:
        print(f"Error during conversion: {e}")
