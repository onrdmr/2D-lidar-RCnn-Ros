import random

from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.GeomAPI import GeomAPI_ProjectPointOnSurf

from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Fuse
from OCC.Core.TopoDS import topods
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_VERTEX, TopAbs_FACE
from OCC.Core.BRep import BRep_Tool
from OCC.Core.gp import gp_Pnt, gp_Vec, gp_Trsf

from OCC.Display.SimpleGui import init_display

from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform

def get_surface_vertices(shape):
    surface_vertices = []
    vertex_iterator = TopExp_Explorer(shape, TopAbs_FACE)

    while vertex_iterator.More():
        face = topods.Face(vertex_iterator.Current())
        surface_vertices.append(face)
        vertex_iterator.Next()

    return surface_vertices

def add_bumps_on_surface(original_shape, shape_generator, num_bumps=3):
    surface_vertices = get_surface_vertices(original_shape)

    for _ in range(num_bumps):
        # Randomly choose a vertex on the surface
        surface_vertex = random.choice(surface_vertices)

        # Get the coordinates of the vertex

        x = random.uniform(-2, 2)  # Replace with your desired range
        y = random.uniform(-2, 2)
        z = random.uniform(-2, 2)

        # Create and return a gp_Pnt with the random coordinates
        deneme =  gp_Pnt(x, y, z)
        
        surface = BRepAdaptor_Surface(surface_vertex, True)


        projection = GeomAPI_ProjectPointOnSurf(deneme, surface_vertex.GetHandle())
        projected_point = projection.NearestPoint()

        x = projected_point.X()
        Y = projected_point.Y()
        Z = projected_point.Z()
        

        # Create a shape using the provided shape_generator function
        new_shape = shape_generator()

        # Calculate the translation vector between the original and new shape
        translation_vector = gp_Trsf()
        translation_vector.SetTranslation(gp_Pnt(0, 0, 0), vertex_coordinates)
        

        # Translate the new shape to the position of the selected vertex
        transform = BRepBuilderAPI_Transform(new_shape, translation_vector, True)
        new_shape = transform.Shape()

        # Combine the new shape with the original using Boolean union
        original_shape = BRepAlgoAPI_Fuse(original_shape, new_shape).Shape()

    return original_shape

def display(shape):
    
    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(shape, update=True)
    start_display()


def main():
    # Create your original shape (replace this with your own shape creation logic)
    original_shape = BRepPrimAPI_MakeSphere(1).Shape()

    # Define a function to generate the desired shape (e.g., a sphere)
    def sphere_generator():
        radius = random.uniform(0.1, 0.5)
        return BRepPrimAPI_MakeSphere(radius).Shape()

    # Add random bumps on the surface of the original shape
    shape_with_bumps = add_bumps_on_surface(original_shape, sphere_generator, num_bumps=3)

    display(shape_with_bumps)

    # Visualize the original shape and the shape with added bumps
    # ...

if __name__ == "__main__":
    main()
