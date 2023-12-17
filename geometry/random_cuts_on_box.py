import random
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeSphere
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.gp import gp_Pnt, gp_Ax2, gp_Dir, gp_XOY, gp_Trsf
from OCC.Display.SimpleGui import init_display

from OCC.Core.BRepAlgoAPI import BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse, BRepAlgoAPI_Common

"""
def make_random_cuts(box_shape, num_cuts=3):
    # Create a transformation for each random cut
    for _ in range(num_cuts):
        # Randomly choose a point to cut along X, Y, or Z axis
        cut_point = gp_Pnt(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
        
        # Randomly choose a vector direction for the cut
        cut_vector = gp_Vec(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
        
        # Create a transformation to perform the cut
        transform = gp_Trsf()
        transform.SetTranslation(cut_vector)
        
        # Apply the transformation to the box
        box_transform = BRepBuilderAPI_Transform(box_shape, transform, True)
        box_shape = box_transform.Shape()

    return box_shape

"""


def get_vertex_coordinates(shape):
    vertex_coordinates = []
    vertex_iterator = TopExp_Explorer(shape, TopAbs_VERTEX)
    
    while vertex_iterator.More():
        vertex = BRep_Tool.Pnt(vertex_iterator.Current())
        vertex_coordinates.append((vertex.X(), vertex.Y(), vertex.Z()))
        vertex_iterator.Next()
    
    return vertex_coordinates

def add_shapes_at_vertices(original_shape, shape_generator, num_shapes=3):
    vertex_coordinates = get_vertex_coordinates(original_shape)

    for _ in range(num_shapes):
        # Randomly choose a vertex
        vertex = random.choice(vertex_coordinates)

        # Create a shape using the provided shape_generator function
        new_shape = shape_generator()

        # Translate the shape to the position of the selected vertex
        translation = gp_Pnt(*vertex) - gp_Pnt(0, 0, 0)
        transform = BRepBuilderAPI_Transform(new_shape, translation, True)
        new_shape = transform.Shape()

        # Combine the new shape with the original using Boolean union
        original_shape = BRepBuilderAPI_Transform(original_shape, translation, True).Shape()
        original_shape = BRepBuilderAPI_Transform(original_shape, gp_Pnt(0, 0, 0), True).Shape()
        original_shape = BRepBuilderAPI_Transform(original_shape, gp_Pnt(*vertex), True).Shape()
        original_shape = BRepBuilderAPI_Transform(original_shape, gp_Pnt(0, 0, 0), True).Shape()

    return original_shape


def get_vertex_coordinates(shape):
    vertex_coordinates = []
    vertex_iterator = TopExp_Explorer(shape, TopAbs_VERTEX)
    
    while vertex_iterator.More():
        vertex = BRep_Tool.Pnt(vertex_iterator.Current())
        vertex_coordinates.append((vertex.X(), vertex.Y(), vertex.Z()))
        vertex_iterator.Next()
    
    return vertex_coordinates


def add_random_bumps(box_shape, num_bumps=3):
    for _ in range(num_bumps):
        # Randomly choose dimensions for the bump
        bump_dimensions = (
            random.uniform(0.2, 0.8),
            random.uniform(0.2, 0.8),
            random.uniform(0.2, 0.8)
        )

        # Randomly choose a position for the bump
        bump_position = (
            random.uniform(-0.8, 0.8),
            random.uniform(-0.8, 0.8),
            random.uniform(-0.8, 0.8)
        )

        # Create a bump at a specific position and with specific dimensions
        bump_box = BRepPrimAPI_MakeBox(gp_Pnt(*bump_position), *bump_dimensions).Shape()

        # Add the bump to the original box using Boolean union
        box_shape = BRepAlgoAPI_Fuse(box_shape, bump_box).Shape()

    return box_shape


def add_bumps_on_surface(box_shape, num_bumps=3):
    for _ in range(num_bumps):
        # Randomly choose dimensions for the bump
        bump_radius = random.uniform(0.2, 0.5)

        # Randomly choose a position for the bump
        bump_position = (
            random.uniform(2, 2),
            random.uniform(2, 2),
            random.uniform(2, 2)
        )

        # Create a spherical bump at a specific position and with a specific radius
        bump_sphere = BRepPrimAPI_MakeSphere(0.1,bump_radius).Shape()

        # Translate the sphere to the desired position
        translation = gp_Trsf()
        translation.SetTranslation(gp_Pnt(random.uniform(-0.8, 0.8), random.uniform(-0.8, 0.8), random.uniform(-0.8, 0.8)), gp_Pnt(*bump_position))
        # Combine the bump with the original box using Boolean union
        box_shape = BRepAlgoAPI_Fuse(box_shape, bump_sphere).Shape()

    return box_shape

def add_intersecting_bumps(box_shape, num_bumps=3):
    for _ in range(num_bumps):
        # Randomly choose dimensions for the bump
        bump_dimensions = (
            random.uniform(0.2, 0.8),
            random.uniform(0.2, 0.8),
            random.uniform(0.2, 0.8)
        )

        # Randomly choose a position for the bump
        bump_position = (
            random.uniform(-0.8, 0.8),
            random.uniform(-0.8, 0.8),
            random.uniform(-0.8, 0.8)
        )

        # Create a bump at a specific position and with specific dimensions
        bump_box = BRepPrimAPI_MakeBox(gp_Pnt(*bump_position), *bump_dimensions).Shape()

        # Perform a common intersection operation to intersect the bump with the original box
        box_shape = BRepAlgoAPI_Common(box_shape, bump_box).Shape()

    return box_shape


def make_random_cuts(box_shape, num_cuts=3):
    for _ in range(num_cuts):
        # Randomly choose dimensions for the cutting box
        cut_box_dimensions = (
            random.uniform(0.2, 0.8),
            random.uniform(0.2, 0.8),
            random.uniform(0.2, 0.8)
        )

        # Randomly choose a position for the cutting box
        cut_box_position = (
            random.uniform(-0.8, 0.8),
            random.uniform(-0.8, 0.8),
            random.uniform(-0.8, 0.8)
        )

        # Create a cutting box with a specific position and dimensions
        cut_box_axis = gp_Ax2(gp_Pnt(*cut_box_position), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0))  # Adjust the directions accordingly
        cut_box = BRepPrimAPI_MakeBox(cut_box_axis, *cut_box_dimensions).Shape()

        # Subtract the cutting box from the original box
        box_shape = BRepAlgoAPI_Cut(box_shape, cut_box).Shape()

    return box_shape


def display(shape):
    
    display, start_display, add_menu, add_function_to_menu = init_display()
    display.DisplayShape(shape, update=True)
    start_display()


def main():
    # Create a box
    box = BRepPrimAPI_MakeBox(2, 2, 2).Shape()

    # Make random cuts on the box
    #deformed_box = make_random_cuts(box, num_cuts=3)
    #bumped_box = add_random_bumps(box, num_bumps=3)



    bumped_box = add_bumps_on_surface(box, num_bumps=3)

    display(bumped_box)

    # Visualize the original and deformed boxes (you can use pythonocc's visualization tools)
    # ...

if __name__ == "__main__":
    main()
