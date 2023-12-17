import meshio
import tripy

def convert_igs_to_stl(igs_path, stl_path):
    # Read IGS file
    mesh = meshio.read(igs_path)

    # Extract vertices and faces
    vertices = mesh.points
    faces = []
    for cell in mesh.cells:
        if cell.type == 'triangle':
            faces.extend(cell.data)

    # Triangulate polygons (if needed)
    for i, face in enumerate(faces):
        if len(face) > 3:
            triangles = tripy.earclip(face)
            faces[i] = triangles

    # Flatten faces list
    faces = [vertex for sublist in faces for vertex in sublist]

    # Write to STL file
    meshio.write_points_cells(stl_path, vertices, [("triangle", faces)])

if __name__ == "__main__":
    # Replace these paths with your IGS and STL file paths
    igs_path = '/home/onur/geometry-read/european-rail-standarts/54E1.IGS'
    stl_path = '/home/onur/geometry-read/file.stl'

    try:
        convert_igs_to_stl(igs_path, stl_path)
        print(f"Conversion successful. STL file saved at: {stl_path}")
    except Exception as e:
        print(f"Error during conversion: {e}")

