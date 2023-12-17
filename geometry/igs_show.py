from OCC.Extend.DataExchange import read_step_file
from OCC.Core.TopoDS import topods
import plotly.graph_objects as go

def read_igs_file(igs_path):
    # Read IGS file
    shape = read_step_file(igs_path)

    # Convert the shape to a TopoDS_Shape
    topo_shape = topods.TopoDS_Shape(shape)

    return topo_shape

def plot_occ_shape(shape):
    # Extract vertices, faces, and edges from the shape
    vertices = []
    faces = []
    edges = []

    # Iterate over the shape to extract information
    explorer = topods.TopoDS_Iterator(shape)
    while explorer.More():
        current = explorer.Value()

        if current.ShapeType() == "VERTEX":
            vertex = topods.TopoDS_Vertex(current)
            pnt = vertex.Pnt()
            vertices.append([pnt.X(), pnt.Y(), pnt.Z()])

        elif current.ShapeType() == "FACE":
            faces.append(current)

        elif current.ShapeType() == "EDGE":
            edges.append(current)

        explorer.Next()

    # Plot the shape using plotly
    fig = go.Figure()

    # Plot vertices
    x, y, z = zip(*vertices)
    fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='markers', marker=dict(size=4), name='Vertices'))

    # Plot edges
    for edge in edges:
        edge_vertices = []
        edge_explorer = topods.TopoDS_Iterator(edge)
        while edge_explorer.More():
            edge_vertex = edge_explorer.Value()
            pnt = topods.TopoDS_Vertex(edge_vertex).Pnt()
            edge_vertices.append([pnt.X(), pnt.Y(), pnt.Z()])
            edge_explorer.Next()

        x, y, z = zip(*edge_vertices)
        x += (x[0],)  # Close the loop
        y += (y[0],)
        z += (z[0],)
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='lines', line=dict(width=2), name='Edges'))

    # Set layout
    fig.update_layout(scene=dict(aspectmode='data'))

    # Show the plot
    fig.show()

if __name__ == "__main__":
    # Replace this path with the actual path to your IGS file
    igs_path = '/home/onur/geometry-read/european-rail-standarts/54E1.IGS'

    try:
        shape = read_igs_file(igs_path)
        plot_occ_shape(shape)
    except Exception as e:
        print(f"Error during file reading or plotting: {e}")

