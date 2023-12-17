from stl import mesh
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

stl_file_path = '/home/onur/.pcg/meshes/wbBCsJoGrk.stl'
#stl_file_path = '/home/onur/.pcg/meshes/rail_defne.stl'
mesh_data = mesh.Mesh.from_file(stl_file_path)

fig = plt.figure()
ax = mplot3d.Axes3D(fig)


ax.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh_data.vectors, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.5))

ax.auto_scale_xyz(mesh_data.x.flatten(), mesh_data.y.flatten(), mesh_data.z.flatten())

# Set axis labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()

vertices = mesh_data.vectors
faces = mesh_data.faces

fist_vertex = vertices[0][0]
first_face = faces[0]

print("First 5 vertices")
for i in range(5):
    print(vertices[i])

scale_factor = 2.0
mesh_data.x *= scale_factor
mesh_data.y *= scale_factor
mesh_data.z *= scale_factor



fig = plt.figure()
ax = mplot3d.Axes3D(fig)


ax.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh_data.vectors, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.5))

plt.show()

new_stl_file_path = '/home/onur/.pcg/onur.stl'
mesh_data.save(new_stl_file_path)



