# examples/Python/Basic/pointcloud.py
from ast import arg

import numpy as np
import open3d as o3d
import os
import math
from mpl_toolkits import mplot3d

import matplotlib.pyplot as plt


def get6DoFPose(line):
    rotationTranslation = line.split()
    print(rotationTranslation)
    
    
    homogenousCoord = np.array(rotationTranslation, dtype=np.float64).reshape((3,4))
    homogenousCoord = np.append(homogenousCoord, [[0,0,0,1]], axis=0)

    print(homogenousCoord)
    R = homogenousCoord[0:3,0:3]
    pos = homogenousCoord[0:3,3]
    print(R , pos)
    
    # assert(1==3)

    
    angles = rotationMatrixToEulerAngles(R)

    return np.concatenate((pos, angles)), homogenousCoord

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z], dtype=np.float32)



if __name__ == "__main__":
    dirPath = "/mnt/c/Users/filiz/Desktop/Ara/kitti_velodyne_bin_to_pcd/dataset_ring_5/"
    posePath = "/mnt/c/Users/filiz/Desktop/Ara/kitti_velodyne_bin_to_pcd/poses/"
    pose = "00.txt"


    filePath = posePath+pose
    file = open(filePath,'r')

    vis = o3d.visualization.Visualizer()
    vis.create_window()

    lines = file.readlines()
    file.close()


    vis = o3d.visualization.Visualizer()
    vis.create_window()

    for i,filename in enumerate(os.listdir(dirPath)):

        # mesh.translate(np.array([posVec[i][0][0],posVec[i][0][1],posVec[i][0][2]],dtype=np.float128),relative=False)

        pcd = o3d.io.read_point_cloud(dirPath+filename)

        vis.add_geometry(pcd)
        vis.run()

        vis.poll_events()
        vis.update_renderer()

        print(filename)
        vis.clear_geometries()
        

        # o3d.visualization.draw_geometries([pcd], )
    vis.destroy_window()
    
