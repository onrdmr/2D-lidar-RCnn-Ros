# examples/Python/Basic/pointcloud.py
from ast import arg
import asyncio
import time
import numpy as np
import open3d as o3d
import os
import math
from multiprocessing import Process, Queue
from threading import Thread

async def draw_pcd_basis(fut,filePath):
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(filePath)
    print(pcd)
    print(np.asarray(pcd.points))
    fut.set_result(pcd)

async def getData(filePath):
    loop = asyncio.get_running_loop()

    # Create a new Future object.
    fut = loop.create_future()

    filePath = filePath
    loop.create_task( draw_pcd_basis(fut, filePath) )

    print("waiting for loading")
    await fut
    pcd = fut.result()
    point_cloud_as_array = np.asarray(pcd.points)

    # Voxelization
    print('voxelization')
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,
                                                                voxel_size=0.05)

    # Downsampled Voxel
    
    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample(voxel_size=0.0005)


    print("Recompute the normal of the downsampled point cloud")
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.1, max_nn=30))



    return pcd, voxel_grid, downpcd


def drawWindow(data, window_name='Open 3D', width=300, height=300):
    (o3d.visualization.draw_geometries([data], window_name=window_name, width=width, height=height, left=50, top=50, point_show_normal=False, mesh_show_wireframe=False, mesh_show_back_face=False))

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

def get_mesh_pose(mesh):
    pose = np.eye(4)
    bb = mesh.get_oriented_bounding_box()
    pose[:3, :3] = bb.R
    pose[:3, 3] = bb.center 
    return pose                                   

def create_mesh(width, height, depth):
        return o3d.geometry.TriangleMesh.create_box(width=width,
                                                        height=height,
                                                        depth=depth)

from mpl_toolkits import mplot3d

import matplotlib.pyplot as plt


def draw_odometry(posVec):
       
    fig = plt.figure()

    posVec = np.array(posVec).transpose()
    print("-----")

    ax = plt.axes(projection='3d')
    ax.scatter3D(posVec[2,:],posVec[0,:],posVec[1,:],)
    

    #ax.quiver(posVec[0,:], posVec[1,:], posVec[2,:], posVec[3,:], posVec[4,:], posVec[5,:], normalize=True)

    plt.show(fig)


if __name__ == "__main__":
    dirPath = "/mnt/c/Users/filiz/Desktop/Ara/kitti_velodyne_bin_to_pcd/dataset/"
    posePath = "/mnt/c/Users/filiz/Desktop/Ara/kitti_velodyne_bin_to_pcd/poses/"
    pose = "00.txt"


    filePath = posePath+pose
    file = open(filePath,'r')

    lines = file.readlines()
    print(len(lines))
    posVec = []
    homogeneousCoords = []
    for line in lines:
        # print(line)
        pose , _ = get6DoFPose(line)
        posVec.append([pose])
        homogeneousCoords.append([_])
        print(pose)
 
    x = Process(target=draw_odometry ,args=(posVec,))
    x.start()
    
    file.close()
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    final_pointcloud = o3d.geometry.PointCloud()
    
    mesh = create_mesh(0.5, 0.5, 0.5)
    transformItr = 0
    print("----------")
    # print(homogeneousCoords[transformItr][0])

    # mesh.transform(homogeneousCoords[transformItr][0])
    
    print(np.array([posVec[transformItr][0][0],posVec[transformItr][0][1],posVec[transformItr][0][2]]))
    mesh.translate(np.array([posVec[transformItr][0][0],posVec[transformItr][0][1],posVec[transformItr][0][2]],dtype=np.float128),relative=False)

    # print(np.array([posVec[transformItr][0][0],posVec[transformItr][0][1],posVec[transformItr][0][2]]).transpose())


    curr_pose = get_mesh_pose(mesh)

    print("current pose of the mesh")
    print(repr(curr_pose))
    print(mesh.get_center())

    # assert(1=""=2)
    firstAnimate=True
    for dir in os.listdir(dirPath):

        pcd, voxel_grid, downpcd = asyncio.run(getData(dirPath + dir))


        pointcloud_as_array = np.asarray(pcd.points)
        Z = 1
        d = 1

        # Go through each point in the array for "slicing"
        final_pointcloud_array = []
        for point in pointcloud_as_array:
            if Z - d < point[2] < Z + d :
                final_pointcloud_array.append(point)

        # Create Open3D point cloud object from array
        final_pointcloud.points = o3d.utility.Vector3dVector(final_pointcloud_array)
        # print(final_pointcloud)
        # print(np.asarray(pcd.points))

        if(firstAnimate == True):
            vis.add_geometry(final_pointcloud)
            vis.add_geometry(mesh)
            vis.poll_events()
            vis.update_renderer()
            firstAnimate = False
            continue
        


        vis.update_geometry(final_pointcloud)
        vis.poll_events()
        vis.update_renderer()
        vis.update_geometry(mesh)
        vis.poll_events()
        vis.update_renderer()
        
        transformItr+=1
        mesh.transform(homogeneousCoords[transformItr][0] @ np.linalg.inv(homogeneousCoords[transformItr-1][0]))
        print(np.array([posVec[transformItr][0][0],posVec[transformItr][0][1],posVec[transformItr][0][2]]))

        # mesh.translate(np.array([posVec[transformItr][0][0],posVec[transformItr][0][1],posVec[transformItr][0][2]]),relative=False)

        curr_pose = get_mesh_pose(mesh)

        print("current pose of the mesh")
        print(repr(curr_pose))

        print(mesh.get_center())

        if transformItr == 180 :
            break 
 
        # p = Process(target=drawWindow, args=(pcd,"point clssoud"))
        # p2 = Process(target=drawWindow, args=(voxel_grid,"voxel grid"))
        # p3 = Process(target=drawWindow, args=(downpcd,"down sampled"))
        # p4 = Process(target=drawWindow, args=(final_pointcloud,"sliced Z = 2"))


        # p.start()
        # p2.start()
        # p3.start()
        # p4.start()

        # p4.join()
        # p3.join()
        # p2.join()
        # p.join()

    # x.join()
    vis.destroy_window()