# -*- coding: utf-8 -*-
"""
Created on Fri Mar 19 18:37:57 2021

@author: Sergio
"""

 

import numpy as np
import open3d
 
def load_ply_point_cloud(filename,voxel_size=0.5):
    print("Load a ply point cloud, print it, and render it")
    mesh= open3d.io.read_triangle_mesh(filename)
    voxel_mesh = open3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh,voxel_size)
    return mesh,voxel_mesh
 
file = "D:\Sergio_vault\study\Trabajo_grado\piezas_Solidworks\pieza_resina1.stl"
 
size = 0.1
mesh,voxel_mesh= load_ply_point_cloud(file,size)
open3d.visualization.draw_geometries([mesh])
 
# Punto de nube
 
points_ar = []
for x in voxel_mesh.voxel_size:
    points_ar.append(x.grid_index)
    points = np.array(points_ar)
    pcd = open3d.geometry.PointCloud()
pcd.points = open3d.utility.Vector3dVector(points)
 
 # Visualización
open3d.visualization.draw_geometries([pcd])
 