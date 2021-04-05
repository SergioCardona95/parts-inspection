# -*- coding: utf-8 -*-
"""
Created on Mon Mar 29 10:50:10 2021

@author: Sergio
"""

import trimesh
import numpy as np 
import open3d as o3d
from pyntcloud import PyntCloud as pytc
import pyransac3d as pyrsc


#Read points cloud 
print("Load a ply point cloud, print it, and render it")
mesh_stl = o3d.io.read_triangle_mesh("D:\Sergio_vault\study\Trabajo_grado\piezas_Solidworks\pieza_ransac\cylinder1_ransac.stl")
cloud_stl= mesh_stl.sample_points_poisson_disk(20000)
o3d.visualization.draw_geometries([cloud_stl])
#get plane of points cloud  
points_stl=np.asarray(cloud_stl.points)
plane1 = pyrsc.Plane()
best_eq, best_inliers = plane1.fit(points_stl,0.05,minPoints=10,maxIteration=1000)
plane_points=points_stl[best_inliers[0]:best_inliers[-1]]


    


#Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize
pcd_plane= o3d.geometry.PointCloud()
pcd_plane.points = o3d.utility.Vector3dVector(plane_points)
o3d.io.write_point_cloud("./data_plane.ply", pcd_plane)
o3d.visualization.draw_geometries([pcd_plane])

#get  cylinder
cylinder = pyrsc.Cylinder()
center,axis,radius,cylinder_inliers=cylinder.fit(points_stl,thresh=0.005,maxIteration=9000)
cylinder_points=points_stl[cylinder_inliers[0]:cylinder_inliers[-1]]

#Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize
pcd_cylinder= o3d.geometry.PointCloud()
pcd_cylinder.points = o3d.utility.Vector3dVector(cylinder_points)
o3d.io.write_point_cloud("./data_cylinder.ply", pcd_cylinder)
o3d.visualization.draw_geometries([pcd_cylinder])


vtx = np.asanyarray(points_stl.get_vertices())




