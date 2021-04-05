# -*- coding: utf-8 -*-
"""
Created on Tue Mar 30 21:08:11 2021

@author: Sergio
"""

import numpy as np
from stl import mesh 
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import pyransac3d as pyrsc
from stl import mesh
import open3d as o3d

###############################mesh object#############################################
mesh_obj= mesh.Mesh.from_file("D:\Sergio_vault\study\Trabajo_grado\piezas_Solidworks\pieza_resina1.stl")
#obtain vertex of mesh_obj
points_obj= np.around(np.unique(mesh_obj.vectors.reshape([int(mesh_obj.vectors.size/3), 3]), axis=0),2)
#points =mesh.vectors.reshape([int(mesh.vectors.size/3), 3])

pcd_obj = o3d.geometry.PointCloud()
pcd_obj.points = o3d.utility.Vector3dVector(points_obj)
o3d.io.write_point_cloud("./data_obj.ply", pcd_obj)
o3d.visualization.draw_geometries([pcd_obj])


circle=pyrsc.Circle()
center,axis,radius,circle_inliers=circle.fit(points_obj,thresh=0.1,maxIteration=9000)
circle_points=points_obj[circle_inliers[0]:circle_inliers[-1]]

#Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize
pcd_circle= o3d.geometry.PointCloud()
pcd_circle.points = o3d.utility.Vector3dVector(circle_points)
o3d.io.write_point_cloud("./data_cylinder.ply", pcd_circle)
o3d.visualization.draw_geometries([pcd_circle])
