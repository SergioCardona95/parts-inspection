# -*- coding: utf-8 -*-
"""
Created on Tue Mar 30 17:41:46 2021

@author: Sergio
"""

import numpy as np
from stl import mesh 
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import open3d.ml as ml3d
from stl import mesh
import open3d as o3d

###############################mesh object#############################################
mesh_obj = mesh.Mesh.from_file("/home/sergio/Documentos/Study/piezas_Freecad/pieza1.stl")
#obtain vertex of mesh_obj
points_obj= np.around(np.unique(mesh_obj.vectors.reshape([int(mesh_obj.vectors.size/3), 3]), axis=0),2)
pcd_stl= o3d.geometry.PointCloud()
pcd_stl.points = o3d.utility.Vector3dVector(points_obj)
o3d.io.write_point_cloud("./data.ply", pcd_stl)
o3d.visualization.draw_geometries([pcd_stl])
normals=mesh_obj.normals
vis = ml3d.vis.Visualizer()
vis.visualize_dataset(pcd_stl, 'all', indices=range(100))