# -*- coding: utf-8 -*-
"""
Created on Thu Dec 31 12:04:57 2020

@author: sergi
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
import numpy as np
from pyntcloud import PyntCloud as pytc
mesh_stl = o3d.io.read_triangle_mesh("D:\Sergio_vault\study\Trabajo_grado\piezas_Solidworks\pieza_resina1.stl")
cloud_stl= mesh_stl.sample_points_poisson_disk(200000)
array_stl=np.asarray(cloud_stl.points)
save_cloud_stl=o3d.io.write_point_cloud("./data_meshStl.ply", cloud_stl)
# you can plot and check
o3d.visualization.draw_geometries([cloud_stl])
#o3d.visualization.draw_geometries([pointcloud])
#o3d.geometry.PointCloud
geonetry_registre=o3d.geometry.PointCloud.get_geometry_type(cloud_stl)
# Calculate volume of cloud points
volume_stl=pytc.from_file("D:\Sergio_vault\study\Trabajo_grado\codigos\python\data_meshStl.ply")
convex_hull_volume_stl_id = volume_stl.add_structure("convex_hull")
convex_hull_stl = volume_stl.structures[convex_hull_volume_stl_id]
volume_cloud_stl=convex_hull_stl.volume

#####mesh get object real######
mesh_obj= o3d.io.read_triangle_mesh("D:\Sergio_vault\study\Trabajo_grado\piezas_Solidworks\pieza_resina1_error.stl")
cloud_obj = mesh_obj.sample_points_poisson_disk(200000)
array_obj=np.asarray(cloud_obj.points)



comparison=o3d.geometry.PointCloud.compute_point_cloud_distance(cloud_stl,cloud_obj)
comparison=np.asarray(comparison)
maximo=np.amax(comparison)
valueMean=np.mean(comparison)
# ##########Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize####
pcd_stl= o3d.geometry.PointCloud()
pcd_stl.points = o3d.utility.Vector3dVector(array_stl)
o3d.io.write_point_cloud("./data.ply", pcd_stl)
o3d.visualization.draw_geometries([pcd_stl])
#Pass numpy array to Open3D.o3d.geometry.PointCloud and visualize  of object
pcd_obj = o3d.geometry.PointCloud()
pcd_obj.points = o3d.utility.Vector3dVector(array_obj)
o3d.io.write_point_cloud("./data_obj.ply", pcd_obj)
o3d.visualization.draw_geometries([pcd_obj])
######### modulos prueba 
# number_of_points=2000
# dimension=o3d.geometry.TriangleMesh.dimension(mesh)
# center_geometry=o3d.geometry.TriangleMesh.get_center(mesh)
# type_geometry=o3d.geometry.TriangleMesh.get_geometry_type(mesh)
# limmax_coor=o3d.geometry.TriangleMesh.get_max_bound(mesh)
# num_points_mues=o3d.geometry.TriangleMesh.sample_points_poisson_disk(mesh, number_of_points, init_factor=5, pcl=None, use_triangle_normal=False, seed=- 1)



#traslate points cloud
#o3d.geometry.PointCloud.traslate()
