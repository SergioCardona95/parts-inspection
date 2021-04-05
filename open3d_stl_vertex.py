# -*- coding: utf-8 -*-
"""
Created on Sat Jan 30 18:30:28 2021

@author: sergi
"""

from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import getVertices as vt 
from stl import mesh
import open3d as o3d
from math import cos,sin
###############################mesh object#############################################

mesh_obj = o3d.io.read_triangle_mesh("D:\Sergio-Pc\Trabajo_grado\piezas_Solidworks\pieza_resina1_error.stl")
dirObj=str("D:\Sergio-Pc\Trabajo_grado\piezas_Solidworks\pieza_resina1_error.stl")

Obj_vt,cloudObj_vt=vt.Vertices(dirObj)  # get vertices  
#points =mesh.vectors.reshape([int(mesh.vectors.size/3), 3])

####tranformada aplicando open3d###
matriz_trans=np.float64([[1,0,0,0],
                         [0,1,0,0],
                         [0,0,1,0],
                         [0,0,0,1]])

o3d.geometry.Geometry3D.transform(mesh_obj,matriz_trans)
o3d.visualization.draw_geometries([mesh_obj,cloudObj_vt])

###########################mesh stl##################################################
mesh_stl=o3d.io.read_triangle_mesh("D:\Sergio-Pc\Trabajo_grado\piezas_Solidworks\pieza_resina1.stl")
dirCad=str("D:\Sergio-Pc\Trabajo_grado\piezas_Solidworks\pieza_resina1.stl")
#points =mesh.vectors.reshape([int(mesh.vectors.size/3), 3])
#points = np.around(np.unique(mesh.vectors.reshape([int(mesh.vectors.size/3), 3]), axis=0),2)
####vertices stl###
stl_vt,cloudstl_vt=vt.Vertices(dirCad)
o3d.visualization.draw_geometries([mesh_stl,cloudstl_vt])

#################figure #######################
figura = plt.figure()
grafica = figura.add_subplot(111,projection = '3d')


[xi, yi , zi] = np.transpose(stl_vt)
[xo, yo , zo] = np.transpose(Obj_vt)

grafica.scatter(xi,yi,zi,    c = 'blue',     marker='o',     label = 'puntos_stl')
grafica.scatter(xo,yo,zo,    c = 'red',     marker='o',     label = 'puntos_obj')

grafica.set_title('puntos, dispersión-scatter')
grafica.set_xlabel('eje x')
grafica.set_ylabel('eje y')
grafica.set_zlabel('eje z')
grafica.set_ylim(0,90)
grafica.legend()
plt.show()
####Save array as point cloud od object ####
pcd_obj= o3d.geometry.PointCloud()
pcd_obj.points = o3d.utility.Vector3dVector(Obj_vt)
####Save array as point cloud ####
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(stl_vt)
o3d.io.write_point_cloud("./data_vertex.ply", pcd)
o3d.visualization.draw_geometries([pcd,pcd_obj])
