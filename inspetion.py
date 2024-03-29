
"""
Created on Wed Apr 28 14:42:43 2021

@author: Daniel
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
import numpy as np
from pyntcloud import PyntCloud as pytc
from getPoints import Vertices as gv
import getPoints as gp
from numpy_to_cloud import  convert as ntc
import math
#directorio pieza
points_cir=[]
points_cy=[]
pieza_dir="/home/daniel/Documentos/Study//piezas_Freecad/study_piece006.stl"
mesh_stl = o3d.io.read_triangle_mesh(pieza_dir)
o3d.visualization.draw_geometries([mesh_stl])
#vertices
array_vertice,pcb=gv(pieza_dir)
#cloud 
cloud_stl,array_stl=gp.points(pieza_dir)
o3d.visualization.draw_geometries([cloud_stl])
#convertir vertice array en nube de puntos 
cloud_obj=ntc(array_vertice)

#Visualizar nube de puntos 
o3d.visualization.draw_geometries([cloud_obj])

#obtener circulo con algotrimo ransca

center_cir,axis_cir,radius_cir,inliers_cir=gp.circle(array_vertice)
#cloud_circle=ntc(inliers)

#volumen de stl
volume_piece=gp.volume_cloud()

#o3d.visualization.draw_geometries([cloud_circle])

#obtener puntos con indices inliers que conforman los circulos 
for i in inliers_cir:
    
    points_cir.append(array_vertice[i])
    
    
    
circle_cloud=ntc(points_cir)
o3d.visualization.draw_geometries([circle_cloud])

# #fix cylinder with ransac
# center_cy,axis_cy,radius_cy,inliers_cy=gp.cylinder(array_stl)
# #obtener puntos con indices inliers que conforman los cylindres
# for i in inliers_cy:
    
#     points_cy.append(array_stl[i])
# cylinder_cloud=ntc(points_cy)
# o3d.visualization.draw_geometries([cylinder_cloud])
#Sergmentacion de planos 
plane_segment=o3d.geometry.PointCloud.segment_plane(circle_cloud,5,20,100)