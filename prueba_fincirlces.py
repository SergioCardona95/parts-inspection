"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
#Created on Fri May 28 09:51:13 2021

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
import numpy as np
from pyntcloud import PyntCloud as pytc
from getPoints import Vertices as gv
import getPoints as gp
from numpy_to_cloud import  convert as ntc
import math
import pandas as pd
import find_cirlces as fdc
#directorio pieza
points_cir=[]
points_cirz1=[]
points_cy=[]
distancia_puntos=[]
ransac_circle=[]
indices=[]
num_circles=0

points_ransac_filtres=[]
coord_z=[]
cont_center=0
clusters_z1=[]
clusters_z2=[]
clusters_z1f=[]
center_ideal_c1=[10,12,0]

num_circles=0
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
distancia_puntos=[]
#Visualizar nube de puntos 
o3d.visualization.draw_geometries([cloud_obj])

#obtener circulo con algotrimo ransca

center_cir,axis_cir,radius_cir,inliers_cir=gp.circle(array_vertice)

clusters_f,center_cirz_f,axis_cirz_f,radius_cirz_f,inliers_cir_f,distan_points_f=fdc.optimizerPoints(np.array(array_vertice),center_ideal_c1)