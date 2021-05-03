
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 28 14:42:43 2021

@author: sergio
"""
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
import numpy as np
from pyntcloud import PyntCloud as pytc
from getPoints import Vertices as gv
import getPoints as gp
from numpy_to_cloud import  convert as ntc

#directorio pieza

pieza_dir="/home/sergio/Documentos/Study/parts-inspection/piezas_Freecad/study_piece010.stl"

#vertices
array_vertice,pcb=gv(pieza_dir)

#convertir vertice array en nube de puntos 
cloud_obj=ntc(array_vertice)

#Visualizar nube de puntos 
o3d.visualization.draw_geometries([cloud_obj])

#obtener circulo con algotrimo ransca

center,axis,radius,inliers=gp.circle(array_vertice)
cloud_circle=ntc(inliers)
o3d.visualization.draw_geometries([cloud_circle])