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

from stl import mesh
import open3d as o3d

###############################mesh object#############################################
mesh_obj = o3d.io.read_triangle_mesh("D:\Sergio_vault\study\Trabajo_grado\piezas_Solidworks\pieza_ransac\cylinder1_ransac.stl")
#obtain vertex of mesh_obj
points_obj= np.around(np.unique(mesh_obj.vectors.reshape([int(mesh_obj.vectors.size/3), 3]), axis=0),2)
