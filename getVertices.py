# -*- coding: utf-8 -*-
"""
Created on Sat Feb  6 20:45:23 2021

@author: sergi
"""

# este script me permite obtener los vertices de un stl y retornarlos 
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from stl import mesh
import open3d as o3d

def  Vertices(directory):
     
     mesh_data= mesh.Mesh.from_file(directory)#read mesh of the directory
     #points =mesh_data.vectors.reshape([int(mesh_data.vectors.size/3), 3]) #get vertices
     points=np.around(np.unique(mesh_data.vectors.reshape([int(mesh_data.vectors.size/3), 3]), axis=0),2)#get vertices
     pcd = o3d.geometry.PointCloud()#  create variable type    o3d.geometry.PointCloud()
     pcd.points = o3d.utility.Vector3dVector(points) #save mesh as points cloud (vertice)
     return points,pcd
 
 