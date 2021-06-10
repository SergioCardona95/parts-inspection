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
import pyransac3d as  pyrsc
from pyntcloud import PyntCloud as pytc
def  Vertices(directory):
     
     mesh_data= mesh.Mesh.from_file(directory)#read mesh of the directory
     #points =mesh_data.vectors.reshape([int(mesh_data.vectors.size/3), 3]) #get vertices
     points=np.around(np.unique(mesh_data.vectors.reshape([int(mesh_data.vectors.size/3), 3]), axis=0),2)#get vertices
     pcd = o3d.geometry.PointCloud()#  create variable type    o3d.geometry.PointCloud()
     pcd.points = o3d.utility.Vector3dVector(points) #save mesh as points cloud (vertice)
     return points,pcd
 



def circle(pts):
    
     
     circle=pyrsc.Circle()
     center,axis,radius,inliers=circle.fit(pts, thresh=7.5, maxIteration=100)
     
     return center,axis,radius,inliers
 
def points (directory):
    
    mesh_stl = o3d.io.read_triangle_mesh(directory)
    cloud_stl= mesh_stl.sample_points_poisson_disk(10000)
    save_cloud_stl=o3d.io.write_point_cloud("./data_meshStl.ply", cloud_stl)
    array_stl=np.asarray(cloud_stl.points)
    return cloud_stl,array_stl

def volume_cloud ():
    
    volume_stl=pytc.from_file("./data_meshStl.ply")
    convex_hull_volume_stl_id = volume_stl.add_structure("convex_hull")
    convex_hull_stl = volume_stl.structures[convex_hull_volume_stl_id]
    volume_cloud_stl=convex_hull_stl.volume
    return volume_cloud_stl
    
def cylinder(pts):
    
    cylinder=pyrsc.Cylinder()
    center,axis,radius,inliers = cylinder.fit(pts, thresh=0.001, maxIteration=1000)
    return center,axis,radius,inliers