#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 15 10:50:49 2021

@author: sergio
"""
import numpy as np
import open3d as o3d
import json 
if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("/home/sergio/Documentos/Study/parts-inspection/data.ply")
    print(pcd)
    array=np.asarray(pcd.points)
    print(np.asarray(pcd.points))
    array_json=array.tolist()
    o3d.visualization.draw_geometries([pcd])
    
    
    
    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    o3d.visualization.draw_geometries([downpcd],zoom=0.3412, front=[0.4257, -0.2125, -0.8795],lookat=[2.6172, 2.0475, 1.532],up=[-0.0694, -0.9768, 0.2024])
    
    
    print("Recompute the normal of the downsampled point cloud")
    downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    o3d.visualization.draw_geometries([downpcd], zoom=0.3412, front=[0.4257, -0.2125, -0.8795],lookat=[2.6172, 2.0475, 1.532],up=[-0.0694, -0.9768, 0.2024],point_show_normal=True)
    