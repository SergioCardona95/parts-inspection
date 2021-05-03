#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 28 14:48:57 2021

@author: sergio
"""
import open3d as o3d
import numpy as np

#este script convierte un array en nube de puntos formato ply

def convert(array):
    pcd= o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(array)
    o3d.io.write_point_cloud("./data.ply", pcd)
    
    
    return pcd


