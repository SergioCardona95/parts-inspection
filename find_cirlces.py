#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 27 14:23:39 2021

@author: daniel
"""

import numpy as np
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


num_cirlces=0
coord_z=[]
clusters=[]
def  optimizerPoints (array_vertices,center_ideal):
    while num_cirlces!=1:
        restri_one=0
        cont_center=0
        distan_points=[]
        points_cirz=[]
        ransac_circle=[]
        indices=[]
        points_ransac_filtres=[]
        points_inliers=[]
        clusters=[]
        center_cir,axis_cir,radius_cir,inliers_cir=gp.circle(array_vertices)
        for i in inliers_cir:
    
            points_inliers.append(array_vertices[i])
            
        inliers_cloud=ntc(points_inliers)
        o3d.visualization.draw_geometries([inliers_cloud])
        
        for i in points_inliers:
            coord_z.append(i[-1])
        
        
        


        for i in range(len(coord_z)):
     
            if  coord_z[i]<=center_ideal[2]+1  and coord_z[i] >=center_ideal[2] -1:
                
                    clusters.append(points_inliers[i])
        clusters=np.array(clusters) 
        cloud=ntc(clusters)
        o3d.visualization.draw_geometries([inliers_cloud])    
        while cont_center!=1 and restri_one!=100:
            print("entro a cont_center",restri_one)
            restri_one+=1
            center_cirz,axis_cirz,radius_cirz,inliers_cirz=gp.circle(clusters)
            distancia_center=math.sqrt(math.pow(center_cirz[0]-center_ideal[0],2)+math.pow(center_cirz[1]-center_ideal[1],2)+math.pow(center_cirz[2]-center_ideal[2],2))
            if distancia_center<=1: 
                for i in inliers_cirz:
    
                    points_cirz.append(clusters[i])
                cont_center+=1
        print("salio de print cont_cont")
        if cont_center==1:
             print("entro a cont center")
             for i in range(len(clusters)):
                 distan_points.append(math.sqrt(math.pow(center_cirz[0]-clusters[i][0],2)+math.pow(center_cirz[1]-clusters[i][1],2)+math.pow(center_cirz[2]-clusters[i][2],2)))
                 if distan_points[i] >=radius_cirz-0.1 and distan_points[i] <=radius_cirz+0.1:
                      
                      ransac_circle.append(distan_points[i])
                      indices.append(i)
                      points_ransac_filtres.append(clusters[i])
                      if len(ransac_circle)>=15  and  i==len(clusters)-1:     
                      
                          print("entro al if",i)
                          clusters=np.array(points_ransac_filtres)
                          center_cirz,axis_cirz,radius_cirz,inliers_cirz=gp.circle(clusters)
                          num_circles+=1
    return clusters,center_cirz,axis_cirz,radius_cirz,inliers_cir,distan_points