
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
import pandas as pd
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
#cloud_circle=ntc(inliers)

#volumen de stl
volume_piece=gp.volume_cloud()

#o3d.visualization.draw_geometries([cloud_circle])

#obtener puntos con indices inliers que conforman los circulos 
for i in inliers_cir:
    
    points_cir.append(array_vertice[i])
    
    
#points_cir.append(center_cir)
circle_cloud=ntc(points_cir)
o3d.visualization.draw_geometries([circle_cloud])
print("sin filtro",len(points_cir))
#paramtetizacion de circulo

#calculo de puntos cacrasteristimos de parametros ransac

# for i in range(len(points_cir)):

#                 distancia_puntos.append(math.sqrt(math.pow(center_cir[0]-points_cir[i][0],2)+math.pow(center_cir[1]-points_cir[i][1],2)+math.pow(center_cir[2]-points_cir[i][2],2)))
#                 if distancia_puntos[-1] >=radius_cir-0.5 and distancia_puntos[-1] <=radius_cir+0.5:
                    
                #      ransac_circle.append(distancia_puntos[-1])
                #      indices.append(i)
                #      points_ransac_filtres.append(points_cir[i])
                # if len(ransac_circle)<=10 and i==len(points_cir)-1:
                    
             
                #  for j in indices:
                        
                #                 points_cir.pop(j)
                       
                # if len(ransac_circle)>10 and i==len(points_cir)-1:
                #     num_circles=+1
                #     cloud_filters=ntc(points_ransac_filtres)
                #     o3d.visualization.draw_geometries([cloud_filters])

                    
print("con filtro",len(points_cir))

#lista con cordenadas de z

for i in points_cir:
    coord_z.append(i[-1])
        
        
        


for i in range(len(coord_z)):
     
    if  coord_z[i]<=center_ideal_c1[2]+1  and coord_z[i] >=center_ideal_c1[2] -1:
        clusters_z1.append(points_cir[i])
        
        
    if  coord_z[i]<=12.5  and coord_z[i] >= 11.5:
        clusters_z2.append(points_cir[i])
clusters_z1=np.array(clusters_z1)
clusters_z2=np.array(clusters_z2)
cloud_z1=ntc(clusters_z1)
cloud_z2=ntc(clusters_z2)

o3d.visualization.draw_geometries([cloud_z1])
while cont_center!=1:

  center_cirz1,axis_cirz1,radius_cirz1,inliers_cirz1=gp.circle(clusters_z1)
  distancia_center=math.sqrt(math.pow(center_cirz1[0]-center_ideal_c1[0],2)+math.pow(center_cirz1[1]-center_ideal_c1[1],2)+math.pow(center_cirz1[2]-center_ideal_c1[2],2))
  if distancia_center<=1: 
     for i in inliers_cirz1:
    
         points_cirz1.append(clusters_z1[i])
     cont_center+=1
circle_cloudz1=ntc(points_cirz1)
o3d.visualization.draw_geometries([circle_cloudz1])
print("radio",radius_cirz1)
bakc_clusters_z1=clusters_z1

#calculo de puntos cacrasteristimos de parametros ransac
while num_circles !=1:
 for i in range(len(clusters_z1)):
 
                distancia_puntos.append(math.sqrt(math.pow(center_cirz1[0]-clusters_z1[i][0],2)+math.pow(center_cirz1[1]-clusters_z1[i][1],2)+math.pow(center_cirz1[2]-clusters_z1[i][2],2)))
                if distancia_puntos[i] >=radius_cirz1-0.01 and distancia_puntos[i] <=radius_cirz1+0.01:
                      
                      ransac_circle.append(distancia_puntos[i])
                      indices.append(i)
                      points_ransac_filtres.append(clusters_z1[i])
               
                      #print("longitud con filtro clusterz1",len(clusters_z1))
                     
                if len(ransac_circle)>=15  and  i==len(clusters_z1)-1:
                    
                          print("entro al if",i)
                          clusters_z1=np.array(points_ransac_filtres)
                          center_cirz1,axis_cirz1,radius_cirz1,inliers_cirz1=gp.circle(clusters_z1)
                          num_circles+=1
                          print("valor num_circles",num_circles)
                            
                           