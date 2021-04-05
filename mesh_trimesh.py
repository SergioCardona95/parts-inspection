# -*- coding: utf-8 -*-
"""
Created on Sat Mar 27 11:29:03 2021

@author: Sergio
"""
import trimesh
import numpy as np
mesh_obj=trimesh.load("D:\Sergio_vault\study\Trabajo_grado\piezas_Solidworks\pieza_resina1.stl")
vol1=mesh_obj.volume
vol2=mesh_obj.convex_hull.volume
volume_cylindre=mesh_obj.bounding_cylinder.volume