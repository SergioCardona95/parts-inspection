import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
import numpy as np
from pyntcloud import PyntCloud as pytc
mesh_stl = o3d.io.read_triangle_mesh("/home/sergio/Documentos/Study/piezas_Freecad/pieza1.stl")
cloud_stl= mesh_stl.sample_points_poisson_disk(20000)
array_stl=np.asarray(cloud_stl.points)
save_cloud_stl=o3d.io.write_point_cloud("./data_meshStl.ply", cloud_stl)
# you can plot and check
o3d.visualization.draw_geometries([cloud_stl])