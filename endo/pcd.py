import numpy as np 
import open3d as o3d


pcd = o3d.io.read_point_cloud("/home/avinash/Downloads/test1.ply")
o3d.visualization.draw_geometries([pcd])