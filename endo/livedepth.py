import numpy as np 
import open3d as o3d
import os 
import glob 
import time
import matplotlib.pyplot as plt


folder_number = "15"

folder = "/home/avinash/Desktop/datasets/endo/depth/rectified"+str(folder_number)+"/"

print(folder)

lfolder = os.path.join(folder,"cropped/image01/*.jpg")
dfolder = os.path.join(folder,"cropped/depth01/*.npy")

limages = sorted(glob.glob(lfolder))
dimages = sorted(glob.glob(dfolder))

print(limages[0])

intriscis = os.path.join(folder,"cropped/intrinscis.txt")
ext = os.path.join(folder,"cropped/extrinsics.txt")

print(intriscis)

K = np.loadtxt(intriscis)

T = np.loadtxt(ext)

b = T[0,3]

print("basline = ", b)

vis = o3d.visualization.VisualizerWithKeyCallback()

vis.create_window(height=540, width=960)


for i in range(len(dimages)):

    color = o3d.io.read_image(limages[i])
    idepth = np.load(dimages[i])

    focal = K[0, 0]
    depth = (focal*b) / (idepth)

    depth = o3d.geometry.Image(depth)

    cam = o3d.camera.PinholeCameraIntrinsic()
    cam.intrinsic_matrix =  K

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False)

    target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)

    target_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

    vis.add_geometry(target_pcd)

    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.25)
    if i >= 0:
        vis.remove_geometry(target_pcd)

    

vis.run()
vis.destroy_window()
