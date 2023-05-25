import numpy as np 
import open3d as o3d
import os 
import glob 
import time
import matplotlib.pyplot as plt


folder_number = "17"

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


for i in range(10):

    color = o3d.io.read_image(limages[i])
    idepth = np.load(dimages[i])

    color1 = o3d.io.read_image(limages[i+1])
    idepth1 = np.load(dimages[i+1])


    focal = K[0, 0]
    depth = (focal*b) / (idepth)
    depth1 = (focal*b) / (idepth1)

    depth = o3d.geometry.Image(depth)
    depth1 = o3d.geometry.Image(depth1)

    cam = o3d.camera.PinholeCameraIntrinsic()
    cam.intrinsic_matrix =  K

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color, depth, convert_rgb_to_intensity=False)

    rgbd1 = o3d.geometry.RGBDImage.create_from_color_and_depth(color1, depth1, convert_rgb_to_intensity=False)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, cam)

    if i == 0:
        total_pcd = pcd 

    pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd1, cam)

    option = o3d.pipelines.odometry.OdometryOption()
    odo_init = np.identity(4)

    [success_color_term, trans_color_term, info] = o3d.pipelines.odometry.compute_rgbd_odometry(
     rgbd, rgbd1, cam, odo_init,
     o3d.pipelines.odometry.RGBDOdometryJacobianFromColorTerm(), option)
    

    print("Using RGB-D Odometry")
    print(trans_color_term)

    pcd1.transform(trans_color_term)


    if False:
        [success_hybrid_term, trans_hybrid_term, info] = o3d.pipelines.odometry.compute_rgbd_odometry(
            rgbd, rgbd1, cam, odo_init,
            o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(), option)

        print("Using Hybrid RGB-D Odometry")
        print(trans_hybrid_term)

        pcd1.transform(trans_hybrid_term)

    if i != 0:
        total_pcd = total_pcd+pcd1

    #o3d.visualization.draw_geometries([total_pcd])

    vis.add_geometry(total_pcd)

    vis.poll_events()
    vis.update_renderer()
    time.sleep(0.25)

vis.run()
vis.destroy_window()


