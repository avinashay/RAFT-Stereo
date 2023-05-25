import numpy as np
import cv2
import os 
import glob 
import shutil


def crop_images(folder_number, output_size):

    intriscis = "/home/avinash/Desktop/datasets/endo/calibration/"+str(folder_number)+"/intrinsics.txt"
    extrinsics = "/home/avinash/Desktop/datasets/endo/calibration/"+str(folder_number)+"/extrinsics.txt"

    folder = "/home/avinash/Desktop/datasets/endo/depth/rectified"+str(folder_number)+"/"

    print(folder)

    lfolder = os.path.join(folder,"image01/*.jpg")
    rfolder = os.path.join(folder,"image02/*.jpg")

    clfolder = os.path.join(folder,"cropped/image01")
    crfolder = os.path.join(folder,"cropped/image02")

    os.makedirs(clfolder,exist_ok=True)
    os.makedirs(crfolder,exist_ok=True)

    K = np.loadtxt(intriscis)
    K = K[:,:3]

    limages = sorted(glob.glob(lfolder))
    rimages = sorted(glob.glob(rfolder))

    limg = cv2.imread(limages[0])
    h,  w = limg.shape[:2]
    h1, w1 = output_size[0], output_size[1]
    xc = (w-w1)//2
    yc = (h-h1)//2

    K[0,2] = K[0,2]-xc
    K[1,2] = K[1,2]-yc

    print("saving new calibration........")

    kfile = os.path.join(folder,'cropped/intrinscis.txt')
    efile = os.path.join(folder,'cropped/extrinsics.txt')

    np.savetxt(kfile, K, fmt='%.3f')

    shutil.copyfile(extrinsics, efile)

    for i in range(len(limages)):

        limg = cv2.imread(limages[i])
        rimg = cv2.imread(rimages[i])
        climg = limg[yc:yc+h1, xc:xc+w1,:]
        crimg = rimg[yc:yc+h1, xc:xc+w1,:]

        filename =  '0' * (10 - len(str(i))) + str(i) + ".jpg"

        lfile = os.path.join(clfolder,filename)
        rfile = os.path.join(crfolder,filename)

        print("saving ",filename)
        cv2.imwrite(lfile,climg)
        cv2.imwrite(rfile,crimg)

    print("Done........")


if __name__ == "__main__":

    folder_number = "06"

    output_size = [384,512]

    crop_images(folder_number, output_size)
 



