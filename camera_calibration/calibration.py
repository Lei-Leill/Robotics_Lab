# https://www.cs.cmu.edu/~16385/s17/Slides/11.1_Camera_matrix.pdf
# Cool slideshow for conceptual ideas

import cv2 
import numpy as np

# corners of the square blocks (vertical and horizontal)
Ch_Dim = (7, 7)
Sq_size = 0.056  #now meters
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001) 

obj_3D = np.zeros((Ch_Dim[0] * Ch_Dim[1], 3), np.float32)
index = 0
for i in range(Ch_Dim[0]):
    for j in range(Ch_Dim[1]):
        obj_3D[index][0] = i * Sq_size
        obj_3D[index][1] = j * Sq_size
        index += 1
# print(obj_3D)
obj_points_3D = []  # 3d point in real world space
img_points_2D = []  # 2d points in image plane.

camera = 'left'

import glob
image_files = glob.glob(rf'camera_calibration\calibration_images\{camera}\*.png')

for image in image_files:
    
    filepath = image
    img = cv2.imread(image)
    image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
     
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(image, Ch_Dim, None)
    if ret == True:
        obj_points_3D.append(obj_3D)
        corners2 = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)
        img_points_2D.append(corners2)

        img = cv2.drawChessboardCorners(image, Ch_Dim, corners2, ret)
        filename = filepath.split('\\')[-1]

        # print(filename)
        output_path = f"camera_calibration/output/{camera}/{filename}"
        # Don't uncomment this
        # cv2.imwrite(output_path, img)

        # cv2.imshow('img', img)
        # cv2.waitKey(0)

    cv2.destroyAllWindows()

# ret, old_mtx, old_dist_coeff, old_R_vecs, old_T_vecs = cv2.calibrateCamera(obj_points_3D, img_points_2D, gray.shape[::-1], None, None)

total_mtx = np.array([[[645.9848022460938, 0, 639.00927734375], [0, 645.2113647460938, 361.0145263671875], [0, 0, 1]],
                      [[644.1821899414062, 0, 647.15283203125], [0, 643.3673706054688, 368.00286865234375], [0, 0, 1]]])
total_dist = np.array([[-0.05537797138094902,0.06713823974132538,-0.0002206100180046633,0.0003125208313576877,-0.02147624082863331], 
                       [-0.056005097925662994,0.06765501201152802,0.00024983854382298887,0.0006985375075601041,0.021450236439704895]])

mtx = total_mtx[0] if camera == 'left' else total_mtx[1]
dist_coeff = total_dist[0] if camera == 'left' else total_dist[1]

idx = 8

ret, R_vecs, T_vecs = cv2.solvePnP(obj_points_3D[idx], img_points_2D[idx], mtx, dist_coeff)

print(image_files[idx])
R, _ = cv2.Rodrigues(R_vecs)
t = T_vecs.reshape(3, 1)

adjusted_t = -R.T @ t

Rt = np.hstack((R, t))

print(Rt)
print(adjusted_t)

print("calibrated")

# Drawing axes
# axes_img_dir = f"camera_calibration/output/{camera}/{camera}_17_Color.png"
# axes_img = cv2.imread(axes_img_dir)
# axes_img = cv2.drawFrameAxes(axes_img, mtx, dist_coeff, R_vecs[idx], T_vecs[idx], length=0.2)
# output_path = f"camera_calibration/output/axes/{camera}_17_Color.png"
# cv2.imwrite(output_path, axes_img)

# Doesn't work anymore :(
# mean_error = 0
# for i in range(len(obj_points_3D)):
#     imgpoints2, _ = cv2.projectPoints(obj_points_3D[i], R_vecs[i], T_vecs[i], mtx, dist_coeff)
#     error = cv2.norm(img_points_2D[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
#     mean_error += error
# print("total error: {}".format(mean_error/len(obj_points_3D)))