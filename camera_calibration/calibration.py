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
    
    filename = image
    img = cv2.imread(image)
    image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
     
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(image, Ch_Dim, None)
    if ret == True:
        obj_points_3D.append(obj_3D)
        corners2 = cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)
        img_points_2D.append(corners2)

        img = cv2.drawChessboardCorners(image, Ch_Dim, corners2, ret)
        # if 'prime' in filename:
        #     output_path = f"camera_calibration/output/prime_right.png"
        #     cv2.imwrite(output_path, img)
        # cv2.imshow('img', img)
        # cv2.waitKey(0)
        # print(filename)
    cv2.destroyAllWindows()

ret, mtx, dist_coeff, R_vecs, T_vecs = cv2.calibrateCamera(obj_points_3D, img_points_2D, gray.shape[::-1], None, None)

mean_error = 0
for i in range(len(obj_points_3D)):
    imgpoints2, _ = cv2.projectPoints(obj_points_3D[i], R_vecs[i], T_vecs[i], mtx, dist_coeff)
    error = cv2.norm(img_points_2D[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error
print("total error: {}".format(mean_error/len(obj_points_3D)))

R, _ = cv2.Rodrigues(R_vecs[-1])
t = T_vecs[-1].reshape(3, 1)

Rt = np.hstack((R, t))

print(Rt)
# print(mtx)

# P = mtx @ Rt
# print(P)


print("calibrated")
# np.savez(
#     f"{calib_data_path}/CalibrationMatrix_college_cpt",
#     Camera_matrix=mtx,
#     distCoeff=dist_coeff
#     RotationalV=R_vecs,
#     TranslationV=T_vecs,
# )