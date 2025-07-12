import numpy as np

# camera extrinsics from calibration process (unit: meter)
# each matrix transforms the chessboard frame to the corresponding camera frame
left_cam = np.array([[-0.10285259,  0.99016408, -0.09484956,  0.11367459],
                     [ 0.65681041, -0.00400559, -0.75404511, -0.28397345],
                     [-0.74700832, -0.13985367, -0.64993809,  0.97922874],
                     [0, 0, 0, 1]])
right_cam = np.array([[-0.99950439, -0.01747422,  0.02618448,  0.08706049],
                      [-0.03119222,  0.66191747, -0.74892744, -0.39036483],
                      [-0.00424504, -0.74937302, -0.66213448,  1.01596251],
                      [0, 0, 0, 1]])

# matrices to transform the camera frame to chessboard frame
left_chess = np.linalg.inv(left_cam)
right_chess = np.linalg.inv(right_cam)

# matrices to transform the chessboard frame to robot base frame (unit: meter)
left_robot = np.transpose(np.array([[0,-1,0,0],
                                    [1,0,0,0],
                                    [0,0,1,0],
                                    [0.395766,0.130377,0.0255017,1]]))
right_robot = np.transpose(np.array([[1,0,0,0],
                                     [0,1,0,0],
                                     [0,0,1,0],
                                     [0.391866,-0.216894,0.0274459,1]]))

# putting it all together to get matrices to transform camera frame to robot frame
left = np.matmul(left_robot, left_chess)
right = np.matmul(right_robot, right_chess)
print("Left camera\n", left)
print("Right camera\n", right)

# Testing the transformation matrices on actual data
left_pose = np.array([[0.0802], [-0.4619], [1.18], [1]])
right_pose = np.array([[0.1286], [0.0153], [0.58], [1]])

left_pose = np.matmul(left, left_pose)
right_pose = np.matmul(right, right_pose)
print(left_pose)
print(right_pose)