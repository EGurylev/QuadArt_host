import root as r
import numpy as np
import cv2
import math

def calc_pose():
    # Calc. pose using solvePnP function
    rot_mat = r.euler2mat(r.roll_cf, r.yaw_cf, r.pitch_cf, 'ZYX')
    rvec_init, _ = cv2.Rodrigues(rot_mat)
    retval, r.rvec, r.tvec = cv2.solvePnP(r.object_points, r.corner_coord, \
        r.camera_matrix, r.dist_coeffs, rvec_init, r.tvec_prev, 1)

    # Calc. the difference between model and measurements
    rot_mat, _ = cv2.Rodrigues(r.rvec)
    homog_mat = np.concatenate((rot_mat, r.tvec), axis=1)
    proj_mat = np.dot(r.camera_matrix, homog_mat)
    image_points, _ = cv2.projectPoints(r.object_points, r.rvec, r.tvec, \
        r.camera_matrix, r.dist_coeffs)
    proj_points = image_points.squeeze()

    # This difference is used for restarting estimation from 
    # known initial conditions when "bad" estimations occur
    coord_diff = np.linalg.norm(proj_points - r.corner_coord)
                
    if coord_diff > 10:
        r.tvec_prev = np.zeros([3,1])
        r.tvec_prev[2] = 5.0
        r.rvec = np.zeros([3,1])
    else:
        r.tvec_prev = r.tvec
        
    # Estimated Euler angles
    _,_,_,rx,ry,rz,euler_angles = cv2.decomposeProjectionMatrix(proj_mat)
    if euler_angles.shape[1] == 1:
        euler_angles = euler_angles.squeeze()     
    r.roll_e = euler_angles[0]
    r.pitch_e = euler_angles[1]
    r.yaw_e = euler_angles[2]
