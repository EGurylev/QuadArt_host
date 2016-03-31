import root as r
import numpy as np
import cv2

def calc_pose():
    # Calc. pose using solvePnP function
    R = r.Euler2Mat(r.roll_cf, r.yaw_cf, r.pitch_cf, 'ZYX')
    rvec_init, _ = cv2.Rodrigues(R)
    retval, r.rvec, r.tvec = cv2.solvePnP(r.objectPoints, r.CornerCoord, \
        r.cameraMatrix, r.distCoeffs, rvec_init, r.tvec_Prev, 1)

    # Calc. the difference between model and measurements
    rotM, _ = cv2.Rodrigues(r.rvec)
    M = np.concatenate((rotM, r.tvec), axis=1)
    projM = np.dot(r.cameraMatrix, M)
    imagePoints, _ = cv2.projectPoints(r.objectPoints, r.rvec, r.tvec, \
        r.cameraMatrix, r.distCoeffs)
    projPoints = imagePoints.squeeze()

    # This difference is used for restarting estimation from 
    # known initial conditions when "bad" estimations occur
    CoordDiff = np.linalg.norm(projPoints - r.CornerCoord)
                
    if CoordDiff > 10:
        r.tvec_Prev = np.zeros([3,1])
        r.tvec_Prev[2] = 5.0
        r.rvec = np.zeros([3,1])
    else:
        r.tvec_Prev = r.tvec
        
    # Estimated Euler angles
    _,_,_,rX,rY,rZ,EulerAngles = cv2.decomposeProjectionMatrix(projM)
    if EulerAngles.shape[1] == 1:
        EulerAngles = EulerAngles.squeeze()     
    r.roll_e = EulerAngles[0]
    r.pitch_e = EulerAngles[1]
    r.yaw_e = EulerAngles[2]
