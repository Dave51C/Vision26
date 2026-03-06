# $Source: /home/scrobotics/src/2025/calibration/RCS/calibrate_from_stills.py,v $
# $Revision: 1.4 $
# $Date: 2025/05/01 19:52:09 $
# $Author: scrobotics $
import numpy as np
import cv2 as cv
import glob
import argparse
import shutil
from datetime import datetime


def write_json (name,width,height,intrinsic,distortion,mean_error):
    Cam_Mtx = intrinsic
    Cam_Dist = distortion
    # I added "created" because we're getting waaaay too many copies floating around.
    NOW = "{:%Y-%m-%d:%H:%M:%S}".format(datetime.now())
    JSON = '{\n   "created"  : "' + NOW + '",\n'
    JSON = JSON + '   "name"     : "{0}_{1}x{2}",\n'.format(name,width,height)
    JSON = JSON + '   "width"    : {0},\n   "height"   : {1},\n'.format(width,height)
    JSON = JSON + '   "meanError": {0},\n'.format(mean_error)
    JSON = JSON + '   "mtx"      :\n'
    JSON = JSON + '   [[{0}, {1}, {2}],\n'.format(Cam_Mtx[0,0],Cam_Mtx[0,1],Cam_Mtx[0,2])
    JSON = JSON + '    [{0}, {1}, {2}],\n'.format(Cam_Mtx[1,0],Cam_Mtx[1,1],Cam_Mtx[1,2])
    JSON = JSON + '    [{0}, {1}, {2}]],\n'.format(Cam_Mtx[2,0],Cam_Mtx[2,1],Cam_Mtx[2,2])
    # Fisheye distortion has 4 coefficients (k1, k2, k3, k4)
    JSON = JSON + '   "dist"     :\n'
    JSON = JSON + '   [{0}, {1}, {2}, {3}],\n'.format(
        Cam_Dist[0,0], Cam_Dist[1,0], Cam_Dist[2,0], Cam_Dist[3,0])
    JSON = JSON + '   "FudgeOffset": 0.0,\n'
    JSON = JSON + '   "FudgeFactor": 1.0\n'
    JSON = JSON + '}\n'
    J = open(name+'_'+str(width)+'x'+str(height)+'.json','w')
    J.write (JSON)
    J.close ()

parser = argparse.ArgumentParser()
parser.add_argument("--name",help="Camera name",type=str,required=True)
args = parser.parse_args()

# termination criteria
criteria   = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 100, 1e-6)
# prepare object points
square_size = 2.0
objp = np.zeros((9*16, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:16].T.reshape(-1, 2) * square_size
objpoints  = [] # 3d point in real world space
imgpoints  = [] # 2d points in image plane.
_img_shape = None
images = glob.glob('stills/img*.jpg')
count = 0
for fname in images:
    img = cv.imread(fname)
    if _img_shape == None:
        _img_shape = img.shape[:2]
        height = img.shape[0]
        width  = img.shape[1]
    else:
        assert _img_shape == img.shape[:2], "All images must share the same size."
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,16), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        count += 1
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        # Draw and display the corners
        cv.drawChessboardCorners(img, (9,16), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(50)
    else:
        print("mv "+fname+" discards")
        shutil.move(fname,'discards')
    if count > 100:
        break
cv.destroyAllWindows()

# Fisheye calibration - K and D must be initialized
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]
tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]

# Fisheye expects (N, 1, 3) and (N, 1, 2) shapes
objpoints_fe = [op.reshape(-1, 1, 3) for op in objpoints]
imgpoints_fe = [ip.reshape(-1, 1, 2) for ip in imgpoints]

ret, mtx, dist, rvecs, tvecs = cv.fisheye.calibrate(
    objpoints_fe,
    imgpoints_fe,
    gray.shape[::-1],
    K,
    D,
    rvecs,
    tvecs,
    flags=cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC,
    criteria=(cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_COUNT, 100, 1e-6)
)

# Reprojection error using fisheye.projectPoints
total_error = 0
for i in range(len(objpoints_fe)):
    imgpoints2, _ = cv.fisheye.projectPoints(
        objpoints_fe[i],
        rvecs[i],
        tvecs[i],
        mtx,
        dist
    )
    error = cv.norm(imgpoints_fe[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
    total_error += error

mean_error = total_error / len(objpoints)
print("Mean reprojection error:", mean_error)

write_json (args.name,width,height,mtx,dist,round(mean_error,5))
