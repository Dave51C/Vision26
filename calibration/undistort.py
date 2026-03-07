import cv2
import numpy as np
import json
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("-i", type=str, help="input image")
parser.add_argument("-c", type=str, help="calibration file")
args = parser.parse_args()
iFile   = args.i
calFile = args.c
img = cv2.imread(iFile)
calData = open(calFile,'r')
j     = json.load(calData)
print(type(j["mtx"]))
output_img   = cv2.undistort(img, np.array(j["mtx"]), np.array(j["dist"]))
cv2.imshow("UNDISTORTED",output_img)
cv2.imshow("ORIGINAL",img)
cv2.waitKey(0)

cv2.destroyAllWindows()
