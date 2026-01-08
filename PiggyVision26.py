# $Source: /home/scrobotics/src/cscore_images_from_queues/RCS/create_camera.py,v $
# $Revision: 1.4 $
# $Date: 2025/12/31 04:24:26 $
# $Author: scrobotics $

class Webcam ():
    def __init__(self, usage):
        import json
        import numpy as np
        from collections import deque
        try:
            with open ('/boot/frc.json','r') as file:
                try:
                    frc = json.load(file)
                except:
                    print ("Can't load /boot/frc.json. Format error?")
        except:
            print ("Can't open /boot/frc.json, will try local copy")
            try:
                with open ('frc.json','r') as file:
                    frc = json.load(file)
            except:
                print ("Can't find a usable frc.json")

        cam_entry = None
        for cam in frc["cameras"]:
            for prop in cam["properties"]:
                if prop["name"] == "usage" and prop["value"] == usage:
                    cam_entry = cam
                    self.CameraName = cam['name']
                    self.width      = cam['width']
                    self.height     = cam['height']
                    self.queue      = deque(maxlen=1)
                    self.buffer=np.zeros(shape=(self.height,self.width,3),dtype=np.uint8)
                    paramFile       = f"{self.CameraName}.json"
                    try:
                        with open(paramFile,'r') as pfile:
                            j = json.load(pfile)
                    except:
                        print("Can't open", paramFile)
                    try:
                        self.mtx  = np.array(j['mtx'])
                    except:
                        print ("Can't set mtx")
                    try:
                        self.dist = np.array(j['dist'])
                    except:
                        print ("Can't set dist")
                    XYZPY = "./"+usage+".json"   # XYZPY: X,Y,Z pitch & yaw
                    try:
                        with open (XYZPY,'r') as file:
                            try:
                                place = json.load(file)
                                self.localX = place['localX']
                                self.localY = place['localY']
                                self.localZ = place['localZ']
                                self.pitch  = place['pitch']
                                self.yaw    = place['yaw']
                            except:
                                 print ("Can't load",XYZPY,"Format error?")
                    except:
                        print ("Can't open",XYZPY)
                    self.robotPose = None
                    break
            if cam_entry:
                break

class BotCam (Webcam):
    """
    BotCam describes the camera's usage and placement w.r.t. the robot. The
    "usage" value should be something meaningful like "DriverCam" or "ScoopCam".
    NOTE: no spaces in the value.
    Placement is the camera's X, Y & Z and pitch and yaw. X and Y are relative
    to the robot's geometric center; Z is relative to the ground plane. The X
    axis points to front of the robot, Y points to the left. Z is up. Pitch and
    yaw are in degrees. Pitch is the camera angle relative to horizontal;
    positive is up, negative is down. Yaw is the camera's rotation about the
    vertical (Z) axis. It's measured counter-clockwise from the X axis so 0
    points forward, 90 points left, 180 points backward and 270 points right.
    """
    list = []
    def __init__(self, usage):
        super().__init__(usage)
        self.usage = usage
        BotCam.list.append(self)    # Keep a list of cameras on bot

"""
TAG_CORNERS breakdown: TagID,[Vertex coords NW,NE,SE,SW]
"""
import numpy as np
import cv2
TAG_CORNERS = {
    0:np.array([[  -3.75,   3.75,   0.00],
                      [   3.75,   3.75,   0.00],
                      [   3.75,  -3.75,   0.00],
                      [  -3.75,  -3.75,   0.00]]),
    1:np.array([[ 656.54,  20.56,  58.50],
                      [ 652.13,  26.63,  58.50],
                      [ 658.20,  31.04,  58.50],
                      [ 662.61,  24.97,  58.50]]),
    2:np.array([[ 662.61, 292.03,  58.50],
                      [ 658.20, 285.96,  58.50],
                      [ 652.13, 290.37,  58.50],
                      [ 656.54, 296.44,  58.50]]),
    3:np.array([[ 458.90, 320.90,  51.25],
                      [ 458.90, 313.40,  51.25],
                      [ 451.40, 313.40,  51.25],
                      [ 451.40, 320.90,  51.25]]),
    4:np.array([[ 361.95, 245.39,  75.42],
                      [ 368.45, 245.39,  71.67],
                      [ 368.45, 237.89,  71.67],
                      [ 361.95, 237.89,  75.42]]),
    5:np.array([[ 361.95,  79.14,  75.42],
                      [ 368.45,  79.14,  71.67],
                      [ 368.45,  71.64,  71.67],
                      [ 361.95,  71.64,  75.42]]),
    6:np.array([[ 531.86, 135.29,  12.13],
                      [ 535.61, 128.80,  12.13],
                      [ 529.12, 125.05,  12.13],
                      [ 525.37, 131.54,  12.13]]),
    7:np.array([[ 543.12, 162.25,  12.13],
                      [ 550.62, 162.25,  12.13],
                      [ 550.62, 154.75,  12.13],
                      [ 543.12, 154.75,  12.13]]),
    8:np.array([[ 525.37, 185.46,  12.13],
                      [ 529.12, 191.95,  12.13],
                      [ 535.61, 188.20,  12.13],
                      [ 531.86, 181.71,  12.13]]),
    9:np.array([[ 496.40, 181.71,  12.13],
                      [ 492.65, 188.20,  12.13],
                      [ 499.14, 191.95,  12.13],
                      [ 502.89, 185.46,  12.13]]),
    10:np.array([[ 485.14, 154.75,  12.13],
                      [ 477.64, 154.75,  12.13],
                      [ 477.64, 162.25,  12.13],
                      [ 485.14, 162.25,  12.13]]),
    11:np.array([[ 502.89, 131.54,  12.13],
                      [ 499.14, 125.05,  12.13],
                      [ 492.65, 128.80,  12.13],
                      [ 496.40, 135.29,  12.13]]),
    12:np.array([[  28.27,  24.97,  58.50],
                      [  32.68,  31.04,  58.50],
                      [  38.75,  26.63,  58.50],
                      [  34.34,  20.56,  58.50]]),
    13:np.array([[  34.34, 296.44,  58.50],
                      [  38.75, 290.37,  58.50],
                      [  32.68, 285.96,  58.50],
                      [  28.27, 292.03,  58.50]]),
    14:np.array([[ 328.93, 237.89,  71.67],
                      [ 322.43, 237.89,  75.42],
                      [ 322.43, 245.39,  75.42],
                      [ 328.93, 245.39,  71.67]]),
    15:np.array([[ 328.93,  71.64,  71.67],
                      [ 322.43,  71.64,  75.42],
                      [ 322.43,  79.14,  75.42],
                      [ 328.93,  79.14,  71.67]]),
    16:np.array([[ 231.98,  -3.90,  51.25],
                      [ 231.98,   3.60,  51.25],
                      [ 239.48,   3.60,  51.25],
                      [ 239.48,  -3.90,  51.25]]),
    17:np.array([[ 165.51, 131.54,  12.13],
                      [ 161.76, 125.05,  12.13],
                      [ 155.27, 128.80,  12.13],
                      [ 159.02, 135.29,  12.13]]),
    18:np.array([[ 147.75, 154.75,  12.13],
                      [ 140.25, 154.75,  12.13],
                      [ 140.25, 162.25,  12.13],
                      [ 147.75, 162.25,  12.13]]),
    19:np.array([[ 159.02, 181.71,  12.13],
                      [ 155.27, 188.20,  12.13],
                      [ 161.76, 191.95,  12.13],
                      [ 165.51, 185.46,  12.13]]),
    20:np.array([[ 187.98, 185.46,  12.13],
                      [ 191.73, 191.95,  12.13],
                      [ 198.22, 188.20,  12.13],
                      [ 194.47, 181.71,  12.13]]),
    21:np.array([[ 205.74, 162.25,  12.13],
                      [ 213.24, 162.25,  12.13],
                      [ 213.24, 154.75,  12.13],
                      [ 205.74, 154.75,  12.13]]),
    22:np.array([[ 194.47, 135.29,  12.13],
                      [ 198.22, 128.80,  12.13],
                      [ 191.73, 125.05,  12.13],
                      [ 187.98, 131.54,  12.13]])
}
def pose (results,mtx,dist):
    """
    pose is called for each camera frame. It returns that camera's robot pose
    value in field coordinates.
    """
    for r in results:
        ret, rvecs, tvecs = cv2.solvePnP(TAG_CORNERS[r.tag_id],r.corners,
             mtx,dist, cv2.SOLVEPNP_IPPE_SQUARE)
        print (tvecs)
    return (0.0, 0.0, 0.0, 0.0, 0.0)

if __name__ == "__main__":
    from pprint import pprint
    DriverCam = BotCam("DriverCam")
    print(DriverCam.__dict__)
    print(DriverCam.__dict__.keys())
    ClimbCam  = BotCam("ClimbCam")
    #pprint(DriverCam.__dict__)
    #pprint(ClimbCam.__dict__)
    #pprint(DriverCam.mtx)
    #pprint(ClimbCam.mtx)
    for item in BotCam.list:
        if item.usage == 'DriverCam':
            print (item.usage)
