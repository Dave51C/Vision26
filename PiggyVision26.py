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

import numpy as np
import cv2
"""
TAG_CORNERS breakdown: TagID,[Vertex coords NW,NE,SE,SW]
"""
TAG_CORNERS = {
    1:np.array([[ 467.64, 296.06,  38.75],
                [ 467.64, 288.56,  38.75],
                [ 467.64, 288.56,  31.25],
                [ 467.64, 296.06,  31.25]]
                )
  , 2:np.array([[ 472.86, 182.60,  48.00],
                [ 465.36, 182.60,  48.00],
                [ 465.36, 182.60,  40.50],
                [ 472.86, 182.60,  40.50]]
                )
  , 3:np.array([[ 445.35, 176.59,  48.00],
                [ 445.35, 169.09,  48.00],
                [ 445.35, 169.09,  40.50],
                [ 445.35, 176.59,  40.50]]
                )
  , 4:np.array([[ 445.35, 162.59,  48.00],
                [ 445.35, 155.09,  48.00],
                [ 445.35, 155.09,  40.50],
                [ 445.35, 162.59,  40.50]]
                )
  , 5:np.array([[ 465.36, 135.09,  48.00],
                [ 472.86, 135.09,  48.00],
                [ 472.86, 135.09,  40.50],
                [ 465.36, 135.09,  40.50]]
                )
  , 6:np.array([[ 467.64,  29.12,  38.75],
                [ 467.64,  21.62,  38.75],
                [ 467.64,  21.62,  31.25],
                [ 467.64,  29.12,  31.25]]
                )
  , 7:np.array([[ 470.59,  21.62,  38.75],
                [ 470.59,  29.12,  38.75],
                [ 470.59,  29.12,  31.25],
                [ 470.59,  21.62,  31.25]]
                )
  , 8:np.array([[ 479.36, 135.09,  48.00],
                [ 486.86, 135.09,  48.00],
                [ 486.86, 135.09,  40.50],
                [ 479.36, 135.09,  40.50]]
                )
  , 9:np.array([[ 158.34, 141.09,  48.00],
                [ 158.34, 148.59,  48.00],
                [ 158.34, 148.59,  40.50],
                [ 158.34, 141.09,  40.50]]
                )
  ,10:np.array([[ 492.88, 155.09,  48.00],
                [ 492.88, 162.59,  48.00],
                [ 492.88, 162.59,  40.50],
                [ 492.88, 155.09,  40.50]]
                )
  ,11:np.array([[ 486.86, 182.60,  48.00],
                [ 479.36, 182.60,  48.00],
                [ 479.36, 182.60,  40.50],
                [ 486.86, 182.60,  40.50]]
                )
  ,12:np.array([[ 470.59, 288.56,  38.75],
                [ 470.59, 296.06,  38.75],
                [ 470.59, 296.06,  31.25],
                [ 470.59, 288.56,  31.25]]
                )
  ,13:np.array([[ 650.92, 295.22,  25.50],
                [ 650.92, 287.72,  25.50],
                [ 650.92, 287.72,  18.00],
                [ 650.92, 295.22,  18.00]]
                )
  ,14:np.array([[ 650.92, 278.22,  25.50],
                [ 650.92, 270.72,  25.50],
                [ 650.92, 270.72,  18.00],
                [ 650.92, 278.22,  18.00]]
                )
  ,15:np.array([[ 650.90, 173.97,  25.50],
                [ 650.90, 166.47,  25.50],
                [ 650.90, 166.47,  18.00],
                [ 650.90, 173.97,  18.00]]
                )
  ,16:np.array([[ 650.90, 156.97,  25.50],
                [ 650.90, 149.47,  25.50],
                [ 650.90, 149.47,  18.00],
                [ 650.90, 156.97,  18.00]]
                )
  ,17:np.array([[ 183.59,  21.62,  38.75],
                [ 183.59,  29.12,  38.75],
                [ 183.59,  29.12,  31.25],
                [ 183.59,  21.62,  31.25]]
                )
  ,18:np.array([[ 178.36, 135.09,  48.00],
                [ 185.86, 135.09,  48.00],
                [ 185.86, 135.09,  40.50],
                [ 178.36, 135.09,  40.50]]
                )
  ,19:np.array([[ 205.87, 141.09,  48.00],
                [ 205.87, 148.59,  48.00],
                [ 205.87, 148.59,  40.50],
                [ 205.87, 141.09,  40.50]]
                )
  ,20:np.array([[ 205.87, 155.09,  48.00],
                [ 205.87, 162.59,  48.00],
                [ 205.87, 162.59,  40.50],
                [ 205.87, 155.09,  40.50]]
                )
  ,21:np.array([[ 185.86, 182.60,  48.00],
                [ 178.36, 182.60,  48.00],
                [ 178.36, 182.60,  40.50],
                [ 185.86, 182.60,  40.50]]
                )
  ,22:np.array([[ 183.59, 288.56,  38.75],
                [ 183.59, 296.06,  38.75],
                [ 183.59, 296.06,  31.25],
                [ 183.59, 288.56,  31.25]]
                )
  ,23:np.array([[ 180.64, 296.06,  38.75],
                [ 180.64, 288.56,  38.75],
                [ 180.64, 288.56,  31.25],
                [ 180.64, 296.06,  31.25]]
                )
  ,24:np.array([[ 171.86, 182.60,  48.00],
                [ 164.36, 182.60,  48.00],
                [ 164.36, 182.60,  40.50],
                [ 171.86, 182.60,  40.50]]
                )
  ,25:np.array([[ 158.34, 176.59,  48.00],
                [ 158.34, 169.09,  48.00],
                [ 158.34, 169.09,  40.50],
                [ 158.34, 176.59,  40.50]]
                )
  ,26:np.array([[ 158.34, 162.59,  48.00],
                [ 158.34, 155.09,  48.00],
                [ 158.34, 155.09,  40.50],
                [ 158.34, 162.59,  40.50]]
                )
  ,27:np.array([[ 164.36, 135.09,  48.00],
                [ 171.86, 135.09,  48.00],
                [ 171.86, 135.09,  40.50],
                [ 164.36, 135.09,  40.50]]
                )
  ,28:np.array([[ 180.64,  29.12,  38.75],
                [ 180.64,  21.62,  38.75],
                [ 180.64,  21.62,  31.25],
                [ 180.64,  29.12,  31.25]]
                )
  ,29:np.array([[   0.30,  22.47,  25.50],
                [   0.30,  29.97,  25.50],
                [   0.30,  29.97,  18.00],
                [   0.30,  22.47,  18.00]]
                )
  ,30:np.array([[   0.30,  39.47,  25.50],
                [   0.30,  46.97,  25.50],
                [   0.30,  46.97,  18.00],
                [   0.30,  39.47,  18.00]]
                )
  ,31:np.array([[   0.32, 143.72,  25.50],
                [   0.32, 151.22,  25.50],
                [   0.32, 151.22,  18.00],
                [   0.32, 143.72,  18.00]]
                )
  ,32:np.array([[   0.32, 160.72,  25.50],
                [   0.32, 168.22,  25.50],
                [   0.32, 168.22,  18.00],
                [   0.32, 160.72,  18.00]]
                )
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
