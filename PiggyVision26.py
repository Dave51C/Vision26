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

    @staticmethod
    def frcYPR(results):
        import cv2
        import numpy as np
        def rotationMatrixToWPILibYawPitchRoll(R):
            """
            Returns yaw, pitch, roll in radians
            WPILib convention:
              yaw   → +Z
              pitch → +Y
              roll  → +X
            """
            sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
            singular = sy < 1e-6
    
            if not singular:
                roll  = np.arctan2(R[2,1], R[2,2])     # X
                pitch = np.arctan2(-R[2,0], sy)        # Y
                yaw   = np.arctan2(R[1,0], R[0,0])     # Z
            else:
                roll  = np.arctan2(-R[1,2], R[1,1])
                pitch = np.arctan2(-R[2,0], sy)
                yaw   = 0.0
            return yaw, pitch, roll
    
        # This roatation matrix transforms conventional axes to wpilib
        R_cam_to_wpilib = np.array([
            [ 0,  0,  1],   # X_wpi =  Z_cv
            [-1,  0,  0],   # Y_wpi = -X_cv
            [ 0, -1,  0]    # Z_wpi = -Y_cv
        ])
        # Compute a pose based on all visible tags.
        for r in results:
            ret, rvec, tvec = cv2.solvePnP(TAG_CORNERS[r.tag_id],
                    r.corners, mtx,dist, cv2.SOLVEPNP_IPPE_SQUARE)

            R_cam, _ = cv2.Rodrigues(rvec)
            R_wpi = R_cam_to_wpilib @ R_cam
        return rotationMatrixToWPILibYawPitchRoll(R_wpi)

import numpy as np
import cv2

# These are the tags for competition. Restore them when neeed.
#"""
#TAG_CORNERS breakdown: TagID,[CtrX,CtrY,CtrZ],ZRot,[Vertex coords NW,NE,SE,SW]
#"""
#TAG_CORNERS = {
#    1:{"center":np.array([ 467.637, 292.314,  35.000]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 467.637, 295.564,  38.250],
#                [ 467.637, 289.064,  38.250],
#                [ 467.637, 289.064,  31.750],
#                [ 467.637, 295.564,  31.750]])
#      }
#  , 2:{"center":np.array([ 469.111, 182.600,  44.250]),
#          "yaw":90.0,
#      "corners":np.array([
#                [ 472.361, 182.600,  47.500],
#                [ 465.861, 182.600,  47.500],
#                [ 465.861, 182.600,  41.000],
#                [ 472.361, 182.600,  41.000]])
#      }
#  , 3:{"center":np.array([ 445.349, 172.844,  44.250]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 445.349, 176.094,  47.500],
#                [ 445.349, 169.594,  47.500],
#                [ 445.349, 169.594,  41.000],
#                [ 445.349, 176.094,  41.000]])
#      }
#  , 4:{"center":np.array([ 445.349, 158.844,  44.250]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 445.349, 162.094,  47.500],
#                [ 445.349, 155.594,  47.500],
#                [ 445.349, 155.594,  41.000],
#                [ 445.349, 162.094,  41.000]])
#      }
#  , 5:{"center":np.array([ 469.111, 135.088,  44.250]),
#          "yaw":270.0,
#      "corners":np.array([
#                [ 465.861, 135.088,  47.500],
#                [ 472.361, 135.088,  47.500],
#                [ 472.361, 135.088,  41.000],
#                [ 465.861, 135.088,  41.000]])
#      }
#  , 6:{"center":np.array([ 467.637,  25.374,  35.000]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 467.637,  28.624,  38.250],
#                [ 467.637,  22.124,  38.250],
#                [ 467.637,  22.124,  31.750],
#                [ 467.637,  28.624,  31.750]])
#      }
#  , 7:{"center":np.array([ 470.586,  25.374,  35.000]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [ 470.586,  22.124,  38.250],
#                [ 470.586,  28.624,  38.250],
#                [ 470.586,  28.624,  31.750],
#                [ 470.586,  22.124,  31.750]])
#      }
#  , 8:{"center":np.array([ 483.111, 135.088,  44.250]),
#          "yaw":270.0,
#      "corners":np.array([
#                [ 479.861, 135.088,  47.500],
#                [ 486.361, 135.088,  47.500],
#                [ 486.361, 135.088,  41.000],
#                [ 479.861, 135.088,  41.000]])
#      }
#  , 9:{"center":np.array([ 492.881, 144.844,  44.250]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [ 492.881, 141.594,  47.500],
#                [ 492.881, 148.094,  47.500],
#                [ 492.881, 148.094,  41.000],
#                [ 492.881, 141.594,  41.000]])
#      }
#  ,10:{"center":np.array([ 492.881, 158.844,  44.250]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [ 492.881, 155.594,  47.500],
#                [ 492.881, 162.094,  47.500],
#                [ 492.881, 162.094,  41.000],
#                [ 492.881, 155.594,  41.000]])
#      }
#  ,11:{"center":np.array([ 483.111, 182.600,  44.250]),
#          "yaw":90.0,
#      "corners":np.array([
#                [ 486.361, 182.600,  47.500],
#                [ 479.861, 182.600,  47.500],
#                [ 479.861, 182.600,  41.000],
#                [ 486.361, 182.600,  41.000]])
#      }
#  ,12:{"center":np.array([ 470.586, 292.314,  35.000]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [ 470.586, 289.064,  38.250],
#                [ 470.586, 295.564,  38.250],
#                [ 470.586, 295.564,  31.750],
#                [ 470.586, 289.064,  31.750]])
#      }
#  ,13:{"center":np.array([ 650.918, 291.469,  21.750]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 650.918, 294.719,  25.000],
#                [ 650.918, 288.219,  25.000],
#                [ 650.918, 288.219,  18.500],
#                [ 650.918, 294.719,  18.500]])
#      }
#  ,14:{"center":np.array([ 650.918, 274.469,  21.750]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 650.918, 277.719,  25.000],
#                [ 650.918, 271.219,  25.000],
#                [ 650.918, 271.219,  18.500],
#                [ 650.918, 277.719,  18.500]])
#      }
#  ,15:{"center":np.array([ 650.904, 170.219,  21.750]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 650.904, 173.469,  25.000],
#                [ 650.904, 166.969,  25.000],
#                [ 650.904, 166.969,  18.500],
#                [ 650.904, 173.469,  18.500]])
#      }
#  ,16:{"center":np.array([ 650.904, 153.219,  21.750]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 650.904, 156.469,  25.000],
#                [ 650.904, 149.969,  25.000],
#                [ 650.904, 149.969,  18.500],
#                [ 650.904, 156.469,  18.500]])
#      }
#  ,17:{"center":np.array([ 183.586,  25.374,  35.000]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [ 183.586,  22.124,  38.250],
#                [ 183.586,  28.624,  38.250],
#                [ 183.586,  28.624,  31.750],
#                [ 183.586,  22.124,  31.750]])
#      }
#  ,18:{"center":np.array([ 182.111, 135.088,  44.250]),
#          "yaw":270.0,
#      "corners":np.array([
#                [ 178.861, 135.088,  47.500],
#                [ 185.361, 135.088,  47.500],
#                [ 185.361, 135.088,  41.000],
#                [ 178.861, 135.088,  41.000]])
#      }
#  ,19:{"center":np.array([ 205.873, 144.844,  44.250]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [ 205.873, 141.594,  47.500],
#                [ 205.873, 148.094,  47.500],
#                [ 205.873, 148.094,  41.000],
#                [ 205.873, 141.594,  41.000]])
#      }
#  ,20:{"center":np.array([ 205.873, 158.844,  44.250]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [ 205.873, 155.594,  47.500],
#                [ 205.873, 162.094,  47.500],
#                [ 205.873, 162.094,  41.000],
#                [ 205.873, 155.594,  41.000]])
#      }
#  ,21:{"center":np.array([ 182.111, 182.600,  44.250]),
#          "yaw":90.0,
#      "corners":np.array([
#                [ 185.361, 182.600,  47.500],
#                [ 178.861, 182.600,  47.500],
#                [ 178.861, 182.600,  41.000],
#                [ 185.361, 182.600,  41.000]])
#      }
#  ,22:{"center":np.array([ 183.586, 292.314,  35.000]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [ 183.586, 289.064,  38.250],
#                [ 183.586, 295.564,  38.250],
#                [ 183.586, 295.564,  31.750],
#                [ 183.586, 289.064,  31.750]])
#      }
#  ,23:{"center":np.array([ 180.637, 292.314,  35.000]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 180.637, 295.564,  38.250],
#                [ 180.637, 289.064,  38.250],
#                [ 180.637, 289.064,  31.750],
#                [ 180.637, 295.564,  31.750]])
#      }
#  ,24:{"center":np.array([ 168.111, 182.600,  44.250]),
#          "yaw":90.0,
#      "corners":np.array([
#                [ 171.361, 182.600,  47.500],
#                [ 164.861, 182.600,  47.500],
#                [ 164.861, 182.600,  41.000],
#                [ 171.361, 182.600,  41.000]])
#      }
#  ,25:{"center":np.array([ 158.341, 172.844,  44.250]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 158.341, 176.094,  47.500],
#                [ 158.341, 169.594,  47.500],
#                [ 158.341, 169.594,  41.000],
#                [ 158.341, 176.094,  41.000]])
#      }
#  ,26:{"center":np.array([ 158.341, 158.844,  44.250]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 158.341, 162.094,  47.500],
#                [ 158.341, 155.594,  47.500],
#                [ 158.341, 155.594,  41.000],
#                [ 158.341, 162.094,  41.000]])
#      }
#  ,27:{"center":np.array([ 168.111, 135.088,  44.250]),
#          "yaw":270.0,
#      "corners":np.array([
#                [ 164.861, 135.088,  47.500],
#                [ 171.361, 135.088,  47.500],
#                [ 171.361, 135.088,  41.000],
#                [ 164.861, 135.088,  41.000]])
#      }
#  ,28:{"center":np.array([ 180.637,  25.374,  35.000]),
#          "yaw":180.0,
#      "corners":np.array([
#                [ 180.637,  28.624,  38.250],
#                [ 180.637,  22.124,  38.250],
#                [ 180.637,  22.124,  31.750],
#                [ 180.637,  28.624,  31.750]])
#      }
#  ,29:{"center":np.array([   0.305,  26.219,  21.750]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [   0.305,  22.969,  25.000],
#                [   0.305,  29.469,  25.000],
#                [   0.305,  29.469,  18.500],
#                [   0.305,  22.969,  18.500]])
#      }
#  ,30:{"center":np.array([   0.305,  43.219,  21.750]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [   0.305,  39.969,  25.000],
#                [   0.305,  46.469,  25.000],
#                [   0.305,  46.469,  18.500],
#                [   0.305,  39.969,  18.500]])
#      }
#  ,31:{"center":np.array([   0.318, 147.469,  21.750]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [   0.318, 144.219,  25.000],
#                [   0.318, 150.719,  25.000],
#                [   0.318, 150.719,  18.500],
#                [   0.318, 144.219,  18.500]])
#      }
#  ,32:{"center":np.array([   0.318, 164.469,  21.750]),
#          "yaw": 0.0,
#      "corners":np.array([
#                [   0.318, 161.219,  25.000],
#                [   0.318, 167.719,  25.000],
#                [   0.318, 167.719,  18.500],
#                [   0.318, 161.219,  18.500]])
#      }
#}

# These are the tags in the Vision Lounge. Use them for testing not competition
"""
TAG_CORNERS breakdown: TagID,[CtrX,CtrY,CtrZ],ZRot,[Vertex coords NW,NE,SE,SW]
"""
TAG_CORNERS = {
    1:{"center":np.array([  12.000,   0.000,  21.500]),
          "yaw":90.0,
      "corners":np.array([
                [  15.250,   0.000,  24.750],
                [   8.750,   0.000,  24.750],
                [   8.750,   0.000,  18.250],
                [  15.250,   0.000,  18.250]])
      }
  , 2:{"center":np.array([  26.000,   0.000,  21.500]),
          "yaw":90.0,
      "corners":np.array([
                [  29.250,   0.000,  24.750],
                [  22.750,   0.000,  24.750],
                [  22.750,   0.000,  18.250],
                [  29.250,   0.000,  18.250]])
      }
  , 3:{"center":np.array([  60.000,   0.000,  21.500]),
          "yaw":90.0,
      "corners":np.array([
                [  63.250,   0.000,  24.750],
                [  56.750,   0.000,  24.750],
                [  56.750,   0.000,  18.250],
                [  63.250,   0.000,  18.250]])
      }
  , 4:{"center":np.array([  96.000,   0.000,  21.500]),
          "yaw":90.0,
      "corners":np.array([
                [  99.250,   0.000,  24.750],
                [  92.750,   0.000,  24.750],
                [  92.750,   0.000,  18.250],
                [  99.250,   0.000,  18.250]])
      }
  , 5:{"center":np.array([ 143.000,  79.500,  21.500]),
          "yaw":180.0,
      "corners":np.array([
                [ 143.000,  82.750,  24.750],
                [ 143.000,  76.250,  24.750],
                [ 143.000,  76.250,  18.250],
                [ 143.000,  82.750,  18.250]])
      }
  , 6:{"center":np.array([ 143.000, 115.500,  21.500]),
          "yaw":180.0,
      "corners":np.array([
                [ 143.000, 118.750,  24.750],
                [ 143.000, 112.250,  24.750],
                [ 143.000, 112.250,  18.250],
                [ 143.000, 118.750,  18.250]])
      }
  , 7:{"center":np.array([ 143.000, 129.500,  21.500]),
          "yaw":180.0,
      "corners":np.array([
                [ 143.000, 132.750,  24.750],
                [ 143.000, 126.250,  24.750],
                [ 143.000, 126.250,  18.250],
                [ 143.000, 132.750,  18.250]])
      }
  , 8:{"center":np.array([ 120.000, 196.500,  21.500]),
          "yaw":270.0,
      "corners":np.array([
                [ 116.750, 196.500,  24.750],
                [ 123.250, 196.500,  24.750],
                [ 123.250, 196.500,  18.250],
                [ 116.750, 196.500,  18.250]])
      }
}

def pose (results,Cam):
    """
    pose is called for each camera frame. It returns that camera's robot pose
    value in field coordinates.
    """
    SumX, SumY, SumYaw, N = 0.0, 0.0, 0.0, 0
    for r in results:
        try:
            ret, rvecs, tvecs = cv2.solvePnP(TAG_CORNERS[r.tag_id]["corners"],r.corners,
                 Cam.mtx,Cam.dist, cv2.SOLVEPNP_IPPE_SQUARE)
        except:
            print (Cam.usage,"failed.",TAG_CORNERS[r.tag_id]["corners"],r.corners,Cam.mtx,Cam.dist)
        ZYX,jac  = cv2.Rodrigues(rvecs)
        CamPos   = -np.matrix(ZYX).T * np.matrix(tvecs)
        CamX, CamY, CamZ = CamPos[0].item(), CamPos[1].item(), CamPos[2].item()
        #CamYaw, _, _  = frcYPR(results)
        BotX = CamX - Cam.localX   # Where this cam thinks robot X is based on single tag
        BotY = CamY - Cam.localY   # Where this cam thinks robot Y is based on single tag
        SumX += BotX               # Sum X values from all tags this cam sees.
        SumY += BotY               # Sum Y values from all tags this cam sees.
        N += 1                     # Count up all the tags this cam sees.
        print (f'{r.tag_id:2d} {Cam.usage:10s} BotXY {BotX:>6.2f} {BotY:6.2f}  CamXYZ {CamX:>6.2f} {CamY:>6.2f}  {CamZ:>6.2f}')
    #print ("AvgX:",SumX/N," AvgY",SumY/N)
    return (SumX/N,SumY/N)         # Average X,Y based on all tags this cam sees.

def rotate(px, py, ox, oy, angle, Integer=False):
    """
    Rotate a point at (px, py) about an origin at (ox, oy) by the given angle.
    The angle is in radians.
    If Integer is True the new values are rounded to the nearest integer
    Return the point's new (X,Y).
    """
    from math import sin, cos
    newx = ox + cos(angle) * (px - ox) - sin(angle) * (py - oy)
    newy = oy + sin(angle) * (px - ox) + cos(angle) * (py - oy)
    if Integer:
        newx = round(newx)
        newy = round(newy)
    return newx, newy


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
