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
"""
TAG_CORNERS breakdown: TagID,[Vertex coords NW,NE,SE,SW]
"""
TAG_CORNERS = {
    1:np.array([[ 470.890, 295.560,  35.000],
                [ 464.390, 295.560,  35.000],
                [ 464.390, 289.060,  35.000],
                [ 470.890, 289.060,  35.000]]
                )
  , 2:np.array([[ 472.360, 179.350,  44.250],
                [ 472.360, 185.850,  44.250],
                [ 465.860, 185.850,  44.250],
                [ 465.860, 179.350,  44.250]]
                )
  , 3:np.array([[ 448.600, 176.090,  44.250],
                [ 442.100, 176.090,  44.250],
                [ 442.100, 169.590,  44.250],
                [ 448.600, 169.590,  44.250]]
                )
  , 4:np.array([[ 448.600, 162.090,  44.250],
                [ 442.100, 162.090,  44.250],
                [ 442.100, 155.590,  44.250],
                [ 448.600, 155.590,  44.250]]
                )
  , 5:np.array([[ 465.860, 138.340,  44.250],
                [ 465.860, 131.840,  44.250],
                [ 472.360, 131.840,  44.250],
                [ 472.360, 138.340,  44.250]]
                )
  , 6:np.array([[ 470.890,  28.620,  35.000],
                [ 464.390,  28.620,  35.000],
                [ 464.390,  22.120,  35.000],
                [ 470.890,  22.120,  35.000]]
                )
  , 7:np.array([[ 467.340,  22.120,  35.000],
                [ 473.840,  22.120,  35.000],
                [ 473.840,  28.620,  35.000],
                [ 467.340,  28.620,  35.000]]
                )
  , 8:np.array([[ 479.860, 138.340,  44.250],
                [ 479.860, 131.840,  44.250],
                [ 486.360, 131.840,  44.250],
                [ 486.360, 138.340,  44.250]]
                )
  , 9:np.array([[ 155.090, 141.590,  44.250],
                [ 161.590, 141.590,  44.250],
                [ 161.590, 148.090,  44.250],
                [ 155.090, 148.090,  44.250]]
                )
  ,10:np.array([[ 489.630, 155.590,  44.250],
                [ 496.130, 155.590,  44.250],
                [ 496.130, 162.090,  44.250],
                [ 489.630, 162.090,  44.250]]
                )
  ,11:np.array([[ 486.360, 179.350,  44.250],
                [ 486.360, 185.850,  44.250],
                [ 479.860, 185.850,  44.250],
                [ 479.860, 179.350,  44.250]]
                )
  ,12:np.array([[ 467.340, 289.060,  35.000],
                [ 473.840, 289.060,  35.000],
                [ 473.840, 295.560,  35.000],
                [ 467.340, 295.560,  35.000]]
                )
  ,13:np.array([[ 654.170, 294.720,  21.750],
                [ 647.670, 294.720,  21.750],
                [ 647.670, 288.220,  21.750],
                [ 654.170, 288.220,  21.750]]
                )
  ,14:np.array([[ 654.170, 277.720,  21.750],
                [ 647.670, 277.720,  21.750],
                [ 647.670, 271.220,  21.750],
                [ 654.170, 271.220,  21.750]]
                )
  ,15:np.array([[ 654.150, 173.470,  21.750],
                [ 647.650, 173.470,  21.750],
                [ 647.650, 166.970,  21.750],
                [ 654.150, 166.970,  21.750]]
                )
  ,16:np.array([[ 654.150, 156.470,  21.750],
                [ 647.650, 156.470,  21.750],
                [ 647.650, 149.970,  21.750],
                [ 654.150, 149.970,  21.750]]
                )
  ,17:np.array([[ 180.340,  22.120,  35.000],
                [ 186.840,  22.120,  35.000],
                [ 186.840,  28.620,  35.000],
                [ 180.340,  28.620,  35.000]]
                )
  ,18:np.array([[ 178.860, 138.340,  44.250],
                [ 178.860, 131.840,  44.250],
                [ 185.360, 131.840,  44.250],
                [ 185.360, 138.340,  44.250]]
                )
  ,19:np.array([[ 202.620, 141.590,  44.250],
                [ 209.120, 141.590,  44.250],
                [ 209.120, 148.090,  44.250],
                [ 202.620, 148.090,  44.250]]
                )
  ,20:np.array([[ 202.620, 155.590,  44.250],
                [ 209.120, 155.590,  44.250],
                [ 209.120, 162.090,  44.250],
                [ 202.620, 162.090,  44.250]]
                )
  ,21:np.array([[ 185.360, 179.350,  44.250],
                [ 185.360, 185.850,  44.250],
                [ 178.860, 185.850,  44.250],
                [ 178.860, 179.350,  44.250]]
                )
  ,22:np.array([[ 180.340, 289.060,  35.000],
                [ 186.840, 289.060,  35.000],
                [ 186.840, 295.560,  35.000],
                [ 180.340, 295.560,  35.000]]
                )
  ,23:np.array([[ 183.890, 295.560,  35.000],
                [ 177.390, 295.560,  35.000],
                [ 177.390, 289.060,  35.000],
                [ 183.890, 289.060,  35.000]]
                )
  ,24:np.array([[ 171.360, 179.350,  44.250],
                [ 171.360, 185.850,  44.250],
                [ 164.860, 185.850,  44.250],
                [ 164.860, 179.350,  44.250]]
                )
  ,25:np.array([[ 161.590, 176.090,  44.250],
                [ 155.090, 176.090,  44.250],
                [ 155.090, 169.590,  44.250],
                [ 161.590, 169.590,  44.250]]
                )
  ,26:np.array([[ 161.590, 162.090,  44.250],
                [ 155.090, 162.090,  44.250],
                [ 155.090, 155.590,  44.250],
                [ 161.590, 155.590,  44.250]]
                )
  ,27:np.array([[ 164.860, 138.340,  44.250],
                [ 164.860, 131.840,  44.250],
                [ 171.360, 131.840,  44.250],
                [ 171.360, 138.340,  44.250]]
                )
  ,28:np.array([[ 183.890,  28.620,  35.000],
                [ 177.390,  28.620,  35.000],
                [ 177.390,  22.120,  35.000],
                [ 183.890,  22.120,  35.000]]
                )
  ,29:np.array([[  -2.950,  22.970,  21.750],
                [   3.550,  22.970,  21.750],
                [   3.550,  29.470,  21.750],
                [  -2.950,  29.470,  21.750]]
                )
  ,30:np.array([[  -2.950,  39.970,  21.750],
                [   3.550,  39.970,  21.750],
                [   3.550,  46.470,  21.750],
                [  -2.950,  46.470,  21.750]]
                )
  ,31:np.array([[  -2.930, 144.220,  21.750],
                [   3.570, 144.220,  21.750],
                [   3.570, 150.720,  21.750],
                [  -2.930, 150.720,  21.750]]
                )
  ,32:np.array([[  -2.930, 161.220,  21.750],
                [   3.570, 161.220,  21.750],
                [   3.570, 167.720,  21.750],
                [  -2.930, 167.720,  21.750]]
                )
}

# These are the tags in the Vision Lounge. Use them for testing not competition
# LOUNGE_TAG_CORNERS breakdown: TagID,[Vertex coords NW,NE,SE,SW]
#TAG_CORNERS = {
#    1:np.array([[  15.250,  -3.250,  21.500],
#                [  15.250,   3.250,  21.500],
#                [   8.750,   3.250,  21.500],
#                [   8.750,  -3.250,  21.500]]
#                )
#  , 2:np.array([[  29.250,  -3.250,  21.500],
#                [  29.250,   3.250,  21.500],
#                [  22.750,   3.250,  21.500],
#                [  22.750,  -3.250,  21.500]]
#                )
#  , 3:np.array([[  63.250,  -3.250,  21.500],
#                [  63.250,   3.250,  21.500],
#                [  56.750,   3.250,  21.500],
#                [  56.750,  -3.250,  21.500]]
#                )
#  , 4:np.array([[  99.250,  -3.250,  21.500],
#                [  99.250,   3.250,  21.500],
#                [  92.750,   3.250,  21.500],
#                [  92.750,  -3.250,  21.500]]
#                )
#  , 5:np.array([[ 146.250,  82.750,  21.500],
#                [ 139.750,  82.750,  21.500],
#                [ 139.750,  76.250,  21.500],
#                [ 146.250,  76.250,  21.500]]
#                )
#  , 6:np.array([[ 146.250, 118.750,  21.500],
#                [ 139.750, 118.750,  21.500],
#                [ 139.750, 112.250,  21.500],
#                [ 146.250, 112.250,  21.500]]
#                )
#  , 7:np.array([[ 146.250, 132.750,  21.500],
#                [ 139.750, 132.750,  21.500],
#                [ 139.750, 126.250,  21.500],
#                [ 146.250, 126.250,  21.500]]
#                )
#  , 8:np.array([[ 116.750, 199.750,  21.500],
#                [ 116.750, 193.250,  21.500],
#                [ 123.250, 193.250,  21.500],
#                [ 123.250, 199.750,  21.500]]
#                )
#}

def pose (results,Cam):
    """
    pose is called for each camera frame. It returns that camera's robot pose
    value in field coordinates.
    """
    SumX, SumY, SumYaw, N = 0.0, 0.0, 0.0, 0
    for r in results:
        try:
            ret, rvecs, tvecs = cv2.solvePnP(TAG_CORNERS[r.tag_id],r.corners,
                 Cam.mtx,Cam.dist, cv2.SOLVEPNP_IPPE_SQUARE)
        except:
            print (Cam.usage,"failed.",TAG_CORNERS[r.tag_id],r.corners,Cam.mtx,Cam.dist)
        ZYX,jac  = cv2.Rodrigues(rvecs)
        CamPos   = -np.matrix(ZYX).T * np.matrix(tvecs)
        CamX, CamY, CamZ = CamPos[0].item(), CamPos[1].item(), CamPos[2].item()
        #CamYaw, _, _  = frcYPR(results)
        BotX = CamX - Cam.localX   # Where this cam thinks robot X is based on single tag
        BotY = CamY - Cam.localY   # Where this cam thinks robot Y is based on single tag
        SumX += BotX               # Sum X values from all tags this cam sees.
        SumY += BotY               # Sum Y values from all tags this cam sees.
        N += 1                     # Count up all the tags this cam sees.
        print (f'{Cam.usage:10s} BotXY {BotX:>6.2f} {BotY:6.2f}  CamXY {CamX:>6.2f} {CamY:>6.2f}')
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
