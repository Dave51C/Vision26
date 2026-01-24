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
                    #self.robotPose = None
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
# TAG_CORNERS breakdown: TagID,[Vertex coords NW,NE,SE,SW]
#TAG_CORNERS = {
#    1:np.array([[ 467.637, 296.064,  38.750],
#                [ 467.637, 288.564,  38.750],
#                [ 467.637, 288.564,  31.250],
#                [ 467.637, 296.064,  31.250]]
#                )
#  , 2:np.array([[ 472.861, 182.600,  48.000],
#                [ 465.361, 182.600,  48.000],
#                [ 465.361, 182.600,  40.500],
#                [ 472.861, 182.600,  40.500]]
#                )
#  , 3:np.array([[ 445.349, 176.594,  48.000],
#                [ 445.349, 169.094,  48.000],
#                [ 445.349, 169.094,  40.500],
#                [ 445.349, 176.594,  40.500]]
#                )
#  , 4:np.array([[ 445.349, 162.594,  48.000],
#                [ 445.349, 155.094,  48.000],
#                [ 445.349, 155.094,  40.500],
#                [ 445.349, 162.594,  40.500]]
#                )
#  , 5:np.array([[ 465.361, 135.088,  48.000],
#                [ 472.861, 135.088,  48.000],
#                [ 472.861, 135.088,  40.500],
#                [ 465.361, 135.088,  40.500]]
#                )
#  , 6:np.array([[ 467.637,  29.124,  38.750],
#                [ 467.637,  21.624,  38.750],
#                [ 467.637,  21.624,  31.250],
#                [ 467.637,  29.124,  31.250]]
#                )
#  , 7:np.array([[ 470.586,  21.624,  38.750],
#                [ 470.586,  29.124,  38.750],
#                [ 470.586,  29.124,  31.250],
#                [ 470.586,  21.624,  31.250]]
#                )
#  , 8:np.array([[ 479.361, 135.088,  48.000],
#                [ 486.861, 135.088,  48.000],
#                [ 486.861, 135.088,  40.500],
#                [ 479.361, 135.088,  40.500]]
#                )
#  , 9:np.array([[ 492.881, 141.094,  48.000],
#                [ 492.881, 148.594,  48.000],
#                [ 492.881, 148.594,  40.500],
#                [ 492.881, 141.094,  40.500]]
#                )
#  ,10:np.array([[ 492.881, 155.094,  48.000],
#                [ 492.881, 162.594,  48.000],
#                [ 492.881, 162.594,  40.500],
#                [ 492.881, 155.094,  40.500]]
#                )
#  ,11:np.array([[ 486.861, 182.600,  48.000],
#                [ 479.361, 182.600,  48.000],
#                [ 479.361, 182.600,  40.500],
#                [ 486.861, 182.600,  40.500]]
#                )
#  ,12:np.array([[ 470.586, 288.564,  38.750],
#                [ 470.586, 296.064,  38.750],
#                [ 470.586, 296.064,  31.250],
#                [ 470.586, 288.564,  31.250]]
#                )
#  ,13:np.array([[ 650.918, 295.219,  25.500],
#                [ 650.918, 287.719,  25.500],
#                [ 650.918, 287.719,  18.000],
#                [ 650.918, 295.219,  18.000]]
#                )
#  ,14:np.array([[ 650.918, 278.219,  25.500],
#                [ 650.918, 270.719,  25.500],
#                [ 650.918, 270.719,  18.000],
#                [ 650.918, 278.219,  18.000]]
#                )
#  ,15:np.array([[ 650.904, 173.969,  25.500],
#                [ 650.904, 166.469,  25.500],
#                [ 650.904, 166.469,  18.000],
#                [ 650.904, 173.969,  18.000]]
#                )
#  ,16:np.array([[ 650.904, 156.969,  25.500],
#                [ 650.904, 149.469,  25.500],
#                [ 650.904, 149.469,  18.000],
#                [ 650.904, 156.969,  18.000]]
#                )
#  ,17:np.array([[ 183.586,  21.624,  38.750],
#                [ 183.586,  29.124,  38.750],
#                [ 183.586,  29.124,  31.250],
#                [ 183.586,  21.624,  31.250]]
#                )
#  ,18:np.array([[ 178.361, 135.088,  48.000],
#                [ 185.861, 135.088,  48.000],
#                [ 185.861, 135.088,  40.500],
#                [ 178.361, 135.088,  40.500]]
#                )
#  ,19:np.array([[ 205.873, 141.094,  48.000],
#                [ 205.873, 148.594,  48.000],
#                [ 205.873, 148.594,  40.500],
#                [ 205.873, 141.094,  40.500]]
#                )
#  ,20:np.array([[ 205.873, 155.094,  48.000],
#                [ 205.873, 162.594,  48.000],
#                [ 205.873, 162.594,  40.500],
#                [ 205.873, 155.094,  40.500]]
#                )
#  ,21:np.array([[ 185.861, 182.600,  48.000],
#                [ 178.361, 182.600,  48.000],
#                [ 178.361, 182.600,  40.500],
#                [ 185.861, 182.600,  40.500]]
#                )
#  ,22:np.array([[ 183.586, 288.564,  38.750],
#                [ 183.586, 296.064,  38.750],
#                [ 183.586, 296.064,  31.250],
#                [ 183.586, 288.564,  31.250]]
#                )
#  ,23:np.array([[ 180.637, 296.064,  38.750],
#                [ 180.637, 288.564,  38.750],
#                [ 180.637, 288.564,  31.250],
#                [ 180.637, 296.064,  31.250]]
#                )
#  ,24:np.array([[ 171.861, 182.600,  48.000],
#                [ 164.361, 182.600,  48.000],
#                [ 164.361, 182.600,  40.500],
#                [ 171.861, 182.600,  40.500]]
#                )
#  ,25:np.array([[ 158.341, 176.594,  48.000],
#                [ 158.341, 169.094,  48.000],
#                [ 158.341, 169.094,  40.500],
#                [ 158.341, 176.594,  40.500]]
#                )
#  ,26:np.array([[ 158.341, 162.594,  48.000],
#                [ 158.341, 155.094,  48.000],
#                [ 158.341, 155.094,  40.500],
#                [ 158.341, 162.594,  40.500]]
#                )
#  ,27:np.array([[ 164.361, 135.088,  48.000],
#                [ 171.861, 135.088,  48.000],
#                [ 171.861, 135.088,  40.500],
#                [ 164.361, 135.088,  40.500]]
#                )
#  ,28:np.array([[ 180.637,  29.124,  38.750],
#                [ 180.637,  21.624,  38.750],
#                [ 180.637,  21.624,  31.250],
#                [ 180.637,  29.124,  31.250]]
#                )
#  ,29:np.array([[   0.305,  22.469,  25.500],
#                [   0.305,  29.969,  25.500],
#                [   0.305,  29.969,  18.000],
#                [   0.305,  22.469,  18.000]]
#                )
#  ,30:np.array([[   0.305,  39.469,  25.500],
#                [   0.305,  46.969,  25.500],
#                [   0.305,  46.969,  18.000],
#                [   0.305,  39.469,  18.000]]
#                )
#  ,31:np.array([[   0.318, 143.719,  25.500],
#                [   0.318, 151.219,  25.500],
#                [   0.318, 151.219,  18.000],
#                [   0.318, 143.719,  18.000]]
#                )
#  ,32:np.array([[   0.318, 160.719,  25.500],
#                [   0.318, 168.219,  25.500],
#                [   0.318, 168.219,  18.000],
#                [   0.318, 160.719,  18.000]]
#                )
#}

# These are the tags in the Vision Lounge. Use them for testing not competition
# LOUNGE_TAG_CORNERS breakdown: TagID,[Vertex coords NW,NE,SE,SW]
TAG_CORNERS = {
    1:np.array([[  15.75,  -0.00,  25.25],
                [   8.25,   0.00,  25.25],
                [   8.25,   0.00,  17.75],
                [  15.75,  -0.00,  17.75]]
                )
  , 2:np.array([[  29.75,  -0.00,  25.25],
                [  22.25,   0.00,  25.25],
                [  22.25,   0.00,  17.75],
                [  29.75,  -0.00,  17.75]]
                )
  , 3:np.array([[  63.75,  -0.00,  25.25],
                [  56.25,   0.00,  25.25],
                [  56.25,   0.00,  17.75],
                [  63.75,  -0.00,  17.75]]
                )
  , 4:np.array([[  99.75,  -0.00,  25.25],
                [  92.25,   0.00,  25.25],
                [  92.25,   0.00,  17.75],
                [  99.75,  -0.00,  17.75]]
                )
  , 5:np.array([[ 143.00,  83.25,  25.25],
                [ 143.00,  75.75,  25.25],
                [ 143.00,  75.75,  17.75],
                [ 143.00,  83.25,  17.75]]
                )
  , 6:np.array([[ 143.00, 119.25,  25.25],
                [ 143.00, 111.75,  25.25],
                [ 143.00, 111.75,  17.75],
                [ 143.00, 119.25,  17.75]]
                )
  , 7:np.array([[ 143.00, 133.25,  25.25],
                [ 143.00, 125.75,  25.25],
                [ 143.00, 125.75,  17.75],
                [ 143.00, 133.25,  17.75]]
                )
  , 8:np.array([[ 116.25, 196.50,  25.25],
                [ 123.75, 196.50,  25.25],
                [ 123.75, 196.50,  17.75],
                [ 116.25, 196.50,  17.75]]
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
