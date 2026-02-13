# $Source: /home/scrobotics/src/2026/RCS/PiggyVision26.py,v $
# $Revision: 2.2 $
# $Date: 2026/02/12 02:42:13 $
# $Author: scrobotics $
import json
import math
import numpy as np
import cv2
# Tag size in inches
TAG_SIZE = 6.5
HALF = TAG_SIZE / 2.0

# AprilTag local coordinate frame (WPILib-compatible)
TAG_OBJECT_POINTS = np.array([
    [-HALF,  HALF, 0.0],   # top-left
    [ HALF,  HALF, 0.0],   # top-right
    [ HALF, -HALF, 0.0],   # bottom-right
    [-HALF, -HALF, 0.0],   # bottom-left
], dtype=np.float32)

class DetectedTags:
    def __init__(self, id, rvec, tvec):
        self.id = id
        self.rvec = rvec
        self.tvec = tvec

class PoseEstimate:
   def __init__(self, robot_xyz, robot_yaw, avg_dist, num_tags):
       self.robot_xyz = robot_xyz
       self.robot_yaw = robot_yaw
       self.avg_dist = avg_dist
       self.num_tags = num_tags

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
                                #self.localX   = place['localX']
                                #self.localY   = place['localY']
                                #self.localZ   = place['localZ']
                                self.localXYZ = np.array([place['localX'],place['localY'],place['localZ']])
                                self.pitch    = place['pitch']
                                self.localYaw = place['yaw']
                            except:
                                 print ("Can't load",XYZPY,"Format error?")
                    except:
                        print ("Can't open",XYZPY)
                    self.robotPose = None
                    self.yaw       = None  # Re-set for each tag. Changes like crazy.
                    self.x         = 0.0
                    self.y         = 0.0
                    self.z         = 0.0
                    self.Skew      = None  # Re-set for each tag. Changes like crazy.
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
    axis points to front of the robot, Y points to the right. Z is up. Pitch and
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

def tag_pose_world(tag_xyz, tag_yaw_deg):
    try:
        yaw = np.deg2rad(tag_yaw_deg)
    
        z_w = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        up  = np.array([0,0,1.0])
    
        x_w = np.cross(up, z_w)
        x_w /= np.linalg.norm(x_w)
    
        y_w = np.cross(z_w, x_w)
    
        R_wt = np.column_stack((x_w, y_w, z_w))
        t_wt = np.array(tag_xyz).reshape(3,1)
    
        return R_wt, t_wt
    except Exception as e:
        print ('tag_pose_world')
        print (e)
        return None, None

def camera_pose_world_from_tag(rvec, tvec, tag_xyz, tag_yaw_deg):
    try:
        # --- solvePnP result ---
        R_ct, _ = cv2.Rodrigues(rvec)
        t_ct = tvec.reshape(3,1)
    
        # --- invert to get camera in tag frame ---
        R_tc = R_ct.T
        t_tc = -R_tc @ t_ct
    
        # --- tag world pose ---
        R_wt, t_wt = tag_pose_world(tag_xyz, tag_yaw_deg)
    
        # --- chain transforms ---
        R_wc = R_wt @ R_tc
        t_wc = R_wt @ t_tc + t_wt
    
        # --- WPILib yaw ---
        yaw = np.arctan2(R_wc[1,0], R_wc[0,0])
        yaw_deg = np.rad2deg(yaw)
    
        # ---- planar AprilTag ambiguity fix ----
        yaw_deg = yaw_deg - 180.0
        if yaw_deg < -180:
            yaw_deg += 360
    
        return t_wc.flatten(), yaw_deg

    except Exception as e:
        print('camera_pose_world_from_tag')
        print(e)
        return None,None

def robot_pose_from_camera(
    camera_xyz,
    camera_yaw_deg,
    cam_offset_xyz,        # camera position in robot frame
    cam_yaw_rel_robot_deg  # camera rotation relative to robot
    ):
    try:
        cam = np.array(camera_xyz)
    
        # Step 1: robot yaw from camera yaw
        robot_yaw = camera_yaw_deg - cam_yaw_rel_robot_deg
    
        # Step 2: rotate camera offset into world frame
        yaw = np.deg2rad(robot_yaw)
        R = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0,0,1]
        ])
    
        offset_world = R @ np.array(cam_offset_xyz)
    
        # Step 3: subtract offset
        robot_world = cam - offset_world
    
        return robot_world, robot_yaw
    except Exception as e:
        print ('robot_pose_from_camera error')
        print (e)

def fuse_camera_pose_multitag(detections, TAG_DB):
    """
    detections: list of detections in current frame
        each detection must contain:
            id, rvec, tvec

    TAG_DB: dictionary of tag world poses from CSV
    """

    weighted_pos_sum = np.zeros(3)
    weight_sum = 0.0

    yaw_vec_sum = np.zeros(2)

    for det in detections:

        tag_id = det.id
        rvec   = det.rvec
        tvec   = det.tvec

        tag_xyz = TAG_DB[tag_id]["center"]
        tag_yaw = TAG_DB[tag_id]["yaw"]

        cam_world, yaw_deg = camera_pose_world_from_tag(
            rvec, tvec, tag_xyz, tag_yaw
        )

        # distance camera→tag
        dist = np.linalg.norm(tvec)
        w = 1.0 / (dist * dist)

        # accumulate position
        weighted_pos_sum += w * cam_world
        weight_sum += w

        # accumulate yaw as vector
        yaw_rad = np.deg2rad(yaw_deg)
        yaw_vec_sum += w * np.array([np.cos(yaw_rad), np.sin(yaw_rad)])

    # final fused position
    cam_world_fused = weighted_pos_sum / weight_sum

    # final fused yaw
    fused_yaw = np.rad2deg(np.arctan2(yaw_vec_sum[1], yaw_vec_sum[0]))

    return cam_world_fused, fused_yaw

def fuse_robot_pose_multicam(camera_results):
    """
    camera_results = list of tuples:
        (robot_xyz, robot_yaw, avg_tag_distance, num_tags)
    """

    pos_sum = np.zeros(3)
    yaw_vec_sum = np.zeros(2)
    weight_sum = 0.0

    for robot_xyz, robot_yaw, avg_dist, num_tags in camera_results:

        w = num_tags / (avg_dist * avg_dist)

        pos_sum += w * np.array(robot_xyz)
        weight_sum += w

        yaw_rad = np.deg2rad(robot_yaw)
        yaw_vec_sum += w * np.array([np.cos(yaw_rad), np.sin(yaw_rad)])

    fused_pos = pos_sum / weight_sum
    fused_yaw = np.rad2deg(np.arctan2(yaw_vec_sum[1], yaw_vec_sum[0]))

    return fused_pos, fused_yaw

# These are the tags for competition. Restore them when neeed.
"""
TAG_CORNERS breakdown: TagID,[CtrX,CtrY,CtrZ],ZRot
"""
TAG_CORNERS = {
    1:{"center":np.array([ 467.637, 292.314,  35.000]),"yaw":180.0}
  , 2:{"center":np.array([ 469.111, 182.600,  44.250]),"yaw":90.0}
  , 3:{"center":np.array([ 445.349, 172.844,  44.250]),"yaw":180.0}
  , 4:{"center":np.array([ 445.349, 158.844,  44.250]),"yaw":180.0}
  , 5:{"center":np.array([ 469.111, 135.088,  44.250]),"yaw":270.0}
  , 6:{"center":np.array([ 467.637,  25.374,  35.000]),"yaw":180.0}
  , 7:{"center":np.array([ 470.586,  25.374,  35.000]),"yaw": 0.0}
  , 8:{"center":np.array([ 483.111, 135.088,  44.250]),"yaw":270.0}
  , 9:{"center":np.array([ 492.881, 144.844,  44.250]),"yaw": 0.0}
  ,10:{"center":np.array([ 492.881, 158.844,  44.250]),"yaw": 0.0}
  ,11:{"center":np.array([ 483.111, 182.600,  44.250]),"yaw":90.0}
  ,12:{"center":np.array([ 470.586, 292.314,  35.000]),"yaw": 0.0}
  ,13:{"center":np.array([ 650.918, 291.469,  21.750]),"yaw":180.0}
  ,14:{"center":np.array([ 650.918, 274.469,  21.750]),"yaw":180.0}
  ,15:{"center":np.array([ 650.904, 170.219,  21.750]),"yaw":180.0}
  ,16:{"center":np.array([ 650.904, 153.219,  21.750]),"yaw":180.0}
  ,17:{"center":np.array([ 183.586,  25.374,  35.000]),"yaw": 0.0}
  ,18:{"center":np.array([ 182.111, 135.088,  44.250]),"yaw":270.0}
  ,19:{"center":np.array([ 205.873, 144.844,  44.250]),"yaw": 0.0}
  ,20:{"center":np.array([ 205.873, 158.844,  44.250]),"yaw": 0.0}
  ,21:{"center":np.array([ 182.111, 182.600,  44.250]),"yaw":90.0}
  ,22:{"center":np.array([ 183.586, 292.314,  35.000]),"yaw": 0.0}
  ,23:{"center":np.array([ 180.637, 292.314,  35.000]),"yaw":180.0}
  ,24:{"center":np.array([ 168.111, 182.600,  44.250]),"yaw":90.0}
  ,25:{"center":np.array([ 158.341, 172.844,  44.250]),"yaw":180.0}
  ,26:{"center":np.array([ 158.341, 158.844,  44.250]),"yaw":180.0}
  ,27:{"center":np.array([ 168.111, 135.088,  44.250]),"yaw":270.0}
  ,28:{"center":np.array([ 180.637,  25.374,  35.000]),"yaw":180.0}
  ,29:{"center":np.array([   0.305,  26.219,  21.750]),"yaw": 0.0}
  ,30:{"center":np.array([   0.305,  43.219,  21.750]),"yaw": 0.0}
  ,31:{"center":np.array([   0.318, 147.469,  21.750]),"yaw": 0.0}
  ,32:{"center":np.array([   0.318, 164.469,  21.750]),"yaw": 0.0}
}

# These are the tags in the Vision Lounge. Use them for testing not competition
#"""
#TAG_CORNERS breakdown: TagID,[CtrX,CtrY,CtrZ],ZRot
#"""
#TAG_CORNERS = {
#    1:{"center":np.array([  12.000,   0.000,  21.500]),"yaw":90.0}
#  , 2:{"center":np.array([  26.000,   0.000,  21.500]),"yaw":90.0}
#  , 3:{"center":np.array([  60.000,   0.000,  21.500]),"yaw":90.0}
#  , 4:{"center":np.array([  96.000,   0.000,  21.500]),"yaw":90.0}
#  , 5:{"center":np.array([ 143.000,  79.500,  21.500]),"yaw":180.0}
#  , 6:{"center":np.array([ 143.000, 115.500,  21.500]),"yaw":180.0}
#  , 7:{"center":np.array([ 143.000, 129.500,  21.500]),"yaw":180.0}
#  , 8:{"center":np.array([ 120.000, 196.500,  21.500]),"yaw":270.0}
#}

def pose (results,Cam):
    from math import atan, atan2, asin, degrees
    from time import sleep
    distances     = []
    detected_tags = [] # Will collect all the tag IDs seen by this camera plus their rvecs & tvecs.
    for r in results:
        try:
            img_pts = r.corners                # This is a hack that re-sequences
            test_img_pts = img_pts[[0,1,2,3]]  # the r.corners values. Why? Because.
            ret, rvec, tvec = cv2.solvePnP(TAG_OBJECT_POINTS,test_img_pts,
                        Cam.mtx,Cam.dist, cv2.SOLVEPNP_IPPE_SQUARE)
            dist = np.linalg.norm(tvec)
            distances.append(dist)
            detected_tags.append (DetectedTags(r.tag_id, rvec, tvec))
        except Exception as e:
            print ('pv.pose error')
            print (e)
            sleep (1)
            return None,None
    cam_world, cam_yaw     = fuse_camera_pose_multitag (detected_tags, TAG_CORNERS)
    #robot_world, robot_yaw = robot_pose_from_camera(cam_world, cam_yaw, Cam.localXYZ, Cam.localYaw)
    robot_xyz,   robot_yaw = robot_pose_from_camera(cam_world, cam_yaw, Cam.localXYZ, Cam.localYaw)
    avg_dist               = np.mean(distances)
    num_tags               = len(distances)
    #print (f'{Cam.usage:10s} {round(cam_world[0],1):5.1f} {round(cam_world[1],1):5.1f} {round(cam_yaw,1):4.1f}')
    #print ("robot     ",round(robot_xyz[0],1),round(robot_xyz[1],1),round(robot_yaw,1))
    #print ()
    return PoseEstimate(robot_xyz, robot_yaw, avg_dist, num_tags)
    #return (robot_world, robot_yaw)

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
