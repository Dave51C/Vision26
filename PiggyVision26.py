# $Source: /home/scrobotics/src/2026/RCS/PiggyVision26.py,v $
# $Revision: 3.2 $
# $Date: 2026/02/26 16:58:28 $
# $Author: scrobotics $
import json
import math
import numpy as np
import cv2
# Tag size in inches
TAG_SIZE = 6.5
HALF = TAG_SIZE / 2.0

# NOTE: This is NOT the WPILib order
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
    def __init__(self, robot_xyz, robot_yaw, avg_distance, num_tags, timestamp):
        self.robot_xyz = robot_xyz
        self.robot_yaw = robot_yaw
        self.avg_distance = avg_distance
        self.num_tags = num_tags
        self.timestamp = timestamp

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
        self.prev_rvec = None
        self.prev_tvec = None
        BotCam.list.append(self)    # Keep a list of cameras on bot

def tag_pose_world(tag_xyz, tag_yaw_deg):
    try:
        yaw = np.deg2rad(tag_yaw_deg)

        # Tag normal (Z_tag)
        z_axis = np.array([ np.cos(yaw), np.sin(yaw), 0.0 ])

        # Tag right (X_tag)
        x_axis = np.array([ -np.sin(yaw), np.cos(yaw), 0.0 ])

        # Tag up (Y_tag)
        y_axis = np.array([ 0.0, 0.0, 1.0 ])

        R_wt = np.column_stack((x_axis, y_axis, z_axis))
        t_wt = np.asarray(tag_xyz, dtype=float)

        return R_wt, t_wt
    except Exception as e:
        print ('tag_pose_world')
        print (e)
        return None, None

def camera_pose_world_from_tag( rvec, tvec, tag_xyz, tag_yaw_deg):
    try:
        R_ct_cv, _ = cv2.Rodrigues(rvec)
        t_ct_cv = tvec.reshape(3)
        
        R_camFix = np.array([ [0, 0, 1], [-1, 0, 0], [0, -1, 0] ])
        
        R_ct = R_camFix @ R_ct_cv
        t_ct = R_camFix @ t_ct_cv
        
        R_tc = R_ct.T
        t_tc = -R_tc @ t_ct
        
        R_wt, t_wt = tag_pose_world(tag_xyz, tag_yaw_deg)
        
        R_wc = R_wt @ R_tc
        t_wc = R_wt @ t_tc + t_wt
        
        camera_yaw = np.rad2deg(np.arctan2(R_wc[1,0], R_wc[0,0])) 
        return t_wc, camera_yaw
    except Exception as e:
        print('camera_pose_world_from_tag')
        print (e)
        return None, None

def camera_to_robot_world(camera_world, camera_yaw_deg, cam):
    try:
        # --- 1) Convert yaw to radians ---
        camera_yaw_rad = np.deg2rad(camera_yaw_deg)
    
        # Robot yaw = camera yaw - mounting yaw
        robot_yaw_rad = ( camera_yaw_rad - np.deg2rad(cam.localYaw))
    
        # --- 2) Rotate camera offset into world frame ---
        offset_forward = cam.localXYZ[0]
        offset_left    = cam.localXYZ[1]
    
        robot_x = camera_world[0] - (
            offset_forward * np.cos(robot_yaw_rad)
            - offset_left  * np.sin(robot_yaw_rad)
        )
    
        robot_y = camera_world[1] - (
            offset_forward * np.sin(robot_yaw_rad)
            + offset_left  * np.cos(robot_yaw_rad)
        )
    
        robot_world = np.array([ robot_x, robot_y, 0.0 ])
        robot_yaw_deg = np.rad2deg(robot_yaw_rad)
    
        return robot_world, robot_yaw_deg
    except Exception as e:
        print ('camera_to_robot_world')
        print (e)
        return None, None

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

def fuse_camera_pose_multitag(detections, TAG_DB, cam_height):
    try:
        weighted_position_sum = np.zeros(3)
        weight_sum = 0.0
    
        yaw_vector_sum = np.zeros(2)
    
        for det in detections:
            tag_id = det.id
            rvec   = det.rvec
            tvec   = det.tvec
            tag_xyz = TAG_DB[tag_id]["center"]
            tag_yaw = TAG_DB[tag_id]["yaw"]
    
            # --- Per-tag camera pose ---
            camera_world, camera_yaw = \
                camera_pose_world_from_tag( rvec, tvec, tag_xyz, tag_yaw)
    
            if camera_world is None:
                continue
    
            # ? Eliminate this by returning R_wt,t_wt from camera_pose_world_from_tag?
            R_wt,t_wt = tag_pose_world(tag_xyz, tag_yaw)

            tag_normal = R_wt[:,2]

            view_dir = camera_world - t_wt
            view_dir /= np.linalg.norm(view_dir)

            angle_factor = abs(np.dot(tag_normal, view_dir))

            distance = np.linalg.norm(tvec)

            weight = (1.0 / (distance * distance)) * angle_factor
           
            weighted_position_sum += weight * camera_world
            weight_sum += weight
    
            # --- Yaw vector accumulation ---
            yaw_rad = np.deg2rad(camera_yaw)
            yaw_vector_sum += weight * np.array([ np.cos(yaw_rad), np.sin(yaw_rad) ])
    
        if weight_sum == 0:
            return None, None
    
        fused_camera_world = weighted_position_sum / weight_sum
    
        fused_camera_yaw = np.rad2deg( np.arctan2( yaw_vector_sum[1], yaw_vector_sum[0]))
    
        return fused_camera_world, fused_camera_yaw
    except Exception as e:
        print ('fuse_camera_pose_multitag')
        print (e)
        return None, None

def fuse_robot_pose_multicam(camera_results):
    """
    camera_results = list of tuples:
        (robot_xyz, robot_yaw, avg_tag_distance, num_tags)
    """
    try:
        pos_sum = np.zeros(3)
        yaw_vec_sum = np.zeros(2)
        weight_sum = 0.0
    
        for robot_xyz, robot_yaw, avg_distance, num_tags in camera_results:
    
            w = num_tags / (avg_distance * avg_distance)
    
            pos_sum += w * np.array(robot_xyz)
            weight_sum += w
    
            yaw_rad = np.deg2rad(robot_yaw)
            yaw_vec_sum += w * np.array([np.cos(yaw_rad), np.sin(yaw_rad)])
    
        fused_pos = pos_sum / weight_sum
        fused_yaw = np.rad2deg(np.arctan2(yaw_vec_sum[1], yaw_vec_sum[0]))

        return fused_pos, fused_yaw
    except Exception as e:
        print ('fuse_robot_pose_multicam')
        print (e)
        return None, None

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
    def show_debugging_info():
        print (f'{Cam.usage:>10s},{r.tag_id:>2d},tag_world={tag_xyz},tag_yaw_deg={tag_yaw_deg}, rvec={rvec},\ntvec={tvec},\ncamera_world={camera_world},camera_yaw={camera_yaw},\nrobot_world={robot_xyz},robot_yaw={robot_yaw}\n\n')
    from math import atan, atan2, asin, degrees
    import time
    frame_timestamp = time.time()
    distances     = []
    detected_tags = [] # Will collect all the tag IDs seen by this camera plus their rvecs & tvecs.
    for r in results:
        try:
            FLAGS = cv2.SOLVEPNP_IPPE_SQUARE
            ret,rvec,tvec = cv2.solvePnP(TAG_OBJECT_POINTS,r.corners,Cam.mtx,Cam.dist,flags=FLAGS)
            distance = np.linalg.norm(tvec)
            distances.append(distance)
            detected_tags.append (DetectedTags(r.tag_id, rvec, tvec))
        except Exception as e:
            print ('pv.pose error')
            print (e)
            return None,None
    tag_xyz = TAG_CORNERS[r.tag_id]["center"]
    tag_yaw_deg = TAG_CORNERS[r.tag_id]["yaw"]
    camera_world, camera_yaw = fuse_camera_pose_multitag (
                               detected_tags, TAG_CORNERS, Cam.localXYZ[2])
    robot_xyz, robot_yaw   = camera_to_robot_world (camera_world, camera_yaw, Cam)
    #print("camera world:", camera_world)
    #print("camera yaw:", camera_yaw)
    #print("robot world:", robot_xyz)
    #print("robot yaw:", robot_yaw)
    #show_debugging_info()

    avg_distance           = np.mean(distances)
    num_tags               = len(distances)
    return PoseEstimate(robot_xyz, robot_yaw, avg_distance, num_tags, frame_timestamp)
    #return robot_xyz, robot_yaw
    #return PoseEstimate(None, None, None, None, None)

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
