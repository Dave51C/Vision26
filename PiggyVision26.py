# $Source: /home/scrobotics/src/2026/RCS/PiggyVision26.py,v $
# $Revision: 4.2 $
# $Date: 2026/04/11 00:52:19 $
# $Author: scrobotics $
import json
import math
import numpy as np
import cv2
import traceback
import time
from ntcore import NetworkTableInstance
inst   = NetworkTableInstance.getDefault()
#BackTbl  = inst.getTable(f"/Vision26/BackCam")
#LeftTbl  = inst.getTable(f"/Vision26/LeftCam")
#RightTbl = inst.getTable(f"/Vision26/RightCam")

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
    def __init__(self, id, rvec, tvec, camera_world=None, camera_yaw=None, err=None):
        self.id   = id
        self.rvec = rvec
        self.tvec = tvec
        self.camera_world = camera_world
        self.camera_yaw = camera_yaw
        self.err = err

class CustomVisionTable:
    def __init__(self, ntinst):
        self.base = ntinst.getTable("CustomVision")

        self.heartbeat = self.base.getIntegerTopic("heartbeat").publish()
        self.hasTag    = self.base.getBooleanTopic("hasTag").publish()

        self._heartbeat = 0
        self.tag_tables = {}

    def get_tag_table(self, tag_id):
        if tag_id not in self.tag_tables:
            print(f"Creating Tag_{tag_id}") 
            sub = self.base.getSubTable(f"Tag_{tag_id}")
            self.tag_tables[tag_id] = TagTable(sub)
        return self.tag_tables[tag_id]

    def publish(self, targets, frame_time, proc_ms):
        self._heartbeat += 1
        self.heartbeat.set(self._heartbeat)
    
        seen_ids = set()
        now = time.time()
        HOLD_TIME = 0.2
    
        if not targets:
            self.hasTag.set(False)
        else:
            self.hasTag.set(True)
    
            # --- Phase 1: update seen tags ---
            for t in targets:
                seen_ids.add(t.id)
    
                table = self.get_tag_table(t.id)
    
                roll, pitch, yaw = rvecToEulerAngles(t.rvec)
                x, y, z = t.tvec.flatten()
    
                table.tagPose.set([
                    float(x), float(y), float(z),
                    float(roll), float(pitch), float(yaw)
                ])
    
                table.targetId.set(int(t.id))
                table.ambiguity.set(float(t.ambiguity) if t.ambiguity is not None else 0.0)
                table.area.set(float(t.area) if t.area is not None else 0.0)
    
                table.timestamp.set(float(frame_time))
                table.procLat.set(float(proc_ms))
                table.netLat.set(0.0)
                table.totalLat.set(float(proc_ms))
    
                # update time + valid
                table._last_seen = now
                table.lastSeen.set(now)
                table.valid.set(True)
                table.camera.set(t.camera)

        # --- Phase 2: invalidate stale tags ---
        for tag_id, table in self.tag_tables.items():
            if (now - table._last_seen) > HOLD_TIME:
                table.valid.set(False)
                table.lastSeen.set(table._last_seen)  # keep topic alive

class TagTable:
    def __init__(self, table):
        def pub(topic):
            t = topic
            t.setRetained(True)
            return t.publish()
        self.tagPose    = pub(table.getDoubleArrayTopic("tagPose"))
        self.targetId   = pub(table.getIntegerTopic("targetId"))
        self.ambiguity  = pub(table.getDoubleTopic("poseAmbiguity"))
        self.area       = pub(table.getDoubleTopic("tagArea"))
        self.timestamp  = pub(table.getDoubleTopic("publishTimestamp"))
        self.procLat    = pub(table.getDoubleTopic("processingLatency"))
        self.netLat     = pub(table.getDoubleTopic("networkLatency"))
        self.totalLat   = pub(table.getDoubleTopic("totalLatency"))
        self.lastSeen   = pub(table.getDoubleTopic("lastSeen"))
        self.valid      = pub(table.getBooleanTopic("valid"))
        self.camera     = pub(table.getStringTopic("camera"))

class Target:
    def __init__(self, id, rvec, tvec, corners,
                 distance, center_error, area, ambiguity, camera=None):
        self.id = id
        self.rvec = rvec
        self.tvec = tvec
        self.corners = corners
        self.distance = distance
        self.center_error = center_error
        self.area = area
        self.ambiguity = ambiguity
        self.camera = camera

class PoseEstimate:
    def __init__( self, robot_xyz, robot_yaw, avg_distance, num_tags, timestamp,\
        tag_ids=None, std_dev_x=None, std_dev_y=None, std_dev_yaw=None,\
        avg_reproj_error=None, ambiguity=None, camera_name=None, thetax=None, thetay=None
    ):
        self.robot_X          = robot_xyz[0] # send as inches / 39.37  # inches -> meter
        self.robot_Y          = robot_xyz[1] # send as inches / 39.37  # inches -> meter
        self.robot_Z          = robot_xyz[2] # send as inches / 39.37  # inches -> meter
        self.robot_yaw        = robot_yaw
        self.avg_distance     = avg_distance # send as inches / 39.37  # inches -> meter
        self.num_tags         = num_tags
        self.timestamp        = timestamp
        self.tag_ids          = tag_ids or []
        self.tag_count        = len(self.tag_ids)
        self.thetax           = thetax
        self.thetay           = thetay
        self.std_dev_x        = std_dev_x
        self.std_dev_y        = std_dev_y
        self.std_dev_yaw      = std_dev_yaw
        self.avg_reproj_error = avg_reproj_error
        self.ambiguity        = ambiguity
        self.camera_name      = camera_name

class Webcam ():
    def __init__(self, name):
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

        for cam in frc["cameras"]:
            if cam["name"] == name:
                self.CameraName = cam['name']
                self.width      = cam['width']
                self.height     = cam['height']
                self.queue      = deque(maxlen=1)
                self.buffer=np.zeros(shape=(self.height,self.width,3),dtype=np.uint8)
                paramFile       = f'{cam["name"]}.json'
                try:
                    with open(paramFile,'r') as pfile:
                        j = json.load(pfile)
                except Exception as e:
                    print("Can't open", paramFile)
                    print (e)
                try:
                    self.mtx  = np.array(j['mtx'])
                except Exception as e:
                    print ("Can't set mtx")
                    print (e)
                try:
                    self.dist = np.array(j['dist'])
                except Exception as e:
                    print ("Can't set dist")
                    print (e)
                self.localXYZ = np.array([j['localX'],j['localY'],j['localZ']])
                self.pitch    = j['pitch']
                self.localYaw = np.deg2rad(j['yaw'])
                #print (self.localYaw, j['yaw'], j['name'])
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
    def __init__(self, name):
        super().__init__(name)
        self.name = name
        BotCam.list.append(self)    # Keep a list of cameras on bot

def compute_std_devs(camera_yaw, avg_distance, avg_reproj_error):
    # Tunable constants
    k_depth = 0.15
    k_lat   = 0.05
    # Camera-frame uncertainties
    depth_std   = k_depth * avg_distance * (1 + avg_reproj_error)
    lateral_std = k_lat   * avg_distance * (1 + avg_reproj_error)
    # Covariance in camera XY (X = lateral, Z = forward)
    cov_cam_xy = np.array([ [lateral_std**2, 0], [0, depth_std**2] ])

    # Rotation into world frame
    theta = camera_yaw
    R = np.array([ [np.cos(theta), -np.sin(theta)], [np.sin(theta),  np.cos(theta)] ])

    cov_world = R @ cov_cam_xy @ R.T

    std_dev_x = np.sqrt(cov_world[0,0])
    std_dev_y = np.sqrt(cov_world[1,1])

    return std_dev_x, std_dev_y

def compute_std_dev_yaw(avg_distance, avg_reproj_error, num_tags, angle_factor):
    k_yaw = 0.1
    # Prevent divide-by-zero
    angle_factor = max(angle_factor, 1e-3)
    num_tags = max(num_tags, 1)
    std_dev_yaw = (
        k_yaw * avg_distance
        * (1 + avg_reproj_error) / (np.sqrt(num_tags) * angle_factor)
    )
    return std_dev_yaw

def compute_reprojection_error( obj_pts, img_pts, rvec, tvec, mtx, dist=None,\
    use_undistorted=False):
    """
    Compute mean reprojection error in pixels.

    Parameters:
        obj_pts : (N,3) 3D object points (e.g. TAG_OBJECT_POINTS)
        img_pts : (N,2) detected image points (corners OR undistorted_pts)
        rvec, tvec : pose from solvePnP
        mtx : camera matrix
        dist : distortion coefficients (None if undistorted)
        use_undistorted : True if img_pts are already undistorted

    Returns:
        mean reprojection error (float, pixels)
    """

    # --- Project points using SAME model as solvePnP ---
    if use_undistorted:
        # Undistorted pipeline → no distortion
        projected, _ = cv2.projectPoints( obj_pts, rvec, tvec, mtx, None)
    else:
        # Normal pinhole pipeline
        projected, _ = cv2.projectPoints( obj_pts, rvec, tvec, mtx, dist)

    # --- Reshape ---
    projected = projected.reshape(-1, 2)
    img_pts   = img_pts.reshape(-1, 2)

    # --- Per-point error ---
    errors = np.linalg.norm(projected - img_pts, axis=1)

    # --- Return mean error ---
    return float(np.mean(errors))

def rvecToEulerAngles(rvec):
    R,_ = cv2.Rodrigues(rvec)
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = math.atan2(R[2,1], R[2,2])   # X
        pitch = math.atan2(-R[2,0], sy)      # Y
        yaw   = math.atan2(R[1,0], R[0,0])   # Z
    else:
        roll  = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = 0
    return np.array([roll, pitch, yaw])

def compute_target_angles_from_ray(px, py, mtx):
    """
    Compute horizontal (thetax) and vertical (thetay) angles
    from pixel coordinates using camera intrinsics.
    Works with undistorted points (pinhole model).
    """
    fx = mtx[0,0]
    fy = mtx[1,1]
    cx = mtx[0,2]
    cy = mtx[1,2]
    # Normalize to camera ray
    x = (px - cx) / fx
    y = (py - cy) / fy
    # Angles
    thetax = np.arctan2(x, 1.0)
    thetay = np.arctan2(y, 1.0)
    return thetax, thetay

def tag_pose_world(tag_xyz, tag_yaw):
    try:
        # Tag normal (Z_tag)
        z_axis = np.array([ np.cos(tag_yaw), np.sin(tag_yaw), 0.0 ])

        # Tag right (X_tag)
        x_axis = np.array([ -np.sin(tag_yaw), np.cos(tag_yaw), 0.0 ])

        # Tag up (Y_tag)
        y_axis = np.array([ 0.0, 0.0, 1.0 ])

        R_wt = np.column_stack((x_axis, y_axis, z_axis))
        t_wt = np.asarray(tag_xyz, dtype=float)

        return R_wt, t_wt
    except Exception as e:
        print ('tag_pose_world')
        print (e)
        traceback.print_exc()
        #return None, None
        return None

def camera_pose_world_from_tag( rvec, tvec, tag_xyz, tag_yaw):
    try:
        R_ct_cv, _ = cv2.Rodrigues(rvec)
        t_ct_cv = tvec.reshape(3)
        
        R_camFix = np.array([ [0, 0, 1], [-1, 0, 0], [0, -1, 0] ])
        
        R_ct = R_camFix @ R_ct_cv
        t_ct = R_camFix @ t_ct_cv
        
        R_tc = R_ct.T
        t_tc = -R_tc @ t_ct
        
        try:
            R_wt, t_wt = tag_pose_world(tag_xyz, tag_yaw)
        except Exception as e:
            print(e,"tag_pose_world") 
            traceback.print_exc()
        R_wc = R_wt @ R_tc
        t_wc = R_wt @ t_tc + t_wt
        
        camera_yaw = np.arctan2(R_wc[1,0], R_wc[0,0])
        return t_wc, camera_yaw
    except Exception as e:
        print('camera_pose_world_from_tag')
        print (e)
        traceback.print_exc()
        #return None, None
        return None

def camera_to_robot_world(camera_world, camera_yaw, cam):
    try:
    
        # Robot yaw = camera yaw - mounting yaw
        robot_yaw_rad = camera_yaw - cam.localYaw
    
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
        #robot_yaw_deg = np.rad2deg(robot_yaw_rad)
    
        return robot_world, robot_yaw_rad
    except Exception as e:
        print ('camera_to_robot_world')
        print (e)
        traceback.print_exc()
        #return None, None
        return None

def robot_pose_from_camera(
    camera_xyz,
    camera_yaw,
    cam_offset_xyz,        # camera position in robot frame
    cam_yaw_rel_robot      # camera rotation relative to robot
    ):
    try:
        cam = np.array(camera_xyz)
    
        # Step 1: robot yaw from camera yaw
        robot_yaw = camera_yaw - cam_yaw_rel_robot
    
        # Step 2: rotate camera offset into world frame
        R = np.array([
            [np.cos(robot_yaw), -np.sin(robot_yaw), 0],
            [np.sin(robot_yaw),  np.cos(robot_yaw), 0],
            [0,0,1]
        ])
    
        offset_world = R @ np.array(cam_offset_xyz)
    
        # Step 3: subtract offset
        robot_world = cam - offset_world
    
        return robot_world, robot_yaw
    except Exception as e:
        print ('robot_pose_from_camera error')
        print (e)
        traceback.print_exc()

def build_targets(results, Cam):
    targets = []

    cx_img = Cam.width / 2.0
    cy_img = Cam.height / 2.0

    for r in results:
        try:
            corners = r.corners.astype(np.float32)

            # --- SolvePnP ---
            if len(Cam.dist) == 4:
                undistorted_pts = cv2.fisheye.undistortPoints(
                    corners.reshape(-1,1,2),
                    Cam.mtx,
                    Cam.dist,
                    P=Cam.mtx
                )

                ret, rvecs, tvecs, reprojErrs = cv2.solvePnPGeneric(
                    TAG_OBJECT_POINTS,
                    undistorted_pts,
                    Cam.mtx,
                    None,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                img_pts = undistorted_pts
                distCoeffs = None

            else:
                ret, rvecs, tvecs, reprojErrs = cv2.solvePnPGeneric(
                    TAG_OBJECT_POINTS,
                    corners,
                    Cam.mtx,
                    Cam.dist,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                img_pts = corners
                distCoeffs = Cam.dist

            if not ret or len(rvecs) == 0:
                continue

            # --- Use best solution ---
            rvec = rvecs[0]
            tvec = tvecs[0]

            # --- Distance ---
            distance = np.linalg.norm(tvec)

            # --- Center error ---
            cx = np.mean(corners[:,0])
            cy = np.mean(corners[:,1])

            dx = cx - cx_img
            dy = cy - cy_img
            center_error = np.sqrt(dx*dx + dy*dy)

            # --- Area ---
            area_px = cv2.contourArea(corners)
            frame_area = Cam.width * Cam.height
            area_percent = 100.0 * area_px / frame_area

            # --- Ambiguity (IPPE dual solution) ---
            if len(reprojErrs) >= 2:
                err1 = reprojErrs[0]
                err2 = reprojErrs[1]
                ambiguity = abs(err1 - err2) / (err1 + err2 + 1e-6)
            else:
                ambiguity = 0.0

            targets.append(Target(
                r.tag_id,
                rvec,
                tvec,
                corners,
                distance,
                center_error,
                area_percent,
                ambiguity,
                Cam.name
            ))

        except Exception as e:
            print("build_targets error:", e)
            traceback.print_exc()
            continue

    return targets

def fuse_robot_pose_multicam(robot_estimates):
    import traceback
    try:
        if robot_estimates is None:
            return None

        valid_estimates = [
            est for est in robot_estimates
            if (
                est is not None and
                isinstance(est.num_tags, int) and est.num_tags > 0 and
                isinstance(est.avg_distance, (int, float)) and est.avg_distance > 0 and
                est.robot_X is not None and
                est.robot_Y is not None
            )
        ]

        if len(valid_estimates) == 0:
            return None

        weighted_pos_sum = np.zeros(2)
        yaw_vec_sum = np.zeros(2)
        weighted_distance_sum = 0.0
        weight_sum = 0.0
        timestamps = []

        for est in valid_estimates:
            w = est.num_tags / (est.avg_distance ** 2)

            weighted_pos_sum += w * np.array([est.robot_X, est.robot_Y])
            yaw_vec_sum += w * np.array([np.cos(est.robot_yaw), np.sin(est.robot_yaw)])
            weighted_distance_sum += w * est.avg_distance

            weight_sum += w

            if isinstance(est.timestamp, (int, float)) and est.timestamp > 0:
                timestamps.append(est.timestamp)

        if weight_sum == 0:
            return None

        fused_xy = weighted_pos_sum / weight_sum
        fused_yaw = np.arctan2(yaw_vec_sum[1], yaw_vec_sum[0])
        fused_avg_distance = weighted_distance_sum / weight_sum
        fused_timestamp = max(timestamps) if timestamps else 0.0

        return PoseEstimate(
            robot_xyz=np.array([fused_xy[0], fused_xy[1], 0.0]),
            robot_yaw=fused_yaw,
            avg_distance=fused_avg_distance,
            num_tags=sum(est.num_tags for est in valid_estimates),
            timestamp=fused_timestamp
        )

    except Exception as e:
        print("fuse_robot_pose_multicam error:", e)
        traceback.print_exc()
        return None

def fuse_camera_pose_multitag(detections, TAG_DB, cam_height):
    try:
        weighted_position_sum = np.zeros(3)
        weight_sum = 0.0
        angle_sum  = 0.0
    
        yaw_vector_sum = np.zeros(2)
    
        for det in detections:
            tag_id = det.id
            rvec   = det.rvec
            tvec   = det.tvec
            tag_xyz = TAG_DB[tag_id]["center"]
            tag_yaw = TAG_DB[tag_id]["yaw"]
    
            # --- Per-tag camera pose ---
            camera_world = det.camera_world
            camera_yaw   = det.camera_yaw
            #camera_world, camera_yaw = \
            #    camera_pose_world_from_tag( rvec, tvec, tag_xyz, tag_yaw)
    
            if camera_world is None:
                continue
    
            # ? Eliminate this by returning R_wt,t_wt from camera_pose_world_from_tag?
            R_wt,t_wt = tag_pose_world(tag_xyz, tag_yaw)

            tag_normal = R_wt[:,2]

            view_dir              = camera_world - t_wt
            norm = np.linalg.norm(view_dir)
            if norm < 1e-6:
                continue
            view_dir             /= norm
            angle_factor          = abs(np.dot(tag_normal, view_dir))
            distance              = np.linalg.norm(tvec)
            weight = (1.0 / (distance**2)) * angle_factor * (1.0 / (det.err + 1e-6))
            weighted_position_sum += weight * camera_world
            weight_sum            += weight
            angle_sum             += weight * angle_factor
    
            # --- Yaw vector accumulation ---
            yaw_vector_sum += weight * np.array([ np.cos(camera_yaw), np.sin(camera_yaw) ])
    
        if weight_sum == 0:
            return None
            #return None, None, None, None

        yaw_confidence     = np.linalg.norm(yaw_vector_sum) / weight_sum
        avg_angle_factor   = angle_sum / weight_sum
        fused_camera_world = weighted_position_sum / weight_sum
        fused_camera_yaw   = np.arctan2( yaw_vector_sum[1], yaw_vector_sum[0])
    
        return fused_camera_world, fused_camera_yaw, avg_angle_factor, yaw_confidence
    except Exception as e:
        print ('fuse_camera_pose_multitag')
        print (e)
        traceback.print_exc()
        return None
        #return None, None, None, None

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

def pose (results,Cam,frame_time):
    def show_debugging_info():
        print (f'{Cam.name:>10s},{r.tag_id:>2d},tag_world={tag_xyz},tag_yaw_deg={tag_yaw_deg}, rvec={rvec},\ntvec={tvec},\ncamera_world={camera_world},camera_yaw={camera_yaw},\nrobot_world={robot_xyz},robot_yaw={robot_yaw}\n\n')
    from math import atan, atan2, asin, degrees
    import time
    reproj_errors = []
    distances     = []
    detected_tags = [] # Will collect all the tag IDs seen by this camera plus their rvecs & tvecs.
    tags_in_frame = []
    tag_weights   = []
    ambiguities   = []
    centers       = []
    cx = Cam.width / 2
    cy = Cam.height / 2
    for r in results:
        try:
            corners = r.corners.astype(np.float32)
            if len(Cam.dist) == 4:
                undistorted_pts = cv2.fisheye.undistortPoints(r.corners.reshape(-1,1,2),Cam.mtx,Cam.dist,P=Cam.mtx)
                ret, rvecs, tvecs, pnperrs = cv2.solvePnPGeneric(TAG_OBJECT_POINTS,undistorted_pts,
                            Cam.mtx,None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                best_idx = np.argmin(pnperrs)
                rvec = rvecs[best_idx]
                tvec = tvecs[best_idx]
                err = compute_reprojection_error( TAG_OBJECT_POINTS, undistorted_pts,\
                      rvec, tvec, Cam.mtx, None, use_undistorted=True)
                undistorted_pts = undistorted_pts.reshape(-1,2)
                center = np.mean(undistorted_pts, axis=0)
                centers.append((r, center))
            else:
                ret, rvecs, tvecs, pnperrs = cv2.solvePnPGeneric(TAG_OBJECT_POINTS,r.corners,
                            Cam.mtx,None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
                best_idx = np.argmin(pnperrs)
                rvec = rvecs[best_idx]
                tvec = tvecs[best_idx]
                err = compute_reprojection_error( TAG_OBJECT_POINTS, r.corners,\
                      rvec, tvec, Cam.mtx, Cam.dist, use_undistorted=False)
                center = np.mean(r.corners, axis=0)
                centers.append((r, center))

            reproj_errors.append(err)
            if len(pnperrs) >= 2:
                errs = sorted(pnperrs)
                err1, err2 = errs[0], errs[1]
                ambiguity = abs(err1 - err2) / (err1 + err2 + 1e-6)
            else:
                ambiguity = 0.0
            ambiguities.append(ambiguity)

            distance = np.linalg.norm(tvec)
            distances.append(distance)

            tags_in_frame.append(r.tag_id)

            tag_xyz = TAG_CORNERS[r.tag_id]["center"]
            tag_yaw = TAG_CORNERS[r.tag_id]["yaw"]
            R_wt, t_wt = tag_pose_world(tag_xyz, tag_yaw)

            camera_world_tmp, camera_yaw_tmp = \
                camera_pose_world_from_tag(rvec, tvec, tag_xyz, tag_yaw)

            view_dir = camera_world_tmp - t_wt
            norm = np.linalg.norm(view_dir)
            if norm < 1e-6:
                continue
            view_dir /= norm
            
            angle_factor = abs(np.dot(R_wt[:,2], view_dir))
            
            err = err if err is not None else 1.0
            weight = (1.0 / (distance**2)) * angle_factor * (1.0 / (err + 1e-6))

            tag_weights.append(weight)
            detected_tags.append( DetectedTags(r.tag_id, rvec, tvec, \
                camera_world_tmp, camera_yaw_tmp, err))

        except Exception as e:
            print ('pv.pose error')
            print (e)
            traceback.print_exc()
            return None
    if len(tag_weights) == 0 or sum(tag_weights) < 1e-6 or len(centers) == 0:
        print("Early reject:",
              "weights=", len(tag_weights),
              "sum=", sum(tag_weights),
              "centers=", len(centers))
        return None
    best             = min(centers, key=lambda item: np.linalg.norm(item[1] - [cx, cy]))
    bestx, besty     = best[1]
    try:
        thetax, thetay   = compute_target_angles_from_ray(bestx, besty, Cam.mtx)
    except Exception as e:
        print(e,"compute_target_angles_from_ray")
        traceback.print_exc()
    reproj_errors = np.array(reproj_errors).flatten()
    avg_reproj_error = np.average(reproj_errors, weights=tag_weights)
    #tag_xyz = TAG_CORNERS[r.tag_id]["center"]
    #tag_yaw = TAG_CORNERS[r.tag_id]["yaw"]

    result = fuse_camera_pose_multitag(detected_tags, TAG_CORNERS, Cam.localXYZ[2])
    if result is None:
        print("fusion returned None")
        return None
    camera_world, camera_yaw, avg_angle_factor, yaw_confidence = result

    if camera_world is None:
        return None
    try:
        robot_xyz, robot_yaw   = camera_to_robot_world (camera_world, camera_yaw, Cam)
    except Exception as e:
        print(e,"camera_to_robot_world")
        traceback.print_exc()
    #show_debugging_info()

    if not (len(reproj_errors) == len(tag_weights) == len(distances) == len(ambiguities)):
        print("Length mismatch:",
              len(reproj_errors),
              len(tag_weights),
              len(distances),
              len(ambiguities))
        return None
    distances     = np.array(distances).flatten()
    ambiguities   = np.array(ambiguities).flatten()
    tag_weights   = np.array(tag_weights).flatten()
    avg_distance         = np.average(distances, weights=tag_weights)
    avg_ambiguity        = np.average(ambiguities, weights=tag_weights)
    num_tags             = len(distances)

    # --- REJECTION GATE ---
    #if num_tags > 1:
    #    amb_thresh = 0.5
    #else:
    #    amb_thresh = 0.3
    #if (
    #    avg_reproj_error is None or
    #    avg_reproj_error > 5.0 or
    #    avg_ambiguity < amb_thresh or
    #    num_tags == 0
    #):
    #    return None

    std_dev_x, std_dev_y = compute_std_devs(camera_yaw, avg_distance, avg_reproj_error)
    std_dev_yaw          = compute_std_dev_yaw(avg_distance, avg_reproj_error,\
                           num_tags, avg_angle_factor)
    std_dev_yaw         /= (yaw_confidence + 1e-3)

    return PoseEstimate(robot_xyz, robot_yaw, avg_distance, num_tags,\
        frame_time, tags_in_frame, std_dev_x, std_dev_y, std_dev_yaw,\
        avg_reproj_error,avg_ambiguity,Cam.name,thetax,thetay)

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

# Convert all tag rotations to radians
for T in TAG_CORNERS:
    TAG_CORNERS[T]["yaw"] = np.deg2rad(TAG_CORNERS[T]["yaw"])

if __name__ == "__main__":
    from pprint import pprint
    camList = ["FrontCam","RightCam","LeftCam"]
    for camera in camList:
        BotCam(camera)
    for item in BotCam.list:
        print (item.name)
        print (item.__dict__)
