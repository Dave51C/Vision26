#!/usr/bin/env python3
# $Source: /home/scrobotics/src/2026/RCS/Vision26.py,v $
# $Revision: 2.2 $
# $Date: 2026/04/11 00:50:03 $
# $Author: scrobotics $

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import cv2
import sys
import apriltag
from collections import deque
import threading
import numpy as np
import traceback
from pprint import pprint
import PiggyVision26 as pv
from math import degrees

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags, _now

configFile = "/boot/frc.json"
#configFile = "./frc.json"

class CameraConfig: pass

class VisionTable:
    def __init__(self):
        base = ntinst.getTable("Vision26")
        self.cameras = {
            name: CameraTable(base, name)
            for name in ["LeftCam", "RightCam", "BackCam"]
        }
        self.fused = CameraTable(base, "Fused")

    def publish_camera(self, name, pose):
        self.cameras[name].publish(pose)

    def publish_fused(self, pose):
        self.fused.publish(pose)

class CameraTable:
    def __init__(self, base_table, name):
        self._heartbeat_counter = 0
        self.table        = base_table.getSubTable(name)
        self.robot_X      = self.table.getDoubleTopic("robot_X").publish()
        self.robot_Y      = self.table.getDoubleTopic("robot_Y").publish()
        self.robot_Z      = self.table.getDoubleTopic("robot_Z").publish()
        self.robot_yaw    = self.table.getDoubleTopic("robot_yaw").publish()
        self.timestamp    = self.table.getDoubleTopic("timestamp").publish()
        self.latency      = self.table.getDoubleTopic("latency").publish()
        self.tag_ids      = self.table.getIntegerArrayTopic("tag_ids").publish()
        self.tag_count    = self.table.getIntegerTopic("tag_count").publish()
        self.avg_distance = self.table.getDoubleTopic("avg_distance").publish()
        self.ambiguity    = self.table.getDoubleTopic("ambiguity").publish()
        self.thetax       = self.table.getDoubleTopic("thetax").publish()
        self.thetay       = self.table.getDoubleTopic("thetay").publish()
        self.reproj       = self.table.getDoubleTopic("reproj_error").publish()
        self.std_x        = self.table.getDoubleTopic("std_x").publish()
        self.std_y        = self.table.getDoubleTopic("std_y").publish()
        self.std_yaw      = self.table.getDoubleTopic("std_yaw").publish()
        self.connected    = self.table.getBooleanTopic("connected").publish()
        self.heartbeat    = self.table.getIntegerTopic("heartbeat").publish()
        self.valid        = self.table.getBooleanTopic("valid").publish()
    def publish(self, pe):
        if pe is None:
            self.valid.set(False)
            return
        self.robot_X.set(pe.robot_X)
        self.robot_Y.set(pe.robot_Y)
        self.robot_Z.set(pe.robot_Z)
        self.robot_yaw.set(pe.robot_yaw)
        self.timestamp.set(pe.timestamp)
        self.latency.set((_now() / 1_000_000.0) - pe.timestamp)
        self.tag_count.set(pe.tag_count)
        self.tag_ids.set(pe.tag_ids if pe.tag_ids is not None else [])
        self.avg_distance.set(pe.avg_distance if pe.avg_distance is not None else 0.0)
        self.ambiguity.set(pe.ambiguity if pe.ambiguity is not None else 0.0)
        self.reproj.set(pe.avg_reproj_error if pe.avg_reproj_error is not None else 0.0)
        self.std_x.set(pe.std_dev_x if pe.std_dev_x is not None else 0.0)
        self.std_y.set(pe.std_dev_y if pe.std_dev_y is not None else 0.0)
        self.std_yaw.set(pe.std_dev_yaw if pe.std_dev_yaw is not None else 0.0)
        self.thetax.set(pe.thetax if pe.thetax is not None else 0.0)
        self.thetay.set(pe.thetay if pe.thetay is not None else 0.0)
        self._heartbeat_counter += 1
        self.heartbeat.set(self._heartbeat_counter)
        self.connected.set(True)
        self.valid.set(True)

team                  = None
server                = False
cameraConfigs         = []
switchedCameraConfigs = []
cameras               = []
CamQs                 = []
Display               = {}
timers                = {}

def overlay(frame,Display,width,height):
    # last param (16) is for anti-alias line type LINE_AA
    col,row = 50,100
    try:
        cv2.line (frame, (int(width/2),0), (int(width/2),height), (0,255,0), 5, 16)
        for label in Display.items():
            cv2.putText (frame, label[0]+"="+str(round(label[1],1)),(col,row),
                     cv2.FONT_HERSHEY_PLAIN, 3.0, (0,0,255), 3)
            row+=50
    except Exception as e:
        print('overlay')
        print(e)

def queueImage (cam):
    import apriltag
    print ("Queueing ",cam.name)
    while True:
        try:
            frame_time, input_img = cam.input_stream.grabFrame(cam.imgBuf)
            if frame_time is None or frame_time <= 0:
                continue
            frame_time = frame_time / 1000000.0 # convert from μs to seconds.
            img_info = (frame_time, input_img)
            cam.queue.append(img_info)
        except:
            print("nothin from",cam.name)
            pass

def customizeCamera(config):
    # Create queue
    camQ = config.name
    match config.name:
        case 'FrontCam':
            print ('customizing FrontCam')
            FrontCam = pv.BotCam('FrontCam')
            FrontCam.input_stream = CameraServer.getVideo('FrontCam')
            FrontCam.imgBuf = np.zeros(shape=(config.height, config.width, 3), dtype=np.uint8)
            ImgT=threading.Thread(target=queueImage,args=(FrontCam,),daemon=True)
            ImgT.start()
            timers['FrontCam'] = 0
            return FrontCam
        case 'LeftCam':
            print ('customizing LeftCam')
            LeftCam = pv.BotCam('LeftCam')
            LeftCam.input_stream = CameraServer.getVideo('LeftCam')
            LeftCam.imgBuf = np.zeros(shape=(config.height, config.width, 3), dtype=np.uint8)
            ImgT=threading.Thread(target=queueImage,args=(LeftCam,),daemon=True)
            ImgT.start()
            timers['LeftCam'] = 0
            return LeftCam
        case 'RightCam':
            print ('customizing RightCam')
            RightCam = pv.BotCam('RightCam')
            RightCam.input_stream = CameraServer.getVideo('RightCam')
            RightCam.imgBuf = np.zeros(shape=(config.height, config.width, 3), dtype=np.uint8)
            ImgT=threading.Thread(target=queueImage,args=(RightCam,),daemon=True)
            ImgT.start()
            timers['RightCam'] = 0
            return RightCam
        case 'BackCam':
            print ('customizing BackCam')
            BackCam = pv.BotCam('BackCam')
            BackCam.input_stream = CameraServer.getVideo('BackCam')
            BackCam.imgBuf = np.zeros(shape=(config.height, config.width, 3), dtype=np.uint8)
            ImgT=threading.Thread(target=queueImage,args=(BackCam,),daemon=True)
            ImgT.start()
            timers['BackCam'] = 0
            return BackCam
        case _:
            print ('Unknown camera name:',config.name)

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    #cam.usage  = config["properties"][0]["value"]
    cam.height = config["height"]
    cam.width  = config["width"]

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    camera = UsbCamera(config.name, config.path)
    server = CameraServer.startAutomaticCapture(camera=camera)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kConnectionKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.addSwitchedCamera(config.name)

    def listener(event):
        data = event.data
        if data is not None:
            value = data.value.value()
            if isinstance(value, int):
                if value >= 0 and value < len(cameras):
                    server.setSource(cameras[value])
            elif isinstance(value, float):
                i = int(value)
                if i >= 0 and i < len(cameras):
                    server.setSource(cameras[i])
            elif isinstance(value, str):
                for i in range(len(cameraConfigs)):
                    if value == cameraConfigs[i].name:
                        server.setSource(cameras[i])
                        break

    NetworkTableInstance.getDefault().addListener(
        NetworkTableInstance.getDefault().getEntry(config.key),
        EventFlags.kImmediate | EventFlags.kValueAll,
        listener)

    return server

def establish_topics():
    global BotPos_tbl,pubRobotWorldX,pubRobotWorldY,pubRobotWorldR,pubHubRng,pubHubHdg
    BotPos_tbl     = ntinst.getTable("BotPos")
    pubRobotWorldX = BotPos_tbl.getDoubleTopic("Robot_X").publish()
    pubRobotWorldY = BotPos_tbl.getDoubleTopic("Robot_Y").publish()
    pubRobotWorldR = BotPos_tbl.getDoubleTopic("Robot_Rot").publish()
    pubHubRng      = BotPos_tbl.getDoubleTopic("Hub_Rng").publish()
    pubHubHdg      = BotPos_tbl.getDoubleTopic("Hub_Hdg").publish()
    return

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    try:
        Cfile   = open('config.json','r')
        j       = json.load(Cfile)
        Horizon = j['Horizon']
        IP      = j['IP']
        Logging = j['Logging']
        LogDir  = j['LogDir']
        Cfile.close()
    except:
        print ("Failed to process config.json")

    # start NetworkTables
    ntinst = NetworkTableInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient4("Vision26")
        ntinst.setServer(IP)
        ntinst.startDSClient()
    establish_topics()
    # start cameras
    # work around wpilibsuite/allwpilib#5055
    #CameraServer.setSize(CameraServer.kSize160x120)
    for config in cameraConfigs:
        cameras.append(startCamera(config))
        CamQs.append(customizeCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    #########################################################################
    # Our code starts. Buckle-up!
    #########################################################################

    V26           = VisionTable() 
    customTable   = pv.CustomVisionTable(ntinst)
    output_stream = CameraServer.putVideo("Overlay", 640, 480)
    options       = apriltag.DetectorOptions(
        families      = "tag36h11",
        quad_decimate = 2,
        quad_blur     = 0.0, 
        refine_edges  = 1,
        refine_decode = 1,
        nthreads      = 4,
        refine_pose   = 0,
        quad_contours = 1)
    detector  = apriltag.Detector(options)
    counter   = 300
    procstart = time.time()
    ballCount = 0
    prev_time = 0
    # loop forever
    while True:
        camera_estimates = []
        for Cam in CamQs:
            try:
                frame_time,frame = Cam.queue[0]       # non-destructive read
                procstart        = time.time()
                start            = frame_time
                if frame_time != timers[Cam.name]:
                    counter -= 1
                    timers[Cam.name] = frame_time
                gray     = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
                results  = detector.detect(gray)
                estimate = None
                if len(results) > 0:
                    try:
                        estimate = pv.pose(results,Cam,frame_time)
                    except Exception as e:
                        print (e)
                V26.publish_camera(Cam.name, estimate)
                if estimate is not None:
                    camera_estimates.append(estimate)
                else:
                    continue
                if len(results) > 0:
                    targets = pv.build_targets(results, Cam)
                    proc_ms = (time.time() - procstart) * 1000.0
                    customTable.publish(targets, frame_time, proc_ms)

                try:
                    if counter < 1:
                        stop    = frame_time
                        counter = 300
                        if stop > start:
                            print (round(counter/(stop - start),1),'fps')
                        start   = stop
                except Exception as e:
                    print ('frame processing')
                    print (e)
                    traceback.print_exc()
                    pass 
            except:
                pass 
        if len(camera_estimates) > 0:
            camera_estimates = [e for e in camera_estimates if e is not None]
            if not camera_estimates:
                fused_estimate = None
            else:
                fused_estimate = pv.fuse_robot_pose_multicam(camera_estimates)
            if fused_estimate is not None:
                V26.publish_fused(fused_estimate)
            else:
                V26.publish_fused(None)
            Display["BOTX"] = round(fused_estimate.robot_X,1)
            Display["BOTY"] = round(fused_estimate.robot_Y,1)
            Display["YAW "] = round(fused_estimate.robot_yaw,1)
            pubRobotWorldX.set(fused_estimate.robot_X)
            pubRobotWorldY.set(fused_estimate.robot_Y)
            pubRobotWorldR.set(fused_estimate.robot_yaw)

            overlay(frame,Display,Cam.width,Cam.height)
            output_stream.putFrame(frame)
            #print ('fused:', round(robot_xyz[0],1), round(robot_xyz[1],1), round(robot_yaw,1))

