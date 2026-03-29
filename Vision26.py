#!/usr/bin/env python3
# $Source: /home/scrobotics/src/2026/RCS/Vision26.py,v $
# $Revision: 1.7 $
# $Date: 2026/03/29 14:23:08 $
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
from pprint import pprint
import PiggyVision26 as pv
from math import degrees

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from ntcore import NetworkTableInstance, EventFlags

configFile = "/boot/frc.json"
#configFile = "./frc.json"

class CameraConfig: pass

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
        frame_time, input_img = cam.input_stream.grabFrame(cam.imgBuf)
        img_info = (frame_time, input_img)
        cam.queue.append(img_info)

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

    output_stream = CameraServer.putVideo("Overlay", 640, 480)
    options = apriltag.DetectorOptions(
        families      = "tag36h11",
        quad_decimate = 2,
        quad_blur     = 0.0, 
        refine_edges  = 1,
        refine_decode = 1,
        nthreads      = 4,
        refine_pose   = 0,
        quad_contours = 1)
    detector = apriltag.Detector(options)
    # loop forever
    counter = 300
    start = time.time()
    ballCount = 0
    prev_time = 0
    while True:
        camera_estimates = []
        for Cam in CamQs:
            try:
                try:
                    frame_time,frame = Cam.queue[0]       # non-destructive read
                    if frame_time != timers[Cam.name]:
                        counter -= 1
                        timers[Cam.name] = frame_time
                    gray = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
                    results = detector.detect(gray)
                    if len(results) > 0:
                        estimate = pv.pose(results,Cam,frame_time)
                        if estimate is not None:
                            camera_estimates.append(estimate)
                    else:
                        continue
                    if counter < 1:
                        stop = time.time()
                        counter = 300
                        print (round(counter/(stop - start),1),'fps')
                        start = stop
                except Exception as e:
                    print ('frame processing')
                    print (e)
                    pass 
            except:
                pass 
        if len(camera_estimates) > 0:
            fused_estimate = pv.fuse_robot_pose_multicam(camera_estimates)
            Display["BOTX"] = round(fused_estimate.robot_xyz[0].item(),1)
            Display["BOTY"] = round(fused_estimate.robot_xyz[1].item(),1)
            Display["YAW "] = round(fused_estimate.robot_yaw,1)
            pubRobotWorldX.set(fused_estimate.robot_xyz[0].item())
            pubRobotWorldY.set(fused_estimate.robot_xyz[1].item())
            pubRobotWorldR.set(fused_estimate.robot_yaw)

            overlay(frame,Display,Cam.width,Cam.height)
            output_stream.putFrame(frame)
            #print ('fused:', round(robot_xyz[0],1), round(robot_xyz[1],1), round(robot_yaw,1))
