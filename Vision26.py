#!/usr/bin/env python3
# $Source: /home/scrobotics/src/2026/RCS/Vision26.py,v $
# $Revision: 1.2 $
# $Date: 2026/01/26 03:31:59 $
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
from PiggyVision26 import Webcam, BotCam, pose

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

def queueImage (cam):
    import apriltag
    print ("Queueing ",cam.usage)
    while True:
        frame_time, input_img = cam.input_stream.grabFrame(cam.imgBuf)
        #print (cam.usage,frame_time)
        cam.queue.append(input_img)

def customizeCamera(config):
    # Create queue
    camQ = config.usage
    match config.usage:
        case 'DriverCam':
            print ('customizing DriverCam')
            DriverCam = BotCam('DriverCam')
            DriverCam.input_stream = CameraServer.getVideo('DriverCam')
            DriverCam.imgBuf = np.zeros(shape=(config.height, config.width, 3), dtype=np.uint8)
            ImgT=threading.Thread(target=queueImage,args=(DriverCam,),daemon=True)
            ImgT.start()
            return DriverCam
        case 'FrontCam':
            print ('customizing FrontCam')
            FrontCam = BotCam('FrontCam')
            FrontCam.input_stream = CameraServer.getVideo('FrontCam')
            FrontCam.imgBuf = np.zeros(shape=(config.height, config.width, 3), dtype=np.uint8)
            ImgT=threading.Thread(target=queueImage,args=(FrontCam,),daemon=True)
            ImgT.start()
            return FrontCam
        case 'ClimbCam':
            print ('customizing ClimbCam')
            ClimbCam = BotCam('ClimbCam')
            ClimbCam.input_stream = CameraServer.getVideo('ClimbCam')
            ClimbCam.imgBuf = np.zeros(shape=(config.height, config.width, 3), dtype=np.uint8)
            ImgT=threading.Thread(target=queueImage,args=(ClimbCam,),daemon=True)
            ImgT.start()
            return ClimbCam
        case 'ExtraCam':
            print ('customizing ExtraCam')
            ExtraCam = BotCam('ExtraCam')
            ExtraCam.input_stream = CameraServer.getVideo('ExtraCam')
            ExtraCam.imgBuf = np.zeros(shape=(config.height, config.width, 3), dtype=np.uint8)
            ImgT=threading.Thread(target=queueImage,args=(ExtraCam,),daemon=True)
            ImgT.start()
            return ExtraCam
        case _:
            print ('Unknown camera usage:',config.usage)

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

    cam.usage  = config["properties"][0]["value"]
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
    print("Starting camera '{}' on {}".format(config.usage, config.path))
    #camera = UsbCamera(config.name, config.path)
    camera = UsbCamera(config.usage, config.path)
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
    # My code starts. Buckle-up!
    #########################################################################

    output_stream = CameraServer.putVideo("Well I'll be dipped!", 640, 480)
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
    while True:
        time.sleep(0.02)
        robotX, robotY, robotYaw, N = 0.0, 0.0,  0.0, 0
        for Cam in CamQs:
            try:
                frame = Cam.queue.pop()
                gray = cv2.cvtColor (frame, cv2.COLOR_BGR2GRAY)
                results = detector.detect(gray)
                if len(results) > 0:
                    Cam.robotPose = pose(results,Cam)
                    BotX, BotY = Cam.robotPose
                    #print (Cam.usage,"X:",BotX,"   Y:",BotY)
                    robotX += BotX
                    robotY += BotY
                    N += 1
                else:
                    continue
                if Cam.usage == 'ClimbCam':
                    output_stream.putFrame(frame)
                counter -= 1
            except:
                pass
            if counter < 1:
                stop = time.time()
                counter = 300
                print (counter/(stop - start),'fps')
                start = stop
        if N > 0:
            print (f'Average          {robotX/N:>6.2f} {robotY/N:6.2f}')
            print (' ')
