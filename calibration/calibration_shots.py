import cv2
import argparse
import numpy as np
import sys
from time import sleep

def open_stream(webcam,width,height):
    stream = cv2.VideoCapture(webcam)  # Initialize webcam
    stream.set (cv2.CAP_PROP_FRAME_WIDTH, width) 
    stream.set (cv2.CAP_PROP_FRAME_HEIGHT,height)
    print (stream.get(cv2.CAP_PROP_FRAME_WIDTH),"x",stream.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return(stream)

def fetch_frame(stream):
    ret, frame = stream.read()          # Capture frame
    return(frame)

ONE   = np.array([[450,10],[450,200]],np.int32)
TWO   = np.array([[350,10],[450,10],[450,100],[350,100],[350,200],[450,200]],np.int32)
THREE = np.array([[350,10],[450,10],[450,100],[350,100],
                 [450,100],[450,200],[350,200]],np.int32)
FOUR  = np.array([[350,10],[350,100],[450,100],[450,10],[450,200]],np.int32)
FIVE  = np.array([[450,10],[350,10],[350,100],[450,100],[450,200],[350,200]],np.int32)

GREEN  = (0,255,0)
RED    = (0,0,255)
CYAN   = (255,255,0)
YELLOW = (0,255,255)

parser = argparse.ArgumentParser()
parser.add_argument("--width",    type=int, default=640, help="defautl 640")
parser.add_argument("--height",   type=int, default=480, help="default 480")
parser.add_argument("--shots",    type=int, default=25,  help="default 25")
parser.add_argument("--startnum", type=int, default=0,   help="default 0")
parser.add_argument("--device",   type=int, default=0,   help="default 0")
args     = parser.parse_args()
width    = args.width
height   = args.height
shots    = args.shots
startnum = args.startnum
device   = args.device

cap      = open_stream(device,width,height)
SEQ      = startnum
DELAY    = 20
LAST     = startnum + shots
while SEQ < LAST:
    T = DELAY
    while T > 0:
        frame = fetch_frame(cap)
        frame = cv2.flip(frame, 1)
        cv2.polylines(frame, [FIVE], False, RED, 20)
        cv2.polylines(frame, [FIVE], False, GREEN, 3)
        if width > 800:
            scale_x = 0.5
            scale_y = 0.5
            smaller = cv2.resize(frame, None, fx=scale_x, fy=scale_y)
            cv2.imshow('counter', smaller)              # Display frame
            T -= 3
        else:
            cv2.imshow   ('counter',frame)
        cv2.waitKey(1)
        T -= 1
    T = DELAY
    while T > 0:
        frame = fetch_frame(cap)
        frame = cv2.flip(frame, 1)
        cv2.polylines(frame, [FOUR], False, RED, 20)
        cv2.polylines(frame, [FOUR], False, GREEN, 3)
        if width > 800:
            scale_x = 0.5
            scale_y = 0.5
            smaller = cv2.resize(frame, None, fx=scale_x, fy=scale_y)
            cv2.imshow('counter', smaller)              # Display frame
            T -= 3
        else:
            cv2.imshow   ('counter',frame)
        cv2.waitKey(1)
        T -= 1
    T = DELAY
    while T > 0:
        frame = fetch_frame(cap)
        frame = cv2.flip(frame, 1)
        cv2.polylines(frame, [THREE], False, RED, 20)
        cv2.polylines(frame, [THREE], False, GREEN, 3)
        if width > 800:
            scale_x = 0.5
            scale_y = 0.5
            smaller = cv2.resize(frame, None, fx=scale_x, fy=scale_y)
            cv2.imshow('counter', smaller)              # Display frame
            T -= 3
        else:
            cv2.imshow   ('counter',frame)
        cv2.waitKey(1)
        T -= 1
    T = DELAY
    while T > 0:
        frame = fetch_frame(cap)
        frame = cv2.flip(frame, 1)
        cv2.polylines(frame, [TWO], False, RED, 20)
        cv2.polylines(frame, [TWO], False, GREEN, 3)
        if width > 800:
            scale_x = 0.5
            scale_y = 0.5
            smaller = cv2.resize(frame, None, fx=scale_x, fy=scale_y)
            cv2.imshow('counter', smaller)              # Display frame
            T -= 3
        else:
            cv2.imshow   ('counter',frame)
        cv2.waitKey(1)
        T -= 1
    T = DELAY
    while T > 0:
        frame = fetch_frame(cap)
        frame = cv2.flip(frame, 1)
        cv2.polylines(frame, [ONE], False, RED, 20)
        cv2.polylines(frame, [ONE], False, GREEN, 3)
        if width > 800:
            scale_x = 0.5
            scale_y = 0.5
            smaller = cv2.resize(frame, None, fx=scale_x, fy=scale_y)
            cv2.imshow('counter', smaller)              # Display frame
            T -= 3
        else:
            cv2.imshow   ('counter',frame)
        cv2.waitKey(1)
        T -= 1

    frame = fetch_frame(cap)
    fname = './stills/img' + '{0:04d}'.format(SEQ) + '.jpg'
    cv2.imwrite (fname,frame)
    print (fname)
    SEQ += 1
    print ('CLICK!')

