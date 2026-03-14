import cv2
import json
import sys
import numpy as np

def getcxcy(cameraname):
	camdata = open (cameraname, 'r')
	j = json.load(camdata)
#	print (j["mtx"], j["mtx"][0][2], j["mtx"][1][2])
	camdata.close()
	return round(j["mtx"][0][2]), round(j["mtx"][1][2])

def open_stream(webcam):
    stream = cv2.VideoCapture(webcam)  # Initialize webcam
    stream.set (cv2.CAP_PROP_FRAME_WIDTH, 800) 
    stream.set (cv2.CAP_PROP_FRAME_HEIGHT,600)
    return(stream)

def fetch_frame(stream):
    ret, frame = stream.read()          # Capture frame
    return(frame)

def show_stream(jfile, webcam=0):
    """
    Streams the attached webcam to the device's display. This assumes you 
    can access the device's display, i.e. you're logged onto the computer
    where you run this. A remote login (VNC) should work but "ssh" won't.
    """
    (cx,cy) = getcxcy(jfile)
    cap = open_stream(webcam)
    while True:                       # Repeat the following until "q" key hit
        frame = fetch_frame(cap)
        cv2.line(frame, (cx, 0), (cx, 600), (0,255,0), 3)
        cv2.line(frame, (0, cy), (800, cy), (0,255,0), 3)
        cv2.imshow('Webcam', frame)              # Display frame
        if cv2.waitKey(1) & 0xFF == ord('q'):    # Check for "q" 
            break
    cap.release()                     # Release the webcam
    cv2.destroyAllWindows()           # Close the window

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", type=int, help="video device number", default=0)
    parser.add_argument("--jfile", type=str, help="json file name")
    args = parser.parse_args()
    show_stream(args.jfile, args.device)
