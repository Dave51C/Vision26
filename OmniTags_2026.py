# $Source: /home/scrobotics/src/2026/sandbox/RCS/OmniTags_2026.py,v $
# $Revision: 1.3 $
# $Date: 2026/01/31 18:01:21 $
# $Author: scrobotics $
"""
OmniTags returns a list of dictionaries. 
"""
def OmniTags (img, Cam, detector):
    from pprint import pprint
    from math import pi,atan2,asin,sqrt,tan,degrees
    from PiggyVision26 import rotate, TAG_CORNERS
    import apriltag
    import time
    import cv2
    import numpy as np

    CamPos   = np.array([0.0, 0.0, 0.0])
    CamPos   = CamPos.reshape(3,1)

    gray = cv2.cvtColor (img, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)
    SumX, SumY, SumYaw, N = 0.0, 0.0, 0.0, 0
    for r in results:
        ret, rvecs, tvecs = cv2.solvePnP(TAG_CORNERS[r.tag_id],r.corners,
                    Cam.mtx,Cam.dist, cv2.SOLVEPNP_IPPE_SQUARE)
        ZYX,jac  = cv2.Rodrigues(rvecs)
        CamPos   = -np.matrix(ZYX).T * np.matrix(tvecs)
        CamX, CamY, CamZ = CamPos[0].item(), CamPos[1].item(), CamPos[2].item()
        BotX = CamX - Cam.localX   # Where this cam thinks robot X is based on single tag
        BotY = CamY - Cam.localY   # Where this cam thinks robot Y is based on single tag
        SumX += BotX               # Sum X values from all tags this cam sees.
        SumY += BotY               # Sum Y values from all tags this cam sees.
        N += 1                     # Count up all the tags this cam sees.
        #CamYaw,CamPitch,CamRoll = Cam.frcYPR(rvecs)
        print (f'{Cam.usage:10s} BotXY {BotX:>6.2f} {BotY:6.2f}  CamXY {CamX:>6.2f} {CamY:>6.2f}')
    #print ("AvgX:",SumX/N," AvgY",SumY/N)
    return (SumX/N,SumY/N)         # Average X,Y based on all tags this cam sees.


if __name__ == "__main__":
    import argparse, time, apriltag
    import cv2, numpy as np
    from pprint import pprint
    from PiggyVision26 import BotCam, Webcam
    options  = apriltag.DetectorOptions(
           families="tag36h11"
           ,nthreads      = 4
           ,quad_decimate = 2
           ,quad_blur     = 0.0
           ,refine_edges  = True
           ,refine_decode = True
           ,refine_pose   = False
           ,quad_contours = True
           )
    parser = argparse.ArgumentParser()
    #parser.add_argument("--tags", nargs='+', type=int, help="List of Tag IDs you're looking for")
    parser.add_argument("--device", type=int, help="video device nbr.")
    args = parser.parse_args()
    BogusCam = BotCam("BogusCam")

    cap    = cv2.VideoCapture(args.device)         # Initialize webcam
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

    detector = apriltag.Detector(options)

    img    = np.zeros(shape=(height, width, 3), dtype=np.uint8)
    while True:                       # Repeat the following until "q" key hit
        ret, frame = cap.read()                    # Capture frame
        cv2.imshow('Webcam', frame)              # Display frame
        if cv2.waitKey(1) & 0xFF == ord('q'):    # Check for "q" 
            break
    cap.release()                     # Release the webcam
    cv2.destroyAllWindows()           # Close the window

    THEN=time.time()
    N = 1
    while N > 0:
        N-=1
        All = OmniTags (frame, BogusCam, detector)
    print(time.time()-THEN)
    #pprint(All)
