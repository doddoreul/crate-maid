# Before anything, we parse all given args and setup everything
import argparse
# Parse args from cli
argParser = argparse.ArgumentParser()
argParser.add_argument("-m", "--move", help="Move the robot", action='store_const', const=True)
argParser.add_argument("-d", "--debug", help="Display debug windows", action='store_const', const=True)
argParser.add_argument("-v", "--verbose", help="Verbose mode", action='store_const', const=True)
args = argParser.parse_args()

# Constants
MOVE = True if args.move == None else False # Should the robot physically move? (debugging)
DEBUG = False if args.debug == None else True
VERBOSE = False if args.verbose == None else True
import utils.utils as utils
from utils.utils import printv
import numpy as np
import cv2
import os
import json
import math
import time
import copy
from difflib import SequenceMatcher

try:
    import RPi.GPIO as GPIO
    hasGPIO = True
except ImportError:
    import SimulRPi.GPIO as GPIO
    hasGPIO = False



RSPEED = 5 # PWM's percentage
CAP_W = 320 # VideoCapture W
CAP_H = 240 # VideoCapture H

# Set the pin numbers for the two pins that you want to use for PWM
P_LEFT = 18
P_RIGHT = 19

# Set the GPIO mode to BCM (Broadcom pin numbering)
GPIO.setmode(GPIO.BCM)

# Set the two pins as outputs
GPIO.setup(P_LEFT, GPIO.OUT)
GPIO.setup(P_RIGHT, GPIO.OUT)

# Create two PWM objects, one for each pin
if (hasGPIO):
    GPIO_L = GPIO.PWM(P_LEFT, 50)
    GPIO_R = GPIO.PWM(P_RIGHT, 50)

# Setup ArUco parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
aruco_params = cv2.aruco.DetectorParameters()

# Open json file, store content then close it
routes_f = open('map.json')
map_data = json.load(routes_f)
routes_f.close()
routes = utils.flatten_json(map_data["nearest_intersection"])

last_pos = {"data": "", "orientation": ""}

# Look for the line, or stop (don't know yet)
def lost():
    printv("I'm lost!")

    if (hasGPIO):
        GPIO_R.ChangeDutyCycle(0)
        GPIO_L.ChangeDutyCycle(0)

# Locate itself in the routes file
def locate(data):
    data = str(data) # Ensure it's a str, not an int

    # Keys with specific suffix in Dictionary
    r = ""
    for i in routes.keys():

        if(i.endswith(".")):
            i = i[:-1]
    
        if (i.endswith("."+data)):
            r = i

    # r is relative position from home.
    return r

# Get the route between current position and goal (destination)
# Should return a route corresponding to the map.json object
# current = current position
# goal = destination
# return: string of route to take
def get_route(current, goal, abs = True):
    # Localizate self with locate 
    self_route = locate(current) # replace arg with self position
    # Localizate goal with locate
    goal_route = locate(goal) # replace arg with goal
    # Compare the two strings for a common intersection (ie: E2)

    #printv("Self: " + self_route)
    #printv("Goal: " + goal_route)

    if (self_route == goal_route):
        printv("Destination reached")
        return []
    else: 
        self_route_lst = self_route.split(".")
        goal_route_lst = goal_route.split(".")
        self_route_lst.reverse()

        #printv("self_route_lst: ", self_route_lst)
        #printv("goal_route_lst: ", goal_route_lst)
        new_full_route_lst = [*self_route_lst, *goal_route_lst]
        
        printv('new_full_route_lst: ', new_full_route_lst)
        
        return new_full_route_lst

# Move to next goal (next intersection, of final goal, whatever)
# TODO: define here how to physically move the robot, and merge it with detect_line to adjust with the line
def reach(goal):
    printv("Moving to: ", goal)
    
    if (goal == last_pos["data"]):
        return False
    return True

# Physically adjust the robot left or right, depending on the line
def adjust_line(dir):

    if (isinstance(dir, int)):

        if(dir < 0):
            if (MOVE and hasGPIO):
                GPIO_R.ChangeDutyCycle(RSPEED)
                GPIO_L.ChangeDutyCycle(0)
            #printv("CCW")
        elif (dir > 0):
            if (MOVE and hasGPIO):
                GPIO_R.ChangeDutyCycle(0)
                GPIO_L.ChangeDutyCycle(RSPEED)
            #printv("CW")
        else: 
            if (MOVE and hasGPIO):
                GPIO_R.ChangeDutyCycle(RSPEED)
                GPIO_L.ChangeDutyCycle(RSPEED)
            #printv("fwd")

        time.sleep(0.08)

        if (hasGPIO):
            GPIO_L.ChangeDutyCycle(0)
            GPIO_R.ChangeDutyCycle(0)

# Orient robot to next goal
# goal = next step, not final goal
def get_orientation(goal):
    current_pos = map_data["four_ways"][str(last_pos['data'])]

    goal_orient = (current_pos.index(goal))*90
    current_orient = last_pos["orientation"]
    #printv("Goal orientation: ", goal_orient)
    #printv("Current orientation: ", current_orient)

    relative_orient = goal_orient - current_orient
    printv("Degrees to turn: ", relative_orient)

    # TODO: calibrate robot to know how long it has to move before reaching the physical intersection
    
# Detect the line via video feed provided by the camera
def detect_line(ret, img):
    w = int(CAP_W/3)
    h = int(CAP_H/3)

    img = cv2.resize(img,(w,h))

    # Gaussian blur
    blur = cv2.GaussianBlur(img,(5,5),0)
    ret,img = cv2.threshold(blur,40,255,cv2.THRESH_BINARY_INV)

    # Find the contours of the frame
    contours,hierarchy = cv2.findContours(img.copy(), 1, cv2.CHAIN_APPROX_NONE)

    # Find the biggest contour (if detected)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)

        # Find centroids
        M = cv2.moments(c)

        if (M['m00'] == 0):
            cx, cy = 0, 0
        else:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        
        # Check if CX is between old CX +/- 5 to be sure it doesn't grab anything too far
        if(cx and cy):
            
            if (DEBUG):
                # Draw X line
                cv2.line(img,(cx,0),(cx,h),(255,255,255),1)
                # Draw Y line
                cv2.line(img,(0,cy),(w,cy),(255,255,255),1)
                cv2.drawContours(img, contours, -1, (255,255,255), 1)
        
            # Move the robot foward, and check if it's still aligned
            # TODO: create moving function
            mx = (w/2)+10
            mn = (w/2)-10
            if cx >= mx:
                adjust_line(5)
            if cx < mx and cx > mn:
                adjust_line(0)
            if cx <= mn:
                adjust_line(-5)

    else:
        lost()

    if (DEBUG):
        w = w*3
        h = h*3
        img_resized = cv2.resize(img,(w,h))
        cv2.imshow('Line detection',img_resized)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return False

    return True

# Detect ArUco via video feed provided by the camera
def detect_ArUco(ret, img):
    c_x1 = (img.shape[0]//3)
    c_x2 = (img.shape[0]//3)*2
    c_y1 = (img.shape[1]//3)
    c_y2 = (img.shape[1]//3)*2

    cropped = img[c_x1: c_x2, c_y1: c_y2]
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 600 pixels
    
    # detect ArUco markers in the input frame
    (corners, ids, rejected) = cv2.aruco.detectMarkers(cropped, aruco_dict, parameters=aruco_params)
    
    if (DEBUG):
        # convert each of the (x, y)-coordinate pairs to integers

        # Draw cropped corners
        cv2.line(img, (c_y1, c_x1), (c_y1, c_x2), (255,0,0), 1) 
        cv2.line(img, (c_y1, c_x2), (c_y2, c_x2), (255,0,0), 1) 
        cv2.line(img, (c_y1, c_x1), (c_y2, c_x1), (255,0,0), 1) 
        cv2.line(img, (c_y2, c_x1), (c_y2, c_x2), (255,0,0), 1) 

    # verify *at least* one ArUco marker was detected
    if len(corners) > 0:
        # flatten the ArUco IDs list

        ids = ids.flatten()
        id = ids[:1] # Filter one code
        
        if(id[0] != ""):
            last_pos["data"] = id[0]

        # loop over the detected ArUCo corners
        for (marker_corner, marker_id) in zip(corners, id):
            # extract the marker corners (which are always returned
            # in top-left, top-right, bottom-right, and bottom-left
            # order)
            corners = marker_corner.reshape((4, 2))
            (top_left, top_right, bottom_right, bottom_left) = corners

            # convert each of the (x, y)-coordinate pairs to integers
            top_right = (int(top_right[0]), int(top_right[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
            top_left = (int(top_left[0]), int(top_left[1]))

            # compute and draw the center (x, y)-coordinates of the
            # ArUco marker
            cX1 = int((bottom_left[0] + bottom_right[0]) / 2.0)
            cX2 = int((top_left[0] + top_right[0]) / 2.0)

            cY1 = int((bottom_left[1] + bottom_right[1]) / 2.0)
            cY2 = int((top_left[1] + top_right[1]) / 2.0)

            if (DEBUG):
                # convert each of the (x, y)-coordinate pairs to integers

                # Draw cropped corners
                cv2.line(img, (c_y1, c_x1), (c_y1, c_x2), (255,0,0), 1) 
                cv2.line(img, (c_y1, c_x2), (c_y2, c_x2), (255,0,0), 1) 
                cv2.line(img, (c_y1, c_x1), (c_y2, c_x1), (255,0,0), 1) 
                cv2.line(img, (c_y2, c_x1), (c_y2, c_x2), (255,0,0), 1) 

                # Draw corners
                cv2.circle(cropped, top_right, 4, (255,0,0), 2) 
                cv2.circle(cropped, bottom_right, 4, (255,0,0), 2) 
                cv2.circle(cropped, bottom_left, 4, (255,0,0), 2) 
                cv2.circle(cropped, top_left, 4, (255,0,0), 2) 

                # draw the bounding box of the ArUCo detection
                cv2.line(cropped, top_left, top_right, (0, 255, 0), 2)
                cv2.line(cropped, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(cropped, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(cropped, bottom_left, top_left, (0, 255, 0), 2)


                # draw the ArUco marker ID on the frame
                cv2.putText(cropped, str(marker_id),
                    (top_left[0], top_left[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

            # Get Orientation
            rad = math.atan2(cX2-cX1, cY2-cY1)

            angle = abs(((rad*180)/math.pi)-180)
            last_pos["orientation"] = angle
            #printv("Angle: ", angle)

    if (DEBUG):
        # show the output frame
        cv2.imshow("ArUco detection", img)
        #cv2.imshow("ArUco cropped", cropped)

    if len(corners) > 0:
        return True
    else : 
        return False

if __name__ == '__main__':

    # Select lowest quality available on PSEye
    video_capture = cv2.VideoCapture(-1)
    video_capture.set(cv2.CAP_PROP_FPS, 15)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_W)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_H)
    
    if not video_capture.isOpened():
        printv("Can't find camera")
        exit()

    goal = input('Where do we go?:') or "29"
    
    printv("Goal: ", goal)
    
    # Start the PWM signals with a duty cycle of 0%
    if (hasGPIO):
        GPIO_L.start(0)
        GPIO_R.start(0)

    while(True):
        ret, img = video_capture.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if ret == False: break # If nothing is found, break the loop

        if ((detect_line(ret, gray) == True) and (detect_ArUco(ret, img) == True)):

            if (last_pos["data"] == ""):
                printv("Looking for a position...")
            else:
                printv("Current position: ", last_pos["data"])

                route = get_route(last_pos["data"], goal)
                
                if (len(route)):  
                    next_goal = route[1]
                    if (reach(next_goal)):
                        get_orientation(next_goal)
                


    video_capture.release()
    cv2.destroyAllWindows()

    # Clean up the GPIO settings
    GPIO.cleanup()
    