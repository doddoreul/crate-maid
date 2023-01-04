import utils.utils as utils
import numpy as np
import cv2
import os
import json
import math
import time
import copy
import argparse
from difflib import SequenceMatcher

try:
    import RPi.GPIO as GPIO
    isPi = True
except ImportError:
    import SimulRPi.GPIO as GPIO
    isPi = False

argParser = argparse.ArgumentParser()
argParser.add_argument("-m", "--move", help="Move the robot", action='store_const', const=True)
argParser.add_argument("-d", "--debug", help="Display debug windows", action='store_const', const=True)
args = argParser.parse_args()

# Constants
MOVE = True if args.move == None else False # Should the robot physically move? (debugging)
DEBUG = False if args.debug == None else True
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
if (isPi):
    GPIO_L = GPIO.PWM(P_LEFT, 50)
    GPIO_R = GPIO.PWM(P_RIGHT, 50)

qrDecoder = cv2.QRCodeDetector()

# Open json file, store content then close it
routes = open('map.json')
routes_data = json.load(routes)
routes.close()
rel_orientation = 0
lst_pos = {"data": "", "orientation": ""}
prev_cx = 0

def lost():
    print("I'm lost!")

    if (isPi):
        GPIO_R.ChangeDutyCycle(0)
        GPIO_L.ChangeDutyCycle(0)
    #adjust_line('stop', 1)
    # TODO: create tippy_tap() function to make the robot "search" for a line

# Locate itself in the routes file
def locate(data):
    data = str(data)

    routes = utils.flatten_json(routes_data["nearest_intersection"])

    # Get key ending with searched route
    #r = {key:val for key, val in routes.items() if key.endswith(data+".")}
    
    # Keys with specific suffix in Dictionary
    r = ""
    for i in routes.keys():
        
        if(i.endswith(".")):
            i = i[:-1]
    
        if (i.endswith("."+data)):
            r = i

    # r is relative position from home.
    return r

# Remove trailing dots
# Return intersection only, not whole route to intersection
def clean_intersection(intersection):
    # Remove the . if last char, for the sake of cleanness
    if (intersection[-1] == "."):
        intersection = intersection[:-1] 
    #print("Intersection", intersection)
    intersection = intersection.split(".")
    #print("Intersection_excl", intersection)
    
    # If intersection equals to 1, means we only have HOME and should return it as it
    if (len(intersection) == 1): 
        intersection = intersection[0]
    else:
        intersection = intersection[-1]

    return intersection

# Merge self route, goal route and intersection into one route
# self = list
# goal = list
# intersection = string
# return list
def merge_routes(self, goal, intersection):
 
    for i in self[:]:
        if i in goal:
            self.remove(i)
            goal.remove(i)
 
    intersection = intersection.split(".") # intersection is string, convert it to list, to concatenate later
    route = self + intersection + goal
    route = list(filter(None, route)) # Remove empty entries

    return route

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

    #print("Self: " + self_route)
    #print("Goal: " + goal_route)

    if (self_route == goal_route):
        print("Destination reached")
        return []
    else: 
        self_route_lst = self_route.split(".")
        goal_route_lst = goal_route.split(".")
        self_route_lst.reverse()

        #print("self_route_lst: ", self_route_lst)
        #print("goal_route_lst: ", goal_route_lst)
        new_full_route_lst = [*self_route_lst, *goal_route_lst]
        
        print('new_full_route_lst: ', new_full_route_lst)
        """ # Strip last entry of each routes to prevent next instruction to match
        s = self_route.split(".")
        s = s[:len(s)-1]
        g = goal_route.split(".")
        g = g[:len(g)-1]
        s = ".".join(s)
        g = ".".join(g)

        # Find a match between the two routes
        match = SequenceMatcher(None, s, g).find_longest_match(0, len(s), 0, len(g))
        # Get corresponding string 
        intersection = self_route[match.a:match.a + match.size]
        #print("Full intersection: ", intersection)

        intersection = clean_intersection(intersection)
        #print("Intersection: ", intersection)
        full_route_lst = merge_routes(self_route_lst, goal_route_lst, intersection)
        #print('full_route_lst: ', full_route_lst)
         """

        
        return new_full_route_lst

# Move to next goal (next intersection, of final goal, whatever)
# TODO: define here how to physically move the robot, and merge it with detect_line to adjust with the line
def reach(goal):
    print("Moving to: ", goal)
    
    if (goal == lst_pos["data"]):
        return False
    return True

# Physically adjust the robot left or right, depending on the line
def adjust_line(dir):

    if (isinstance(dir, int)):
        """         # Crate's width = 36cm
        # Timing @ 5%: 10.2
        r = 36
        speed = ((2*(math.pi)*r)/100)/10.2 # Speed, in m/s
        # 10.2s = 360°
        f = (10.2*5)/RSPEED
        deg = f/360
        td = deg*dir
        # Slow speed factor = 10.2/5 : 2.04
        if (td<0.1): td = 0.1 """

        if(dir < 0):
            if (MOVE and isPi):
                GPIO_R.ChangeDutyCycle(RSPEED)
                GPIO_L.ChangeDutyCycle(0)
            #print("CCW")
        elif (dir > 0):
            if (MOVE and isPi):
                GPIO_R.ChangeDutyCycle(0)
                GPIO_L.ChangeDutyCycle(RSPEED)
            #print("CW")
        else: 
            if (MOVE and isPi):
                GPIO_R.ChangeDutyCycle(RSPEED)
                GPIO_L.ChangeDutyCycle(RSPEED)
            #print("fwd")

        time.sleep(0.08)

        if (isPi):
            GPIO_L.ChangeDutyCycle(0)
            GPIO_R.ChangeDutyCycle(0)

# Orient robot to next goal
# goal = next step, not final goal
def get_orientation(goal):
    current_pos = routes_data["four_ways"][str(lst_pos['data'])]

    goal_orient = (current_pos.index(goal))*90
    current_orient = lst_pos["orientation"]
    #print("Goal orientation: ", goal_orient)
    #print("Current orientation: ", current_orient)

    relative_orient = goal_orient - current_orient
    print("Degrees to turn: ", relative_orient)

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
        #if ((prev_cx <= cx - 5 and prev_cx >= cx + 5) or prev_cx == 0):
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

# Detect QR code via video feed provided by the camera
def detect_QR(ret, img):
    #read camera intrinsic parameters.
    cmtx, dist = utils.read_camera_parameters()

    ret_qr, points = qrDecoder.detect(img)

    if ret_qr:
        data, bbox = qrDecoder.decode(img, points)
        if data:
            
            # Store data in QR code array
            lst_pos["data"] = data if (data != "") else lst_pos["data"]
            print("Actual position: ", lst_pos["data"])

            axis_points, rvec, tvec = utils.get_qr_coords(cmtx, dist, points)
            
            #check axes points are projected to camera view.
            if len(axis_points) > 0:
                axis_points = axis_points.reshape((4,2))

                x1 = axis_points[1][0]
                y1 = axis_points[1][1]
                x0 = axis_points[0][0]
                y0 = axis_points[0][1]

                angle = math.atan2(x0-x1,y0-y1)
                angle_degrees = -(math.floor((-180-math.degrees(angle))*100)/100)

                lst_pos["orientation"] = angle_degrees
                print("Angle: ", lst_pos["orientation"])
                
                return True
        
        if lst_pos["data"] == "":
            print('Waiting to catch QR Code data')

    return False

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
            lst_pos["data"] = id[0]

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
            lst_pos["orientation"] = angle
            #print("Angle: ", angle)

    if (DEBUG):
        # show the output frame
        cv2.imshow("ArUco detection", img)
        #cv2.imshow("ArUco cropped", cropped)

    if len(corners) > 0:
        return True
    else : 
        return False
    


def tick(qty):
    # 120 = 360°
    
    i=0
    while(i<qty):
        GPIO_R.ChangeDutyCycle(RSPEED)
        GPIO_L.ChangeDutyCycle(0)
        time.sleep(0.1)
        i = i+1


# Setup ArUco parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
aruco_params = cv2.aruco.DetectorParameters()


if __name__ == '__main__':

    # Select lowest quality available on PSEye
    video_capture = cv2.VideoCapture(-1)
    video_capture.set(cv2.CAP_PROP_FPS, 15)
    video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAP_W)
    video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAP_H)
    
    if not video_capture.isOpened():
        print("Can't find camera")
        exit()

    goal = input('Where do we go?:') or "29"
    
    print("Goal: ", goal)
    
    # Start the PWM signals with a duty cycle of 0%
    if (isPi):
        GPIO_L.start(0)
        GPIO_R.start(0)

    while(True):
        ret, img = video_capture.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if ret == False: break # If nothing is found, break the loop

        if ((detect_line(ret, gray) == True) and (detect_ArUco(ret, img) == True)):

            if (lst_pos["data"] == ""):
                print("Looking for a position...")
            else:
                print("Current position: ", lst_pos["data"])

                route = get_route(lst_pos["data"], goal)
                
                if (len(route)):  
                    next_goal = route[1]
                    if (reach(next_goal)):
                        get_orientation(next_goal)
                


    video_capture.release()
    cv2.destroyAllWindows()

    # Clean up the GPIO settings
    GPIO.cleanup()
    