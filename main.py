import utils.utils as utils
import numpy as np
import cv2
import json
import math
import time
from difflib import SequenceMatcher

video_capture = cv2.VideoCapture(-1)
video_capture.set(3, 160)
video_capture.set(4, 120)

qrDecoder = cv2.QRCodeDetector()

# Open json file, store content then close it
routes = open('map.json')
routes_data = json.load(routes)
routes.close()
relative_orientation = 0
last_qr = {"data": "", "orientation": ""}

def lost():
    print("I'm lost!")
    # TODO: create tippy_tap() function to make the robot "search" for a line

# Locate itself in the routes file
def locate(data):
    routes = utils.flatten_json(routes_data["nearest_intersection"])
    # Get key ending with searched route
    r = {key:val for key, val in routes.items() if key.endswith((data, data+"."))}
    try:
        r = list(r.keys())[0]
    except:
        print("*** Missing 'self key' in nearest_intersection?") 
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

    self_route_lst = self_route.split(".")
    goal_route_lst = goal_route.split(".")
    self_route_lst.reverse()
    
    # Strip last entry of each routes to prevent next instruction to match
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

    full_route_lst = merge_routes(self_route_lst, goal_route_lst, intersection)
    
    return full_route_lst

# Move to next goal (next intersection, of final goal, whatever)
# TODO: define here how to physically move the robot, and merge it with detect_line to adjust with the line
def reach(goal):
    print("Moving to: ", goal)
    
    if (goal == last_qr["data"]):
        return False
    return True

# Physically adjust the robot left or right, depending on the line
def adjust_line(direction):
    if (direction == "left"):
        #print("Turn Left!")
        return True
    elif (direction == "right"):
        #print("Turn Right")
        return True
    else:
        return False

# Orient robot to next goal
# goal = next step, not final goal
def get_orientation(goal):

    current_pos = routes_data["four_ways"][last_qr['data']]
    #print("Current position: ", current_pos)

    goal_orient = (current_pos.index(goal))*90
    current_orient = last_qr["orientation"]
    #print("Goal orientation: ", goal_orient)
    #print("Current orientation: ", current_orient)

    relative_orient = goal_orient - current_orient 

    print("Degrees to turn: ", relative_orient)
    # TODO: calibrate robot to know how long it has to move before reaching the physical intersection
    
# Detect the line via video feed provided by the camera
def detect_line():

    # Capture the frames
    ret, frame = video_capture.read()

    # Crop the image
    crop_img = frame[60:120, 0:160]

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Gaussian blur
    blur = cv2.GaussianBlur(gray,(5,5),0)
    ret,thresh = cv2.threshold(blur,60,255,cv2.THRESH_BINARY_INV)

    # Find the contours of the frame
    contours,hierarchy = cv2.findContours(thresh.copy(), 1, cv2.CHAIN_APPROX_NONE)

    # Find the biggest contour (if detected)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
    
        if (M['m00'] == 0):
            print("Meh, m00 = 0")
        else:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

        # Draw lines to preview
        cv2.line(frame,(cx,0),(cx,720),(255,0,0),1)
        cv2.line(frame,(0,cy),(1280,cy),(255,0,0),1)
        cv2.drawContours(frame, contours, -1, (0,255,0), 1)

        # Move the robot foward, and check if it's still aligned
        # TODO: create moving function
        if cx >= 120:
            adjust_line("right")
        if cx < 120 and cx > 50:
            adjust_line("forward")
        if cx <= 50:
            adjust_line("left")

    else:
        lost()
        
    #Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False

    return True

# Detect QR code via video feed provided by the camera
def detect_QR():
    #read camera intrinsic parameters.
    cmtx, dist = utils.read_camera_parameters()

    ret, img = video_capture.read()
    if ret == False: return False

    ret_qr, points = qrDecoder.detect(img)
    cv2.imshow('frame', img)

    if ret_qr:
        data, bbox, rectifiedImage = qrDecoder.detectAndDecode(img)
        if data:

            # Store data in QR code array
            last_qr["data"] = data if (data != "") else last_qr["data"]
            print("Actual position: ", last_qr["data"])

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

                last_qr["orientation"] = angle_degrees
                print("Angle: ", last_qr["orientation"])
                
                return True
        
        if last_qr["data"] == "":
            print('Waiting to catch QR Code data')

    return False


if __name__ == '__main__':

    goal = input('Where do we go?:') or "HS2"
    print("Goal: ", goal)

    while(True):
        if ((detect_line() == True) and (detect_QR() == True)):

            if (last_qr["data"] != goal):
                route = get_route(last_qr["data"], goal)
                next_goal = route[1]
                if (reach(next_goal)):
                    get_orientation(next_goal)
            
            else :
                print("Destination reached!")


        k = cv2.waitKey(20)
        if k == 27: break #27 is ESC key.

    video_capture.release()
    cv2.destroyAllWindows()

