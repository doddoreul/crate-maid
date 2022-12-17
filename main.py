import numpy as np
import cv2
import json
import math
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

def move(direction):
    if (direction == "left"):
        print("Turn Left!")
        return True
    elif (direction == "right"):
        print("Turn Right")
        return True
    elif (direction == "forward"):
        return True
    elif (direction == "reverse"):
        return True
    else:
        return True

def lost():
    print("I'm lost!")

# Locate itself in the routes file
def locate(data):
    routes = flatten_json(routes_data["nearest_intersection"])
    # Get key ending with searched route
    r = {key:val for key, val in routes.items() if key.endswith((data, data+"."))}
    r = list(r.keys())[0]
    # r is relative position from home.
    print("Actual position: ", data)
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
        #intersection = ".".join(intersection)

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
def get_route(current, goal):
    # Localizate self with locate
    self_route = locate(current) # replace arg with self position
    # Localizate goal with locate
    goal_route = locate(goal) # replace arg with goal
    # Compare the two strings for a common intersection (ie: E2)

    print("Self: " + self_route)
    print("Goal: " + goal_route)

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
    print("Full intersection: ", intersection)
    intersection = clean_intersection(intersection)
    print("Short intersection: ", intersection)

    full_route_lst = merge_routes(self_route_lst, goal_route_lst, intersection)
    full_route = ".".join(full_route_lst)
    print(full_route)

    return full_route_lst

# Return directions assuming the robot is moving foward
def get_directions(route):
    print(route)

    # First key is always current location, let's ignore it
    dir_lst = routes_data["directions"]
    del route[0]
    for i in route:
        print(dir_lst[i])

    print(dir_lst)

# Go to destination via specified route
# Recursive function till the destination is reached
def follow_route(route):
    # TODO
    print("Moving robot to destination via specified route...")

def flatten_json(y):
    out = {}
 
    def flatten(x, name=''):
 
        # If the Nested key-value
        # pair is of dict type
        if type(x) is dict:
 
            for a in x:
                flatten(x[a], name + a + '.')
 
        # If the Nested key-value
        # pair is of list type
        elif type(x) is list:
 
            i = 0
 
            for a in x:
                flatten(a, name + str(i) + '.')
                i += 1
        else:
            out[name[:-1]] = x
 
    flatten(y)
    return out



def detectQR():
    relative_orientation
    # Read a frame from the camera
    _, frame = video_capture.read()

    # Detect the QR Code in the image
    data, bbox, rectifiedImage = qrDecoder.detectAndDecode(frame)

    # Wait 10 ms
    key = cv2.waitKey(10)
    if key == 27:  # 'Esc' key to quit
        return False
    
    # If QRCode's data is recognized, get it's orientation angle
    #if (data):
        #print(get_orientation(frame))

    #Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False

    # Display the contents of the QR Code
    if data:
        print("QR Code data:", data)
        locate(data) # Return position relative to HOME
        #input("Press enter to continue")
        return True
    else:
        #print("No QR Code was detected")
        return True

def detectLine():

    # Capture the frames
    ret, frame = video_capture.read()

    # Crop the image
    crop_img = frame[60:120, 0:160]

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Gaussian blur
    blur = cv2.GaussianBlur(gray,(5,5),0)

    # Color thresholding
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

        if cx >= 120:
            move("right")
        if cx < 120 and cx > 50:
            move("forward")
        if cx <= 50:
            move("left")

    else:
        lost()
        print("I don't see the line")

    #Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False

# Look for a QR Code to position itself
def tippy_tap():
    # TODO
    return False




def read_camera_parameters(filepath = 'camera_parameters/intrinsic.dat'):

    inf = open(filepath, 'r')

    cmtx = []
    dist = []

    #ignore first line
    line = inf.readline()
    for _ in range(3):
        line = inf.readline().split()
        line = [float(en) for en in line]
        cmtx.append(line)

    #ignore line that says "distortion"
    line = inf.readline()
    line = inf.readline().split()
    line = [float(en) for en in line]
    dist.append(line)

    #cmtx = camera matrix, dist = distortion parameters
    return np.array(cmtx), np.array(dist)

def get_qr_coords(cmtx, dist, points):

    #Selected coordinate points for each corner of QR code.
    qr_edges = np.array([[0,0,0],
                         [0,1,0],
                         [1,1,0],
                         [1,0,0]], dtype = 'float32').reshape((4,1,3))

    #determine the orientation of QR code coordinate system with respect to camera coorindate system.
    ret, rvec, tvec = cv2.solvePnP(qr_edges, points, cmtx, dist)

    #Define unit xyz axes. These are then projected to camera view using the rotation matrix and translation vector.
    unitv_points = np.array([[0,0,0], [1,0,0], [0,1,0], [0,0,1]], dtype = 'float32').reshape((4,1,3))
    if ret:
        points, jac = cv2.projectPoints(unitv_points, rvec, tvec, cmtx, dist)
        return points, rvec, tvec

    #return empty arrays if rotation and translation values not found
    else: return [], [], []


def detectQR2(cmtx, dist):
    # Orientation algo shamelessly stolen here: https://github.com/TemugeB/QR_code_orientation_OpenCV

    ret, img = video_capture.read()
    if ret == False: return False

    ret_qr, points = qrDecoder.detect(img)

    if ret_qr:
        data, bbox, rectifiedImage = qrDecoder.detectAndDecode(img)
        # Store data in QR code array
        last_qr["data"] = data

        axis_points, rvec, tvec = get_qr_coords(cmtx, dist, points)
        
        #BGR color format
        #colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0,0,0)]

        #check axes points are projected to camera view.
        if len(axis_points) > 0:
            axis_points = axis_points.reshape((4,2))

            origin = (int(axis_points[0][0]),int(axis_points[0][1]) )
            x = axis_points[1][0]
            y = axis_points[1][1]
            angle = math.atan2(axis_points[0][0]-x,axis_points[0][1]-y)
            angle_degrees = -(math.floor((-180-math.degrees(angle))*100)/100)
            last_qr["orientation"] = angle_degrees
            print("angle: ", angle_degrees)

            '''
            for p, c in zip(axis_points[1:], colors[:3]):
                p = (int(p[0]), int(p[1]))
                #print(p)

                #Sometimes qr detector will make a mistake and projected point will overflow integer value. We skip these cases. 
                if origin[0] > 5*img.shape[1] or origin[1] > 5*img.shape[1]:break
                if p[0] > 5*img.shape[1] or p[1] > 5*img.shape[1]:break

                cv2.line(img, origin, p, c, 5)
            '''
        
        locate(data) # Return position relative to HOME


    cv2.imshow('frame', img)

    return True




if __name__ == '__main__':

    #read camera intrinsic parameters.
    cmtx, dist = read_camera_parameters()

    #goal = input('Where do we go?: ')
    '''route = get_route("HS4", "HS3")
    directions = get_directions(route)'''

    # False while debugging
    while(True):
        '''if(detectLine() == False):
            break'''

        if(detectQR2(cmtx, dist) == False):
            break

        k = cv2.waitKey(20)
        if k == 27: break #27 is ESC key.

    video_capture.release()
    cv2.destroyAllWindows()

