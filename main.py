import numpy as np
import cv2
import json

video_capture = cv2.VideoCapture(-1)
video_capture.set(3, 160)
video_capture.set(4, 120)

qrDecoder = cv2.QRCodeDetector()

# Open json file, store content then close it
routes = open('map.json')
routes_data = json.load(routes)
routes.close()

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

def locate(data):
    routes = flatten_json(routes_data["nearest_intersection"])

    # Get key ending with searched route
    r = {key:val for key, val in routes.items() if key.endswith(data)}
    r = list(r.keys())[0]
    # r is relative position from home.
    return r

def goto(room):

    return True

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
    # Read a frame from the camera
    _, frame = video_capture.read()

    # Detect the QR Code in the image
    data, bbox, rectifiedImage = qrDecoder.detectAndDecode(frame)

    # Wait 10 ms
    key = cv2.waitKey(10)
    if key == 27:  # 'Esc' key to quit
        return False
    
    #Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False

    # Display the contents of the QR Code
    if data:
        print("QR Code data:", data)
        locate(data)
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


locate("HS3")
while(True):
    if(detectLine() == False):
        break

    if(detectQR() == False):
        break



