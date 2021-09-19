from cv2 import cv2
import numpy as np
import sys
import math
import Lines
import time
class hough_algorithm:

    def __init__(self, name=None):
        # Files, setup, OpenCV Tutorial
        # Master, demo
        # green filter
        self.name = name
        self.init_time = time.time()
        self.LOWER_GREEN = np.array([31, 43, 23])
        self.UPPER_GREEN = np.array([255, 255, 100])

        # gaussian blur / dilate
        self.K_SIZE = (3, 3)

        # canny edge
        self.CANNY_THRESH_1 = 600
        self.CANNY_THRESH_2 = 400

        # prob. hough
        self.THRESHOLD = 40
        self.MIN_LINE_LENGTH = 40
        self.MAX_LINE_GAP = 15

        # resize factor
        self.resizeFactor = 1

    # processFrame function that is called to process a frame of a video
    # takes in frame mat object obtained from cv2 video.read()
    def processFrame(self, frame):
        print('running process on ' +self.name)
        def nothing(x):
            pass

        # create trackbar with the name of the current class and set to default values at start
        tuner_name = "Hough Tuner " + str(self.name)
        cv2.imshow(tuner_name, np.zeros((1,400,3), np.uint8))
        
        cv2.createTrackbar('threshold',tuner_name,1,200,nothing)   
        cv2.createTrackbar('min line length',tuner_name,10,400,nothing)
        cv2.createTrackbar('max line gap',tuner_name,1,500,nothing)
        cv2.createTrackbar('rho',tuner_name,1,5,nothing)
        cv2.createTrackbar('theta',tuner_name,1, 180,nothing)
        cv2.createTrackbar('canny low',tuner_name,1, 2000,nothing)
        cv2.createTrackbar('canny high',tuner_name,1, 2000,nothing)
        cv2.createTrackbar('blur kernel size',tuner_name,1, 100,nothing)

        if (time.time() - self.init_time) < 2.0:
            # initial values for PID control
            cv2.setTrackbarPos('threshold',tuner_name, 40)   
            cv2.setTrackbarPos('min line length',tuner_name, 40)
            cv2.setTrackbarPos('max line gap',tuner_name, 15)
            cv2.setTrackbarPos('rho',tuner_name, 1)
            cv2.setTrackbarPos('theta',tuner_name, 1)
            cv2.setTrackbarPos('canny low', tuner_name, 400)
            cv2.setTrackbarPos('canny high', tuner_name, 600)
            cv2.setTrackbarPos('blur kernel size', tuner_name, 3)
        cv2.waitKey(1)
        # create mask by filtering image colors
        k = cv2.getTrackbarPos('blur kernel size', tuner_name)*2 + 1
        self.K_SIZE =  (k,k)
        mask = self.createMask(frame)
        
        # dilate the mask
        mask = cv2.dilate(mask, np.ones((k,k), np.uint8), iterations = 1)
        cv2.imshow('front mask', mask)
        # Resize frame to smaller size to allow faster processing
        frame = self.resize(frame, self.resizeFactor)
        mask = self.resize(mask, self.resizeFactor)

        # Perform Canny Edge Detection
        canny_low = cv2.getTrackbarPos('canny low', tuner_name)
        canny_high = cv2.getTrackbarPos('canny high', tuner_name)
        edges = cv2.Canny(mask, canny_low, canny_high)
  
        # # Sobel Edge detector
        # scale = 10
        # delta = 30
        # ddepth = cv2.CV_16S
        # grad_x = cv2.Sobel(mask, ddepth, 1, 0, ksize=3, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)
        # # Gradient-Y
        # # grad_y = cv2.Scharr(gray,ddepth,0,1)
        # grad_y = cv2.Sobel(mask, ddepth, 0, 1, ksize=3, scale=scale, delta=delta, borderType=cv2.BORDER_DEFAULT)
        # abs_grad_x = cv2.convertScaleAbs(grad_x)
        # abs_grad_y = cv2.convertScaleAbs(grad_y)
        # edges = cv2.addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0)
        edges = cv2.dilate(edges, np.ones((3,3)), iterations=1)
        cv2.imshow('edges '+str(self.name), edges)
        cv2.waitKey(1)

        # get values from Trackbar
        rho = cv2.getTrackbarPos('rho', tuner_name)  # distance resolution in pixels of the Hough grid
        theta = cv2.getTrackbarPos('theta', tuner_name) * np.pi / 180  # angular resolution in radians of the Hough grid
        # minimum number of votes (intersections in Hough grid cell)
        threshold = cv2.getTrackbarPos('threshold', tuner_name)
        min_line_length = cv2.getTrackbarPos('min line length', tuner_name)  # minimum number of pixels making up a line
        max_line_gap = cv2.getTrackbarPos('max line gap', tuner_name)  # maximum gap in pixels between connectable line segments

        # Run Hough on edge detected image
        # Output "lines" is an array containing endpoints of detected line segments
        # lines = cv2.HoughLinesP(edges, rho=rho, theta=theta, threshold=threshold, lines=np.array([]),
        #                     minLineLength=min_line_length, maxLineGap=max_line_gap)

        # lines = cv2.HoughLines(edges, rho=rho, theta=theta,
        #                     threshold=threshold, min_theta=math.pi/180, max_theta=math.pi)

        lines = cv2.HoughLinesP(
                            edges,
                            rho,
                            theta,
                            threshold,
                            np.array([]),
                            min_line_length,
                            maxLineGap=max_line_gap
                            )

        # lines = cv2.HoughLines(edges, rho=rho, theta=theta,
        #                     threshold=threshold, min_theta=math.pi/180, max_theta=math.pi)

        # write values onto frame
        print_location = 50
        for label, value in (('rho (pixels)', rho), ('theta (deg)', theta*180/np.pi), ('threshold', threshold), ('min line length', min_line_length), ('max line gap', max_line_gap), ('canny low', canny_low), ('canny high', canny_high), ('kernel size', self.K_SIZE)):
            frame = cv2.putText(frame, label + " " + str(value), (50, print_location), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,123,123), 2, cv2.LINE_AA)
            print_location += 30
        
        if lines is not None:
            i = 0
            while i < len(lines):
                line = lines[i]
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                j = i + 1
                while j < len(lines):
                    other_line = lines[j]
                    other_rho = lines[j][0][0]
                    other_theta = lines[j][0][1]
                    if math.atan(abs(-math.cos(theta)/math.sin(theta) + math.cos(other_theta)/math.sin(other_theta))) < math.pi/180 * 10:
                        # abs(other_rho - rho) < 60 and abs(other_theta-theta) < math.pi/180 * 15:
                        lines = np.delete(lines, j, 0)
                        lines[i][0][0] = (other_rho + rho)/2
                        lines[i][0][1] = (other_theta + theta)/2
                    else:
                        j += 1
                i += 1

        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                # print('rho', rho, 'theta', theta)
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                const = 2000
                pt1 = (int(x0 + const*(-b)), int(y0 + const*(a)))
                pt2 = (int(x0 - const*(-b)), int(y0 - const*(a)))
                cv2.line(frame, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

        intersect_points = []
        slopes = []
        if lines is not None:
            i = 0
            while (i < len(lines)):
                line = lines[i]
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                m = -math.cos(theta)/math.sin(theta)
                slopes.append(m)
                b = rho/math.sin(theta)
                for next_line in lines[i+1:]:
                    next_rho = next_line[0][0]
                    next_theta = next_line[0][1]
                    next_m = -math.cos(next_theta)/math.sin(next_theta)
                    next_b = next_rho/math.sin(next_theta)
                   
                    print('b: ', b, 'next_b:', next_b, 'm:', m, 'next_m:', next_m)
                    if abs(next_m - m) < 0.00001:
                        x = 99999
                    else:
                        x = int((b - next_b)/(next_m - m))
                    y = int(m*x + b)
                    intersect_points.append((x, y))

                i += 1

        sum_x, sum_y = 0, 0
        for point in intersect_points:
            sum_x += point[0]
            sum_y += point[1]
            cv2.circle(frame, point, 10, [0, 255, 0], -1)
        if len(intersect_points) > 0:
            line_cX = int(sum_x / len(intersect_points))
            line_cY = int(sum_y / len(intersect_points))
        else:
            line_cX = int(frame.shape[0]/2)
            line_cY = int(frame.shape[1]/2)
        
        
   
        return frame, (line_cX, line_cY)

    # helper function to create a mask
    # takes in frame mat object, returns mask mat object
    def createMask(self, frame):
        # Convert to hsv format to allow for easier colour filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Filter image and allow only shades of green to pass
        mask = cv2.inRange(hsv, self.LOWER_GREEN, self.UPPER_GREEN)
        # Apply gaussian blur (can be removed)
        mask = cv2.GaussianBlur(mask, self.K_SIZE, 2)

        return mask


    # helper function to resize a frame mat object
    def resize(self, frame, factor):
        # resize frame to smaller size to allow faster processing
        return cv2.resize(frame, (int(frame.shape[1] / factor), int(frame.shape[0] / factor)))
