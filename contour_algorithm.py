import numpy as np
import cv2 
import sys
 
class BigContours:
    def __init__(self):
        #HSV green thresholds
        self.low_green = np.array([25, 52, 72])
        self.high_green = np.array([102, 255, 255])
        
        #filtering parameters
        self.averagingKernelSize = (5,5)
        self.gaussKernelSize = (3,3)
        self.dilateKernelSize = (5,5)
        self.sigmaX = 0
 
 
        #contour parameters
        self.contourColor = (0,129,255)
        self.minContourArea = 3000
 
        #Canny edge parameters
        self.cannyLowThld = 75
        self.cannyHighThld = 150

        #dimensions
        self.HEIGHT = 720
        self.WIDTH = 1280
        
 
    def processFrame(self, frame):
        black_frame = np.uint8(np.zeros((720, 1280, 3)))
        mask = self.createBinaryMask(frame) 
        
        #Perform canny edge detection
        edges = cv2.Canny(mask,self.cannyLowThld,self.cannyHighThld)
        blurred_edges = self.gaussianBlur(edges)
        
        cnt, contour_frame = self.getContours(mask)
        cv2.drawContours(black_frame, cnt, -1, self.contourColor, 3)
        cv2.fillPoly(black_frame, pts=cnt, color=self.contourColor)
        lines, slopes, ellipseFrame = self.ellipseSlopes(cnt, black_frame)
        intersections, points = self.intersectPoint(ellipseFrame, lines)
        vanishpoint = self.vanishingPoint(ellipseFrame, points)
        
        cv2.imshow("contours", black_frame)
        cv2.imshow("ellipses", ellipseFrame)
 
 
 
 
 
    def createBinaryMask(self, frame):
        # Run averaging filter to blur the frame 
        kernel = np.ones((5,5),np.float32)/25
        frame = cv2.filter2D(frame,-1,kernel)
 
        # Convert to hsv format to allow for easier colour filtering
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Filter image and allow only shades of green to pass
        mask = cv2.inRange(hsv, self.low_green, self.high_green)
        
        return mask
 
    def gaussianBlur(self, frame):
        # Apply gaussian blur
        mask = cv2.GaussianBlur(frame, self.gaussKernelSize, self.sigmaX)
        return mask
 
    def ellipseSlopes(self, cnt, black_frame):
        slopes = []
        lines = []
        for cnt in cnt:
            if cv2.contourArea(cnt) > self.minContourArea:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                black_frame = cv2.drawContours(black_frame, [box], 0, (255, 255, 255), 2)
                ellipse = cv2.fitEllipse(cnt)
                cv2.ellipse(black_frame,ellipse,(255,255,255),2)
                rows,cols = black_frame.shape[:2]
                [vx,vy,x,y] = cv2.fitLine(cnt, cv2.DIST_L2,0,0.01,0.01)
                slope = vy/vx
                slopes.append(slope)
                lefty = int((-x*vy/vx) + y)
                righty = int(((cols-x)*vy/vx)+y)
                black_frame = cv2.line(black_frame,(cols-1,righty),(0,lefty),(255,255,0),9)
                lines.append([cols-1,righty,0,lefty])
 
        return lines, slopes, black_frame
 
 
 
    def intersectPoint(self, frame, lines):
        intersections = []
        points = []
        lines_left = []
        lines_right = []
 
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line
                if x2 == x1:
                    continue
                else:
                    slope = (y2-y1)/(x2-x1)
                if slope > 1 or slope < -1:
                    cv2.line(frame, (x1,y1), (x2,y2), (0,0,255), 1)
                    if slope > 0:
                        lines_right.append(line)
                    else:
                        lines_left.append(line)
 
            for lineL in lines_left:
                for lineR in lines_right:
                    x1L, y1L, x2L, y2L = lineL
                    x1R, y1R, x2R, y2R = lineR
                    intersect = self.getIntersection(((x1L, y1L), (x2L, y2L)), ((x1R, y1R), (x2R, y2R)))
                    if type(intersect) is bool:
                        continue
                    intersections.append(intersect)
                    points.append(intersect[0])
            
        return intersections, points
    
    
    def getIntersection(self, line1, line2):
        s1 = np.array(line1[0])
        e1 = np.array(line1[1])
 
        s2 = np.array(line2[0])
        e2 = np.array(line2[1])
 
        a1 = (s1[1] - e1[1]) / (s1[0] - e1[0])
        b1 = s1[1] - (a1 * s1[0])
 
        a2 = (s2[1] - e2[1]) / (s2[0] - e2[0])
        b2 = s2[1] - (a2 * s2[0])
 
        if abs(a1 - a2) < sys.float_info.epsilon:
            return False
 
        x = (b2 - b1) / (a1 - a2)
        y = a1 * x + b1
        return (x, y)
 
 
    def vanishingPoint(self, frame, points):
        if len(points) is not 0:
            IntersectingX = np.average(points)
            cv2.circle(frame, (int(IntersectingX), 200), 8, (255, 255, 255), -1)
 
        return (int(IntersectingX), 200)
 
 
    def getContours(self, binary_mask):
    # input binary mask, returns list of contours and blank frame with contours filled in
     frame = np.zeros((self.HEIGHT, self.WIDTH, 3))
     ret, thresh = cv2.threshold(binary_mask, 0, 254, 0)
     contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
     cnt = contours
     cv2.drawContours(frame, cnt, -1, (0,255,0), 2)
     cv2.fillPoly(frame, pts =cnt, color=(0,255,0))
 
     return cnt, frame