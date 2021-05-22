import cv2
import numpy as np
import time

cap = cv2.VideoCapture('sim_vid.mp4')
import Lines
import scanning_algorithm
###### PARAMS ###########
# LOWER_GREEN = np.array([17, 26, 0])
# UPPER_GREEN = np.array([93, 255, 255])

# for wheat
LOWER_GREEN = np.array([31, 43, 23])
UPPER_GREEN = np.array([255, 255, 100])

kernel=np.ones((5,5), np.uint8)


#########################
def nothing(x):
    pass

threshold_window_name = 'thresholding image'
threshold_trackbar_name = "threshold value"
cv2.namedWindow(threshold_window_name)
cv2.createTrackbar(threshold_trackbar_name, threshold_window_name, 0, 255, nothing)

low_hue_trackbar_name = "low hue value"
cv2.createTrackbar(low_hue_trackbar_name, threshold_window_name, 0, 255, nothing)
high_hue_trackbar_name = "high hue value"
cv2.createTrackbar(high_hue_trackbar_name, threshold_window_name, 0, 255, nothing)

low_canny_trackbar_name = "low canny value"
cv2.createTrackbar(low_canny_trackbar_name, threshold_window_name, 0, 255, nothing)
high_canny_trackbar_name = "high canny value"
cv2.createTrackbar(high_canny_trackbar_name, threshold_window_name, 0, 255, nothing)


# creates an array of x,y points for a line starting from a point on the top edge extedning to a point on the bottom edge
def create_line(start_x, start_y, end_x, end_y):
    x_diff = abs(start_x - end_x)
    y_diff = abs(start_y - end_y)
    
    # keep the length constant between the two arrays
    length = int(max(x_diff, y_diff) / 5)

    x = np.linspace(start_x, end_x, length+1).astype(int)
    y = np.linspace(start_y, end_y, length+1).astype(int)
    
    # array of x, y coordinates defining the line
    line = np.stack((x, y), axis=1)

    return line

WIDTH = 800
HEIGHT = 800

generate_lines_time = time.time()
lines = []
for top_x in range(0, WIDTH, 30):
    for bottom_x in range(0, WIDTH, 30):
        line = create_line(top_x, 0, bottom_x, HEIGHT-1)
        lines.append(line)

lines = np.array(lines)
print('elapsed time to generate lines', time.time()-generate_lines_time)

num_of_lines = 10

scanning_algorithm = scanning_algorithm.scanning_algorithm(WIDTH, HEIGHT)
while(True):
    ret, frame = cap.read()

    if (ret):
        frame, vPoint = scanning_algorithm.processFrame(frame, show=True)
        print(vPoint)
        

        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # # Filter image and allow only shades of green to pass
        # mask = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        # # Apply gaussian blur (can be removed)
        # mask = cv2.GaussianBlur(mask, (3,3), 2)
        # # mask = cv2.dilate(mask, kernel, iterations = 1)
        # # mask = cv2.erode(mask, kernel, iterations= 1)
        # # print(mask.shape)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        # mask = ~mask
        # lines_array = []

        # # find_lines_time = time.time()

        # for line in lines:
        #     row = line[:,0]
        #     col = line[:,1]
        #     extracted = mask[col, row]
        #     lines_array.append((np.sum(extracted)/len(extracted), line))
        # lines_array = np.array(lines_array)

        # values = lines_array[:, 0]
        # largest_indices = (-values).argsort()[:num_of_lines]
        # most_prominent_lines = lines_array[:, 1][largest_indices]

        # # print('time to find lines', time.time()-find_lines_time)

        # # convert to lines as defined in Lines.py
        # converted_lines = []
        # for line in most_prominent_lines:
        #     converted_line = [line[0][0], line[0][1], line[-1][0], line[-1][1]]
        #     converted_lines.append(converted_line)
        
        # intersections, points = Lines.getIntersections(converted_lines)
        # vanishing_point = Lines.drawVanishingPoint(frame, points)
        # for line in converted_lines:
        #     frame = cv2.line(frame, (line[0],line[1]), (line[2], line[3]), (255,255,255), 1) 

        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # threshold_value = cv2.getTrackbarPos(threshold_trackbar_name, threshold_window_name)
        # _, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV) 

        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # low_hue = cv2.getTrackbarPos(low_hue_trackbar_name, threshold_window_name)
        # high_hue = cv2.getTrackbarPos(high_hue_trackbar_name, threshold_window_name)
        # hsv_thresh = cv2.inRange(hsv, (low_hue, 0, 0), (high_hue, 255, 255))

        # low_canny = cv2.getTrackbarPos(low_canny_trackbar_name, threshold_window_name)
        # high_canny = cv2.getTrackbarPos(high_canny_trackbar_name, threshold_window_name)

        # canny = cv2.Canny(hsv_thresh, low_canny, high_canny)

        # cv2.imshow('frame', frame)
        # cv2.imshow('mask', mask)
        # cv2.imshow('threshold', thresh)
        # cv2.imshow('hsv threshold', hsv_thresh)
        # cv2.imshow('canny edge detection', canny)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

    else:
        break
    
cap.release()
cv2.destroyAllWindows() 
