import sys

import numpy
from cv2 import cv2
import numpy as np

"""
This class deals with everything to do with lines in our program
Every Algorithm calls functions from this class to:
    1. Draw the lines detected by OpenCV onto a frame
    2. Calculate intersections given a set of lines
    3. Draw the vanishing point onto the frame, given a set of x values
"""


def polar2points(rho, theta):
    """
    Helper function that,
    converts a polar definition of a line to a 2 point definition.
    Args:
        rho: input line rho
        theta: input line theta

    Returns:
        end_pts: [(x1, y1), (x2, y2)]
    """

    end_pts = []

    a = np.math.cos(theta)
    b = np.math.sin(theta)
    x0 = a * rho
    y0 = b * rho
    pt1 = (int(x0 + 2000 * (-b)), int(y0 + 2000 * a))
    pt2 = (int(x0 - 2000 * (-b)), int(y0 - 2000 * a))

    end_pts.append(pt1)
    end_pts.append(pt2)

    return end_pts


def getIntersections(lines):
    """
    Calculates and returns all the intersection points between positive and negative gradient lines
    also returns a set of only the x co-ordinates for the intersections, for easier processing in the main methods
    Args:
        lines: [[votes, rho, theta]]

    Returns:
        intersections: [(x,y)]
        points: [x]
    """
    intersections = []
    points = []
    lines_left = []
    lines_right = []

    # check the lines array, only run if the lines array is not empty
    if lines is not None:

        # for each line in the lines array, we determine its slope.
        for line in lines:
            x1, y1, x2, y2 = assignCoordinateValues(line)

            # if the line is a vertical line, skip it
            if x2 == x1:
                continue
            else:
                slope = (y2 - y1) / (x2 - x1)

            # we only want to process lines that have a steep slope
            # keep in mind the x values increase to the right, y values increase downwards in an image
            if slope > 1 or slope < -1:
                # cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)

                # we split the lines into two different lists by their slopes (positive vs. negative)
                # we call these two list lines_right and lines_left
                if slope > 0:
                    lines_right.append(line)
                else:
                    lines_left.append(line)

        # for each possible pairs of line that can be made from lines_right and lines_left,
        # we find an intersection point
        for lineL in lines_left:
            for lineR in lines_right:
                x1L, y1L, x2L, y2L = assignCoordinateValues(lineL)
                x1R, y1R, x2R, y2R = assignCoordinateValues(lineR)

                # calls the getIntersection helper function
                intersect = getIntersection(((x1L, y1L), (x2L, y2L)), ((x1R, y1R), (x2R, y2R)))
                if type(intersect) is bool:
                    continue

                # the intersections array is an array of points coordinates (x, y)
                # the points array is an array of the x coordinates of the points
                intersections.append(intersect)
                points.append(intersect[0])

    return intersections, points


def assignCoordinateValues(line):
    """
    Helper method,
    Since we will be working with different types of lines, we need a way to convert any type of line into
    the required form of (x1, y1), (x2, y2), which makes it easier to draw onto frame.
    This method takes any line as an input and returns it with our desired format
    :param line:
                Can be of three types currently (please update if new types of lines are utilized)
                1. Hough Algorithm: [((x1,y1), (x2,y2))] (element of outer array is of type numpy.ndarray)
                2. Contour Algorithm: [x1 y1 x2 y2]
                3. Mini Contour Algorithm: [[votes, rho, theta]]
    :return: [x1 y1 x2 y2] (converts line to point form)
    """

    # Type 1
    if isinstance(line[0], numpy.ndarray) and len(line[0]) == 4:
        x1, y1, x2, y2 = line[0]
    # Type 2
    elif isinstance(line, list) and len(line) == 4:
        x1, y1, x2, y2 = line
    # Type 3
    else:
        points = polar2points(line[0][1], line[0][2])
        x1, y1 = points[0]
        x2, y2 = points[1]

    return x1, y1, x2, y2


def getIntersection(line1, line2):
    """
    Helper function that returns the intersection point of 2 lines
    :param line1: ((x1,y1), (x2,y2))
    :param line2: ((x1,y1), (x2,y2))
    :return: (x, y)
    """
    # line has the following structure ((x1,y1), (x2,y2))
    s1, e1 = np.array(line1)
    s2, e2 = np.array(line2)

    # a1 is the slope of line1
    a1 = (s1[1] - e1[1]) / (s1[0] - e1[0])
    # b1 is the y-int of line1
    b1 = s1[1] - (a1 * s1[0])

    # a2 is the slope of line2
    a2 = (s2[1] - e2[1]) / (s2[0] - e2[0])
    # b2 is the y-int of line2
    b2 = s2[1] - (a2 * s2[0])

    # check if a1 and a2 are the same (epsilon is a very small value: 10^-16)
    if abs(a1 - a2) < sys.float_info.epsilon:
        return False

    # x coordinate is obtained by equating two mx+b equations where m is a1, a2 and b is b1, b2
    x = (b2 - b1) / (a1 - a2)
    # y coordinate is just obtained by plugging in one of the slope equations (since point must be on both lines)
    y = a1 * x + b1
    return x, y


# helper function to draw lines on given frame
def drawLinesOnFrame(lines, frame):
    """
    Helper function that draws lines on a frame
    :param lines: set of all lines
    :param frame: frame on which lines need to be drawn
    :return: the updated frame
    """
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = assignCoordinateValues(line)

            # Avoids math error, and we can skip since we don't care about horizontal lines
            if x1 == x2:
                continue
            slope = (float(y2 - y1)) / (x2 - x1)
            # Check if slope is sufficiently large, since we are interested in vertical lines
            if abs(slope) > 1:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2, cv2.LINE_AA)
    return frame


def drawVanishingPoint(frame, points):
    """
    Function that draws the vanishing point onto a frame
    :param frame: the frame on which the point needs to be drawn
    :param points: a list of x co-ordinates from which the vanishing point will be calculated
    :return: (x, y), where x is the median intersection point and y is a constant value
    """
    if len(points) != 0:
        IntersectingX = np.median(points)
        cv2.circle(frame, (int(IntersectingX), int(frame.shape[1] / 2)), 8, (255, 0, 0), -1)

        return (int(IntersectingX), int(frame.shape[1] / 2))
