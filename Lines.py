# RhoTheta to points
# LineEq to points
import sys

import numpy
from cv2 import cv2
import numpy as np


def polar2cartesian(rho: float, theta_rad: float, rotate90: bool = False):
    """
    Converts line equation from polar to cartesian coordinates

    Args:
        rho: input line rho
        theta_rad: input line theta
        rotate90: output line perpendicular to the input line

    Returns:
        m: slope of the line
           For horizontal line: m = 0
           For vertical line: m = np.nan
        b: intercept when x=0
    """
    x = np.cos(theta_rad) * rho
    y = np.sin(theta_rad) * rho
    m = np.nan
    if not np.isclose(x, 0.0):
        m = y / x
    if rotate90:
        if m is np.nan:
            m = 0.0
        elif np.isclose(m, 0.0):
            m = np.nan
        else:
            m = -1.0 / m
    b = 0.0
    if m is not np.nan:
        b = y - m * x

    return m, b


def polar2points(rho: float, theta: float):
    """
    Returns end points of the line on the end of the image
    Args:
        rho: input line rho
        theta: input line theta

    Returns:
        list: [(x1, y1), (x2, y2)]
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
    intersections = []
    points = []
    lines_left = []
    lines_right = []

    # check the lines array, only run if the lines array is not empty
    if lines is not None:

        # for each line in the lines array, we determine its slope.
        for line in lines:
            x1, x2, y1, y2 = assignCoordinateValues(line)

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
                x1L, x1R, x2L, x2R, y1L, y1R, y2L, y2R = assignCoordinateValuesLeftyRighty(lineL, lineR)

                # calls the getIntersection helper function
                intersect = getIntersection(((x1L, y1L), (x2L, y2L)), ((x1R, y1R), (x2R, y2R)))
                if type(intersect) is bool:
                    continue

                # the intersections array is an array of points coordinates (x, y)
                # the points array is an array of the x coordinates of the points
                intersections.append(intersect)
                points.append(intersect[0])

    return intersections, points


def assignCoordinateValuesLeftyRighty(lineL, lineR):
    if isinstance(lineL[0], numpy.ndarray) and len(lineL[0]) == 4:
        x1L, y1L, x2L, y2L = lineL[0]
        x1R, y1R, x2R, y2R = lineR[0]
    elif isinstance(lineL, list) and len(lineL) == 4:
        x1L, y1L, x2L, y2L = lineL
        x1R, y1R, x2R, y2R = lineR
    else:
        x1L, y1L = lineL[1]
        x2L, y2L = lineL[2]
        x1R, y1R = lineR[1]
        x2R, y2R = lineR[2]
    return x1L, x1R, x2L, x2R, y1L, y1R, y2L, y2R


def assignCoordinateValues(line):
    if isinstance(line[0], numpy.ndarray) and len(line[0]) == 4:
        x1, y1, x2, y2 = line[0]
    elif isinstance(line, list) and len(line) == 4:
        x1, y1, x2, y2 = line
    else:
        x1, y1 = line[1]
        x2, y2 = line[2]
    return x1, x2, y1, y2


# helper function to help determine the intersection point coordinate given two lines
def getIntersection(line1, line2):
    # line has the following structure ((x1,y1), (x2,y2))
    s1 = np.array(line1[0])
    e1 = np.array(line1[1])

    s2 = np.array(line2[0])
    e2 = np.array(line2[1])

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
    if lines is not None:
        for line in lines:
            x1, x2, y1, y2 = assignCoordinateValues(line)

            # Avoids math error, and we can skip since we don't care about horizontal lines
            if x1 == x2:
                continue
            slope = (float(y2 - y1)) / (x2 - x1)
            # Check if slope is sufficiently large, since we are interested in vertical lines
            if abs(slope) > 1:
                cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 6, cv2.LINE_AA)
    return frame


# helper function to draw the vanishing point on frame
def drawVanishingPoint(frame, points):
    if len(points) != 0:
        IntersectingX = np.median(points)
        cv2.circle(frame, (int(IntersectingX), int(frame.shape[1] / 2)), 8, (255, 0, 0), -1)

        return (int(IntersectingX), int(frame.shape[1] / 2))
