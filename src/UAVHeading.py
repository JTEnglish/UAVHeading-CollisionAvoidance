# File: UAVHeading.py
# Author: Jacob English, je787413@ohio.edu
############################################

import math

'''
 Class: UAVHeading
'''
class UAVHeading:
    position = ()
    waypoint = ()
    speed = 0
    time = 0
    thetaRef = 0
    thetaPossible = 0
    staticAreaLength = False

    '''
     UAVHeading Function: __init__
        Parameters: 
                    pos: UAV position (x, y),
                    waypt: UAV target position (x, y),
                    speed: UAV Speed (m/s),
                    time: time to calculate maximum distance for flight (s),
                    heading: UAV heading (degrees),
                    tPossible: possible turn angle for UAV (degrees)
        Description:
                    Constructor for UAVHeading Class.
    '''
    def __init__(self, pos, waypt, speed, time, heading, tPossible):
        self.position = pos
        self.waypoint = waypt
        self.speed = speed
        self.time = time
        self.thetaRef = heading
        # self.thetaRef = 90 - heading
        self.thetaPossible = tPossible
        # self.staticAreaLength = False

    '''
     UAVHeading Function: possibleFlightArea
        Parameters: NONE
        Description:
                    Returns a polygon defining the possible flight
                    area for the UAV calculated using the init values.
    '''
    def possibleFlightArea(self, area_length):
        theta_ref = math.radians(self.thetaRef)
        theta_possible = math.radians(self.thetaPossible)

        if self.staticAreaLength:
            area_length = self.staticAreaLength

        points = [self.position]

        for div in range(-2, -5, -1):
            print(div, theta_ref + (theta_possible / div))
            pt_x = self.position[0] + (area_length * math.cos(theta_ref + (theta_possible / div)))
            pt_y = self.position[1] + (area_length * math.sin(theta_ref + (theta_possible / div)))
            points.append((pt_x, pt_y))

        # +-0
        pt_x = self.position[0] + (area_length * math.cos(theta_ref))
        pt_y = self.position[1] + (area_length * math.sin(theta_ref))
        points.append((pt_x, pt_y))

        for div in range(4, 1, -1):
            print(div, theta_ref + (theta_possible / div))
            pt_x = self.position[0] + (area_length * math.cos(theta_ref + (theta_possible / div)))
            pt_y = self.position[1] + (area_length * math.sin(theta_ref + (theta_possible / div)))
            points.append((pt_x, pt_y))

        points.append(self.position)
        return points

    '''
     UAVLine Function: __lineIntersect
        Parameters:
                    line1: [(x0, y0), (x1, y1)],
                    line2: [(x0, y0), (x1, y1)]
        Description:
                    Returns intersection point (x, y) of two lines.
    '''
    def __lineIntersect(self, line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
           raise ValueError('lines do not intersect')

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y

    '''
     UAVLine Function: __distance
        Parameters:
                    a: point (x, y),
                    b: point (x, y)
        Description:
                    Returns the distance from point a to b.
    '''
    def __distance(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    '''
     UAVLine Function: __isBetween
        Parameters:
                    pt0: point (x, y),
                    intersect: point (x, y),
                    pt1: point (x, y),
        Description:
                    Returns True if the intersect point is on the
                    line segment defined by pt0 and pt1. If not,
                    the function returns False.
    '''
    def __isBetween(self, pt0, intersect, pt1):
        distAB = self.__distance(pt0, intersect)
        distBC = self.__distance(intersect, pt1)
        distAC = self.__distance(pt0, pt1)

        # triangle inequality
        return math.isclose((distAB + distBC), distAC)

    '''
     UAVLine Function: __findIntersects
        Parameters:
                    uavh_other: UAVHeading object to avoid
        Description:
                    Finds intersection points between the
                    UAVLine path and the possible flight area polygon
                    of the UAVHeading uavh_other.
                    Returns:
                        - Intersection list
                        - UAVHeading possible flight polygon point list
    '''
    def __findIntersects(self, uavh_other):
        intersects = []
        other_area_points = []
        self_line = [(self.position[0], self.position[1]), (self.waypoint[0], self.waypoint[1])]
        # print('self line', self_line)

        distance_to_other = self.__distance(self.position, uavh_other.position)

        ######### STUB DISTANCE THRESHOLD
        if distance_to_other < 0.01:
            other_area_points = uavh_other.possibleFlightArea((2 * distance_to_other))
            for j in range(len(other_area_points) -1):
                other_line = [other_area_points[j], other_area_points[j+1]]
                # print('\tother line', other_line)
                try:
                    point = self.__lineIntersect(self_line, other_line)

                    if (self.__isBetween(self_line[0], point, self_line[1]) and self.__isBetween(other_line[0], point, other_line[1])):
                        intersects.append(point)
                except ValueError:
                    continue
            if len(intersects) == 1: # UAV 0 position possibly in UAV 1 flight area
                if not uavh_other.staticAreaLength: # set to static flight area length
                    uavh_other.staticAreaLength = distance_to_other / 2
                other_area_points = uavh_other.possibleFlightArea(uavh_other.staticAreaLength)
                for j in range(len(other_area_points) -1):
                    other_line = [other_area_points[j], other_area_points[j+1]]
                    # print('\tother line', other_line)
                    try:
                        point = self.__lineIntersect(self_line, other_line)

                        if (self.__isBetween(self_line[0], point, self_line[1]) and self.__isBetween(other_line[0], point, other_line[1])):
                            intersects.append(point)
                    except ValueError:
                        continue
            elif uavh_other.staticAreaLength: # if there are 2 intersections and the static flight area length is set, reset
                uavh_other.staticAreaLength = False
        print(intersects)
        return intersects, other_area_points

        '''
        UAVLine Function: avoid
            Parameters:
                        uavh_other: UAVHeading object to avoid
            Description:
                        Returns the list of waypoints generated by
                        the A* search algorithm.
        '''
        def avoid(self, uavh_other):
            intersects, area_points = self.__findIntersects(uavh_other)
            if len(intersects) == 0:
                raise ValueError('Nothing to Avoid.')

            max_dist = 0
            select_point = self.position
            for pt in intersects:
                dist = self.__distance(self.position, pt)
                if dist > max_dist:
                    max_dist = dist
                    select_point = pt
            return select_point, area_points