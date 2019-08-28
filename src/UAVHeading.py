# File: UAVHeading.py
# Author: Jacob English, je787413@ohio.edu
############################################

import math
import matplotlib.pyplot as plt
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from AStar import a_star_planning
from UAVHcfg import *
from TerminalColors import TerminalColors as TC

'''
 Class: UAVHeading
'''
class UAVHeading:
    position = []
    waypoint = []
    speed = 0
    time = 0
    thetaRef = 0
    thetaPossible = 0

    staticAreaLength = False
    shift_x = 0
    shift_y = 0
    lastClear = False

    '''
     UAVHeading Function: __init__
        Parameters: 
                    pos: UAV position (x, y),
                    waypt: UAV target position (x, y),
                    speed: UAV Speed (m/s),
                    heading: UAV heading (degrees),
                    tPossible: possible turn angle for UAV (degrees)
        Description:
                    Constructor for UAVHeading Class.
    '''
    def __init__(self, pos, waypt, speed, heading, tPossible):
        self.position = list(pos)
        self.waypoint = list(waypt)
        self.speed = speed
        self.thetaRef = heading
        # self.thetaRef = 90 - heading
        self.thetaPossible = tPossible
        # self.staticAreaLength = False


    '''
     UAVHeading Function: __weightedSideDecision
        Parameters:
                    uav0: UAVHeading running the avoidance function,
                    uav_others: list of other UAVHeading objects,
                    keepOutZones: list of keep out zone objects from scenario environment
        Description:
                    Returns a polygon defining the possible flight
                    area for the UAV calculated using the init values.
    '''
    def __weightedSideDecision(self, uav0, uav_others, keepOutZones):
        side_sum = 0

        if (45 < self.thetaRef < 135) or (225 < self.thetaRef < 315): # use y position difference
        
            side_sum += DECISION_WEIGHTS[0] * abs(self.position[1] - uav0.position[1])
            side_sum += DECISION_WEIGHTS[1] * abs(self.position[1] - uav0.waypoint[1])

            for uav in uav_others:
                side_sum -= DECISION_WEIGHTS[2] * abs(self.position[1] - uav.position[1])
            for koz in keepOutZones:
                kx = [pt[0] for pt in koz]
                ky = [pt[0] for pt in koz]
                centroid = (sum(kx) / len(koz), sum(ky) / len(koz))
                kPoly = Polygon(koz)
                side_sum -= DECISION_WEIGHTS[3] * kPoly.area * abs(self.position[1] - centroid[1])

            if abs(self.thetaRef - 90) > abs(self.thetaRef - 270):
                if side_sum > 0:
                    return 1
                else:
                    return -1
            else:
                if side_sum > 0:
                    return -1
                else:
                    return 1
        else: # use x position difference

            side_sum += DECISION_WEIGHTS[0] * abs(self.position[0] - uav0.position[0])
            side_sum += DECISION_WEIGHTS[1] * abs(self.position[0] - uav0.waypoint[0])

            for uav in uav_others:
                side_sum -= DECISION_WEIGHTS[2] * abs(self.position[0] - uav.position[0])
            for koz in keepOutZones:
                kx = [pt[0] for pt in koz]
                ky = [pt[0] for pt in koz]
                centroid = (sum(kx) / len(koz), sum(ky) / len(koz))
                kPoly = Polygon(koz)
                side_sum -= DECISION_WEIGHTS[3] * kPoly.area * abs(self.position[0] - centroid[0])

            if abs(self.thetaRef) > abs(self.thetaRef - 180):
                if side_sum > 0:
                    return -1
                else:
                    return 1
            else:
                if side_sum > 0:
                    return 1
                else:
                    return -1

    '''
     UAVHeading Function: possibleFlightArea
        Parameters: NONE
        Description:
                    Returns a polygon defining the possible flight
                    area for the UAV calculated using the init values.
    '''
    def possibleFlightArea(self, area_length, uav0, uavh_others, static_kozs):
        theta_ref = math.radians(self.thetaRef)
        theta_possible = math.radians(self.thetaPossible)
    
        side_decision = 0

        points = [list(self.position)]

        if self.staticAreaLength:
            area_length = self.staticAreaLength
            side_decision = self.__weightedSideDecision(uav0, uavh_others, static_kozs) # stub uav_others and koz lists for now

            if side_decision < 0:
                points[-1][0] = self.position[0] + (3 * area_length * math.cos(theta_ref - (theta_possible / 2)))
                points[-1][1] = self.position[1] + (3 * area_length * math.sin(theta_ref - (theta_possible / 2)))

        for div in range(-2, -5, -1):
            pt_x = self.position[0] + (area_length * math.cos(theta_ref + (theta_possible / div)))
            pt_y = self.position[1] + (area_length * math.sin(theta_ref + (theta_possible / div)))
            points.append([pt_x, pt_y])

        # +-0
        pt_x = self.position[0] + (area_length * math.cos(theta_ref))
        pt_y = self.position[1] + (area_length * math.sin(theta_ref))
        points.append([pt_x, pt_y])

        for div in range(4, 1, -1):
            pt_x = self.position[0] + (area_length * math.cos(theta_ref + (theta_possible / div)))
            pt_y = self.position[1] + (area_length * math.sin(theta_ref + (theta_possible / div)))
            points.append([pt_x, pt_y])

        if self.staticAreaLength and side_decision > 0:
            points[-1][0] = self.position[0] + (2 * area_length * math.cos(theta_ref + (theta_possible / 2)))
            points[-1][1] = self.position[1] + (2 * area_length * math.sin(theta_ref + (theta_possible / 2)))

        points.append(list(self.position))

        # if uav0 is in possible flight area, recalculate with length/2
        pt = Point(uav0.position[0], uav0.position[1])
        koz_polygon = Polygon(points)
        if koz_polygon.contains(pt):
            print(TC.FAIL + '[HOTFIX - Line 152 | Area Length for Head-On Collision]' + TC.ENDC)
            if self.staticAreaLength:
                self.staticAreaLength = self.staticAreaLength / 2
            else:
                self.staticAreaLength = area_length / 4
            points = self.possibleFlightArea(area_length, uav0, uavh_others, static_kozs)

        return points

    '''
     UAVHeading Function: __lineIntersect
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
     UAVHeading Function: __distance
        Parameters:
                    a: point (x, y),
                    b: point (x, y)
        Description:
                    Returns the distance from point a to b.
    '''
    def __distance(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    '''
     UAVHeading Function: __isBetween
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
     UAVHeading Function: __findIntersects
        Parameters:
                    uavh_other: UAVHeading object to avoid
        Description:
                    Finds intersection points between the
                    UAVHeading path and the possible flight area polygon
                    of the UAVHeading uavh_other.
                    Returns:
                        - Intersection list
                        - UAVHeading possible flight polygon point list
    '''
    def __findIntersects(self, uavh_others, static_kozs):
        intersects = []
        koz_areas = []
        self_line = [(self.position[0], self.position[1]), (self.waypoint[0], self.waypoint[1])]

        # check for potential UAV collisions
        for uavh_other in uavh_others:
            tmp_intersects = []
            other_area_points = []
            distance_to_other = self.__distance(self.position, uavh_other.position)

            if distance_to_other < DISTANCE_THRESHOLD:
                other_area_points = uavh_other.possibleFlightArea((2 * distance_to_other), self, uavh_others, static_kozs)
                for j in range(len(other_area_points) -1):
                    other_line = [other_area_points[j], other_area_points[j+1]]
                    try:
                        point = self.__lineIntersect(self_line, other_line)

                        if (self.__isBetween(self_line[0], point, self_line[1]) and self.__isBetween(other_line[0], point, other_line[1])):
                            tmp_intersects.append(point)
                    except ValueError:
                        continue

                koz_areas.append(other_area_points)
                intersects = intersects + tmp_intersects

        # check for potential static KoZ collisions
        for area in static_kozs:
            koz_areas.append(area)
            for i in range(len(area) - 1):
                other_line = [area[i], area[i+1]]
                try:
                    point = self.__lineIntersect(self_line, other_line)

                    if (self.__isBetween(self_line[0], point, self_line[1]) and self.__isBetween(other_line[0], point, other_line[1])):
                        intersects.append(point)
                        break
                except ValueError:
                    continue

        return intersects, koz_areas

    '''
    UAVHeading Function: __scale_border
        Parameters:
                    border: List of points to define search
                            border for A*,
                    center: center point of border region,
                    offset: value to offset border by
        Description:
                    Returns the list points to define the scaled border.
    '''
    def __scale_border(self, border, center, offset):
        for pt in border:
            if pt[0] > center[0]:
                pt[0] += offset
            else:
                pt[0] -= offset
            if pt[1] > center[1]: 
                pt[1] += offset
            else:
                pt[1] -= offset
        return border

    '''
    UAVHeading Function: __intermediates
        Parameters:
                    p1: first point (x,y),
                    p2: end point (x,y),
                    interval: distance between points on line
        Description:
                    Returns the list of points spaced between
                    p1 and p2.
    '''
    def __intermediates(self, p1, p2, interval):
        """ Credit:
            https://stackoverflow.com/questions/43594646/how-to-calculate-the-coordinates-of-the-line-between-two-points-in-python
        """
        nb_points = int(self.__distance(p1, p2) / interval)

        x_spacing = (p2[0] - p1[0]) / (nb_points + 1)
        y_spacing = (p2[1] - p1[1]) / (nb_points + 1)

        return [[p1[0] + i * x_spacing, p1[1] +  i * y_spacing] 
            for i in range(1, nb_points+1)]

    '''
    UAVHeading Function: __midpoint
        Parameters:
                    a: first point (x,y),
                    b: second point (x,y)
        Description:
                    Returns the midpoint of a and b
    '''
    def __midpoint(self, a, b):
        a = (float(a[0]), float(a[1]))
        b = (float(b[0]), float(b[1]))
        return [ (a[0]+b[0])/2, (a[1]+b[1])/2 ]

    '''
    UAVHeading Function: __format_astar_input
        Parameters:
                    koz: area points list to avoid from 
                         other UAV
        Description:
                    Returns formatted data for A*:
                        - Start Position
                        - Goal Position
                        - Border for Search Area
                        - KeepOut Zone Points for other UAV


                        - use_pseudo_target
    '''
    def __format_astar_input(self, kozList, staticAreaLength):
        if staticAreaLength:
            print(TC.OKBLUE + '\t<Using static avoid area length>' + TC.ENDC)
        
        # Make Border - find min and max for x and y values
        x_min, y_min = self.position[0], self.position[1]
        x_max, y_max = self.position[0], self.position[1]

        # pseudo_target = self.__midpoint(self.position, self.waypoint)

        # check if pseudo-target is reachable
        # pt = Point(pseudo_target[0], pseudo_target[1])
        # koz_polygon = Polygon(koz)
        # koz_scale = koz_polygon.buffer(2 * INTERVAL_SIZE) # buffer size for uav0 in A* search

        use_pseudo_target = False #not koz_scale.contains(pt)

        if not use_pseudo_target:# and not staticAreaLength:
            print(TC.OKBLUE + '\t<Using real target position>' + TC.ENDC)

            # compare with target position
            if x_min > self.waypoint[0]:
                x_min = self.waypoint[0]
            if y_min > self.waypoint[1]:
                y_min = self.waypoint[1]

            if x_max < self.waypoint[0]:
                x_max = self.waypoint[0]
            if y_max < self.waypoint[1]:
                y_max = self.waypoint[1]
        else:
            print(TC.OKBLUE + '\t<Using pseudo-target position>' + TC.ENDC)

            # compare with pseudo-target position
            if x_min > pseudo_target[0]:
                x_min = pseudo_target[0]
            if y_min > pseudo_target[1]:
                y_min = pseudo_target[1]

            if x_max < pseudo_target[0]:
                x_max = pseudo_target[0]
            if y_max < pseudo_target[1]:
                y_max = pseudo_target[1]

        if not staticAreaLength:
            # compare with uav other position
            if x_min > koz[0][0]:
                x_min = koz[0][0]
            if y_min > koz[0][1]:
                y_min = koz[0][1]

            if x_max < koz[0][0]:
                x_max = koz[0][0]
            if y_max < koz[0][1]:
                y_max = koz[0][1]
        else:
            # compare with all koz points
            for koz in kozList:
                for pt in koz:
                    if x_min > pt[0]:
                        x_min = pt[0]
                    if y_min > pt[1]:
                        y_min = pt[1]

                    if x_max < pt[0]:
                        x_max = pt[0]
                    if y_max < pt[1]:
                        y_max = pt[1]
        
        border_pts = [[x_max, y_max], 
                      [x_max, y_min],
                      [x_min, y_max],
                      [x_min, y_min]]

        # add padding to border
        center = self.__midpoint((x_max, y_max), (x_min, y_min))
        border_pts = self.__scale_border(border_pts, center, (4 * INTERVAL_SIZE))

        # shift (minx, miny) to (0, 0) for A*
        if (border_pts[3][0] < 0): # x min < 0
            self.shift_x = abs(border_pts[3][0])
        elif (border_pts[3][0] > 0): # x min > 0
            self.shift_x = -abs(border_pts[3][0])
        if (border_pts[3][1] < 0): # y min < 0
            self.shift_y = abs(border_pts[3][1])
        elif (border_pts[3][1] > 0): # y min > 0
            self.shift_y = -abs(border_pts[3][1])
        
        # shift border corners
        for i in range(len(border_pts)):
            border_pts[i][0] += self.shift_x
            border_pts[i][1] += self.shift_y

        # add interval points for border
        border_pts += self.__intermediates(border_pts[0], border_pts[1], INTERVAL_SIZE)
        border_pts += self.__intermediates(border_pts[1], border_pts[3], INTERVAL_SIZE)
        border_pts += self.__intermediates(border_pts[2], border_pts[0], INTERVAL_SIZE)
        border_pts += self.__intermediates(border_pts[3], border_pts[2], INTERVAL_SIZE)

        # modifying koz list passed by reference causes a bug for using real target case
        _kozList = [] 
        # shift KeepOut zone points
        for koz in kozList:
            tmp = []
            for pt in koz:
                tmp.append([(pt[0] + self.shift_x), (pt[1] + self.shift_y)])
            _kozList.append(tmp)

        # add interval points for koz
        koz_pts = []
        for _koz in _kozList:
            for i in range(len(_koz) -1):
                koz_pts += self.__intermediates(_koz[i], _koz[i+1], INTERVAL_SIZE)
            koz_pts += self.__intermediates(_koz[-1], _koz[0], INTERVAL_SIZE)
            koz_pts += self.__intermediates(_koz[0], _koz[1], INTERVAL_SIZE)

        # shift start and goal positions
        start_pt = [(self.position[0] + self.shift_x),
                    (self.position[1] + self.shift_y)]
        goal_pt = []
        if not use_pseudo_target:
            goal_pt = [(self.waypoint[0] + self.shift_x),
                       (self.waypoint[1] + self.shift_y)]
        else:
            goal_pt = [(pseudo_target[0] + self.shift_x),
                    (pseudo_target[1] + self.shift_y)]

        return start_pt, goal_pt, border_pts, koz_pts, use_pseudo_target

    '''
    UAVHeading Function: avoid
        Parameters:
                    uavh_other: UAVHeading object to avoid
        Description:
                    Returns the list of waypoints generated by
                    the A* search algorithm.
    '''
    def avoid(self, uavh_others, static_kozs):
        intersects, avoid_areas = self.__findIntersects(uavh_others, static_kozs)
        if len(intersects) == 0:
            if not self.lastClear:
                print(TC.OKGREEN + 'PATH CLEAR.' + TC.ENDC)
            self.lastClear = True
            return [self.waypoint], avoid_areas

        print(TC.WARNING + 'AVOID.' + TC.ENDC)
        self.lastClear = False

        use_pseudo_target = False
        try: # get optimal path to destination
            # format UAVHeading data for A* input
            start, goal, border, koz, use_pseudo_target = self.__format_astar_input(avoid_areas, True)#bool(uavh_other.staticAreaLength))

            ox, oy = [], []
            for pt in border:
                ox.append(pt[0])
                oy.append(pt[1])
                
            for pt in koz:
                ox.append(pt[0])
                oy.append(pt[1])
                
            if SHOW_ANIMATION:  # pragma: no cover
                plt.plot(ox, oy, ".k", label='Search Area Obstacles')
                plt.plot(start[0], start[1], "xr", label='UAV0 Position')
                plt.plot(goal[0], goal[1], "xb", label='UAV0 Goal')
                plt.grid(True)
                plt.axis("equal")

            path_x, path_y = a_star_planning(start[0], start[1],
                                             goal[0], goal[1],
                                             ox, oy,
                                             INTERVAL_SIZE, (2 * INTERVAL_SIZE))
        except ValueError:
            print(TC.FAIL + '\t\t**No valid path found.**' + TC.ENDC)
            return [], avoid_areas

        if SHOW_ANIMATION:  # pragma: no cover
            plt.plot(path_x, path_y, "-r", label='Shortest Path')
            plt.legend()
            plt.show()

        # format A* output for waypoint list
        path_pts = []
        if use_pseudo_target:
            path_pts.append(self.waypoint)
        for i in range(len(path_x)):
            pt = []
            pt.append(path_x[i] - self.shift_x)
            pt.append(path_y[i] - self.shift_y)

            # ignore extra waypoints that are between the previous and next
            if (i > 0) and (i < len(path_x) - 1):
                last_pt = []
                last_pt.append(path_x[i-1] - self.shift_x)
                last_pt.append(path_y[i-1] - self.shift_y)

                next_pt = []    
                next_pt.append(path_x[i+1] - self.shift_x)
                next_pt.append(path_y[i+1] - self.shift_y)

                if not (self.__isBetween(last_pt, pt, next_pt)):
                    path_pts.append(pt)
            else:
                path_pts.append(pt)
        return path_pts, avoid_areas