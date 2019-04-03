# File: UAVHeading.py
# Author: Jacob English, je787413@ohio.edu
############################################

import math

'''
 Class: UAVHeading
'''
class UAVHeading:
    position = ()
    speed = 0
    time = 0
    thetaRef = 0
    thetaPossible = 0

    '''
     UAVHeading Function: __init__
        Parameters: 
                    pos: UAV position (x, y),
                    speed: UAV Speed (m/s),
                    time: time to calculate maximum distance for flight (s),
                    heading: UAV heading (degrees),
                    tPossible: possible turn angle for UAV (degrees)
        Description:
                    Constructor for UAVHeading Class.
    '''
    def __init__(self, pos, speed, time, heading, tPossible):
        self.position = pos
        self.speed = speed
        self.time = time
        self.thetaRef = heading
        # self.thetaRef = 90 - heading
        self.thetaPossible = tPossible
        self.staticAreaLength = False

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