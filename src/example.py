#!/usr/bin/env python3

############################################
# File: example.py
# Author: Jacob English, je787413@ohio.edu
#
# Example usage for UAVHeading Class
############################################

from UAVHeading import UAVHeading

def main():
                    # (pos, waypt, speed, heading, tPossible)
    uav0 = UAVHeading((1.515593587271553, -132.53722833033987),
                      (1.5156, -132.5111),
                      50.0,
                      90.0,
                      30.0)

    uav1 = UAVHeading((1.5098039166143598, -132.531099),
                      (1.5267, -132.531),
                      50.0,
                      0.0,
                      30.0)

    waypoints, _ = uav0.avoid(uav1)
    print(waypoints)

                    # (pos, waypt, speed, heading, tPossible)
    uav0 = UAVHeading((1.5156, -132.5423),
                      (1.5156, -132.5111),
                      50.0,
                      90.0,
                      30.0)

    uav1 = UAVHeading((1.5156, -132.5321),
                      (1.5156, -132.403),
                      50.0,
                      270.0,
                      30.0)

    waypoints, _ = uav0.avoid(uav1)
    print(waypoints)

if __name__ == '__main__':
    main()