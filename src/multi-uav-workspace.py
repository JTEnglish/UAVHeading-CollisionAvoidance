#!/usr/bin/env python3
import matplotlib.pyplot as plt
from UAVHeading import UAVHeading

def main():
                    # (pos, waypt, speed, heading, tPossible)
    uav0 = UAVHeading((7, 0),
                      (20, 0),
                      50.0,
                      0.0,
                      30.0)

    uav1 = UAVHeading((10, -3),
                      (1.5267, -132.531),
                      50.0,
                      90.0,
                      30.0)
    
    uav2 = UAVHeading((13, 0),
                      (1.5267, -132.531),
                      50.0,
                      180.0,
                      30.0)

    koz0 = [(15,0), (19,2), (20,1), (16,-1), (15,0)]

    uav_others = [uav1, uav2]
    static_kozs = [koz0]

    # plot uav0 position and goal
    plt.plot(uav0.position[0], uav0.position[1], 'oc', label='UAV0 Position')
    plt.plot(uav0.waypoint[0], uav0.waypoint[1], 'xc', label='UAV0 Goal')

    # plot uav and koz positions
    for uav in uav_others:
        plt.plot(uav.position[0], uav.position[1], 'or', label='Other UAV')
    for koz in static_kozs:
        x = [pt[0] for pt in koz]
        y = [pt[1] for pt in koz]
        plt.plot(x, y, '--r', label='Keep Out Zone')
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()

    uav0.avoid(uav_others, static_kozs)

if __name__ == '__main__':
    main()