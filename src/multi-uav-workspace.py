#!/usr/bin/env python3
import matplotlib.pyplot as plt
from UAVHeading import UAVHeading

def main():
                    # (pos, waypt, speed, heading, tPossible)
    uav0 = UAVHeading((0, 0),
                      (6, 0),
                      50.0,
                      0.0,
                      30.0)

    uav1 = UAVHeading((2, 0),
                      (1.5267, -132.531),
                      50.0,
                      180.0,
                      30.0)

    uav2 = UAVHeading((3, -1),
                      (1.5156, -132.403),
                      50.0,
                      135.0,
                      30.0)

    uav3 = UAVHeading((4, 1),
                      (1.5156, -132.403),
                      50.0,
                      270.0,
                      30.0)

    uav_others = [uav1, uav2, uav3]

    # _, kozs = uav0.findIntersects(uav_others)
    # for k in kozs:
    #     print(k, '\n')
    #     koz = k[0]
    #     x = [pt[0] for pt in koz]
    #     y = [pt[1] for pt in koz]

    #     plt.plot(x, y, '--r')

    # plt.plot([uav0.position[0], uav0.waypoint[0]], [uav0.position[1], uav0.waypoint[1]], 'k')
    # plt.axis('equal')
    # plt.show()

    uav0.avoid(uav_others)

if __name__ == '__main__':
    main()