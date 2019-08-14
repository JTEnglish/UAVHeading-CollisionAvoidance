#!/usr/bin/env python3
import matplotlib.pyplot as plt
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

    uav2 = UAVHeading((1.5156, -132.5321),
                      (1.5156, -132.403),
                      50.0,
                      270.0,
                      30.0)

    uav_others = [uav1, uav2]

    _, kozs = uav0.findIntersects(uav_others)
    for k in kozs:
        koz = k[0]
        x = [pt[0] for pt in koz]
        y = [pt[1] for pt in koz]

        plt.plot(x, y, '--r')

    plt.plot([uav0.position[0], uav0.waypoint[0]], [uav0.position[1], uav0.waypoint[1]], 'k')
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    main()