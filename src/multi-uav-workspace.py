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

    koz0 = [(15,0), (19,2), (20,1), (16,-1)]

    uav_others = [uav1, uav2]
    static_kozs = [koz0]

    uav0.avoid(uav_others, static_kozs)

if __name__ == '__main__':
    main()