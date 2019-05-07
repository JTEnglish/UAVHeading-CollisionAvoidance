# UAVHeading-CollisionAvoidance
A*-based collision avoidance for UAV path planning

## Required Modules
* [Shapely](https://pypi.org/project/Shapely/)
* [Matplotlib](https://matplotlib.org/)

## Avoidance Scenarios
#### Head-On Collision
![Head-On Collision Path Planning](https://github.com/JTEnglish/UAVHeading-CollisionAvoidance/raw/master/images/searchArea1.png "Head-On Collision Path Planning")
#### Non-Head-On Collision
![Non-Head-On Collision Path Planning](https://github.com/JTEnglish/UAVHeading-CollisionAvoidance/raw/master/images/searchArea0.png "Non-Head-On Collision Path Planning")

## Simulation Results
#### 50 m/s Head-On Collision
![50 m/s Head-On](https://github.com/JTEnglish/UAVHeading-CollisionAvoidance/raw/master/images/avoid-head-on.png "Head-On Collision")
#### 50 m/s Head-On Collision (Targeted)
![50 m/s Targeted Head-On](https://github.com/JTEnglish/UAVHeading-CollisionAvoidance/raw/master/images/avoid-50mps-head-on.png "Targeted Head-On Collision")
#### 50 m/s Perpendicular Collision Scenario
![50 m/s Perpendicular](https://github.com/JTEnglish/UAVHeading-CollisionAvoidance/raw/master/images/avoid-90.png "Perpendicular Collision")


## Testing
#### UAVHcfg.py
Configuration file for UAVHeading Class
```python
INTERVAL_SIZE = 0.0012          # defines resolution for A* search
DISTANCE_THRESHOLD = 0.01       # upper bound distance for running UAVHeading.avoid(UAVHeading) function

DECISION_WEIGHTS = [            # weights for sidedWeightDecision in head-on avoidance scenario
    1.0,                            # UAV-0
    6.0,                            # Goal Position for UAV-0
    0.75,                           # Other UAVs
    0.25                            # KeepOut Zoness
]

SHOW_ANIMATION = False          # graph path planning search animation in avoidance function
```
#### example.py
Import UAVHeading Class
```python
from UAVHeading import UAVHeading
```
Initialize UAV Objects
```python
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
```
Run Avoidance Algorithm
```python
waypoints, _ = uav0.avoid(uav1)
```
