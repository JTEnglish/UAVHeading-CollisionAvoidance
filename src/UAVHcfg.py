INTERVAL_SIZE = 0.0012           # defines resolution for A* search
DISTANCE_THRESHOLD = 0.015       # upper bound distance for running UAVHeading.avoid(UAVHeading) function

DECISION_WEIGHTS = [            # weights for sidedWeightDecision in head-on avoidance scenario
    1.0,                            # UAV-0
    6.0,                            # Goal Position for UAV-0
    0.75,                           # Other UAVs
    0.25                            # KeepOut Zones
]