#!/usr/bin/env python

# Global declarations, see jr_common.h
V1 = 0
V2 = 1
V3 = 2
B = 3

STATION_A = 0
STATION_B = 1
STATION_C = 2
STATION_D = 3
STATION_E = 4
STATION_F = 5
STATION_G = 6
STATION_H = 7

VERTICAL = 0
HORIZONTAL = 1

def pose_lookup(station, dev_type, dev_ori):
    if (station == STATION_F):
        print "pose special case corner"
    else:
        if (dev_type == V1):
            if (dev_ori == VERTICAL):
                # Vertical gate
                return 0.45, 0.03, 0.45
            else:
                # Horizontal gate
                return 0.35, 0.0, 0.4
        elif (dev_type == V2):
            # Large valve
            return 0.35, 0.0, 0.4
        elif (dev_type == V3):
            if (dev_ori == VERTICAL):
                # Vertical shuttle
                return 0.45, 0.03, 0.45
            else:
                # Horizontal shuttle
                return 0.35, 0.0, 0.36
        elif (dev_type == B):
            # Breaker
            return 0.35, 0.0, 0.4
        else:
            print "[warning]: wrong dev_type value in pose_lookup", dev_type
        