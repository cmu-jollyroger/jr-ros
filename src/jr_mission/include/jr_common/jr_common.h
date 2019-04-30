/**
 * @file jr_common.h
 * @author Haowen Shi
 * @date 23 Apr 2019
 * @brief Mission related common declarations
 */

#ifndef __JR_COMMON_H__
#define __JR_COMMON_H__

#define NUM_STATIONS (8)
#define DEFAULT_RAIL (RAIL_LONG)
#define GET_STATION_RAIL(S) (S <= STATION_E ? RAIL_LONG : RAIL_SHORT)

enum GuideRail {
  RAIL_LONG = 0,
  RAIL_SHORT = 1,
};

enum StationID {
    STATION_A = 0,
    STATION_B = 1,
    STATION_C = 2,
    STATION_D = 3,
    STATION_E = 4,
    STATION_F = 5,
    STATION_G = 6,
    STATION_H = 7
};

enum DeviceType {
    V1 = 0, /**< Gate Valve */
    V2 = 1, /**< Large Valve */
    V3 = 2, /**< Shuttle Valve */
    B = 3,  /**< Breaker */
};

enum DeviceOrientation {
    VERTICAL = 0,    /**< Rotation plane parallel to station */
    HORIZONTAL = 1,  /**< Rotation plane parallel to wall */
};

#endif /* __JR_COMMON_H__ */