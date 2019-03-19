/**
 * @file mecanum_ctrl.hpp
 * @author Haowen Shi
 * @date 16 Mar 2019
 * @brief Mecanum wheel control constants and helpers
 */

#ifndef __MEC_CTRL_HPP__
#define __MEC_CTRL_HPP__

/* math relevant */
/* radian coefficient */
#define RADIAN_COEF (57.3f)
/* circumference ratio */
#define PI (3.14159f)

/** @brief Diameter of mecanum wheels (mm) */
#define WHEEL_DIAMETER (100)
/** @brief Perimeter of mecanum wheels (mm) */
#define WHEEL_PERIMETER (PI * WHEEL_DIAMETER)
/** @brief Wheel track distance (mm) */
#define WHEEL_TRACK (500)
/** @brief Wheel base distance (mm) */
#define WHEEL_BASE (550)

/** @brief The deceleration ratio of chassis motor */
#define CHASSIS_DECELE_RATIO (1.0f)
/* single 3508 motor maximum speed, unit is rpm */
#define MAX_WHEEL_RPM (40)   //44rpm = 350mm/s
/* chassis maximum translation speed, unit is mm/s */
#define MAX_CHASSIS_VX_SPEED (330)  //41.5rpm
#define MAX_CHASSIS_VY_SPEED (330)
/* chassis maximum rotation speed, unit is degree/s */
#define MAX_CHASSIS_VR_SPEED (50)

#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

/* arrangement of motors */
#define MOTOR_FR (0)
#define MOTOR_FL (1)
#define MOTOR_BL (2)
#define MOTOR_BR (3)

/** @brief Mecanum calculation interface */
void mecanum_calc(
    float vx,
    float vy,
    float vw,
    int16_t speed[]
);

#endif /* __MEC_CTRL_HPP__ */