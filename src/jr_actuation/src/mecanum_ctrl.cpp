/**
 * @file mecanum_ctrl.cpp
 * @author Haowen Shi
 * @date 16 Mar 2019
 * @brief Mecanum wheel control
 */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "mecanum_ctrl.hpp"
/**
  * @brief mecanum chassis velocity decomposition
  * @param input : ↑=+vx(mm/s)  ←=+vy(mm/s)  ccw=+vw(deg/s)
  *        output: every wheel speed(rpm)
  * @note  1=FR 2=FL 3=BL 4=BR
  */
void mecanum_calc(float vx, float vy, float vw, int16_t speed[])
{
    static float rotate_ratio_fr = 1.f;
    static float rotate_ratio_fl = 1.f;
    static float rotate_ratio_bl = 1.f;
    static float rotate_ratio_br = 1.f;
    static float wheel_rpm_ratio;
    
    float rotate_x_offset = 0;
    float rotate_y_offset = 0;
    
    rotate_ratio_fr = ((WHEEL_BASE + WHEEL_TRACK) / 2.0f \
                        - rotate_x_offset + rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_fl = ((WHEEL_BASE + WHEEL_TRACK) / 2.0f \
                        - rotate_x_offset - rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_bl = ((WHEEL_BASE + WHEEL_TRACK) / 2.0f \
                        + rotate_x_offset - rotate_y_offset) / RADIAN_COEF;
    rotate_ratio_br = ((WHEEL_BASE + WHEEL_TRACK) / 2.0f \
                        + rotate_x_offset + rotate_y_offset) / RADIAN_COEF;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * CHASSIS_DECELE_RATIO);
    
    VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s
    
    int16_t wheel_rpm[4];
    float   max = 0;
    
    wheel_rpm[MOTOR_FR] = (-vx - vy - vw * rotate_ratio_fr) * wheel_rpm_ratio;
    wheel_rpm[MOTOR_FL] = ( vx - vy - vw * rotate_ratio_fl) * wheel_rpm_ratio;
    wheel_rpm[MOTOR_BL] = ( vx + vy - vw * rotate_ratio_bl) * wheel_rpm_ratio;
    wheel_rpm[MOTOR_BR] = (-vx + vy - vw * rotate_ratio_br) * wheel_rpm_ratio;

    //find max item
    for (int i = 0; i < 4; i++)
    {
        if (abs(wheel_rpm[i]) > max)
        max = abs(wheel_rpm[i]);
    }
    //equal proportion
    if (max > MAX_WHEEL_RPM)
    {
        float rate = MAX_WHEEL_RPM / max;
        for (int i = 0; i < 4; i++)
        wheel_rpm[i] *= rate;
    }
    memcpy(speed, wheel_rpm, 4 * sizeof(int16_t));
}