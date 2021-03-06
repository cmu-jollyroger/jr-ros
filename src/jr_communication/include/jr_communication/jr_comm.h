/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of�
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.� See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file jr_comm.h
 *  @version 1.0
 *  @date Apr 2019
 *
 *  @brief the information from computer
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *  @copyright Haowen Shi, Carnegie Mellon University.
 *
 */

#ifndef __JR_COMM_H__
#define __JR_COMM_H__

#include <stddef.h>
#include <stdint.h>

/** @brief  Length of PC FIFO buffer */
#define COMPUTER_FIFO_BUFLEN (1000)


/**
  * @brief  JollyRoger shipbot comms packet opcode
  */
typedef enum
{
  /* Controller -> PC */
  CHASSIS_DATA_ID     = 0x0010, /**< 50Hz fixed, chassis info */
  INFANTRY_ERR_ID     = 0x0013, /**< 50Hz fixed, error info */

  /* PC -> Controller */
  CHASSIS_CTRL_ID     = 0x00A0, /**< 50Hz fixed, chassis ctrl */
  ERROR_LEVEL_ID      = 0x00A3, /**< upon error, PC error */
  INFANTRY_STRUCT_ID  = 0x00A4, /**< before start, robot config */
} infantry_data_id_e;

typedef enum
{
  BOTTOM_DEVICE        = 0,
  CHASSIS_M1_OFFLINE   = 1,
  CHASSIS_M2_OFFLINE   = 2,
  CHASSIS_M3_OFFLINE   = 3,
  CHASSIS_M4_OFFLINE   = 4,
  TOF_0_OFFLINE        = 5,
  TOF_1_OFFLINE        = 6,
  TOF_2_OFFLINE        = 7,
  TOF_3_OFFLINE        = 8,
  TOF_4_OFFLINE        = 9,
  TOF_5_OFFLINE        = 10,
  CHASSIS_CONFIG_ERR   = 11,
  ERROR_LIST_LENGTH    = 12,
} err_id_e;

/** @brief  First offline error ID */
#define ERR_OFFLINE_FIRST CHASSIS_M1_OFFLINE

/** @brief  Last offline error ID */
#define ERR_OFFLINE_LAST TOF_5_OFFLINE

typedef enum
{
  DEVICE_NORMAL = 0,
  ERROR_EXIST   = 1,
  UNKNOWN_STATE = 2,
} bottom_err_e;

typedef enum
{
  GLOBAL_NORMAL        = 0,
  SOFTWARE_WARNING     = 1,
  SOFTWARE_ERROR       = 2,
  SOFTWARE_FATAL_ERROR = 3,
  CHASSIS_ERROR        = 5,
  HARAWARE_ERROR       = 6,
} err_level_e;

typedef enum
{
  NO_CONFIG      = 0,
  DEFAULT_CONFIG = 1,
  CUSTOM_CONFIG  = 3,
} struct_config_e;

typedef enum
{
  SPD_CTRL       = 0,
  ENC_CTRL_X     = 1,
  ENC_CTRL_Y     = 2,
  ENC_CTRL_W     = 3
} move_cmd_e;

/********** the information send to computer ***********/

/**
  * @brief  chassis information [0x10]
  */
typedef struct __attribute__((__packed__))
{
  uint8_t ctrl_mode;      /**< chassis control mode */
  float   gyro_palstance; /**< chassis palstance(degree/s) from gyroscope */
  float   gyro_angle;     /**< chassis angle(degree) relative to ground from gyroscope */
  float   ecd_palstance;  /**< chassis palstance(degree/s) from chassis motor encoder calculated */
  float   ecd_calc_angle; /**< chassis angle(degree) relative to ground from chassis motor encoder calculated */
  int16_t x_spd;          /**< chassis x-axis move speed(mm/s) from chassis motor encoder calculated */
  int16_t y_spd;          /**< chassis y-axis move speed(mm/s) from chassis motor encoder calculated */
  int32_t x_position;     /**< chassis x-axis position(mm) relative to the starting point */
  int32_t y_position;     /**< chassis y-axis position(mm) relative to the starting point */
  int16_t tof_0;
  int16_t tof_1;
  int16_t tof_2;
  int16_t tof_3;
  int16_t tof_4;
  int16_t tof_5;
  uint8_t sw_l;           /**< left limit switch */
  uint8_t sw_r;           /**< right limit switch */
  uint8_t enc_exec_done;  /**< encoder execution done, 0 if still in progress */
} chassis_info_t;

/**
  * @brief  robot error information [0x13]
  */
typedef struct __attribute__((__packed__))
{
  bottom_err_e err_sta;                 /* bottom error state */
  bottom_err_e err[ERROR_LIST_LENGTH];  /* device error list */
} infantry_err_t;

/**
  * @brief  robot structure config response [0x14]
  */
typedef struct __attribute__((__packed__))
{
  struct_config_e chassis_config;
} config_response_t;

/********** information from host PC **********/

typedef struct __attribute__((__packed__))
{
  int16_t x_offset;   /* offset(mm) relative to the x-axis of the chassis center */
  int16_t y_offset;   /* offset(mm) relative to the y-axis of the chassis center */
  float   w_spd;      /* rotation speed(deg/s) of chassis, or angle(deg) if enc */
} chassis_rotate_t;

/**
  * @brief  chassis control information [0xA0]
  */
typedef struct __attribute__((__packed__))
{
  uint8_t          ctrl_mode; /* chassis control mode */
  uint8_t          move_cmd;  /* command type */
  int16_t          x_spd;     /* x-axis speed(mm/s), or displacement(mm) if enc */
  int16_t          y_spd;     /* y-axis speed(mm/s), or displacement(mm) if enc */
  chassis_rotate_t w_info;    /* rotation control of chassis */
} chassis_ctrl_t;

/**
  * @brief  robot system error level [0xA3]
  */
typedef struct __attribute__((__packed__))
{
  err_level_e err_level; /* the error level is included in err_level_e enumeration */
} global_err_level_t;

/**
  * @brief  infantry structure configuration information [0xA4]
  */
typedef struct __attribute__((__packed__))
{
  uint16_t         chassis_config;  /* chassis structure config state */
  uint16_t         wheel_perimeter; /* the perimeter(mm) of wheel */
  uint16_t         wheel_track;     /* wheel track distance(mm) */
  uint16_t         wheel_base;      /* wheelbase distance(mm) */
  float            pid_vel_p;
  float            pid_vel_i;
  float            pid_vel_d;
  float            pid_pos_p;
  float            pid_pos_i;
  float            pid_pos_d;
} infantry_structure_t;

/********* variables **********/
/**
  * @brief  the data structure send to pc
  */
typedef struct
{
  /* data send */
  chassis_info_t    chassis_information;
  infantry_err_t    bottom_error_data;
  config_response_t structure_config_data;
} send_pc_t;
/**
  * @brief  the data structure receive from pc
  */
typedef struct
{
  /* data receive */
  chassis_ctrl_t       chassis_control_data;
  global_err_level_t   global_error_level;
  infantry_structure_t structure_data;
} receive_pc_t;

/* data send */
extern send_pc_t    pc_send_mesg;
/* data receive */
extern receive_pc_t pc_recv_mesg;

void pc_data_handler(uint8_t *p_frame);

#endif /* __JR_COMM_H__ */
