/**
 * @file jrcomms.h
 * @author Haowen Shi
 * @date 16 Apr 2019
 * @brief A communication library for microcontroller.
 * 
 */

#ifndef __JRCOMMS_H__
#define __JRCOMMS_H__

#include <sys/types.h>

#include "jr_comm.h"

/**
 * @brief Communication status.
 */
typedef enum {
  COM_OK = 0,   /**< Success */
  COM_FAIL,     /**< Failure */
  COM_TIMEOUT,  /**< Timeout */
} JRCOM_Status;

int jrcomm_init(char *tty_path);

/**
 * @brief Sends chassis velocity command to microcontroller.
 */
void jrcomm_send_chassis_command(int16_t vx, int16_t vy, float w_spd);

/**
 * @brief Sends chassis encoder command to microcontroller.
 */
void jrcomm_send_chassis_encoder(move_cmd_e cmd_type, int16_t cmd_val);

/**
 * @brief Sends chassis configuration such as PID and wheel/track distances.
 */
void jrcomm_send_chassis_config(infantry_structure_t config);

void jrcomm_send_chassis_reset(void);

void *jrcom_recv_thread(void *argu);

#endif /* __JRCOMMS_H__ */
