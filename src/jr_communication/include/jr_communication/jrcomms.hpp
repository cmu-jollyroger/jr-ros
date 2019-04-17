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

void jrcomm_send_chassis_command(int16_t vx, int16_t vy, float w_spd);

void *jrcom_recv_thread(void *argu);

#endif /* __JRCOMMS_H__ */