/**
 * @file jrcomms.cpp
 * @author Haowen Shi, Fiona Li
 * @date 19 March 2019
 * @brief A communication library for microcontroller.
 */

#include "jr_comm.h"
#include "protocol.h"
#include "communicate.h"
#include <assert.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <stdlib.h>
#include <time.h>
#include <jr_communication/ChassisInfo.h>

#define UART_BUFF_SIZE (500)
#define COMPUTER_FRAME_BUFLEN UART_BUFF_SIZE

#define TOF_CALIB_RATIO (0.91)

/* Chassis stats */
send_pc_t mky_chassis_stats;

static int tty_fd = 0;

/* UART receive buffer */
static uint8_t computer_rx_buf[UART_BUFF_SIZE];

/* UART transmit buffer */
static uint8_t computer_tx_buf[COMPUTER_FRAME_BUFLEN];

/* External global values */
extern jr_communication::ChassisInfo chassis_info_msg;

/* For debug */
int pc_seq            = 0;
int once_lost_num     = 0;
int lost_pack_percent = 0;

int pack_num_cnt   = 0;
int lost_num_sum_t = 0;
int pack_lost      = 0;

/* ttyUSB serial device helpers */
static int set_interface_attribs (int fd, int speed, int parity);
static void set_blocking(int fd, int should_block);

static ssize_t _com_receive(char *buf, size_t len);
static ssize_t _com_transmit(char *buf, size_t len);

static void unix_error(const char *msg) {
  printf("%s: %s\n", strerror(errno), msg);
}

ssize_t _com_receive(char *buf, size_t len) {
  if (tty_fd == 0) return -1;
  ssize_t recv_len = read(tty_fd, buf, len);
  return recv_len;
}

ssize_t _com_transmit(char *buf, size_t len) {
  if (tty_fd == 0) return -1;
  ssize_t tx_len = write(tty_fd, buf, len);
  return tx_len;
}

int jrcomm_init(char *tty_path) {

  tty_fd = open (tty_path, O_RDWR | O_NOCTTY | O_SYNC);
  if (tty_fd < 0) {
    unix_error("error opening usbtty, check supplied tty_path");
    return tty_fd;
  }

  set_interface_attribs (tty_fd, B115200, 0);  // set speed to 115200 bps, 8n1 (no parity)
  set_blocking (tty_fd, 0);                // set no blocking

  fprintf(stderr, "jrcomm_init successful with fd=%d\n", tty_fd);

  return 0;
}

ssize_t jrcomm_receive(char *buf, size_t len) {
  return _com_receive(buf, len);
}

ssize_t jrcomm_transmit(char *buf, size_t len) {
  return _com_transmit(buf, len);
}

static int set_interface_attribs (int fd, int speed, int parity) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    unix_error("error from tcgetattr");
    return errno;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    unix_error("error from tcsetattr");
    return -1;
  }
  return 0;
}

static void set_blocking(int fd, int should_block) {
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(fd, &tty) != 0) {
    unix_error("error from tggetattr");
    return;
  }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    unix_error("error setting term attributes");
  }
}

/**
  * @brief     pack data to bottom device
  * @param[in] cmd_id:  command id of data
  * @param[in] *p_data: pointer to the data to be sent
  * @param[in] len:     the data length
  * @usage     data_pack_handle(CHASSIS_CTRL_ID, &chassis_control_data, sizeof(chassis_ctrl_t))
  */

int data_pack_handle(uint16_t cmd_id, uint8_t *p_data, uint16_t len)
{
  memset(computer_tx_buf, 0, COMPUTER_FRAME_BUFLEN);
  frame_header_t *p_header = (frame_header_t*)computer_tx_buf;

  p_header->sof          = UP_REG_ID;
  p_header->data_length  = len;

  memcpy(&computer_tx_buf[HEADER_LEN], (uint8_t*)&cmd_id, CMD_LEN);
  append_crc8_check_sum(computer_tx_buf, HEADER_LEN);

  memcpy(&computer_tx_buf[HEADER_LEN + CMD_LEN], p_data, len);
  append_crc16_check_sum(computer_tx_buf, HEADER_LEN + CMD_LEN + len + CRC_LEN);

  return sizeof(frame_header_t) + CMD_LEN + len + CRC_LEN;
}

/**
 * Update global information data when packet arrives
 */
static void data_handle(uint8_t *p_frame) {
  frame_header_t *p_header = (frame_header_t *)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;

  chassis_info_t *chassis_info = (chassis_info_t *)data_addr;

  // printf("data_handle called\n");

  if (cmd_id == CHASSIS_DATA_ID) {
    chassis_info_msg.x_spd = chassis_info->x_spd;
    chassis_info_msg.y_spd = chassis_info->y_spd;
    chassis_info_msg.x_position = chassis_info->x_position;
    chassis_info_msg.y_position = chassis_info->y_position;
    chassis_info_msg.dist0 = chassis_info->tof_0 * TOF_CALIB_RATIO;
    chassis_info_msg.dist1 = chassis_info->tof_1 * TOF_CALIB_RATIO;
    chassis_info_msg.dist2 = chassis_info->tof_2 * TOF_CALIB_RATIO;
    chassis_info_msg.dist3 = chassis_info->tof_3 * TOF_CALIB_RATIO;
    chassis_info_msg.dist4 = chassis_info->tof_4 * TOF_CALIB_RATIO;
    chassis_info_msg.dist5 = chassis_info->tof_5 * TOF_CALIB_RATIO;
    chassis_info_msg.sw_l = chassis_info->sw_l;
    chassis_info_msg.sw_r = chassis_info->sw_r;
    chassis_info_msg.enc_exec_done = chassis_info->enc_exec_done;
  }
}

void *jrcom_recv_thread(void *argu) {
  uint8_t byte = 0;
  int32_t read_len;
  int32_t buff_read_index;

  uint16_t      data_len;
  unpack_step_e unpack_step;
  int32_t       index;
  uint8_t       protocol_packet[PROTOCAL_FRAME_MAX_SIZE];

  fprintf(stderr, "jrcom_recv_thread\n");

  while (1)
  {
    read_len = read(tty_fd, computer_rx_buf, UART_BUFF_SIZE);
    // fprintf(stderr, "\rRead %d bytes", read_len);
    fflush(stderr);
    buff_read_index = 0;

    while (read_len--)
    {
      byte = computer_rx_buf[buff_read_index++];

      // if (unpack_step != STEP_HEADER_SOF) {
      //   fprintf(stderr, "%x ", byte);
      //   fprintf(stderr, "unpack_step = %d\n", unpack_step);
      // }

      switch(unpack_step)
      {
        case STEP_HEADER_SOF:
        {
          if(byte == UP_REG_ID)
          {
            unpack_step = STEP_LENGTH_LOW;
            protocol_packet[index++] = byte;
          }
          else
          {
            index = 0;
          }
        }break;

        case STEP_LENGTH_LOW:
        {
          data_len = byte;
          protocol_packet[index++] = byte;
          unpack_step = STEP_LENGTH_HIGH;
        }break;

        case STEP_LENGTH_HIGH:
        {
          data_len |= (byte << 8);
          protocol_packet[index++] = byte;

          if(data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
          {
            unpack_step = STEP_FRAME_SEQ;
          }
          else
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;
          }
        }break;

        case STEP_FRAME_SEQ:
        {
          protocol_packet[index++] = byte;
          unpack_step = STEP_HEADER_CRC8;
        }break;

        case STEP_HEADER_CRC8:
        {
          protocol_packet[index++] = byte;

          if (index == HEADER_LEN)
          {
            if ( verify_crc8_check_sum(protocol_packet, uint16_t(HEADER_LEN)) )
            {
              unpack_step = STEP_DATA_CRC16;
            }
            else
            {
              unpack_step = STEP_HEADER_SOF;
              index = 0;
            }
          }
        }break;

        case STEP_DATA_CRC16:
        {
          if (index < (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
            protocol_packet[index++] = byte;
          }
          if (index >= (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;

            if ( verify_crc16_check_sum(protocol_packet, uint32_t(HEADER_LEN + CMD_LEN + data_len + CRC_LEN)) )
            {
              // Upon successfully decoding a packet, handle it.
              data_handle(protocol_packet);
            }
          }
        }break;

        default:
        {
          unpack_step = STEP_HEADER_SOF;
          index = 0;
        }break;
      }
    }
  }

  return NULL;
}

/**@brief  Chassis modes */
typedef enum
{
  CHASSIS_RELAX          = 0, /**< Chassis no power */
  CHASSIS_STOP           = 1, /**< Chassis is stopped/breaking */
  CHASSIS_MOVING         = 4, /**< Chassis is moving */
} chassis_mode_e;

void jrcomm_send_chassis_command(int16_t vx, int16_t vy, float w_spd) {
  chassis_ctrl_t control;
  control.ctrl_mode = CHASSIS_MOVING;
  control.move_cmd = SPD_CTRL;
  control.x_spd = vx;
  control.y_spd = vy;
  control.w_info.x_offset = 0;
  control.w_info.y_offset = 0;
  control.w_info.w_spd = w_spd;

  int packed_size = data_pack_handle(
          CHASSIS_CTRL_ID, (uint8_t *)&control, sizeof(control));
  jrcomm_transmit((char *)computer_tx_buf, packed_size);
}

void jrcomm_send_chassis_encoder(move_cmd_e cmd_type, int16_t cmd_val) {
  chassis_ctrl_t control;
  control.ctrl_mode = CHASSIS_MOVING;
  control.move_cmd = cmd_type;
  switch (cmd_type) {
    case ENC_CTRL_X: {
      control.x_spd = cmd_val;
      break;
    }
    case ENC_CTRL_Y: {
      control.y_spd = cmd_val;
      break;
    }
    case ENC_CTRL_W: {
      control.w_info.w_spd = (float) cmd_val;
      break;
    }
  }

  int packed_size = data_pack_handle(
          CHASSIS_CTRL_ID, (uint8_t *)&control, sizeof(control));
  jrcomm_transmit((char *)computer_tx_buf, packed_size);
}

void jrcomm_send_chassis_config(infantry_structure_t config) {
  int packed_size = data_pack_handle(
    INFANTRY_STRUCT_ID, (uint8_t *)&config, sizeof(config));
  jrcomm_transmit((char *)computer_tx_buf, packed_size);
}

void jrcomm_send_chassis_reset(void) {
  global_err_level_t level;
  int packed_size = data_pack_handle(
    ERROR_LEVEL_ID, (uint8_t *)&level, sizeof(level));
  jrcomm_transmit((char *)computer_tx_buf, packed_size);
}
