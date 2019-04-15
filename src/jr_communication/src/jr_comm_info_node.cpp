//
// Created by FionaLee on 3/23/19.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Info_package/TOF_info.msg>
#include <Info_package/Motor_info.msg>

ros::Publisher chassis_info_pub;
ros::Publisher tof_info_pub;
ros::Publisher motor_pid_info_pub;
#define MSG_QUEUE_SIZE (1)

typedef enum
{
INFO_TOF = 0,
INFO_MOTOR  = 1,
} info_type;

typedef __packed struct
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
float spd_p;
float spd_i;
float spd_d;
float pos_p;
float pos_i;
float pos_d;
uint8_t sw_l;           /**< left limit switch */
uint8_t sw_r;           /**< right limit switch */
} chassis_info_t;

void data_handle(uint8_t* protocol_packet) {
  frame_header_t *p_header = (frame_header_t *)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;

  chassis_info_t *chassis_info = (chassis_info_t)data_addr;

  if (cmd_id == CHASSIS_DATA_ID) {
    Info_package::TOF_info tof_msg;
    tof_msg.dist0 = chassis_info->tof_0;
    tof_msg.dist1 = chassis_info->tof_1;
    tof_msg.dist2 = chassis_info->tof_2;
    tof_msg.dist3 = chassis_info->tof_3;
    tof_msg.dist4 = chassis_info->tof_4;
    tof_msg.dist5 = chassis_info->tof_5;
    tof_info_pub.publish(tof_msg);

    Info_package::Chassis_info chassis_msg;
    chassis_msg.x_spd = chassis_info->x_spd;
    chassis_msg.y_spd = chassis_info->y_spd;
    chassis_msg.x_position = chassis_info->x_position;
    chassis_msg.y_position = chassis_info->y_position;
    chassis_info_pub.publish(chassis_msg);

    Info_package::Motor_PID_info motor_pid_msg;
    motor_pid_msg.spd_p = chassis_info->spd_p;
    motor_pid_msg.spd_i = chassis_info->spd_i;
    motor_pid_msg.spd_d = chassis_info->spd_d;
    motor_pid_msg.pos_p = chassis_info->pos_p;
    motor_pid_msg.pos_i = chassis_info->pos_i;
    motor_pid_msg.pos_d = chassis_info->pos_d;
    tof_info_pub.publish(motor_pid_msg);
  }
}

void *jrcom_recv_thread(void *argu) {
  uint8_t byte = 0;
  int32_t read_len;
  int32_t buff_read_index;
  unpack_step_e unpack_step;


  uint16_t      data_len;
  int32_t       index;
  uint8_t       protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  while (1)
  {
    read_len = read(tty_fd, computer_rx_buf, UART_BUFF_SIZE);
    buff_read_index = 0;

    while (read_len--) {
      byte = computer_rx_buf[buff_read_index++];
      switch (unpack_step) {
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
          break;
        }
        case STEP_LENGTH_LOW:
        {
          data_len = byte;
          protocol_packet[index++] = byte;
          unpack_step = STEP_LENGTH_HIGH;
          break;
        }
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
          break;
        }
        case STEP_FRAME_SEQ:
        {
          protocol_packet[index++] = byte;
          unpack_step = STEP_HEADER_CRC8;
          break;
        }
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
          break;
        }
        default:
        {
          unpack_step = STEP_HEADER_SOF;
          index = 0;
          break;
        }
      }
    }
  }
}

void process_msg() {
  jrcom_recv_thread();
}


void print_usage(void) {
  printf("usage: rosrun jr_comm_info_node\n");
}

int main(int argc, char **argv) {
  if (argc < 1) {
    print_usage();
    return -1;
  }

  // More initialization here
  jrcomm_init("tty_name");
  ros::init(argc, argv, "jr_comm_info_node");

  ros::NodeHandle node;

  chassis_info_pub = node.advertise<Info_package::Chassis_info>(
          "jr_chassis", MSG_QUEUE_SIZE);
  tof_info_pub = node.advertise<Info_package::TOF_info>(
          "jr_tof", MSG_QUEUE_SIZE);
  motor_pid_info_pub = node.advertise<Info_package::Motor_PID_info>(
          "jr_motor_pid", MSG_QUEUE_SIZE);


  // Running at 10Hz
  ros::Rate loop_rate(10);

  while(ros::ok()) {
    process_msg();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}