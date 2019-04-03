//
// Created by FionaLee on 3/23/19.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <Info_package/TOF_info.msg>
#include <Info_package/Motor_info.msg>

ros::Publisher motor_info_pub;
ros::Publisher tof_info_pub;
#define MSG_QUEUE_SIZE (1)

typedef enum
{
INFO_TOF = 0,
INFO_MOTOR  = 1,
} info_type;

void data_handle(uint8_t* protocol_packet) {
  frame_header_t *p_header = (frame_header_t *)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
  ros::Publisher pub;
  if (p_header->sof == INFO_TOF) {

  } else if (p_header->sof == INFO_MOTOR) {
    Info_package::Motor_info msg;
    msg.spd0 = data_addr[0];
    msg.spd1 = data_addr[1];
    msg.spd2 = data_addr[2];
    msg.spd3 = data_addr[3];
    motor_info_pub.publish(msg);
    return;

  } else {
    Info_package::TOF_info msg;
    msg.dist0 = data_addr[0];
    msg.dist1 = data_addr[1];
    msg.dist2 = data_addr[2];
    msg.dist3 = data_addr[3];
    msg.dist4 = data_addr[4];
    msg.dist5 = data_addr[5];
    tof_info_pub.publish(msg);
    return;
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
          if((byte == INFO_TOF)|| (byte == INFO_MOTOR))
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

  motor_info_pub = node.advertise<Info_package::Motor_info>(
          "jr_motor", MSG_QUEUE_SIZE);
  tof_info_pub = node.advertise<Info_package::TOF_info>(
          "jr_tof", MSG_QUEUE_SIZE);


  // Running at 10Hz
  ros::Rate loop_rate(10);

  while(ros::ok()) {
    process_msg();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}