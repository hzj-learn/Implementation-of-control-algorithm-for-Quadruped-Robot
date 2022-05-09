/*!
 * @file rt_spi.h
 * @brief SPI通讯到MCU通讯板
 */
//#ifdef linux

#include <byteswap.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#include <linux/spi/spidev.h>
#include "rt/rt_spi.h"
#include <lcm/lcm-cpp.hpp>

unsigned char spi_mode = SPI_MODE_0;
unsigned char spi_bits_per_word = 8;
unsigned int spi_speed = 6000000;
uint8_t lsb = 0x01;

int spi_1_fd = -1;
int spi_2_fd = -1;
int spi_open();
static spine_cmd_t g_spine_cmd;
static spine_data_t g_spine_data;

spi_command_t spi_command_drv;
spi_data_t spi_data_drv;
spi_torque_t spi_torque;
pthread_mutex_t spi_mutex;

const float max_torque[3] = {17.f, 17.f, 26.f};  // TODO CHECK WITH BEN
const float wimp_torque[3] = {6.f, 6.f, 6.f};    // TODO CHECK WITH BEN
const float disabled_torque[3] = {0.f, 0.f, 0.f};

//仅用于实际机器人 
const float abad_side_sign[4] = {-1.f, -1.f, 1.f, 1.f};
const float hip_side_sign[4] = {-1.f, 1.f, -1.f, 1.f};
const float knee_side_sign[4] = {-.6429f, .6429f, -.6429f, .6429f};

//仅用于实际机器人
const float abad_offset[4] = {0.f, 0.f, 0.f, 0.f};
const float hip_offset[4] = {M_PI / 2.f, -M_PI / 2.f, -M_PI / 2.f, M_PI / 2.f};
const float knee_offset[4] = {K_KNEE_OFFSET_POS, -K_KNEE_OFFSET_POS,
                              -K_KNEE_OFFSET_POS, K_KNEE_OFFSET_POS};



/*!
 * 功能：计算SPI消息校验和
 * @param data : input
 * @param len : length (in 32-bit words)
 * @return
 */
uint32_t xor_checksum(uint32_t *data, size_t len) 
{
  uint32_t t = 0;
  for (size_t i = 0; i < len; i++) t = t ^ data[i];
  return t;
}

/*!
 * 功能：仿真spi板来估计转矩
 * spi_command_t：SPI指令
 * spi_data_t：   SPI数据
 * spi_torque_t： SPI估计力矩
 * board_num：    SPI的片选信号
 * 步骤：
 * （1）计算三个关节的力矩输出
 * （2）输出力矩限幅
 */
void fake_spine_control(spi_command_t *cmd, spi_data_t *data,
                        spi_torque_t *torque_out, int board_num) 
{
  /*（1）计算三个关节的力矩输出*/
  //tau_abad
  torque_out->tau_abad[board_num] =
      cmd->kp_abad[board_num] *
          (cmd->q_des_abad[board_num] - data->q_abad[board_num]) +
      cmd->kd_abad[board_num] *
          (cmd->qd_des_abad[board_num] - data->qd_abad[board_num]) +
      cmd->tau_abad_ff[board_num];
//tau_hip
  torque_out->tau_hip[board_num] =
      cmd->kp_hip[board_num] *
          (cmd->q_des_hip[board_num] - data->q_hip[board_num]) +
      cmd->kd_hip[board_num] *
          (cmd->qd_des_hip[board_num] - data->qd_hip[board_num]) +
      cmd->tau_hip_ff[board_num];
//tau_knee
  torque_out->tau_knee[board_num] =
      cmd->kp_knee[board_num] *
          (cmd->q_des_knee[board_num] - data->q_knee[board_num]) +
      cmd->kd_knee[board_num] *
          (cmd->qd_des_knee[board_num] - data->qd_knee[board_num]) +
      cmd->tau_knee_ff[board_num];

/*（2）输出力矩限幅*/
  const float *torque_limits = disabled_torque;
//（2.1）每个关节的力矩限制参数分配
  if (cmd->flags[board_num] & 0b1) 
  {
    if (cmd->flags[board_num] & 0b10)
      torque_limits = wimp_torque;
    else
      torque_limits = max_torque;
  }
//（2.2）每个关节的力矩根据上面分配的参数进行限幅
  //tau_abad
  if (torque_out->tau_abad[board_num] > torque_limits[0])
    torque_out->tau_abad[board_num] = torque_limits[0];
  if (torque_out->tau_abad[board_num] < -torque_limits[0])
    torque_out->tau_abad[board_num] = -torque_limits[0];
//tau_hip
  if (torque_out->tau_hip[board_num] > torque_limits[1])
    torque_out->tau_hip[board_num] = torque_limits[1];
  if (torque_out->tau_hip[board_num] < -torque_limits[1])
    torque_out->tau_hip[board_num] = -torque_limits[1];
//tau_knee
  if (torque_out->tau_knee[board_num] > torque_limits[2])
    torque_out->tau_knee[board_num] = torque_limits[2];
  if (torque_out->tau_knee[board_num] < -torque_limits[2])
    torque_out->tau_knee[board_num] = -torque_limits[2];
}



/*!
 * 功能：初始化SPI函数
 */
void init_spi() 
{
  //（1）创建SPI命令和数据的格式大小
  size_t command_size = sizeof(spi_command_t);
  size_t data_size = sizeof(spi_data_t);
  //（2）把SPI命令和数据搬运建立连接关系
  memset(&spi_command_drv, 0, sizeof(spi_command_drv));
  memset(&spi_data_drv, 0, sizeof(spi_data_drv));
  //（3）若SPI线程接收没打开，则报错
  if (pthread_mutex_init(&spi_mutex, NULL) != 0)    
  {
     printf("[ERROR: RT SPI] Failed to create spi data mutex\n");
  }
  //（4）若SPI命令格式大小不一致报错
  if (command_size != K_EXPECTED_COMMAND_SIZE)      
  {
    printf("[RT SPI] Error command size is %ld, expected %d\n", command_size,
           K_EXPECTED_COMMAND_SIZE);
  } 
  else
  {
    printf("[RT SPI] command size good\n");
  }
  //（5）若SPI数据格式大小不一致报错 
  if (data_size != K_EXPECTED_DATA_SIZE)           
  {
    printf("[RT SPI] Error data size is %ld, expected %d\n", data_size,
           K_EXPECTED_DATA_SIZE);
  } 
  else
  {
     printf("[RT SPI] data size good\n");
  }
  //（6）打开通讯
 printf("[RT SPI] Open\n");
  spi_open();                                      //（6）打开通讯
}


/*!
 * 功能：打开SPI设备
 */
int spi_open() 
{
  int rv = 0;
  spi_1_fd = open("/dev/spidev2.0", O_RDWR);
  if (spi_1_fd < 0) perror("[ERROR] Couldn't open spidev 2.0");
  spi_2_fd = open("/dev/spidev2.1", O_RDWR);
  if (spi_2_fd < 0) perror("[ERROR] Couldn't open spidev 2.1");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_WR_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_mode (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_MODE, &spi_mode);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_mode (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_bits_per_word (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_BITS_PER_WORD, &spi_bits_per_word);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_bits_per_word (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (1)");
  rv = ioctl(spi_2_fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_wr_max_speed_hz (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (1)");
  rv = ioctl(spi_2_fd, SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_max_speed_hz (2)");

  rv = ioctl(spi_1_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (1)");

  rv = ioctl(spi_2_fd, SPI_IOC_RD_LSB_FIRST, &lsb);
  if (rv < 0) perror("[ERROR] ioctl spi_ioc_rd_lsb_first (2)");
  return rv;
}

int spi_driver_iterations = 0;

/*!
 * 功能：将spi命令转换为spine（通讯板）命令
 */
void spi_to_spine(spi_command_t *cmd, spine_cmd_t *spine_cmd, int leg_0) 
{
  for (int i = 0; i < 2; i++) //三个关节
  {
    //（1）三个关节期望位置指令转换
    spine_cmd->q_des_abad[i] =
        (cmd->q_des_abad[i + leg_0] * abad_side_sign[i + leg_0]) +
        abad_offset[i + leg_0];
    spine_cmd->q_des_hip[i] =
        (cmd->q_des_hip[i + leg_0] * hip_side_sign[i + leg_0]) +
        hip_offset[i + leg_0];
    spine_cmd->q_des_knee[i] =
        (cmd->q_des_knee[i + leg_0] / knee_side_sign[i + leg_0]) +
        knee_offset[i + leg_0];

    //（2）三个关节期望速度指令转换
    spine_cmd->qd_des_abad[i] =
        cmd->qd_des_abad[i + leg_0] * abad_side_sign[i + leg_0];
    spine_cmd->qd_des_hip[i] =
        cmd->qd_des_hip[i + leg_0] * hip_side_sign[i + leg_0];
    spine_cmd->qd_des_knee[i] =
        cmd->qd_des_knee[i + leg_0] / knee_side_sign[i + leg_0];

    //（3）三个关节KP增益指令转换
    spine_cmd->kp_abad[i] = cmd->kp_abad[i + leg_0];
    spine_cmd->kp_hip[i] = cmd->kp_hip[i + leg_0];
    spine_cmd->kp_knee[i] = cmd->kp_knee[i + leg_0];

    //（4）三个关节KD增益指令转换
    spine_cmd->kd_abad[i] = cmd->kd_abad[i + leg_0];
    spine_cmd->kd_hip[i] = cmd->kd_hip[i + leg_0];
    spine_cmd->kd_knee[i] = cmd->kd_knee[i + leg_0];

    //（5）三个关节力矩指令转换
    spine_cmd->tau_abad_ff[i] =
        cmd->tau_abad_ff[i + leg_0] * abad_side_sign[i + leg_0];
    spine_cmd->tau_hip_ff[i] =
        cmd->tau_hip_ff[i + leg_0] * hip_side_sign[i + leg_0];
    spine_cmd->tau_knee_ff[i] =
        cmd->tau_knee_ff[i + leg_0] * knee_side_sign[i + leg_0];

    //（6）三个关节指令标志位
    spine_cmd->flags[i] = cmd->flags[i + leg_0];
  }
  //（7）看看组合起来是不是32位数据
  spine_cmd->checksum = xor_checksum((uint32_t *)spine_cmd, 32);    
}

/*!
 * 功能：将通讯板数据转换为spi数据
 */
void spine_to_spi(spi_data_t *data, spine_data_t *spine_data, int leg_0) 
{
  for (int i = 0; i < 2; i++) 
  {
    //（1）三个关节期望位置指令转换
    data->q_abad[i + leg_0] = (spine_data->q_abad[i] - abad_offset[i + leg_0]) *
                              abad_side_sign[i + leg_0];
    data->q_hip[i + leg_0] = (spine_data->q_hip[i] - hip_offset[i + leg_0]) *
                             hip_side_sign[i + leg_0];
    data->q_knee[i + leg_0] = (spine_data->q_knee[i] - knee_offset[i + leg_0]) *
                              knee_side_sign[i + leg_0];
    //（2）三个关节期望速度指令转换
    data->qd_abad[i + leg_0] =
        spine_data->qd_abad[i] * abad_side_sign[i + leg_0];
    data->qd_hip[i + leg_0] = spine_data->qd_hip[i] * hip_side_sign[i + leg_0];
    data->qd_knee[i + leg_0] =
        spine_data->qd_knee[i] * knee_side_sign[i + leg_0];
    //（3）三个关节指令标志位
    data->flags[i + leg_0] = spine_data->flags[i];
  }
    //（4）看看组合起来是不是14位数据
  uint32_t calc_checksum = xor_checksum((uint32_t *)spine_data, 14);    
  if (calc_checksum != (uint32_t)spine_data->checksum)                  //如果组合起来不是14位数据，报错
    printf("SPI ERROR BAD CHECKSUM GOT 0x%hx EXPECTED 0x%hx\n", calc_checksum,
           spine_data->checksum);
}


/*!
 * 功能：从通讯板发送接收数据和命令函数
 * command：  命令
 * data：     数据
 */
void spi_send_receive(spi_command_t *command, spi_data_t *data) 
{
  //（1）更新驱动程序状态标志 
  spi_driver_iterations++;
  data->spi_driver_status = spi_driver_iterations << 16;
  //（2）定义发送和接收缓冲器 
  uint16_t tx_buf[K_WORDS_PER_MESSAGE];
  uint16_t rx_buf[K_WORDS_PER_MESSAGE];
  //（3）两块MCU，实现两个SPI的通讯
  for (int spi_board = 0; spi_board < 2; spi_board++) 
  {
    //1）将命令复制到通讯板类型：
    spi_to_spine(command, &g_spine_cmd, spi_board * 2);

    //2）定义指向命令/数据通讯板数组的指针
    uint16_t *cmd_d = (uint16_t *)&g_spine_cmd;
    uint16_t *data_d = (uint16_t *)&g_spine_data;

    //3）定义零接收缓冲器
    memset(rx_buf, 0, K_WORDS_PER_MESSAGE * sizeof(uint16_t));

    //4）复制到发送缓冲区翻转字节
    for (int i = 0; i < K_WORDS_PER_MESSAGE; i++)
      tx_buf[i] = (cmd_d[i] >> 8) + ((cmd_d[i] & 0xff) << 8);

    //5）定义每个字有两个字节长 
    size_t word_len = 2;  // 16 bit word

    //6）定义spi消息结构
    struct spi_ioc_transfer spi_message[1];

    //7）定义零消息结构
    memset(spi_message, 0, 1 * sizeof(struct spi_ioc_transfer));

    //8）设置消息结构
    for (int i = 0; i < 1; i++) 
    {
      spi_message[i].bits_per_word = spi_bits_per_word;
      spi_message[i].cs_change = 1;
      spi_message[i].delay_usecs = 0;
      spi_message[i].len = word_len * 66;
      spi_message[i].rx_buf = (uint64_t)rx_buf;
      spi_message[i].tx_buf = (uint64_t)tx_buf;
    }

    //9）进行spi通信
    int rv = ioctl(spi_board == 0 ? spi_1_fd : spi_2_fd, SPI_IOC_MESSAGE(1),
                   &spi_message);
    (void)rv;

    //10）反向翻转字节
    for (int i = 0; i < 30; i++)
      data_d[i] = (rx_buf[i] >> 8) + ((rx_buf[i] & 0xff) << 8);

    //11)从通讯板搬数据到uboard 
    spine_to_spi(data, &g_spine_data, spi_board * 2);
  }
}


/*!
 * 功能：运行SPI驱动函数
 */
void spi_driver_run() 
{
  //（1）仿真模式下，计算估计转矩数据
  for (int i = 0; i < 4; i++) 
  {
    fake_spine_control(&spi_command_drv, &spi_data_drv, &spi_torque, i);//仿真模式下，计算估计转矩数据
  }
  //（2）驱动准备就绪
  pthread_mutex_lock(&spi_mutex);                       //SPI线程上锁
  spi_send_receive(&spi_command_drv, &spi_data_drv);    //从通讯板发送接收数据和命令
  pthread_mutex_unlock(&spi_mutex);                     //SPI线程解锁
}

/*!
 * 功能：获取spi命令函数
 */
spi_command_t *get_spi_command() 
{
  return &spi_command_drv;
}

/*!
 *功能： 获取spi数据函数
 */
spi_data_t *get_spi_data() 
{ return &spi_data_drv; }

//#endif
