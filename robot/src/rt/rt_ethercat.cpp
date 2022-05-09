#include "rt/rt_ethercat.h"
#include <stdio.h>
#include <string.h>
#include <inttypes.h> 
#include <stdlib.h>
#include <time.h>
#include <mutex>
#include <SOEM/soem/ethercat.h>
#include "SOEM/soem/ethercat.h"
#include "SOEM/osal/osal.h"
#include "SOEM/osal/linux/osal_defs.h"
#include "rt/rt_ethercat.h"
#include "SimUtilities/ti_boardcontrol.h"


#define EC_TIMEOUTMON 500

static char IOmap[4096];
static OSAL_THREAD_HANDLE thread1;
static int expectedWKC;
static boolean needlf;
static volatile int wkc;
static boolean inOP;
static uint8 currentgroup = 0;

#define ADAPTER_NAME "enp2s0"

/*!
 * 功能：以太网通讯报错提示
 * 备注：都是以太网网通讯协议的实现，没必要细看，硬件驱动
 */
static void degraded_handler() 
{
  // estop();  //关闭gpio
  printf("[EtherCAT Error] Logging error...\n");
  time_t current_time = time(NULL);
  char* time_str = ctime(&current_time);
  printf("ESTOP. EtherCAT became degraded at %s.\n", time_str);
  printf("[EtherCAT Error] Stopping RT process.\n");
  exit(0);
}


/*!
 * 功能：运行以太网通讯函数
 * 备注：都是以太网网通讯协议的实现，没必要细看，硬件驱动
 */
static int run_ethercat(const char *ifname) 
{
  int i, oloop, iloop, chk;
  needlf = FALSE;
  inOP = FALSE;

  /* 初始化SOEM，将socket绑定到ifname */
  if (ec_init(ifname))
  {
    printf("[EtherCAT Init] Initialization on device %s succeeded.\n",ifname);
    /* 查找和自动配置从属对象 */

    if ( ec_config_init(FALSE) > 0 )
    {
      printf("[EtherCAT Init] %d slaves found and configured.\n",ec_slavecount);
      if(ec_slavecount < 4)
      {
        printf("[RT EtherCAT] Warning: Expected %d legs, found %d.\n", 4, ec_slavecount);
      }

      ec_config_map(&IOmap);
      ec_configdc();

      printf("[EtherCAT Init] Mapped slaves.\n");
      /* 等待所有从机进入安全状态 */
      ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

      for(int slave_idx = 0; slave_idx < ec_slavecount; slave_idx++) {
        printf("[SLAVE %d]\n", slave_idx);
        printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
        printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
        printf("\n");
     }

      oloop = ec_slave[0].Obytes;
      if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
      if (oloop > 8) oloop = 8;
      iloop = ec_slave[0].Ibytes;
      if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
      if (iloop > 8) iloop = 8;

      printf("[EtherCAT Init] segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

      printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
      expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
      printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
      ec_slave[0].state = EC_STATE_OPERATIONAL;
      /* 发送一个有效的进程数据以使从进程中的输出满意 */
      ec_send_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      /* 请求所有从机的操作状态 */
      ec_writestate(0);
      chk = 40;
      /* 等待所有从机进入操作状态 */
      do
      {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
      }
      while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

      if (ec_slave[0].state == EC_STATE_OPERATIONAL )
      {
        printf("[EtherCAT Init] Operational state reached for all slaves.\n");
        inOP = TRUE;
        return 1;

      }
      else
      {
        printf("[EtherCAT Error] Not all slaves reached operational state.\n");
        ec_readstate();

        for(i = 1; i<=ec_slavecount ; i++)
        {
          if(ec_slave[i].state != EC_STATE_OPERATIONAL)
          {

            printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                    i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
          }
        }
      }
    }
    else
    {
      printf("[EtherCAT Error] No slaves found!\n");
    }
  }
  else
  {
    printf("[EtherCAT Error] No socket connection on %s - are you running run.sh?\n",ifname);
  }
  return 0;
}


/*!
 * 功能：以太网校验程序
 * 备注：都是以太网网通讯协议的实现，没必要细看。，硬件驱动
 */
static int err_count = 0;
static int err_iteration_count = 0;
#define K_ETHERCAT_ERR_PERIOD 100   // EtherCAT错误是在循环迭代的这段时间内测量的 
#define K_ETHERCAT_ERR_MAX 20       //每个循环迭代周期发生故障之前的最大etherCAT错误数
static OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
  (void)ptr;
  int slave = 0;
  while(1)
  {
    if(err_iteration_count > K_ETHERCAT_ERR_PERIOD)    //计数错误
    {
      err_iteration_count = 0;
      err_count = 0;
    }
    if(err_count > K_ETHERCAT_ERR_MAX)                //发生故障的最大etherCAT错误数大于阈值
    {
      printf("[EtherCAT Error] EtherCAT connection degraded.\n");
      printf("[Simulink-Linux] Shutting down....\n");
      degraded_handler();
      break;
    }
    err_iteration_count++;

    if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
    {
      if (needlf)
      {
        needlf = FALSE;
        printf("\n");
      }
      /* 一个或多个从机没有反应 */
      ec_group[currentgroup].docheckstate = FALSE;
      ec_readstate();
      for (slave = 1; slave <= ec_slavecount; slave++)
      {
        if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
        {
          ec_group[currentgroup].docheckstate = TRUE;
          if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
          {
            printf("[EtherCAT Error] Slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
            ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
            ec_writestate(slave);
            err_count++;
          }
          else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
          {
            printf("[EtherCAT Error] Slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
            ec_slave[slave].state = EC_STATE_OPERATIONAL;
            ec_writestate(slave);
            err_count++;
          }
          else if(ec_slave[slave].state > 0)
          {
            if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("[EtherCAT Status] Slave %d reconfigured\n",slave);
            }
          }
          else if(!ec_slave[slave].islost)
          {
            /* 重新检查状态 */
            ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (!ec_slave[slave].state)
            {
              ec_slave[slave].islost = TRUE;
              printf("[EtherCAT Error] Slave %d lost\n",slave);
              err_count++;
            }
          }
        }
        if (ec_slave[slave].islost)
        {
          if(!ec_slave[slave].state)
          {
            if (ec_recover_slave(slave, EC_TIMEOUTMON))
            {
              ec_slave[slave].islost = FALSE;
              printf("[EtherCAT Status] Slave %d recovered\n",slave);
            }
          }
          else
          {
            ec_slave[slave].islost = FALSE;
            printf("[EtherCAT Status] Slave %d found\n",slave);
          }
        }
      }
      if(!ec_group[currentgroup].docheckstate)
        printf("[EtherCAT Status] All slaves resumed OPERATIONAL.\n");
    }
    osal_usleep(50000);
  }
}


/*!
 * 功能：以太网通讯初始化函数
 * 备注：有必要细看，硬件驱动
 */
void rt_ethercat_init()
{
  //（1）终端打印提示正在初始化以太网
  printf("[EtherCAT] Initializing EtherCAT\n");    
  //（2）初始化以太网监视线程，用第三方库函数实现的                                 
  osal_thread_create((void*)&thread1, 128000, (void*)&ecatcheck, (void*) &ctime);   

  //（3）尝试100次初始化连接以太网，直到成功
  int i;
  int rc;
  for(i = 1; i < 100; i++)                                                          
  {
    printf("[EtherCAT] Attempting to start EtherCAT, try %d of 100.\n", i);
    rc = run_ethercat(ADAPTER_NAME);      //1）启动以太网通讯，把以太网的数据搬过来，硬件驱动
    if(rc) break;                         //2）若启动成功就不用重复启动了                                  
    osal_usleep(1000000);                 //3）启动若不成功，休眠一下在尝试
  }

  //（4）验证以太网通讯是否正常，终端打印出来
  if(rc) //以太网接收到数据，证明初始化成功，打印成功提示
  {
    printf("[EtherCAT] EtherCAT successfully initialized on attempt %d \n", i); 
  }
  else  //以太网接收不到数据，证明初始化失败，报错
  {
    printf("[EtherCAT Error] Failed to initialize EtherCAT after 100 tries. \n");
  }
  
}



/* 
 * 功能：启动etherCAT通信，通过EtherCAT发送和接收数据
 * 备注：在模拟中，通过LCM发送数据
 *      在robt上，验证EtherCAT连接是否正常，发送数据、接收数据，并检查丢失的数据包
 * 有必要细看
 * 步骤：
 * （1）检查连接
 * （2）发送
 * （3）接收
 *  (4)检查丢包
 */
static int wkc_err_count = 0;
static int wkc_err_iteration_count = 0;
static std::mutex command_mutex, data_mutex;
void rt_ethercat_run()
{
  /*（1）检查以太网连接是否正常*/
  if(wkc_err_iteration_count > K_ETHERCAT_ERR_PERIOD)
  {
    wkc_err_count = 0;
    wkc_err_iteration_count = 0;
  }
  if(wkc_err_count > K_ETHERCAT_ERR_MAX)
  {
    printf("[EtherCAT Error] Error count too high!\n");
    degraded_handler();//以太网通讯报错提示，程序在降级处理程序中终止。
  }

  /*（2）发送以太网数据*/
  command_mutex.lock();       //1）以太网线程上锁，进入临界区
  ec_send_processdata();      //2）发送以太网数据，驱动库函数实现
  command_mutex.unlock();     //3）以太网线程解锁，退出临界区

  /*（3）接收以太网数据*/
  data_mutex.lock();                            //1）以太网线程上锁，进入临界区
  wkc = ec_receive_processdata(EC_TIMEOUTRET);  //2）接收以太网数据，驱动库函数实现
  data_mutex.unlock();                          //3）以太网线程解锁，退出临界区

  /*(4)检查数据是否丢包*/
  if(wkc < expectedWKC)
  {
    printf("\x1b[31m[EtherCAT Error] Dropped packet (Bad WKC!)\x1b[0m\n");
    wkc_err_count++;
  }
  wkc_err_iteration_count++;
}


/*!
 * 功能：以太网通讯接收数据，接收upboard上的数据
 * 旧版的没有用到，新版是用库实现
 */
void rt_ethercat_get_data(TiBoardData* data) 
{
  data_mutex.lock();                     //(1)以太网线程上锁，进入临界区
                                         //(2)直接从缓冲器中读取接收到的upboard数据
  for(int slave = 0; slave < 4; slave++) 
  {
    TiBoardData* slave_src = (TiBoardData*)(ec_slave[slave + 1].inputs);
    if(slave_src)
      data[slave] = *(TiBoardData*)(ec_slave[slave + 1].inputs);
  }
  data_mutex.unlock();                  //(3)以太网线程解锁，退出临界区
}


/*!
 * 功能：以太网通讯设置命令，发送数据到upboard上
 * 旧版的没有用到，新版是用库实现
 */
void rt_ethercat_set_command(TiBoardCommand* command) 
{
  command_mutex.lock();                 //（1）以太网线程上锁，进入临界区
                                        //(2)直接从缓冲器中发送数据到的upboard数据
  for(int slave = 0; slave < 4; slave++) 
  {
    TiBoardCommand* slave_dest = (TiBoardCommand*)(ec_slave[slave + 1].outputs);
    if(slave_dest)
      *(TiBoardCommand*)(ec_slave[slave + 1].outputs) = command[slave];
  }
  command_mutex.unlock();                //（3）以太网线程解锁，退出临界区
}
