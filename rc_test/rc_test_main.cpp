#include <cstdio>
#include <thread>
#include <unistd.h>

#include "rt/rt_sbus.h"
#include "rt/rt_rc_interface.h"
//遥控器测试
static struct {
 double     mode;
 double     impedance_scale;
 double     enable;
 double     emergency_damp;
 double     zero_leg[4];
 double     p_des[3];
 double     v_des[3];
 double     rpy_des[3];
 double     omega_des[3];
 double     p_des_slew_min[3];
 double     p_des_slew_max[3];
 double     rpy_des_slew_max[3];
 double     v_des_slew_min[3];
 double     v_des_slew_max[3];
 double     omegab_des_slew_max[3];
 double     emergency_damp_kd;
 double     alexa_mode;
 double     rc_configured;
 double     bonus_knee_torque;
 double     variable[3];
 double     want_cheater_mode;
} main_control_settings;

const char* switch_names[3] = {
    "UP", "--", "DN"
};

/*
     [UP]                        [UP]
  [UP]  [UP]  [ 0.1]  [-1.0]  [UP]  [UP]
         [-0.0 -0.0]  [ 0.0  0.0]
 */
//打印手柄按键和功能的对应关系
void print_sbus(Taranis_X7_data& data) 
{
  printf("\n\n\n"
         "     [%s]                        [%s]\n"
         "  [%s]  [%s]  [%4.1f]  [%4.1f]  [%s]  [%s]\n"
         "         [%4.1f %4.1f]  [%4.1f %4.1f]\n",
         switch_names[(uint32_t)data.left_upper_switch],
         switch_names[(uint32_t)data.right_upper_switch],
         switch_names[(uint32_t)data.left_lower_left_switch],
         switch_names[(uint32_t)data.left_lower_right_switch],
         data.knobs[0], data.knobs[1],
         switch_names[(uint32_t)data.right_lower_left_switch],
         switch_names[(uint32_t)data.right_lower_right_switch],
         data.left_stick[0], data.left_stick[1], data.right_stick[0], data.right_stick[1]
  );
}


//打印设置的期望速度
void print_left_y() 
{
 // printf("[%6.3f]\n", data.left_stick[1]);
  printf("cmd: %.3f\n", main_control_settings.v_des[0]);
  int cmd_int = (main_control_settings.v_des[0] * 50) + 50;
  for(int i = 0; i < cmd_int; i++) 
  {
    printf("*");
  }
  printf("\n");
  printf("mode: %d\n\n", (int)main_control_settings.mode);
}


//手柄发送过来的数据解包，用于设置运动模式、步态、躯干速度方向等
void sbus_packet_complete() 
{
  Taranis_X7_data data;
  update_taranis_x7(&data);                                   //（1）手柄按键及摇杆的数字量数据换算【手柄的模拟量在其他地方】
  float v_scale = data.knobs[0]*1.2f + 1.2f;                  //（2）设置手柄摇杆的振幅常量，相当于摇杆模拟量的增益，范围从0.5到1.5

                                                              //（3）设置手柄上的按键和机器人控制模式及参数的对应关系
  auto estop_switch = data.right_lower_right_switch;          //设置停止模式开关
  auto backflip_prepare_switch = data.left_lower_left_switch; //设置准备后空翻模式开关
  auto backflip_go_switch = data.left_upper_switch;           //设置开启后空翻模式开关
  auto left_select = data.left_lower_right_switch;            //设置左移动模式开关
  auto right_select = data.right_lower_left_switch;           //设置右移动模式开关


  switch(estop_switch)                                       //（4）进行运动模式选择
  {
    case SWITCH_UP:                     //上按键--停止模式
      main_control_settings.mode = RC_mode::OFF;
      break;

    case SWITCH_MIDDLE:                 //中按键--恢复站立模式
      main_control_settings.mode = RC_mode::RECOVERY_STAND;
      break;

    case SWITCH_DOWN:                   //下按键--运动模式
      main_control_settings.mode = RC_mode::LOCOMOTION; 

      if(left_select == SWITCH_UP && right_select == SWITCH_UP) // 站立模式
      {
        main_control_settings.mode = RC_mode::QP_STAND;
      }

      if(backflip_prepare_switch == SWITCH_MIDDLE)              //后空翻模式
       {
        main_control_settings.mode = RC_mode::BACKFLIP_PRE;     //后空翻准备模式

        if(backflip_go_switch == SWITCH_DOWN)                 //开始后空翻模式
        {
          main_control_settings.mode = RC_mode::BACKFLIP;
        }
      }

                                                                //（5）步态选择
      int gait_id = left_select * 3 + right_select;
      constexpr int gait_table[9] = {
                                     0, //stand
                                     0, // trot
                                     1, // bounding
                                     2, // pronking
                                     3, // gallop
                                     5, // trot run
                                     6, // walk
                                     7, // walk2
                                     8, // pace
                                     };

      if(main_control_settings.mode == RC_mode::LOCOMOTION)    //（6）运动模式下，通过手柄设置躯干的位置、速度、偏航角度
      {
        main_control_settings.variable[0] = gait_table[gait_id];              //设置步态（表）
        main_control_settings.v_des[0] = v_scale * data.left_stick[1] * 0.5;  //设置躯干x的速度，   公式：躯干x的速度=摇杆模拟量的增益*左摇杆模拟量*0.5
        main_control_settings.v_des[1] = v_scale * data.left_stick[0] * -1.;  //设置躯干y的速度，   公式：躯干y的速度=摇杆模拟量的增益*左摇杆模拟量*0.5
        main_control_settings.v_des[2] = 0;                                   //设置躯干z的速度
        main_control_settings.p_des[2] = 0.25;                                //设置期望的位置

        main_control_settings.omega_des[0] = 0;                               //设置期望的滚动角
        main_control_settings.omega_des[1] = 0;                               //设置期望的倾斜角
        main_control_settings.omega_des[2] = -v_scale * data.right_stick[0];  //计算期望的偏航角，偏航角=摇杆模拟量的增益*右摇杆模拟量

        main_control_settings.rpy_des[1] = v_scale * data.right_stick[0];     //计算期望的偏航角，偏航角=摇杆模拟量的增益*右摇杆模拟量

      } 
      else if(main_control_settings.mode == RC_mode::QP_STAND)  //（6）站立平衡模式下，通过手柄设置躯干的位置和转角
      {
        //计算期望的rpy
        main_control_settings.rpy_des[0] = data.left_stick[0] * 1.4;
        main_control_settings.rpy_des[1] = data.right_stick[1] * 0.46;
        main_control_settings.rpy_des[2] = -data.right_stick[0];
        //计算期望的xyz位置
        main_control_settings.p_des[0] = 0;
        main_control_settings.p_des[1] = -0.667 * main_control_settings.rpy_des[0];
        main_control_settings.p_des[2] = data.left_stick[1] * .12;
      }

      break;
  }

}




#include <cmath>

//若上一次设置的期望速度和这一次设置的期望速度差值过大，机器人是无法跳变的，报错
static double last_cmd = 0.;
void check_d_rate() 
{
  if(std::abs(last_cmd - main_control_settings.v_des[0]) > 0.2) //若上一次设置的期望速度和这一次设置的期望速度差值过大，机器人是无法跳变的，报错
  {
    printf("RATE error! (%6.3f -> %6.3f)\n", last_cmd, main_control_settings.v_des[0]);
  }
  last_cmd = main_control_settings.v_des[0];      //设置接收到的最后的指令
}


std::thread lag_threads[12];


//打印标签
void lag() 
{
  for(auto& x : lag_threads) 
  {
    x = std::thread([]()
    {
      int64_t beans = 0;
      for(int64_t i = 2; i < 1000000000; i++) 
      {
        beans += i * i - i + 1223123123/i + sqrt((double)i);
      }
      printf("done %ld\n", beans);
    });
  }
}



int main(int argc, char** argv) 
{
  (void)argc;
  (void)argv;
  //（1）判断使用GUI控制还是手柄控制
  int use_computer_port = 1;                  //默认使用计算机控制
  if(argc == 2 && argv[1][0] == 'r')          //若使用手柄
  {
    use_computer_port = 0;
  }
  lag();                                      //（2）打印显示标签
  auto port = init_sbus(use_computer_port);   //（3）初始化手柄sbus端口
                                              //（4）运行sbus线程处理函数
  auto sbus_thread = std::thread([&]()        
  {
    while(true) 
    {
      receive_sbus(port);                     //从port接收手柄的数据，接收串行底层数据包
      if (receive_sbus(port))                 //如果接收到手柄信息
      {
        sbus_packet_complete();               //手柄发送过来的数据包，用于设置运动模式、步态、躯干速度方向等
      }
    }
  });

  for(;;) 
  {
    for(int i = 0; i < 30; i++) 
    {
      usleep(1000);       //休眠1ms
      check_d_rate();     //若上一次设置的期望速度和这一次设置的期望速度差值过大，机器人是无法跳变的，报错 
    }
    print_left_y();       //打印设置的期望速度
  }

  sbus_thread.join();     //开启sbus线程
  return 0;
}
