#include "DataReader.hpp"
#include <Configuration.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
/**
 * 功能：数据读取
 */
DataReader::DataReader(const RobotType& type, FSM_StateName stateNameIn) : _type(type) 
{
  if (_type == RobotType::MINI_CHEETAH)   // MINI_CHEETAH机器人
  {
    if (stateNameIn == FSM_StateName::BACKFLIP)               //在后空翻状态
    {
      load_control_plan(THIS_COM "config/mc_flip.dat");       //加载.dat文件
      printf("[Backflip DataReader] Setup for mini cheetah\n");
    }
    else if (stateNameIn == FSM_StateName::FRONTJUMP)         //在前跳状态
    {
      load_control_plan(THIS_COM "config/front_jump_pitchup_v2.dat");//加载.dat文件
      printf("[Front Jump DataReader] Setup for mini cheetah\n");
    }
  } 
  else                                    //CHEETAH 3 机器人
  {
    printf("[Backflip DataReader] Setup for cheetah 3\n");
    load_control_plan(THIS_COM "user/WBC_Controller/WBC_States/BackFlip/data/backflip.dat");  //加载.dat文件
  }
  printf("[Backflip DataReader] Constructed.\n");
}


/**
 * 功能：加载.dat文件
 */
void DataReader::load_control_plan(const char* filename) 
{
  printf("[Backflip DataReader] Loading control plan %s...\n", filename);
  FILE* f = fopen(filename, "rb");                                        //（1）打开名称为"rb"的文件
  if (!f)                                       //加载控制文件失败，报错
  {
    printf("[Backflip DataReader] Error loading control plan!\n");
    return;
  }
  fseek(f, 0, SEEK_END);
  uint64_t file_size = ftell(f);
  fseek(f, 0, SEEK_SET);

  printf("[Backflip DataReader] Allocating %ld bytes for control plan\n",
         file_size);

  plan_buffer = (float*)malloc(file_size + 1);

  if (!plan_buffer)                             //数据复制错误
  {
    printf("[Backflip DataReader] malloc failed!\n");
    return;
  }

  uint64_t read_success = fread(plan_buffer, file_size, 1, f);
  if (!read_success)                            //读取文件不成功
    {
    printf("[Backflip DataReader] Error: fread failed.\n");
  }

  if (file_size % sizeof(float))                //文件大小不匹配
  {
    printf(
        "[Backflip DataReader] Error: file size isn't divisible by size of "
        "float!\n");
  }

  fclose(f);

  plan_loaded = true;
  plan_timesteps = file_size / (sizeof(float) * plan_cols);
  printf("[Backflip DataReader] Done loading plan for %d timesteps\n",
         plan_timesteps);
}


/**
 * 功能：加载.dat文件
 */
float* DataReader::get_initial_configuration() 
{
  if (!plan_loaded) //文件没有加载成功
  {
    printf(
        "[Backflip DataReader] Error: get_initial_configuration called without "
        "a plan!\n");
    return nullptr;
  }

  return plan_buffer + 3;
}


/**
 * 功能：定时加载.dat文件
 */
float* DataReader::get_plan_at_time(int timestep) 
{
  if (!plan_loaded)                               //文件没有加载成功
  {
    printf(
        "[Backflip DataReader] Error: get_plan_at_time called without a "
        "plan!\n");
    return nullptr;
  }

  if (timestep < 0 || timestep >= plan_timesteps) //加载文件超时
  {
    printf(
        "[Backflip DataReader] Error: get_plan_at_time called for timestep %d\n"
        "\tmust be between 0 and %d\n",
        timestep, plan_timesteps - 1);
    timestep = plan_timesteps - 1;
  }

  return plan_buffer + plan_cols * timestep;
}


/**
 * 功能：卸载载.dat文件
 */
void DataReader::unload_control_plan() 
{
  free(plan_buffer);
  plan_timesteps = -1;
  plan_loaded = false;
  printf("[Backflip DataReader] Unloaded plan.\n");
}
