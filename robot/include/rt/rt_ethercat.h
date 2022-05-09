#ifndef PROJECT_RT_ETHERCAT_H
#define PROJECT_RT_ETHERCAT_H

#include <cstdint>

void rt_ethercat_init();                                    //以太网通讯初始化函数
void rt_ethercat_run();                                     //运行以太网通讯函数
struct TiBoardData;
struct TiBoardCommand;
void rt_ethercat_get_data(TiBoardData* data);               //以太网通讯接收数据函数
void rt_ethercat_set_command(TiBoardCommand* command);      //以太网通讯设置命令函数
#endif //PROJECT_RT_ETHERCAT_H
