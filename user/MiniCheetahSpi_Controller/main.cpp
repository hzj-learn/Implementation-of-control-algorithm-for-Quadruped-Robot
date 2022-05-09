
#include <main_helper.h>
#include "MiniCheetahSpi_Controller.h"


/*!
 * 功能：主函数，调用SPI通讯控制器
 */
int main(int argc, char** argv) 
{
  main_helper(argc, argv, new MiniCheetahSpi_Controller());
  return 0;
}