/*!
 * @file main.cpp
 * @brief Main Function for the WBC Controller
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <main_helper.h>
#include "WBC_Controller.hpp"

/**
 * 功能：运行WBC_Controller
 */
int main(int argc, char** argv) 
{
  main_helper(argc, argv, new WBC_Controller());
  return 0;
}
