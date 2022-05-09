#include "include/Utilities/utilities.h"

#include <ctime>
#include <iomanip>
#include <iostream>


/*!
 * 写字符串到文件里
 */
void writeStringToFile(const std::string& fileName,
                       const std::string& fileData) 
{
  FILE* fp = fopen(fileName.c_str(), "w");
  if (!fp) {
    printf("Failed to fopen %s\n", fileName.c_str());
    throw std::runtime_error("Failed to open file");
  }
  fprintf(fp, "%s", fileData.c_str());
  fclose(fp);
}


/*!
 * 获取现在的时间和数据
 */
std::string getCurrentTimeAndDate() 
{
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream ss;
  ss << std::put_time(&tm, "%c");
  return ss.str();
}

/*!
 * Todo: 做些更好的事情来跟踪我们相对于配置目录
 */
std::string getConfigDirectoryPath() { return "../config/"; }

std::string getLcmUrl(s64 ttl)
{
  assert(ttl >= 0 && ttl <= 255);
  return "udpm://239.255.76.67:7667?ttl=" + std::to_string(ttl);
}