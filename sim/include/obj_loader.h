/*! @file obj_loader.h
 *  @brief 加载.obj文件的实用程序，包含机器人的三维模型。
 */

#ifndef OBJLOADER_H
#define OBJLOADER_H

#include <string>
#include <vector>

void load_obj_file(std::string fileName, std::vector<float>& positions,
                   std::vector<float>& normals);

#endif  // OBJLOADER_H
