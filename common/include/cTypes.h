/*! @file cTypes.h
 *  @brief Common types that can also be included in C code
 *该文件包含C和C++代码之间共享的类型。这个
 *低级驱动程序（RT文件夹）都在C中，其他所有的东西都是C++。
 *因为这个文件包含在C和C++中，所以它不能包含C++类型
 *别名（“使用”）、命名空间或模板。
 */

#ifndef PROJECT_CTYPES_H
#define PROJECT_CTYPES_H

#include <stddef.h>  // for size_t
#include <stdint.h>

// stdint默认整数类型的简短版本
typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

#endif  // PROJECT_CTYPES_H
