#ifndef _common_types
#define _common_types
#include <stdint.h>
#include <eigen3/Eigen/Dense>

//添加此行将添加打印语句和健全性检查，这些语句太慢，无法实时使用。
//#define K_DEBUG

typedef double dbl;
typedef float flt;

//尽可能使用浮点类型
typedef float fpt;

//与MATLAB接口时使用的浮点型
typedef double mfp;
typedef int mint;

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;
typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;
typedef int64_t  s64;

#endif
