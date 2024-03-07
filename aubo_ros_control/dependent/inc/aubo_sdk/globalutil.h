#ifndef GLOBALUTIL_H
#define GLOBALUTIL_H

#include <math.h>
#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

#include <string>
#include "trace/robotlogprint.h"

namespace  aubo_robot_logtrace
{
enum LOG_LEVEL {LL_INFO=0, LL_DEBUG, LL_WARN, LL_ERROR, LL_FATAL};

#define W_INFO(fmt,...) do {				\
    if (RobotLogPrint::getRobotLogPrintPtr() != NULL)	\
    {                                       \
        RobotLogPrint::getRobotLogPrintPtr()->printTrace(0, fmt, ##__VA_ARGS__); \
    }                                       \
} while (0)

#define W_DEBUG(fmt,...) do {				\
    if (RobotLogPrint::getRobotLogPrintPtr() != NULL)	\
    {                                       \
        RobotLogPrint::getRobotLogPrintPtr()->printTrace(1, fmt, ##__VA_ARGS__); \
    }                                       \
} while (0)

#define W_WARN(fmt,...) do {				\
    if (RobotLogPrint::getRobotLogPrintPtr() != NULL)	\
    {                                       \
        RobotLogPrint::getRobotLogPrintPtr()->printTrace(2, fmt, ##__VA_ARGS__); \
    }                                       \
} while (0)

#define W_ERROR(fmt,...) do {				\
    if (RobotLogPrint::getRobotLogPrintPtr() != NULL)	\
    {                                       \
        RobotLogPrint::getRobotLogPrintPtr()->printTrace(3, fmt, ##__VA_ARGS__); \
    }                                       \
} while (0)

#define W_FATAL(fmt,...) do {				\
    if (RobotLogPrint::getRobotLogPrintPtr() != NULL)	\
    {                                       \
        RobotLogPrint::getRobotLogPrintPtr()->printTrace(4, fmt, ##__VA_ARGS__); \
    }                                       \
} while (0)

}


class commonUtil
{

public:
    //延时函数
    static void delay(int seconds, int mSeconds);

public:
    //去除开头和末尾的空格
    static std::string trim(std::string str);

    //去除多余的空格
    static std::string removeSurplusSpaces(const std::string &s);

public:
    //二阶低通滤波 初始化
    static void iir_low_filter_Init(double F0,double Fs,double Ly,double Coe[5]);

    //二阶低通滤波运行
    static void iir_low_filter_run(double *dat_in,double *dat_o,double *coe,int num);

};



#endif // GLOBALUTIL_H
