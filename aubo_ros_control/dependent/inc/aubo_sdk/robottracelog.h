#ifndef ROBOT_TRACELOG_H
#define ROBOT_TRACELOG_H


#if defined(__WIN32__) || defined (WIN32) || defined(NO_USE_LOG4)
#else

#include <log4cplus/loggingmacros.h>
#include <log4cplus/configurator.h>
#include <log4cplus/helpers/loglog.h>
#include <log4cplus/helpers/stringhelper.h>
#include <log4cplus/loggingmacros.h>

using namespace log4cplus;
using namespace log4cplus::helpers;

#endif

using namespace std;

enum LOG_LEVEL {LL_INFO=0, LL_DEBUG, LL_WARN, LL_ERROR, LL_FATAL};

class RobotTraceLog
{
public:
    //实例化
    static RobotTraceLog *instance();

    //构造函数
    RobotTraceLog(const char *doConfig);

public:
    bool initTraceLog(const char *doConfig);
    void printTrace(LOG_LEVEL level, const char *buf);

private:
#if defined(__WIN32__) || defined (WIN32) || defined(NO_USE_LOG4)
#else
    Logger m_traceLogger;
#endif
};

#endif // TRACELOG_H
