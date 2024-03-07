#ifndef ROBOTLOGPRINT_H
#define ROBOTLOGPRINT_H

#include "pthread.h"

class RobotLogPrint
{
    enum{ MAX_BUF = 2048 };
public:
    typedef void (*LogPrintCallback) (int logLevel, const char *str, void *arg);

public:
    RobotLogPrint();

    void   printTrace(int level, const char *format, ...);

    void   registerRobotLogPrintCallback(LogPrintCallback callback, void *arg);

    static RobotLogPrint *getRobotLogPrintPtr();

private:
    char m_dataBuf[MAX_BUF];

    pthread_mutex_t        m_logPrintMutex;

    LogPrintCallback       m_logPrintCallback;

    void                  *m_arg;

    static RobotLogPrint*  s_robotLogPrintPtr;
};

#endif // ROBOTLOGPRINT_H
