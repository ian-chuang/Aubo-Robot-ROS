#ifndef ERRORINFOSERVICE_H
#define ERRORINFOSERVICE_H

#include <map>
#include "internalMateType.h"

class ErrorInfoService
{

private:
    ErrorInfoService();


public:
    static std::string getErrDescByCode(aubo_robot_namespace::RobotErrorCode code);

public:
//    static int robotErrorInfoNotifyEventResolve(const std::string &source, std::string &dest);

//    static int ResolveRobotErrorInfo(const RobotErrorInfo &robotErrInfo, std::string &jsonText);


private:
    static void initErrInfoDescMaping();

private:
    static bool S_isInitInfoDescMaping;     //映射是否被初始化

    static std::map<aubo_robot_namespace::RobotErrorCode,std::string> S_errInfoDescMaping;
};

#endif // ERRORINFOSERVICE_H
