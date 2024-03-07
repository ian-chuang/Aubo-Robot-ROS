#ifndef ROBOTIOSERVICE_H
#define ROBOTIOSERVICE_H

#include <map>
#include <set>
#include <vector>
#include <string>

#include "AuboRobotMetaType.h"


class RobotControlServices;
class RobotIoService
{
public:
    RobotIoService(RobotControlServices *p);

public:      //接口板接口功能

    /** 获取接口板的配置 **/
    int  getBoardIOConfig(const std::vector<aubo_robot_namespace::RobotIoType> &ioType, std::vector<aubo_robot_namespace::RobotIoDesc> &configVector);

    /** 获取接口板的状态 **/
    int getBoardIOStatus(const std::vector<aubo_robot_namespace::RobotIoType>  ioType, std::vector<aubo_robot_namespace::RobotIoDesc> &statusVector);

    /** 通过名称获取接口板指定IO的状态 **/
    int getBoardIOStatus(aubo_robot_namespace::RobotIoType type, std::string name, double &value);

    /** 通过地址获取接口板指定IO的状态 **/
    int getBoardIOStatus(aubo_robot_namespace::RobotIoType type, int    addr,      double &value);

    /** 通过名称设置接口板指定IO的状态 **/
    int setBoardIOStatus(aubo_robot_namespace::RobotIoType type, std::string name, double value);

    /** 通过地址设置接口板指定IO的状态 **/
    int  setBoardIOStatus(aubo_robot_namespace::RobotIoType type, int    addr,      double value);


public:      //工具端接口功能

    /** 设置工具端电源电压类型 **/
    int setToolPowerVoltageType(aubo_robot_namespace::ToolPowerType type);

    /** 获取工具端电源电压类型 **/
    int getToolPowerVoltageType(aubo_robot_namespace::ToolPowerType &type);

    /** 设置工具端数字量类型  RobotToolDI或者RobotToolDO **/
    int setToolDigitalIOType(aubo_robot_namespace::ToolDigitalIOAddr addr, aubo_robot_namespace::ToolIOType type);

    /** 获取工具端电源电压状态 **/
    int getToolPowerVoltageStatus(double &value);

    /**
     * @brief setToolPowerTypeAndDigitalIOType
     *        IO配置:配置工具端IO的电源电压类型和IO类型
     */
    int setToolPowerTypeAndDigitalIOType(aubo_robot_namespace::ToolPowerType type,
                                         aubo_robot_namespace::ToolIOType io0,
                                         aubo_robot_namespace::ToolIOType io1,
                                         aubo_robot_namespace::ToolIOType io2,
                                         aubo_robot_namespace::ToolIOType io3);

    /** 获取工具端所有数字量IO状态 **/
    int getAllToolDigitalIOStatus(std::vector<aubo_robot_namespace::RobotIoDesc> &statusVector);

    /** 获取工具端所有AI状态  **/
    int getAllToolAIStatus(std::vector<aubo_robot_namespace::RobotIoDesc> &statusVector);

    /** 通过名称获取工具端IO的状态 **/
    int getToolIoStatusByName(std::string name, double &value);

    /** 设置工具端DO的状态 通过地址 **/
    int setToolDOStatus(aubo_robot_namespace::ToolDigitalIOAddr addr,  aubo_robot_namespace::IO_STATUS value);

    /** 设置工具端DO的状态 通过名称 **/
    int setToolDOStatus(std::string name, aubo_robot_namespace::IO_STATUS value);




private:

    /** 初始化RobotIoDesc 结构体 ***/
    void initRobotIoDesc(aubo_robot_namespace::RobotIoDesc &robotIoDesc, const std::string &ioId, aubo_robot_namespace::RobotIoType ioType,
                         const std::string &ioName, int         ioAddr, double      ioValue);

    /** 初始化接口板IO的配置 **/
    void initBoardIoConfig();

    /** 初始化工具端IO的配置 **/
    void initToolIoConfig();

private:
    std::set<int>           m_boardUserDIAddrSet;            //用户DI的地址集合
    std::set<int>           m_boardUserDOAddrSet;            //用户DO的地址集合
    std::set<int>           m_boardUserAIAddrSet;            //用户AI的地址集合
    std::set<int>           m_boardUserAOAddrSet;            //用户AO的地址集合
    std::set<std::string>   m_boardUserDINameSet;            //用户DI的名称集合
    std::set<std::string>   m_boardUserDONameSet;            //用户DO的名称集合
    std::set<std::string>   m_boardUserAINameSet;            //用户AI的名称集合
    std::set<std::string>   m_boardUserAONameSet;            //用户AO的名称集合

    std::set<int>           m_boardControllerDIAddrSet;      //Controler DI的地址集合
    std::set<int>           m_boardControllerDOAddrSet;      //Controler DO的地址集合
    std::set<int>           m_boardControllerAIAddrSet;      //Controler AI的地址集合
    std::set<int>           m_boardControllerAOAddrSet;      //Controler AO的地址集合
    std::set<std::string>   m_boardControllerDINameSet;      //Controler DI的名称集合
    std::set<std::string>   m_boardControllerDONameSet;      //Controler DO的名称集合
    std::set<std::string>   m_boardControllerAINameSet;      //Controler AI的名称集合
    std::set<std::string>   m_boardControllerAONameSet;      //Controler AO的名称集合

    std::set<int>           m_boardIoTypeSet;                //接口板IO类型集合
    std::map<aubo_robot_namespace::RobotIoType, std::map<std::string, int> > m_boardIOMaping;             //接口板IO完整映射
    std::map<aubo_robot_namespace::RobotIoType, std::set<int> >              m_boardIOTypeAddrSetMaping;  //接口板IO 类型-地址集合 完整映射
    std::map<aubo_robot_namespace::RobotIoType, std::set<std::string> >      m_boardIOTypeNameSetMaping;  //接口板IO 类型-名称集合 完整映射

    std::map<std::string, int>       m_toolPowerMaping;
    std::map<std::string, int>       m_toolDiagnosisIONameAddrMaping;
    std::map<int, std::string>       m_toolDiagnosisIOAddrNameMaping;
    std::map<std::string, int>       m_toolAIMaping;

    std::set<int>                    m_toolIoTypeSet;             //工具端IO类型集合
    std::set<int>                    m_toolDiagnosisIOAddrSet;

    RobotControlServices            *m_robotBaseService;          //机械臂基础服务
};




#endif // ROBOTIOSERVICE_H
