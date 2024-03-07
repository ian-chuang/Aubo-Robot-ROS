#ifndef ROBOTBASESERVICES_H
#define ROBOTBASESERVICES_H

class RobotBaseServices
{
public:
    RobotBaseServices();


private:
    /** 处理响应：处理来自服务器的所有响应　**/
    virtual  void responseProcess(int socketFd, const char *buff, int size);

    /** 请求-响应模式(一问一答) **/
    int   sendRequestAndWaitResponse  (const RobotRequest &request, RobotResponse &response);

    /** 发送请求并判断是否正确响应，不提取数据信息 **/
    int   sendRequestCheckResponseResult (const RobotRequest &robotRequest);

    /** 发送请求并判断是否正确响应 提取数据信息 **/
    int   sendRequestGetResponseResult(const RobotRequest &robotRequest, RobotResponse &robotResponse);



private:
    int  requestServiceOnlyCheckSendResultMode(RobotCommandType robotCommandType, void *protobufTextPtr, int protobufTextLength );

    int  requestServiceGetResponseContentMode (RobotCommandType robotCommandType, void *protobufTextPtr, int protobufTextLength, RobotResponse &response);





private:

    RobotResponse       *m_robotCommandResponseSet;   //命令响应状态集合

    pthread_mutex_t     m_robotCommandMutexSet[RobotCommandCount];   //命令响应状态集合


    aubo_robot_namespace::JointStatus   m_jointStatus[aubo_robot_namespace::ARM_DOF];

    aubo_robot_namespace::ROBOT_SERVICE_STATE m_robotStartupDoneResult;


    void                              *m_realTimeJointStatusCallbackArg;
    RealTimeJointStatusCallback        m_realTimeJointStatusCallback;

    void                              *m_realTimeRoadPointCallbackArg;
    RealTimeRoadPointCallback          m_realTimeRoadPointCallback;

    void                              *m_robotEventCallbackArg;
    RobotEventCallback                 m_robotEventCallback;

    void                              *m_robotEndSpeedCallbackArg;
    RealTimeEndSpeedCallback           m_robotEndSpeedCallback;

    bool                               m_robotMoveIsAtTrackTarget;

    bool                               m_robotMoveIsStop;

    pthread_cond_t      m_startupDoneConditon;
    pthread_mutex_t     m_startupDoneMutex;

    pthread_cond_t      m_shutDownDoneConditon;
    pthread_mutex_t     m_shutDownDoneMutex;

    pthread_cond_t      m_robotHandShakeFinishConditon;
    pthread_mutex_t     m_robotHandShakeFinishMutex;
    bool                m_robotHandShakeSuccFlag;

public:

    pthread_cond_t      m_atTrackTargetPosConditon;
    pthread_mutex_t     m_atTrackTargetPosMutex;

};

#endif // ROBOTBASESERVICES_H








#ifndef ROBOTCONTROLSERVICES_H
#define ROBOTCONTROLSERVICES_H

#include  <vector>
#include  <iostream>

#include "robotcommunicationclient.h"
#include "robotcommand.h"
#include "internalMateType.h"


class RobotControlServices : public RobotCommunicationClient
{
    enum {WAIT_RESPONSE_MAX_TIME_USEC = 800*1000};
    enum {SCAN_RESPONSE_TIME_USEC     = 200};

public:
    RobotControlServices();


public:

    /** 处理响应：处理来自服务器的所有响应　**/
    virtual  void responseProcess(int socketFd, const char *buff, int size);

    /** 请求-响应模式(一问一答) **/
    int   sendRequestWaitResponse  (const RobotRequest &requestInfo, RobotResponse &responseInfo);

    /** 请求不等待响应模式 **/
    int   sendRequestNoWaitResponse(const RobotRequest &requestInfo);

    /** 发送请求并判断是否正确响应，不提取数据信息 **/
    int   sendRequestCheckResponseResult (const RobotRequest &robotRequest);

    /** 发送请求并判断是否正确响应 提取数据信息 **/
    int   sendRequestGetResponseResult(const RobotRequest &robotRequest, RobotResponse &robotResponse);

    aubo_robot_namespace::RobotErrorCode getErrCodeByServerResponse(int code);



public:

    //发送无正文命令请求
    int sendNoTextCommandRequest(RobotCommandType cmdType);


    /**************************************************************************************************************************
     ************************************************业务处理部分****************************************************************
     **************************************************************************************************************************/

public:   /** 系统接口 **/

    //业务接口:登录服务
    int  loginService(const char *host, int port, const std::string &userName, const std::string &possword);

    //业务接口:退出登录服务
    int  logoutService();

    /** 机械臂握手 **/
    int robotHandShake();

    int robotHandShakeService(bool isBlock);


public:  /** 状态推送接口 **/

    //业务接口:设置是否允许实时关节状态推送
    int setRealTimeJointStatusPush(bool enable);

    //业务接口:设置是否允许实时关节角信息推送
    int setRealTimeJointAnglePush(bool enable);

    //业务接口:设置是否允许实时末端速度推送
    int setRealTimeEndSpeedPush(bool enable);

    //业务接口:处理服务器推送过来的实时关节状态信息
    void realTimeJointStatusService(RobotResponse &response);

    //业务接口:处理服务器推送过来的实时关节角信息
    void realTimeJointAngleService(RobotResponse &response);

    //业务接口:处理服务器推送过来的实时末端速度信息
    void realTimeEndSpeedService(RobotResponse &response);

    //业务接口:处理服务器推送的事件通知
    void realTimeRobotEventInfoService(RobotResponse &response);

    /** 获取当前关节角信息　**/
    int getCurrentJointAngle(aubo_robot_namespace::JointParam &jointAngle);



public:
    int startupOfflineExcitTrajService(const char *trackFile, aubo_robot_namespace::Robot_Dyn_identify_traj type, int subtype);

    int getDynIdentifyResultsService(std::vector<int> &paramVector);



public:  /** 机械臂相关参数设置与获取的接口  **/

    int getRobotJointStatus(aubo_robot_namespace::JointStatus *jointStatus, int size);

    //业务接口:获取TCP参数
    int  getTcpParamService(RobotTcpParam &tcpParam);

    //业务接口:设置TCP参数
    int  setTcpParamService(const RobotTcpParam &tcpParam);

    //业务接口:设置工具的动力学参数
    int  setToolDynamicsParamService(const aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);

    //业务接口:获取工具的动力学参数
    int  getToolDynamicsParamService(aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);

    //业务接口:设置工具的运动学参数
    int  setToolKinematicsParamService(const aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);

    //业务接口:获取工具的运动学参数
    int  getToolKinematicsParamService(aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);


    //业务接口:获取重力分量
    int getRobotGravityComponent(aubo_robot_namespace::RobotGravityComponent &gravityComponent);

    //业务接口: 获取当前碰撞等级
    int getRobotCollisionCurrentService(RobotCollisionCurrent &collisionCurrent);

    //业务接口: 设置碰撞等级
    int  setRobotCollisionClassService(int grade);

    //业务接口:获取设备信息
    int getRobotDevInfoService(aubo_robot_namespace::RobotDevInfo &devInfo);

    //业务接口:获取当前机械臂模式   仿真或真实
    int  getRobotWorkModeService(aubo_robot_namespace::RobotWorkMode &mode);

    //业务接口:设置当前机械臂模式   仿真或真实
    int setRobotWorkModeService(aubo_robot_namespace::RobotWorkMode &mode);

    //业务接口: 获取机械臂安全配置
    int  getRobotSafetyConfig(aubo_robot_namespace::RobotSafetyConfig &safetyConfig);

    //业务接口: 获取机械臂安全配置
    int  setRobotSafetyConfig(const aubo_robot_namespace::RobotSafetyConfig &safetyConfig);

    //业务接口: 获取机械臂安全状态
    int  getOrpeSafetyStatus(aubo_robot_namespace::OrpeSafetyStatus &safetyStatus);

    int  getRobotCurrentStateService(aubo_robot_namespace::RobotState &state);

    //碰撞恢复
    int  collisionRecover();

    int  getIsRealRobotExist(bool &value);

    int  getRobotDiagnosisInfo(aubo_robot_namespace::RobotDiagnosis &robotDiagnosisInfo);

    //设置关节碰撞补偿（范围0.00~0.51度）
    int  setRobotJointOffsetService(aubo_robot_namespace::RobotJointOffset &jointOffset);


public:  /** 机械臂控制接口 **/

    int  robotControlService(const aubo_robot_namespace::RobotControlCommand cmd);

    int  armPowerControlService(bool value);

    int  rootReleaseBrake();


    int  robotStartupService  (const RobotTcpParam &tcpParam, uint8 collisionClass,
                               bool readPose, bool staticCollisionDetect, int maxAcc);

    int  robotStartup(const RobotTcpParam &tcpParam, uint8 collisionClass,
                      bool readPose, bool staticCollisionDetect, int maxAcc, aubo_robot_namespace::ROBOT_SERVICE_STATE &result, bool IsBolck = true);


    int  robotShutdownService();

    int  robotShutdown(bool IsBolck = true);


public:  /** 安全IO **/

    int setRobotAtOriginPoseService();


    int safeIoAboutCommunication(RobotCommandType commandType, const std::vector<int> &paramVeror);

    /**
     * @brief 通知接口板上位机暂停状态
     * @param data
     * @return
     */
    int setRobotOrpePause(uint8 data);

    /**
     * @brief 通知接口板上位机停止状态
     * @param data
     * @return
     */
    int setRobotOrpeStop(uint8 data);

    /**
     * @brief 通知接口板上位机错误
     * @param 16个字节的错误数据，每个错误占一个bit
     * @return
     */
    int setRobotOrpeError(uint8 data[], int len);

    /**
     * @brief 解除系统紧急停止输出信号 0-无动作 1-解除
     * @param data
     * @return
     */
    int clearSystemEmergencyStop(uint8 data);

    /**
     * @brief 解除缩减模式错误 0-无效 1-解除
     * @param data
     * @return
     */
    int clearReducedModeError(uint8 data);

    /**
     * @brief 防护重置成功 0-无动作 1-解除
     * @param data
     * @return
     */
    int robotSafetyguardResetSucc(uint8 data);



public:  /**  关于接口板IO的基础接口 **/

    int  getInterfaceBoardAllDIStatusService(std::vector<aubo_robot_namespace::RobotDiagnosisIODesc> &diagnosisIOStatusVector); //业务接口: 获取接口板数字量输入的数据

    int  getInterfaceBoardAllDOStatusService(std::vector<aubo_robot_namespace::RobotDiagnosisIODesc> &diagnosisIOStatusVector); //业务接口: 获取接口板数字量输出的数据

    int  getInterfaceBoardAllAIStatusService(std::vector<aubo_robot_namespace::RobotAnalogIODesc> &analogIOStatusVector);       //业务接口: 获取接口板模拟量输入的数据

    int  getInterfaceBoardAllAOStatusService(std::vector<aubo_robot_namespace::RobotAnalogIODesc> &analogIOStatusVector);       //业务接口: 获取接口板模拟量输出的数据

    int  setInterfaceBoardDOStatusService(int addr, aubo_robot_namespace::IO_STATUS status);   //业务接口:设置接口板DO的状态

    int  setInterfaceBoardAOStatusService(int addr, double status);                            //业务接口:设置接口板AO的状态


    int  getRobotDigitalIOStatusService(int commandType, int addr, int &status);               //业务接口: 获取接口板DIDO的状态

    int  getInterfaceBoardDIStatusService(int addr, aubo_robot_namespace::IO_STATUS &status);  //业务接口:获取接口板DI的状态

    int  getInterfaceBoardDOStatusService(int addr, aubo_robot_namespace::IO_STATUS &status);  //业务接口:获取接口板DO的状态


    int  getRobotAnalogIOStatusService(int commandType, int addr, double &status);             //业务接口: 获取接口板DIDO的状态

    int  getInterfaceBoardAIStatusService(int addr, double &status);                           //业务接口:获取接口板AI的状态

    int  getInterfaceBoardAOStatusService(int addr, double &status);                            //业务接口:获取接口板AO的状态



public:  /** 关于tool IO的基础接口 **/

    int  setToolPowerVoltageTypeService(aubo_robot_namespace::ToolPowerType value);

    int  getToolPowerVoltageTypeService(aubo_robot_namespace::ToolPowerType &type);

    int  setToolDigitalIOTypeService   (aubo_robot_namespace::ToolDigitalIOAddr addr, aubo_robot_namespace::ToolIOType type);

    int  getToolPowerVoltageStatusService   (double &status);

    int  getToolAllDigitalIOStatusService(std::vector<aubo_robot_namespace::RobotDiagnosisIODesc> &diagnosisIOStatusVector);

    int  getToolAllAIStatusService (std::vector<aubo_robot_namespace::RobotAnalogIODesc> &analogIOStatusVector);

    int  setToolDigitalIOStatusService(aubo_robot_namespace::ToolDigitalIOAddr addr,  aubo_robot_namespace::IO_STATUS value);

    int  getToolAllIOStatusService(aubo_robot_namespace::RobotToolAllIOStatus &toolAllIOStatus);



public:
    int  updateRobotBoardFirmwareService(aubo_robot_namespace::update_board_firmware_cmd cmd, const void *data, uint16 length);

    int  getBoardFirmwareUpdateResultService(bool &value);

    int  getRobotEthernetDeviceNameService(std::string &ethernetDeviceName);


public:

    void  getCurrentJointStatus(aubo_robot_namespace::JointStatus jointStatus[aubo_robot_namespace::ARM_DOF]);

    void  robotServiceRegisterRealTimeJointStatusCallbackService(RealTimeJointStatusCallback ptr, void  *arg);

    void  robotServiceRegisterRealTimeRoadPointCallbackService(RealTimeRoadPointCallback ptr, void  *arg);

    void  robotServiceRegisterRealTimeEndSpeedCallbackService(const RealTimeEndSpeedCallback ptr, void  *arg);

    void  robotServiceRegisterRobotEventInfoCallbackService(RobotEventCallback ptr, void  *arg);

    bool  getRobotMoveIsAtTrackTarget();

    bool  getRobotMoveIsStop();

    void  setRobotMoveAtTrackTargetFlag(bool flag);

    void  setRobotMoveStopFlag(bool flag);



public:
    int  requestService(RobotCommandType robotCommandType, void *protobufTextPtr, int protobufTextLength );



private:

    RobotResponse       *m_robotCommandResponseSet;   //命令响应状态集合

    pthread_mutex_t     m_robotCommandMutexSet[RobotCommandCount];   //命令响应状态集合


    aubo_robot_namespace::JointStatus   m_jointStatus[aubo_robot_namespace::ARM_DOF];

    aubo_robot_namespace::ROBOT_SERVICE_STATE m_robotStartupDoneResult;


    void                              *m_realTimeJointStatusCallbackArg;
    RealTimeJointStatusCallback        m_realTimeJointStatusCallback;

    void                              *m_realTimeRoadPointCallbackArg;
    RealTimeRoadPointCallback          m_realTimeRoadPointCallback;

    void                              *m_robotEventCallbackArg;
    RobotEventCallback                 m_robotEventCallback;

    void                              *m_robotEndSpeedCallbackArg;
    RealTimeEndSpeedCallback           m_robotEndSpeedCallback;

    bool                               m_robotMoveIsAtTrackTarget;

    bool                               m_robotMoveIsStop;

    pthread_cond_t      m_startupDoneConditon;
    pthread_mutex_t     m_startupDoneMutex;

    pthread_cond_t      m_shutDownDoneConditon;
    pthread_mutex_t     m_shutDownDoneMutex;

    pthread_cond_t      m_robotHandShakeFinishConditon;
    pthread_mutex_t     m_robotHandShakeFinishMutex;
    bool                m_robotHandShakeSuccFlag;

public:

    pthread_cond_t      m_atTrackTargetPosConditon;
    pthread_mutex_t     m_atTrackTargetPosMutex;
};




#endif // ROBOTCONTROLSERVICES_H
