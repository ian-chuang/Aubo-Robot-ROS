#ifndef ROBOTCONTROLSERVICES_H
#define ROBOTCONTROLSERVICES_H

#include  <vector>
#include  <queue>
#include  <iostream>

#include "internalMateType.h"
#include "robotcommunicationclient.h"


//#define EVENT_CALLBACK_LINKED_LIST_MODE


#define LL_ADD(item, list) do {				\
    item->prev = NULL;						\
    item->next = list;						\
    if (list != NULL) list->prev = item;	\
    list = item;							\
} while (0)

#define LL_REMOVE(item, list) do {			\
    if (item->prev != NULL) item->prev->next = item->next;	\
    if (item->next != NULL) item->next->prev = item->prev;	\
    if (list == item) list = item->next;					\
    item->prev = item->next = NULL;							\
} while (0)

class  AuboRobotEvent
{
public:
    AuboRobotEvent()
    {
        eventType = aubo_robot_namespace::robot_event_unknown;
        eventCode = 0;
        eventContent = "";
    }

    AuboRobotEvent(int  type, int  code, std::string  &content)
    {
        eventType = type;
        eventCode = code;
        eventContent = content;
    }

public:
    int          eventType;       //事件类型号
    int          eventCode;       //
    std::string  eventContent;    //事件内容
    AuboRobotEvent *next;
    AuboRobotEvent *prev;
};

class RobotEvent
{
public:
    //默认构造
    RobotEvent()
    {
        m_evnetCode    =  0;
        m_eventContent = "";
        m_eventType    =  aubo_robot_namespace::robot_event_unknown;
    }

    //构造
    RobotEvent(int  type, int  code, const std::string &content)
    {
        m_eventType    = type;
        m_evnetCode    = code;
        m_eventContent = content;
    }

    //拷贝构造
    RobotEvent(const RobotEvent &robotEvent)
    {
        this->m_eventType    = robotEvent.m_eventType;
        this->m_evnetCode    = robotEvent.m_evnetCode;
        this->m_eventContent = robotEvent.m_eventContent;
    }

    //等号重载
    RobotEvent &operator=(const RobotEvent &robotEvent)
    {
        if ( this == &robotEvent )
        {
           return *this;
        }

        this->m_eventType    = robotEvent.m_eventType;
        this->m_evnetCode    = robotEvent.m_evnetCode;
        this->m_eventContent = robotEvent.m_eventContent;

        return *this;
    }

    //析构函数
    ~RobotEvent()
    {

    }

public:
    int         m_eventType;
    int         m_evnetCode;
    std::string m_eventContent;
};


class RobotEventQueue
{
    enum {QUEUE_SIZE=4096};
public:
    RobotEventQueue()
    {

    }

    void init()
    {
        m_wIdx = 0;
        m_rIdx = 0;
    }

    bool pop(RobotEvent &robotEvent)
    {
        bool ret = false;
        if (m_rIdx != m_wIdx)
        {
            robotEvent = m_queue[m_rIdx&(QUEUE_SIZE-1)];
            m_rIdx++;
            ret = true;
        }

        return ret;
    }

    bool push(const RobotEvent &robotEvent)
    {
        bool ret;

        if (m_wIdx != m_rIdx+QUEUE_SIZE)
        {
            m_queue[m_wIdx&(QUEUE_SIZE-1)] = robotEvent;
            m_wIdx++;
            ret = true;
        }
        else
        {
            ret = false;
        }

        return ret;
    }

    void clear()
    {
        m_rIdx = m_wIdx;
    }


    int  getUnpopSize()
    {
        return (m_wIdx-m_rIdx);
    }

    bool empty()
    {
        if(m_rIdx == m_wIdx)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

private:
    int        m_wIdx;
    int        m_rIdx;
    RobotEvent m_queue[QUEUE_SIZE];
};

class ScriptDoneInfo
{
public:
    ScriptDoneInfo()
    {
        m_label     ="unknow";
        m_eventType =  aubo_robot_namespace::robot_event_unknown;
        m_eventContent = "";
    }

    ScriptDoneInfo(std::string label, int eventType, const std::string &eventContent)
    {
        m_label        = label;
        m_eventType    = eventType;
        m_eventContent = eventContent;
    }

    //拷贝构造
    ScriptDoneInfo(const ScriptDoneInfo &scriptDoneInfo)
    {
        this->m_label        = scriptDoneInfo.m_label;
        this->m_eventType    = scriptDoneInfo.m_eventType;
        this->m_eventContent = scriptDoneInfo.m_eventContent;
    }

    //等号重载
    ScriptDoneInfo &operator=(const ScriptDoneInfo &scriptDoneInfo)
    {
        if ( this == &scriptDoneInfo )
        {
           return *this;
        }

        this->m_label        = scriptDoneInfo.m_label;
        this->m_eventType    = scriptDoneInfo.m_eventType;
        this->m_eventContent = scriptDoneInfo.m_eventContent;

        return *this;
    }

    //析构函数
    ~ScriptDoneInfo()
    {

    }

public:
    std::string  m_label;
    int          m_eventType;
    std::string  m_eventContent;
};



class RobotControlServices : public RobotCommunicationClient
{
    enum {WAIT_RESPONSE_MAX_TIME_MSEC = 5000};                               //等待消息超时事件   单位ms
    enum {WAIT_RESPONSE_MAX_TIME_USEC = WAIT_RESPONSE_MAX_TIME_MSEC*1000};   //等待消息超时事件   单位us
    enum {SCAN_RESPONSE_TIME_USEC     = 200};

    enum {PAUSE_DONE_EVNET_TIMED_LOCK_MSEC = 450};


public:
    RobotControlServices();

    ~RobotControlServices();

public:

    /** 处理响应：处理来自服务器的所有响应　**/
    virtual  void responseProcess(int socketFd, const char *requestContentPtr, int requestContentLength);

    virtual  void disconnectProcess();

    virtual  int  heartbeatService();

    /** 请求-响应模式(一问一答) **/
    int   sendRequestWaitResponse  (const CommunicationRequest &requestInfo, CommunicationResponse &responseInfo);

    /** 根据服务器返回的错误码获取对外的错误码　**/
    aubo_robot_namespace::RobotErrorCode getErrCodeByServerResponse(int code);

    /** 发送请求并判断结果　不提取数据信息**/
    int  requestServiceOnlyCheckSendResultMode(RobotCommandType robotCommandType, void *protobufTextPtr, int protobufTextLength );

    /** 发送请求并提取数据信息 **/
    int  requestServiceGetResponseContentMode (RobotCommandType robotCommandType, void *protobufTextPtr, int protobufTextLength, CommunicationResponse &robotResponse);


public:  /** 事件处理　**/

    /** 事件处理:处理服务器推送过来的实时关节状态信息　**/
    void realTimeJointStatusService(CommunicationResponse &response);

    /** 事件处理:处理服务器推送过来的实时关节角信息　 **/
    void realTimeJointAngleService(CommunicationResponse &response);

    /** 事件处理:处理服务器推送过来的实时末端速度信息**/
    void realTimeEndSpeedService(CommunicationResponse &response);

    /** 事件处理:处理服务器推送过来的实时Tcp末端速度信息**/
    void realTimeTcpEndSpeedService(CommunicationResponse &response);

    /** 事件处理:处理服务器推送过来的实时movep num **/
    void realTimeMovepProgressNotifyService(CommunicationResponse &response);

    /** 事件处理:处理服务器推送的事件通知　**/
    void realTimeRobotEventResponseService(CommunicationResponse &response);

    void robotRealTimeEventProcess(aubo_robot_namespace::RobotEventInfo robotEvent);

    void pushEventToList(const aubo_robot_namespace::RobotEventInfo &robotEvent);

    static void   *robotEventProcessThread (void *args);     //处理实时事件


    /**************************************************************************************************************************
     ************************************************业务处理部分****************************************************************
     **************************************************************************************************************************/
public:

    /** 业务接口:登录服务 **/
    int  loginService(const char *host, int port, const std::string &userName, const std::string &possword, aubo_robot_namespace::RobotType &robotType, aubo_robot_namespace::RobotDhPara &robotDhPara);

    /** 业务接口:退出登录服务 **/
    int  logoutService();

    /** 内部接口:获取算法层参数 **/
    int  getJoint6Rotate360EnableFlag(bool &joint6Rot360Enable);

    /** 内部接口:获取算法层参数 **/
    int  getJoint1Rotate360EnableFlag(bool &joint1Rot360Enable);

    /** 机械臂握手 **/
    int  robotHandShakeService(bool isBlock);

    /** 机械臂初始化　**/
    int  robotStartupService(const RobotTcpParam &tcpParam, uint8 collisionClass,
                             bool readPose, bool staticCollisionDetect, int maxAcc,
                             aubo_robot_namespace::ROBOT_SERVICE_STATE &result, bool IsBolck = true);

    /** 机械臂反初始化　**/
    int  robotShutdownService(bool IsBolck = true);

public:/** 运动控制　**/
    int   robotMoveControlService(CommunicationMateType::CommunicationCommandType cmdType);

public:

    /** 业务接口:设置是否允许实时关节状态推送  **/
    int  setRealTimeJointStatusPush(bool enable);

    /** 业务接口:设置是否允许实时关节角信息推送 **/
    int  setRealTimeJointAnglePush(bool enable);

    /** 业务接口:设置是否允许实时末端速度推送 **/
    int  setRealTimeEndSpeedPush(bool enable);

    /** 业务接口:设置是否允许实时Tcp末端速度推送 **/
    int  setRealTimeTcpEndSpeedPush(bool enable);

    /** 业务接口:关于机械臂末端工具动力学参数的设置和获取 **/
    int  getTcpParamService(RobotTcpParam &tcpParam);

    int  setTcpParamService(const RobotTcpParam &tcpParam);

    int  getToolDynamicsParamService(aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);

    int  setToolDynamicsParamService(const aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);

    int  setToolKinematicsParamService(const aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);

    /** 业务接口: 机械臂碰撞等级的设置和获取 **/
    int  setRobotCollisionClassService(int grade);

    int  getRobotCollisionCurrentService(RobotCollisionCurrent &collisionCurrent);

    /** 业务接口:机械臂工作模式(仿真或真实)的设置和获取 **/
    int  getRobotWorkModeService (aubo_robot_namespace::RobotWorkMode &mode);

    int  setRobotWorkModeService (aubo_robot_namespace::RobotWorkMode  mode);

    /** 在工程运行的时候使能/失能力控模式 **/
    int  setEnableForceTeachWhenProjectIsRunning (bool  enable);


    /** 业务接口:获取设备信息 devInfo **/
    int  getRobotDevInfoService  (aubo_robot_namespace::RobotDevInfo  &devInfo);

    /** 业务接口:获取重力分量 **/
    int  getRobotGravityComponent(aubo_robot_namespace::RobotGravityComponent &gravityComponent);

    /** 业务接口:获取机械臂关节状态 **/
    int  getRobotJointStatus(aubo_robot_namespace::JointStatus *jointStatus, int size);

    /** 业务接口:获取机械臂关节类型参数 **/
    int  getRobotJointTypeParam(aubo_robot_namespace::JointTypeParameter *jointTypeParam, int size);

    /** 获取当前关节角信息　**/
    int  getCurrentJointAngle(aubo_robot_namespace::JointParam &jointAngle);

    int  getForceSensorData(aubo_robot_namespace::ForceSensorData &data);

    /** 根据提供的参考点得到补偿后的点 **/
    int  getCorrectedWaypoint(const aubo_robot_namespace::wayPoint_S &source, aubo_robot_namespace::wayPoint_S &target);

    /** 从算法控制层获取当前的路点信息 **/
    int  getJointAngleFromController(aubo_robot_namespace::JointParam &jointAngle);

    int getTargetWaypointFromController(aubo_robot_namespace::wayPoint_S &target_waypoint);

    /** 业务接口:获取真实机械臂是否存在 **/
    int  getIsRealRobotExist(bool &value);

    /** 业务接口:碰撞恢复 **/
    int  collisionRecover();

    /** 业务接口:关节超限恢复 **/
    int  robotJointPosRecover(bool confirm);

    /** 业务接口:获取机械臂的诊断信息 **/
    int  getRobotDiagnosisInfo(aubo_robot_namespace::RobotDiagnosis &robotDiagnosisInfo);

    /** 业务接口:获取机械臂的当前的状态 **/
    int  getRobotCurrentStateService(aubo_robot_namespace::RobotState &state);

    /** 业务接口:设置关节碰撞补偿（范围0.00~0.51度） **/
    int  setRobotJointOffsetService(aubo_robot_namespace::RobotJointOffset &jointOffset);

    /** 业务接口: 获取机械臂安全配置 **/
    int  getRobotSafetyConfig(aubo_robot_namespace::RobotSafetyConfig &safetyConfig);

    /** 业务接口: 设置机械臂安全配置 **/
    int  setRobotSafetyConfig(const aubo_robot_namespace::RobotSafetyConfig &safetyConfig);

    /** 业务接口: 获取机械臂安全状态 **/
    int  getOrpeSafetyStatus(aubo_robot_namespace::OrpeSafetyStatus &safetyStatus);
    /** 解析机器人服务器版本号，将版本号转换成int类型**/
    int serverVersionToInt(int versionCode);

public:
    int startupOfflineExcitTrajService(const char *trackFile, aubo_robot_namespace::Robot_Dyn_identify_traj type, int subtype);

    int getDynIdentifyResultsService(std::vector<int> &paramVector);


public:  /** 机械臂控制接口 **/

    int  robotControlService(const aubo_robot_namespace::RobotControlCommand cmd);

    int  armPowerControlService(bool value);

    int  rootReleaseBrake();


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

    int  setInterfaceBoardDOStatusService(int addr, aubo_robot_namespace::IO_STATUS status);     //业务接口:设置接口板DO的状态

    int  setInterfaceBoardAOStatusService(int addr, double status);                              //业务接口:设置接口板AO的状态


    int  getRobotDigitalIOStatusService(RobotCommandType commandType, int addr, int &status);    //业务接口: 获取接口板DIDO的状态

    int  getInterfaceBoardDIStatusService(int addr, aubo_robot_namespace::IO_STATUS &status);    //业务接口:获取接口板DI的状态

    int  getInterfaceBoardDOStatusService(int addr, aubo_robot_namespace::IO_STATUS &status);    //业务接口:获取接口板DO的状态


    int  getRobotAnalogIOStatusService(RobotCommandType commandType, int addr, double &status);  //业务接口: 获取接口板DIDO的状态

    int  getInterfaceBoardAIStatusService(int addr, double &status);                             //业务接口:获取接口板AI的状态

    int  getInterfaceBoardAOStatusService(int addr, double &status);                             //业务接口:获取接口板AO的状态



public:  /** 关于tool IO的基础接口 　１路电源可配置＋４个数字量可配置＋２路模拟量输入　　**/

    int  setToolPowerVoltageTypeService(aubo_robot_namespace::ToolPowerType value);

    int  getToolPowerVoltageTypeService(aubo_robot_namespace::ToolPowerType &type);

    int  getToolPowerVoltageStatusService   (double &status);

    /**
     * @brief setToolPowerTypeAndDigitalIOTypeService
     *        IO配置:配置工具端IO的电源电压类型和IO类型
     */
    int  setToolPowerTypeAndDigitalIOTypeService(aubo_robot_namespace::ToolPowerType type,
                                         aubo_robot_namespace::ToolIOType io0,
                                         aubo_robot_namespace::ToolIOType io1,
                                         aubo_robot_namespace::ToolIOType io2,
                                         aubo_robot_namespace::ToolIOType io3);

    int  setToolDigitalIOTypeService   (aubo_robot_namespace::ToolDigitalIOAddr addr, aubo_robot_namespace::ToolIOType type);

    int  getToolAllDigitalIOStatusService(std::vector<aubo_robot_namespace::RobotDiagnosisIODesc> &diagnosisIOStatusVector);

    int  getToolAllAIStatusService (std::vector<aubo_robot_namespace::RobotAnalogIODesc> &analogIOStatusVector);

    int  setToolDigitalIOStatusService(aubo_robot_namespace::ToolDigitalIOAddr addr,  aubo_robot_namespace::IO_STATUS value);

    int  getToolAllIOStatusService(RobotToolAllIOStatus &toolAllIOStatus);

    int setToolSerialConfigService(aubo_robot_namespace::ToolBusType type,
                            aubo_robot_namespace::ToolSerialBaudRate baud,
                            aubo_robot_namespace::ToolSerialStopBitNum stop,
                            aubo_robot_namespace::ToolSerialCheck check,
                            aubo_robot_namespace::ToolSerialCheckCmd check_cmd);

    int getToolSerialConfigService(aubo_robot_namespace::ToolBusType &type,
                            aubo_robot_namespace::ToolSerialBaudRate &baud,
                            aubo_robot_namespace::ToolSerialStopBitNum &stop,
                            aubo_robot_namespace::ToolSerialCheck &check,
                            aubo_robot_namespace::ToolSerialCheckCmd &check_cmd);



public:
    int  updateRobotBoardFirmwareService(aubo_robot_namespace::update_board_firmware_cmd cmd, const void *data, uint16 length);

    int  getBoardFirmwareUpdateResultService(bool &value);

    int  getRobotEthernetDeviceNameService(std::string &ethernetDeviceName);

    int  getServerVersionInfoService(std::string &versionInfo);

    //升级关节驱动器
    int  startUpdateJointFirmware(int jointId, std::string updateFilePath);
    int  getUpdateJointFirmwareResult(bool &isFinished, bool &isSucceed);
    int  setUpdateJointFirmwareFinished(int jointId);

    int getMovepSequenceNum(int &num);


public:  //关于驱动器PID参数
    int  getJointCommonData(aubo_robot_namespace::JointCommonData jointCommonDataArray[], int size);

    int  setJointParamService(int type, int jointID, int value);

    int  jointSaveDataFlashService(int jointID);

    int  setRobotBaseParameters(const aubo_robot_namespace::RobotBaseParameters &baseParameters);

    int  getRobotBaseParameters(aubo_robot_namespace::RobotBaseParameters &baseParameters);

    int  setRobotJointsParameter(const aubo_robot_namespace::RobotJointsParameter &jointsParameter);

    int  getRobotJointsParameter(aubo_robot_namespace::RobotJointsParameter &jointsParameter);

    int  refreshRobotArmParamter();


public:  //脚本管理
    int  scriptRunRequest(const std::string scriptPath, const std::string scriptLabel);

    int  scriptRunService(const std::string scriptPath, const std::string scriptLabel, bool IsBolck);

    void pushEventToScriptRunDoneEventQueue(ScriptDoneInfo scriptDoneInfo);

    ScriptDoneInfo frontToScriptRunDoneEventQueue();

    void clearScriptRunDoneEventQueue();

    bool isScriptRunDoneEventQueueEmpty();
    int registerPercentageEventRequest(const std::string scriptPath, const std::string percent_str);
    int RegisterPercentageEvent(double percent,const std::string scriptPath,bool IsBolck);
public:

    aubo_robot_namespace::RobotEventType  getMoveFinishEventType() const;

    void  setMoveFinishEventType(aubo_robot_namespace::RobotEventType eventType);

    void  pushEventToMoveFinishEventQueue(aubo_robot_namespace::RobotEventType eventType);

    aubo_robot_namespace::RobotEventType  frontToMoveFinishEventQueue();

    void  clearMoveFinishEventQueue();


    pthread_cond_t  *getRobotMoveAtTrackTargetPosConditonPtr() const;

    pthread_mutex_t *getRobotMoveAtTrackTargetPosMutexPtr() const;

    void  robotServiceRegisterRealTimeJointStatusCallbackService(RealTimeJointStatusCallback ptr, void  *arg);

    void  robotServiceRegisterRealTimeRoadPointCallbackService(RealTimeRoadPointCallback ptr, void  *arg);

    void  robotServiceRegisterRealTimeEndSpeedCallbackService(const RealTimeEndSpeedCallback ptr, void  *arg);

    void  robotServiceRegisterRobotEventInfoCallbackService(RobotEventCallback ptr, void  *arg);

    void  robotServiceRegisterMovepProgressNotifyCallbackService(RealTimeMovepStepNumNotifyCallback ptr, void  *arg);

    void  robotServiceRegisterRealTimeTcpEndSpeedCallbackService(const RealTimeTcpEndSpeedCallback ptr, void  *arg);

    int getVersionCode();

private:

    //创建用于存放响应的响应结合
    CommunicationResponse  *m_robotCommandResponseSet;

    //用于多线程同时调用接口　　线程安全
    pthread_mutex_t         m_robotCommandMutexSet[CommunicationMateType::CommunicationTypeCount];

    //利用条件变量实现startup接口的阻塞形式
    pthread_cond_t                            m_startupDoneConditon;
    pthread_mutex_t                           m_startupDoneMutex;
    aubo_robot_namespace::ROBOT_SERVICE_STATE m_robotStartupDoneResult;

    //利用条件变量实现ScriptRun的阻塞形式
    pthread_cond_t                            m_scriptRunDoneConditon;
    pthread_mutex_t                           m_scriptRunDoneMutex;
    std::queue<ScriptDoneInfo>                m_scriptRunDoneEventQueue;

    //利用条件变量实现shutDown接口的阻塞形式
    pthread_cond_t                            m_shutDownDoneConditon;
    pthread_mutex_t                           m_shutDownDoneMutex;

    //利用条件变量实现HandShake接口的阻塞形式
    pthread_cond_t                            m_robotHandShakeFinishConditon;
    pthread_mutex_t                           m_robotHandShakeFinishMutex;
    bool                                      m_robotHandShakeSuccFlag;

    //利用条件变量实现stop接口的阻塞形式
    pthread_cond_t                           m_robotMoveControlStopConditon;
    pthread_mutex_t                          m_robotMoveControlStopMutex;

    //利用条件变量实现pause接口的阻塞形式
    pthread_cond_t                           m_robotMoveControlPauseConditon;
    pthread_mutex_t                          m_robotMoveControlPauseMutex;

    //利用条件变量实现continue接口的阻塞形式
    pthread_cond_t                           m_robotMoveControlContinueConditon;
    pthread_mutex_t                          m_robotMoveControlContinueMutex;

    pthread_mutex_t                          m_robotMoveControlMutex;


    //处理实时事件的回调函数
    RobotEventCallback                        m_robotEventCallback;
    void                                     *m_robotEventCallbackArg;

    //处理实时末端速度的回调函数
    RealTimeEndSpeedCallback                  m_robotEndSpeedCallback;
    void                                     *m_robotEndSpeedCallbackArg;

    //处理实cp末端速度的回调函数
    RealTimeTcpEndSpeedCallback               m_robotTcpEndSpeedCallback;
    void                                     *m_robotTcpEndSpeedCallbackArg;

    //处理实时movep进度通知的回调函数
    RealTimeMovepStepNumNotifyCallback        m_movepProgressNotifyCallback;
    void                                     *m_movepProgressNotifyCallbackArg;

    //处理实时关节状态的回调函数
    RealTimeJointStatusCallback               m_realTimeJointStatusCallback;
    void                                     *m_realTimeJointStatusCallbackArg;

    //处理实时路点的回调函数
    RealTimeRoadPointCallback                 m_realTimeRoadPointCallback;
    void                                     *m_realTimeRoadPointCallbackArg;

    //利用条件变量实现MOVE接口的阻塞形式
    pthread_cond_t                           *m_robotMoveAtTrackTargetPosConditonPtr;
    pthread_mutex_t                          *m_robotMoveAtTrackTargetPosMutexPtr;


    //这个变量判断机械臂运动完成的成功与否
    aubo_robot_namespace::RobotEventType      m_moveFinishEventType;

    std::queue<aubo_robot_namespace::RobotEventType>    m_moveFinishEventQueue;

    pthread_t                                           m_threadId;                   //线程ID

    std::queue<aubo_robot_namespace::RobotEventInfo>    m_eventInfoQueue;

    pthread_mutex_t                                     m_eventInfoQueueMutex;

private:  //事件处理
    pthread_t           m_eventThrowThreadid;
    pthread_mutex_t     m_eventlistMtx;
    pthread_cond_t      m_eventlistCond;
    AuboRobotEvent     *m_eventlist;   //任务队列

    RobotEventQueue     m_robotEventQueue;   //事件队列
public:
    static int m_versionCode;
};




#endif // ROBOTCONTROLSERVICES_H
