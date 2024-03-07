#ifndef COMMUNICATIONMATETYPE_H
#define COMMUNICATIONMATETYPE_H

#include <map>
#include <string>
#include <stdint.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>


#ifndef  INVALID_SOCKET
#define  INVALID_SOCKET  (~0)           //无效的Socket
#endif


class CommunicationMateType
{
public:
    typedef enum
    {
        /** 系统相关　**/
        CommunicationType_Login,                           //命令类型之---系统登陆
        CommunicationType_Logout,                          //命令类型之---系统退出

        /** 事件　**/
        CommunicationType_Event,                           //命令类型之---事件

        /** 关节状态　**/
        CommunicationType_RealTimeJointStatus,             //命令类型之---实时关节状态推送
        CommunicationType_EnableRealTimeJointStatusPush,   //命令类型之---允许推送实时关节状态信息
        CommunicationType_DisableRealTimeJointStatusPush,  //命令类型之---禁止推送实时关节状态信息
        CommunicationType_GetRobotJointStatus,             //命令类型之---获取关节状态信息

        /** 实时路点　**/
        CommunicationType_RealTimeRobotJointAngle,         //命令类型之---实时关节角信息推送
        CommunicationType_EnableRealTimeJointAnglePush,    //命令类型之---允许推送实时关节角信息
        CommunicationType_DisableRealTimeJointAnglePush,   //命令类型之---禁止推送实时关节角信息
        CommunicationType_GetRobotJointAngle,              //命令类型之---获取关节角信息

        /** 末端速度　**/
        CommunicationType_RealTimeRobotEndSpeed,           //命令类型之---实时末端速度推送
        CommunicationType_EnableRealTimeEndSpeedPush,      //命令类型之---允许推送实时关节角信息
        CommunicationType_DisableRealTimeEndSpeedPush,     //命令类型之---禁止推送实时关节角信息

        /** 诊断信息**/
        CommunicationType_RealTimeRobotDiagnosis,          //命令类型之---实时诊断信息推送
        CommunicationType_GetRobotDiagnosis,               //命令类型之---获取诊断信息


        /** 机械臂控制相关的***/
        CommunicationType_RobotStartup,                    //命令类型之---机械臂启动
        CommunicationType_RobotShutdown,                   //命令类型之---机械臂关闭
        CommunicationType_RobotControl,                    //命令类型之---机械臂控制
        CommunicationType_CollisionRecover,                //命令类型之---碰撞恢复
        CommunicationType_SetRobotPowerStatus,             //命令类型之---设置机械臂电源状态
        CommunicationType_SetRobotReleaseBrake,            //命令类型之---松刹车


        /** 机械臂运动相关　**/
        CommunicationType_RobotMove,                       //命令类型之---机械臂运动
        CommunicationType_RobotTeachMoveStart,             //命令类型之---机械臂示教运动开始
        CommunicationType_RobotTeachMoveStop,              //命令类型之---机械臂示教运动停止
        CommunicationType_RobotMoveControl,                //命令类型之---机械臂运动控制    暂停,继续,停止
        CommunicationType_GetRobotCurrentState,            //命令类型之---机械臂当前状态

        /** 信息获取与设置　**/
        CommunicationType_SetRobotTcpParam,                //命令类型之---设置TCP中心
        CommunicationType_GetRobotTcpParam,                //命令类型之---获取TCP中心
        CommunicationType_SetToolDynamicsParam,            //命令类型之---设置工具动力学参数
        CommunicationType_GetToolDynamicsParam,            //命令类型之---获取工具动力学参数
        CommunicationType_SetToolKinematicsParam,          //命令类型之---设置工具运动学参数
        CommunicationType_GetToolKinematicsParam,          //命令类型之---获取工具运动学参数

        CommunicationType_SetRobotMode,                    //命令类型之---设置机械臂仿真模式/真实模式
        CommunicationType_GetRobotMode,                    //命令类型之---获取机械臂仿真模式/真实模式
        CommunicationType_GetRobotGravityComponent,        //命令类型之---获取重力分量
        CommunicationType_GetRobotCollisionCurrent,        //命令类型之---获取碰撞等级
        CommunicationType_GetIsRealRobotExist,             //命令类型之---真实机械臂是否存在
        CommunicationType_GetDeviceInfo,                   //命令类型之---读取机械臂设备信息
        CommunicationType_SetMaxAcc,                       //命令类型之---设置最大速度
        CommunicationType_SetRobotCollisionGrade,          //命令类型之---设置机械臂的碰撞等级


        /** 接口板相关 **/
        CommunicationType_GetInterfaceBoardAllDIData,      //命令类型之---获取所有机械臂接口板DI状态
        CommunicationType_GetInterfaceBoardAllDOData,      //命令类型之---获取所有机械臂接口板DO状态
        CommunicationType_GetInterfaceBoardAllAIData,      //命令类型之---获取所有机械臂接口板AI状态
        CommunicationType_GetInterfaceBoardAllAOData,      //命令类型之---获取所有机械臂接口板AO状态
        CommunicationType_GetInterfaceBoardAllData,        //命令类型之---获取所有机械臂接口板All状态
        CommunicationType_GetInterfaceBoardOneDIStatus,    //命令类型之---获取所有机械臂接口板某个DI状态
        CommunicationType_GetInterfaceBoardOneDOStatus,    //命令类型之---获取所有机械臂接口板某个DO状态
        CommunicationType_GetInterfaceBoardOneAIStatus,    //命令类型之---获取所有机械臂接口板某个AI状态
        CommunicationType_GetInterfaceBoardOneAOStatus,    //命令类型之---获取所有机械臂接口板某个AO状态
        CommunicationType_SetInterfaceBoardOneDOStatus,    //命令类型之---获取所有机械臂接口板某个DI状态
        CommunicationType_SetInterfaceBoardOneAOStatus,    //命令类型之---获取所有机械臂接口板某个DI状态

        /** 工具端相关 **/
        CommunicationType_GetToolPowerVoltageType,         //命令类型之---配置工具端电源
        CommunicationType_SetToolPowerVoltageType,         //命令类型之---配置工具端电源
        CommunicationType_SetToolPowerTypeAndDigitalIOType,//命令类型之---配置工具端电源AND数字量IO类型
        CommunicationType_SetToolDigitalIOType,            //命令类型之---配置工具端数字量IO类型
        CommunicationType_GetToolPowerVoltage,             //命令类型之---获取工具端电源电压
        CommunicationType_GetToolAllDigitalIOStatus,       //命令类型之---获取工具端所有数字量IO状态
        CommunicationType_GetToolAllAIStatus,              //命令类型之---获取工具端所有AI状态
        CommunicationType_SetToolDigitalIOStatus,          //命令类型之---设置工具端所有数字量IO状态


        /** 安全IO　**/
        CommunicationType_SetRobotAtOriginPose,
        CommunicationType_GetRobotSafetyConfig,            //命令类型之---获取机械臂安全配置
        CommunicationType_SetRobotSafetyConfig,            //命令类型之---获取机械臂安全配置
        CommunicationType_GetOrpeSafetyStatus,             //命令类型之---获取机械臂安全状态
        CommunicationType_SafetyIoAbout_setRobotOrpePause,
        CommunicationType_SafetyIoAbout_setRobotOrpeStop,
        CommunicationType_SafetyIoAbout_setRobotOrpeError,
        CommunicationType_SafetyIoAbout_clearSystemEmergencyStop,
        CommunicationType_SafetyIoAbout_clearReducedModeError,
        CommunicationType_SafetyIoAbout_robotSafetyguardResetSucc,


        //system poweroff
        CommunicationType_SystemHalt,                   //关闭系统


        CommunicationType_OfflineTrackWaypointAppend,
        CommunicationType_OfflineTrackWaypointClear,
        CommunicationType_OfflineTrackMoveStartup,
        CommunicationType_OfflineTrackMoveStop,

        CommunicationType_EnterTcp2CanbusMode,
        CommunicationType_LeaveTcp2CanbusMode,
        CommunicationType_SetRobotPosData2Canbus,

        CommunicationType_StartupOfflineExcitTraj,
        CommunicationType_GetDynIdentifyResults,

        CommunicationType_RobotHandShake,

        CommunicationType_GetToolAllIoStatus,              //命令类型之---设置工具端所有IO状态


        CommunicationType_UpdateRobotBoardFirmware,
        CommunicationType_FirmwareUpdateResult,

        CommunicationType_GetRobotEthernetDeviceName,

        CommunicationType_RobotMoveFastStop,

        CommunicationType_RobotSetJointOffset, //设置关节碰撞补偿


        CommunicationType_getEnableJoint6Rot360Flag,


        CommunicationType_RobotMoveControlType,
        CommunicationType_RobotMoveControlType_SlowStop,
        CommunicationType_RobotMoveControlType_FastStop,
        CommunicationType_RobotMoveControlType_Pause,
        CommunicationType_RobotMoveControlType_Continue,


        //RobotConveyorTrack
        CommunicationType_setConveyorEncoderReset,
        CommunicationType_setConveyorStartup,
        CommunicationType_setConveyorStop,
        CommunicationType_setConveyorDir,
        CommunicationType_setRobotCameraCalib,
        CommunicationType_setConveyorVelc,
        CommunicationType_setEncoderValPerMeter,
        CommunicationType_setStartWindowUpstream,
        CommunicationType_setStartWindowDownstream,
        CommunicationType_setConveyorTrackDownstream,
        CommunicationType_enableConveyorTrack,
        CommunicationType_getConveyorEncoderVal,
        CommunicationType_appendObject2ConveyorTrackQueue,
        CommunicationType_setRobotConveyorTrackMaxVelc,
        CommunicationType_setRobotConveyorTrackMaxAcc,
        CommunicationType_setRobotConveyorSystemDelay,
        CommunicationType_setRobotTool,

        CommunicationType_setWeaveMoveParameters,
        CommunicationType_setJointRangeOfMotion,

        CommunicationType_setRobotRecognitionParam,
        CommunicationType_getRobotRecognitionParam,

        CommunicationType_getJointAngleFromController,

        CommunicationType_communicationHeart,

        CommunicationType_setSeamTrackingParameters,

        CommunicationType_getSeamTrackingParameters,

        CommunicationType_getCorrectedWaypoint,

        CommunicationType_getJointRangeOfMotion,

        CommunicationType_SetEnableForceTeachWhenProjectIsRunning,   //在工程运行的时候使能/失能力控模式

        //关于驱动器PID参数
        CommunicationType_getJointCommonData,
        CommunicationType_setJointCommonData,
        CommunicationType_jointSaveDataFlash,
        CommunicationType_GetRobotVersionInfo,
        CommunicationType_getEnableJoint1Rot360Flag,
        CommunicationType_enterRobotReduceMode,
        CommunicationType_exitRobotReduceMode,


        //Sensor
        CommunicationType_getForceSensorData,
        CommunicationType_CalibToolAndSensor,
        CommunicationType_EnableForceControlPlan,
        CommunicationType_SetForceControlAttributes,
        CommunicationType_ForceControlCalibrationZero,

        CommunicationType_MovepProgressNotify,
        CommunicationType_SlowStopAndBoardEvent,

        //机械臂参数表
        CommunicationType_setRobotBaseParameter,
        CommunicationType_getRobotBaseParameter,
        CommunicationType_setRobotJointsParameter,
        CommunicationType_getRobotJointsParameter,
        CommunicationType_refreshRobotArmParamter,

        CommunicationType_robotJointPosRecover,

        //调速模式
        CommunicationType_getRegulateSpeedModeConfig,
        CommunicationType_setRegulateSpeedModeConfig,
        CommunicationType_enableRegulateSpeedMode,

        //力控模式
        CommunicationType_enableForceControlMode = 138,
        CommunicationType_getForceControlModeAdmittancePatam,
        CommunicationType_setForceControlModeAdmittancePatam,
        CommunicationType_getForceControlModeExploreForceParam,
        CommunicationType_setForceControlModeExploreForceParam,

        //movep 关节范围限制
        CommunicationType_getMovePJointRangeLimitParam,
        CommunicationType_setMovePJointRangeLimitParam,

        //力控模式
        CommunicationType_enableForceControlMode_V4010000 = 144,
        CommunicationType_getForceControlModeAdmittancePatam_V4010000,
        CommunicationType_setForceControlModeAdmittancePatam_V4010000,
        CommunicationType_getForceControlModeExploreForceParam_V4010000,
        CommunicationType_setForceControlModeExploreForceParam_V4010000,

        //movep 关节范围限制
        CommunicationType_getMovePJointRangeLimitParam_V4010000,
        CommunicationType_setMovePJointRangeLimitParam_V4010000,

        CommunicationType_setReduceParameter,

        CommunicationType_scriptRun,

        CommunicationType_getTargetWaypointFromController,

        CommunicationType_updateJointFirmware = 160,
        CommunicationType_JointFirmwareUpdateResult,

        CommunicationType_setToolSerialConfig,
        CommunicationType_getToolSerialConfig,
        CommunicationType_GetTrackWaypoints,
        CommunicationType_getMovepSequenceNumber,

        CommunicationType_getJointPositionLimit = 180,
        CommunicationType_GetRobotForceSensorData = 190,

        CommunicationType_percentageEvent=300,

        CommunicationType_StartServoj,                     //启动 servoj
        CommunicationType_StopServoj,                      //停止 servoj
        CommunicationType_Servoj,                          //servoj下发目标跟随点

        CommunicationType_GetJointTypeParameter = 310,     //获取关节类型参数

        /** TCP末端速度　**/
        CommunicationType_RealTimeRobotTCPEndSpeed,           //命令类型之---实时TCP末端速度推送
        CommunicationType_EnableRealTimeTCPEndSpeedPush,      //命令类型之---允许推送实时TCP末端速度
        CommunicationType_DisableRealTimeTCPEndSpeedPush,     //命令类型之---禁止推送实时TCP末端速度

        CommunicationType_StartMoveGroup,                  //启动 moveGroup
        CommunicationType_EndOfMoveGroup,                  //设置 moveGroup 结束点（关闭交融）

        //command count
        CommunicationTypeCount,

        //unknown command
        CommunicationType_Unknown,


    }CommunicationCommandType;


    typedef enum
    {
        SetJointCommonParam_CurrentIP,
        SetJointCommonParam_CurrentII,
        SetJointCommonParam_CurrentID,
        SetJointCommonParam_SpeedP,
        SetJointCommonParam_SpeedI,
        SetJointCommonParam_SpeedD,
        SetJointCommonParam_SpeedDS,
        SetJointCommonParam_PosP,
        SetJointCommonParam_PosI,
        SetJointCommonParam_PosD,
        SetJointCommonParam_PosDS,
    }SetJointCommonParamType;

public:
    static CommunicationMateType *instance();

    static std::string getCommandDescByType(CommunicationCommandType type);


private:
    CommunicationMateType();

    void initCommunicationCommandTypeDescMap();


private:

    std::map<CommunicationCommandType, std::string>  m_communicationCommandTypeDescMap;
};

typedef CommunicationMateType::CommunicationCommandType RobotCommandType;



class CommunicationRequest
{
public:
    CommunicationRequest()
    {
        init();
    }

    CommunicationRequest(const CommunicationRequest &request){
        m_socketFd   = request.m_socketFd;
        m_textLength = request.m_textLength;
        m_commandType= request.m_commandType;
        this->m_textPtr=new char[request.m_textLength];
        memcpy(this->m_textPtr, request.m_textPtr, request.m_textLength);
    }

    ~CommunicationRequest()
    {
        destroy();
    }

public:
    void init()
    {
        m_socketFd   = INVALID_SOCKET;
        m_textPtr    = NULL;
        m_textLength = 0;
        m_commandType= CommunicationMateType::CommunicationType_Unknown;
    }

    void setProperty(int socketFd, CommunicationMateType::CommunicationCommandType commandType, char *textPtr, int textLength)
    {
        m_socketFd   = socketFd;
        m_textPtr    = textPtr;
        m_textLength = textLength;
        m_commandType= commandType;
    }

    void destroy()
    {
        if(m_textPtr != NULL)
        {
            delete[] m_textPtr;
            m_textPtr = NULL;
        }
        setProperty(INVALID_SOCKET, CommunicationMateType::CommunicationType_Unknown, NULL, 0);
    }

public:
    int     m_socketFd;
    char   *m_textPtr;
    int     m_textLength;
    CommunicationMateType::CommunicationCommandType    m_commandType;
};




class CommunicationResponse
{
public:
    CommunicationResponse()
    {
        init();

        pthread_cond_init (&m_conditon, NULL);
        pthread_mutex_init(&m_mutex,    NULL);
    }

    CommunicationResponse(const CommunicationResponse &response){
        m_socketFd   = response.m_socketFd;
        m_textLength = response.m_textLength;
        m_commandType= response.m_commandType;
        this->m_textPtr=new char[response.m_textLength];
        memcpy(this->m_textPtr, response.m_textPtr, response.m_textLength);

        pthread_cond_init (&m_conditon, NULL);
        pthread_mutex_init(&m_mutex,    NULL);
    }

    ~CommunicationResponse()
    {
        destroy();
        pthread_cond_destroy(&m_conditon);
        pthread_mutex_destroy(&m_mutex);
    }

public:
    void init()
    {
        m_socketFd   = INVALID_SOCKET;
        m_textPtr    = NULL;
        m_textLength = 0;
        m_commandType= CommunicationMateType::CommunicationType_Unknown;
    }


    void setProperty(int socketFd, CommunicationMateType::CommunicationCommandType commandType, char *textPtr, int textLength)
    {
        m_socketFd   = socketFd;
        m_textPtr    = textPtr;
        m_textLength = textLength;
        m_commandType= commandType;
    }

    void destroy()
    {
        if(m_textPtr != NULL)
        {
            delete m_textPtr;
            m_textPtr = NULL;
        }



        setProperty( INVALID_SOCKET, CommunicationMateType::CommunicationType_Unknown, NULL, 0);
    }

public:
    int     m_socketFd;
    char   *m_textPtr;
    int     m_textLength;
    pthread_mutex_t  m_mutex;
    pthread_cond_t   m_conditon;
    CommunicationMateType::CommunicationCommandType m_commandType;
};


//通用响应格式
class CommunicationCommonResultResponse
{
public:
    CommunicationCommonResultResponse()
    {
        m_errorCode = 0;

        m_errorMsg  = "";
    }

public:
    int          m_errorCode;    //错误号
    std::string  m_errorMsg ;    //错误信息
};



//呼叫运动控制函数返回值定义
enum call_robot_motion_func_result
{
    call_robot_motion_func_failed = 0,        //规则无效，呼叫运动控制函数失败
    call_robot_motion_func_succ_wait_done,    //规则有效，需调用算法层, 操作者需要等待完成事件
    call_robot_motion_func_succ_nowait,       //规则有效，不调用算法层，状态机立即切换，操作者不需要等待完成事件
    call_robot_motion_func_succ_nocall_wait   //规则有效，不调用算法层，但操作者需要等待完成事件
};



#endif // COMMUNICATIONMATETYPE_H
