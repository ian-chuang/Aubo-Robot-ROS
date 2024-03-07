#ifndef PROTOENCODEDECODE_H
#define PROTOENCODEDECODE_H

#include <iostream>

#include "communicationmatetype.h"
#include "robotcommon.pb.h"      //protoBuf 通信数据类型
#include "robotParameters.pb.h"
#include "internalMateType.h"
#include "sensor.pb.h"
#include "robotutilservice.h"

#include "forcecontrol.h"

class ProtoEncodeDecode
{
private:
    ProtoEncodeDecode();

private:
    //路点类型转换
    static bool roadPointTypeConversion(const aubo::robot::common::ProtoRoadPoint &protoRoadPoint, aubo_robot_namespace::wayPoint_S &roadPoint);

    static bool roadPointTypeConversion(const aubo_robot_namespace::wayPoint_S &roadPoint, aubo::robot::common::ProtoRoadPoint &protoRoadPoint);

    //
    static bool jointCartUTypeConversion(const aubo::robot::common::joint_cart_U &src, joint_cart_U &target);

    static bool jointCartUTypeConversion(const joint_cart_U &src,  aubo::robot::common::joint_cart_U &target);

    //姿态类型 转换
    static bool cartesianOriUTypeConversion(const aubo::robot::common::cartesianOri_U &src, cartesian_Ori_U &target);

    static bool cartesianOriUTypeConversion(const cartesian_Ori_U &src, aubo::robot::common::cartesianOri_U &target);

    //运动属性类型转换
    static bool RobotMovePatamTypeConversion(const aubo::robot::common::RobotMove &protoRobotMove, RobotMoveProfile &moveProfile, std::vector<aubo_robot_namespace::wayPoint_S> &roadpointVector);

    static bool RobotMovePatamTypeConversion(const RobotMoveProfile &moveProfile, const std::vector<aubo_robot_namespace::wayPoint_S> &roadpointVector, aubo::robot::common::RobotMove &target);

    static void AuboRobotMoveProfileType2ProtoType(const RobotMoveProfile &moveProfile, aubo::robot::common::RobotMoveProfile &target);

    static void ProtoRobotMoveProfileType2AuboType(const aubo::robot::common::RobotMoveProfile &ProtoMoveProfile, RobotMoveProfile &target);

    //TCP参数
    static bool robotTcpParamTypeConversion(const RobotTcpParam &src, aubo::robot::communication::RobotTcpParam &target);

    static bool robotTcpParamTypeConversion(const aubo::robot::communication::RobotTcpParam &src,  RobotTcpParam&target);

    //关节状态 类型转换
    static bool jointStatusTypeConversion(const aubo_robot_namespace::JointStatus &src, aubo::robot::common::ProtoJointStatus &target);

    static bool jointStatusTypeConversion(const aubo::robot::common::ProtoJointStatus &src, aubo_robot_namespace::JointStatus &target);

    //机械臂诊断信息
    static bool robotDiagnosisTypeConversionToProto(const aubo_robot_namespace::RobotDiagnosis &src, aubo::robot::communication::RobotDiagnosis &target);

    static bool robotDiagnosisTypeConversionToStruct(const aubo::robot::communication::RobotDiagnosis &src,  aubo_robot_namespace::RobotDiagnosis &target);

    //重力分量
    static bool robotGravityComponentTypeConversion(const aubo_robot_namespace::RobotGravityComponent &src, aubo::robot::communication::RobotGravityComponent &target);

    static bool robotGravityComponentTypeConversion(const aubo::robot::communication::RobotGravityComponent &src,  aubo_robot_namespace::RobotGravityComponent &target);

    //
    static bool robotCollisionCurrentTypeConversion(const RobotCollisionCurrent &src,  aubo::robot::communication::RobotCollisionCurrent &target);

    static bool robotCollisionCurrentTypeConversion(const aubo::robot::communication::RobotCollisionCurrent &src,  RobotCollisionCurrent &target);


    static bool RobotSafetyConfigTypeConversion(const aubo_robot_namespace::RobotSafetyConfig &src, aubo::robot::common::ProtoRobotSafetyConfig &target);

    static bool RobotSafetyConfigTypeConversion(const aubo::robot::common::ProtoRobotSafetyConfig &src, aubo_robot_namespace::RobotSafetyConfig &target);

    static bool OrpeSafetyStatusTypeConversion(const  aubo_robot_namespace::OrpeSafetyStatus  &src, aubo::robot::common::ProtoOrpeSafetyStatus &target);

    static bool OrpeSafetyStatusTypeConversion(const  aubo::robot::common::ProtoOrpeSafetyStatus  &src, aubo_robot_namespace::OrpeSafetyStatus &target);

    //工具参数类型转换
    /** 工具类型转换:Proto工具类型中提取工具动力学参数　**/
    static bool toolParamTypeConversion(const  aubo::robot::common::ToolParam &protoToolParam, aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);

    /** 工具类型转换:Proto工具类型中提取工具运动学参数　**/
    static bool toolParamTypeConversion(const  aubo::robot::common::ToolParam &protoToolParam, aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);

    /** 工具类型转换:工具动力学参数生成Proto工具类型**/
    static bool toolParamTypeConversion(const  aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam,     aubo::robot::common::ToolParam &protoToolParam);

    /** 工具类型转换:工具运动学参数生成Proto工具类型**/
    static bool toolParamTypeConversion(const  aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam, aubo::robot::common::ToolParam &protoToolParam);



public:
    /** Robot事件类型转换 **/
    static void robotEventTypeConversion(const aubo_robot_namespace::RobotEventInfo &src, aubo::robot::communication::ProtoCommunicationRobotEvent &target);

    static void robotEventTypeConversion(const aubo::robot::communication::ProtoCommunicationRobotEvent &src,  aubo_robot_namespace::RobotEventInfo &target);

    /** 设备信息的类型转换 **/
    static void RobotDevInfoTypeConversion(const aubo::robot::communication::OurRobotDevInfo &src, aubo_robot_namespace::RobotDevInfo &target);


private:  /** 关于IO的类型转换 **/

    static void RobotDiagnosisIODescTypeConversion(const aubo_robot_namespace::RobotDiagnosisIODesc &src,  aubo::robot::communication::ProtoRobotDiagnosisIODesc &target);

    static void RobotDiagnosisIODescTypeConversion(const aubo::robot::communication::ProtoRobotDiagnosisIODesc &src, aubo_robot_namespace::RobotDiagnosisIODesc &target);

    static void RobotAnalogIODescTypeConversion   (const aubo_robot_namespace::RobotAnalogIODesc &src,     aubo::robot::communication::ProtoRobotAnalogIODesc &target);

    static void RobotAnalogIODescTypeConversion   (const aubo::robot::communication::ProtoRobotAnalogIODesc &src,    aubo_robot_namespace::RobotAnalogIODesc &target);

private: //焊接
    static  bool  ProtoSeamTrackToSeamTrack(const aubo::robot::common::ProtoSeamTrack_t &protoSeamTrack, aubo_robot_namespace::SeamTracking &target);

    static  bool  SeamTrackToProtoSeamTrack(const aubo_robot_namespace::SeamTracking &source, aubo::robot::common::ProtoSeamTrack_t &protoSeamTrack);



public:  /** 关于通用数据类型的序列化和反序列化 **/

    static  void  makeProtoCommunicationGeneralData(aubo::robot::communication::ProtoCommunicationGeneralData &protoCommunicationGeneralData,const std::vector<int> &propertyVector1, const std::vector<bool> &propertyVector2);

    static  bool  ProtoCommunicationGeneralDataMessageSerialize(const aubo::robot::communication::ProtoCommunicationGeneralData &protoCommunicationGeneralData, char **ptr, int *size);

    static  void  resolveProtoCommunicationGeneralData(const aubo::robot::communication::ProtoCommunicationGeneralData &protoCommunicationGeneralData,
                                                       std::vector<int> &propertyVector1,  std::vector<bool> &propertyVector2, int &errorCode);

    static  bool  ProtoCommunicationGeneralDataMessageParse(const char *buffer, int size,  aubo::robot::communication::ProtoCommunicationGeneralData &protoCommunicationGeneralData);


public:
    static bool ProtoJointCommonDataToRobotStruct(const aubo::robot::communication::ProtoJointCommonData &protoJointCommonData, aubo_robot_namespace::JointCommonData &jointCommonData);


    /** ************************************************************************************************************************************************
      *******************************************************生成数据流   数据序列化********************************************************************
      ***************************************************************************************************************************************************/


public:  //产生数据流   ProtoBuf对象序列化

    /** 生成数据: 登录 **/
    static  bool getRequest_login     (char **buffer, int *bufferSize, const std::string &name, const std::string &passwd);

    /** 生成数据: 路点类型 **/
    static  bool getRequest_roadpoint (char **ptr, int *size, const aubo_robot_namespace::wayPoint_S &roadPoint);

    /** 生成数据: 设置TCP参数 **/
    static  bool getRequest_tcpParam  (char **ptr, int *size, const RobotTcpParam &tcpParam);

    static  bool getRequest_setToolDynamicsParam(char **ptr, int *size,   const  aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);

    static  bool getRequest_setToolKinematicsParam(char **ptr, int *size, const aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);

    static  bool getRequest_startupOfflineExcitTraj(char **ptr, int *size, std::string name, int type, int subtype);

    /** 生成数据: 机械臂控制 **/
    static  bool getRequest_robotControl(char **ptr, int *size, const aubo_robot_namespace::RobotControlCommand cmd);

    /** 生成数据: 设置机械臂电源状态 **/
    static  bool getRequest_setRobotPowerStatus(char **ptr, int *size, bool value);

    /** 生成数据: 设置机械臂工作模式 **/
    static  bool getRequest_setRobotWorkMode(char **ptr, int *size, aubo_robot_namespace::RobotWorkMode mode);

    /** 生成数据: 设置机械臂运动控制 **/
    static  bool getRequest_robotMoveControl(char **ptr, int *size, aubo_robot_namespace::RobotMoveControlCommand value);

    /** 生成数据: 设置接口板最大加速度 **/
    static  bool  getRequest_setRobotBoardMaxAcc(char **ptr, int *size, int maxAcc);

    /** 生成数据: 设置碰撞等级 **/
    static  bool  getRequest_setRobotCollision  (char **ptr, int *size, int grade);

    /** 生成数据: 机械臂启动  **/
    static  bool  getRequest_robotStartup(char **ptr, int *size, const RobotTcpParam &tcpParam, uint8 collisionClass, bool readPose, bool staticCollisionDetect, int maxAcc);

    /** 生成数据: 事件信息   **/
    static  bool  getResponse_Event  (char *ptr, int *size, const aubo_robot_namespace::RobotEventInfo &robotEvent);


    /** 生成数据: safe io    **/
    static  bool  getRequest_setSafetyConfig  (char **ptr, int *size, const aubo_robot_namespace::RobotSafetyConfig &safetyConfig);

    /** 生成数据: safe io    **/
    static  bool  getRequest_safeIoParamAbout(char **ptr, int *size, const std::vector<int> &paramVector);

public:
    /** 生成数据: 机械臂运动 **/
    static  bool getRequest_robotMove(char **ptr, int *size,const RobotMoveProfile &moveProfile, const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);

    //示教运动　　其中坐标系通过CoordCalibrateByToolEndPoint给出
    static  bool getRequest_robotTeachMove(char **ptr, int *size, const RobotMoveProfile &moveProfile,
                                           const CoordCalibrateByToolEndPoint &userCoordSystem);

    //示教运动　　其中坐标系通过CoordCalibrateByJointAngleAndTool给出
    static  bool getRequest_robotTeachMove(char **ptr, int *size, const RobotMoveProfile &moveProfile,
                                           const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem);

public:
    static bool  getRequest_firmwareUpgrade(char **ptr, int *size, int cmd, const void *firmwareContent, int length);

public:
    static bool getRequest_Servoj(char **ptr, int *size, const std::vector<double> &target_pos, double duration, double smooth_scale, double delay_sacle, bool is_last_point);

public:   /** 关于IO的请求数据生成 **/

    /** 数字量IO **/
    static  bool  getRequest_robotDiagnosisIOData(char **ptr, int *size, const std::vector<aubo_robot_namespace::RobotDiagnosisIODesc> &diagnosisIOVector);

    /** 模拟量IO **/
    static  bool  getRequest_robotAnalogIOData   (char **ptr, int *size, const std::vector<aubo_robot_namespace::RobotAnalogIODesc> &analogIOVector     );

public:
    /** 生成数据: 关节碰撞补偿 **/
    static  bool  getRequest_robotRobotJointOffsetData(char **ptr, int *size, aubo_robot_namespace::RobotJointOffset &jointOffset);


public:
    static  bool  getRequest_intTypeValueVector(char **ptr, int *size, const std::vector<int> &intTypeValueVector);

    static  bool  getRequest_doubleTypeValueVector(char **ptr, int *size, const std::vector<double> &doubleTypeValueVector);


public:
    static  bool getRequest_ConveyorTrackValuePoint(char **ptr, int *size, const aubo_robot_namespace::Pos &pos, const aubo_robot_namespace::Ori &ori, int timestamp);

public:   //焊接
    static  bool getRequest_SeamTracking(char **ptr, int *size, const aubo_robot_namespace::SeamTracking &seamTracking);

    /** ************************************************************************************************************************************************
    *******************************************************解析数据流   数据反序列化********************************************************************
    ***************************************************************************************************************************************************/

public:

    /** 解析响应: 通用响应格式 **/
    static  bool  resolveResponse_commonFormat (const char *buffer, int size, CommunicationCommonResultResponse &response);

    /** 解析响应:　机械臂事件 **/
    static bool  resolveResponse_robotEvent(const char *buffer, int size, aubo_robot_namespace::RobotEventInfo &robotEvent);

    /** 解析响应: 末端速度 **/
    static  bool  resolveResponse_robotEndSpeed(const char *buffer, int size, double &speed);

    /** 解析响应: MoveP num **/
    static  bool  resolveResponse_movepProgressNotify(const char *buffer, int size, int &num);

    /** 解析请求: 关节角数据 **/
    static  bool  resolveResponse_jointAngle(const char *buffer, int size, aubo_robot_namespace::JointParam &jointAngle, int &errorCode);

    static  bool  resolveResponse_tcpforcesensorData(const char *buffer, int size, aubo_robot_namespace::ForceSensorData &data, int &errorCode);

    /** 解析请求: 关节角数据 **/
    static  bool  resolveResponse_waypoint(const char *buffer, int size, aubo_robot_namespace::wayPoint_S &waypoint, int &errorCode);

    /** 解析请求: 机械臂关节状态 **/
    static  bool  resolveResponse_jointStatus(const char *buffer, int size,  aubo_robot_namespace::JointStatus *jointStatusBuffer, int len,  int &errorCode);

    /** 解析响应: 机械臂诊断信息 **/
    static  bool  resolveResponse_robotDiagnosis(const char *buffer, int size, aubo_robot_namespace::RobotDiagnosis &robotDiagnosis);

    /** 解析响应: 机械臂诊断信息 **/
    static bool resolveResponse_robotDiagnosisInfo(const char *buffer, int size, aubo_robot_namespace::RobotDiagnosis &robotDiagnosis, int &errorCode);

    /** 解析响应:设备信息 **/
    static  bool  resolveResponse_devInfo(const char *buffer, int size, aubo_robot_namespace::RobotDevInfo &devInfo, int &errorCode);

    /** 解析请求: 机械臂TCP参数**/
    static  bool  resolveResponse_tcpParam(const char *buffer, int size, RobotTcpParam &tcpParam, int &errorCode);

    static  bool  resolveResponse_getToolDynamicsParam(const char *buffer, int size,   aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam , int &errorCode);

    static  bool  resolveResponse_getToolKinematicsParam(const char *buffer, int size, aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam, int &errorCode);

    static  bool  resolveResponse_getDynIdentifyResults(const char *buffer, int size,  std::vector<int> &paramVector, int &errorCode);

    /** 解析请求: 重力分量 **/
    static  bool  resolveResponse_gravityComponent(const char *buffer, int size, aubo_robot_namespace::RobotGravityComponent &gravityComponent, int &errorCode);

    /** 解析响应: 碰撞电流 **/
    static  bool  resolveResponse_collisionCurrent(const char *buffer, int size, RobotCollisionCurrent &collisionCurrent, int &errorCode);

    /** 解析响应: 工作模式 **/
    static  bool  resolveResponse_robotWorkMode(const char *buffer, int size, aubo_robot_namespace::RobotWorkMode &mode, int &errorCode);

    /** 解析响应: 机械臂当前状态 **/
    static  bool  resolveResponse_robotState(const char *buffer, int size, aubo_robot_namespace::RobotState &state, int &errorCode);

    /** 解析响应: 真实机械臂是否存在 **/
    static bool resolveResponse_isRealRobotExist(const char *buffer, int size, bool &value, int &errorCode);

    /** 解析响应: 安全配置 **/
    static  bool  resolveResponse_safetyConfig(const char *buffer, int size, aubo_robot_namespace::RobotSafetyConfig &safetyConfig, int &errorCode);

    /** 解析响应: 安全状态 **/
    static  bool  resolveResponse_safetyStatus(const char *buffer, int size, aubo_robot_namespace::OrpeSafetyStatus  &safetyStatus, int &errorCode);

    static  bool  resolveResponse_DhParam(const char *buffer, int size, aubo_robot_namespace::RobotType &robotType, aubo_robot_namespace::RobotDhPara &robotDhPara, int &errorCode);

    static  bool  resolveResponse_moveControlResule(const char *buffer, int size, call_robot_motion_func_result &funcRet, int &errorCode);


public:   /** 解析响应: 关于IO的响应解析 **/

    /** 数字量IO **/
    static  bool  resolveResponse_robotDiagnosisIOData(const char *buffer, int size, std::vector<aubo_robot_namespace::RobotDiagnosisIODesc> &diagnosisIOStatusVector, int &errorCode);

    /** 模拟量IO **/
    static  bool  resolveResponse_robotAnalogIOData   (const char *buffer, int size, std::vector<aubo_robot_namespace::RobotAnalogIODesc>    &analogIOStatusVector,    int &errorCode);

    static  bool  resolveResponse_robotToolIoStatus   (const char *buffer, int size, RobotToolAllIOStatus &toolAllIOStatus, int &errorCode);


public:
    static  bool  resolveResponse_ethernetDeviceName(const char *buffer, int size, std::string &ethernetDeviceName, int &errorCode);

    static  bool  resolveResponse_jointCommonData(const char *buffer, int size, std::vector<aubo_robot_namespace::JointCommonData> &jointCommonDataVector, int &errorCode);

public://通用数据类型
    static  bool  resolveResponse_intTypeValueVector(const char *buffer, int size, std::vector<int> &intTypeValueVector,int &errorCode);

    static  bool  resolveResponse_doubleTypeValueVector(const char *buffer, int size, std::vector<double> &doubleTypeValueVector,int &errorCode);

public: //焊接
    static  bool  resolveResponse_SeamTracking(const char *buffer, int size, aubo_robot_namespace::SeamTracking &seamTracking, int &errorCode);

public:  //Sensor

    //力传感器六个维度数据(力+力矩)
    static  bool  ProtoWrenchToWrench(const peripheral::ProtoWrench &source, WrenchParam &target);

    static  bool  WrenchToProtoWrench(const WrenchParam &source, peripheral::ProtoWrench &target);

    //力传感器标定结果
    static  bool  ProtoFtSensorCalResultToRobotType(const peripheral::ProtoFtSensorCalibResult &source, FtSensorCalResult &target);

    static  bool  FtSensorCalResultToProtoType(const FtSensorCalResult &source, peripheral::ProtoFtSensorCalibResult &target);

    //力传感器标定参数
    static  bool  FtSensorCalibParamToProtoType(const aubo_robot_namespace::JointParam jointParamGroup[3], const WrenchParam wrenchParamGroup[3], peripheral::ProtoFtSensorCalibParam &target);

public:

    static  bool  getRequest_FtSensorCalibParam(char **ptr, int *size, aubo_robot_namespace::JointParam jointParamGroup[3], WrenchParam wrenchGroup[3]);

    static  bool  resolveResponse_FtSensorCalibResult(const char *buffer, int size, FtSensorCalResult &calibrationResult, int &errorCode);

    static  bool  resolveResponse_Wrench(const char *buffer, int size, WrenchParam &wrenchParam, int &errorCode);

    static  bool  getResponse_Wrench(char **ptr, int *size, WrenchParam &wrenchData, int errorCode, const std::string &errorMsg);

    static  bool  getResponse_FtSensorCalibResult(char **ptr, int *size, const FtSensorCalResult &calibrationResult, int errorCode, const std::string &errorMsg);

public:
    static  void  ProtoRobotCommonResponseInit(aubo::robot::common::RobotCommonResponse* protoErrnoInfo, int errorCode, const std::string &errorMsg);

    static  bool  ProtobufDataSerializeToArray(const google::protobuf::Message* msg, char **ptr, int *size);

public:
    static  bool RobotInfo_ProtoToAubo(const aubo::robot::paramerter::ProtoRobotInfo &protoRobotInfo, aubo_robot_namespace::RobotInfo &robotInfo);

    static  bool RobotInfo_AuboToProto(const aubo_robot_namespace::RobotInfo &robotInfo, aubo::robot::paramerter::ProtoRobotInfo &protoRobotInfo);

    static  bool RobotDynamicsParameters_ProtoToAubo(const aubo::robot::paramerter::ProtoRobotDynamicsParameter &ProtoDynamicsParameter, aubo_robot_namespace::RobotDynamicsParameters &dynamicsParameter);

    static  bool RobotDynamicsParameters_AuboToProto(const aubo_robot_namespace::RobotDynamicsParameters &dynamicsParameter, aubo::robot::paramerter::ProtoRobotDynamicsParameter &ProtoDynamicsParameter );

    static  bool RobotHandguidingParameter_ProtoToAubo(const aubo::robot::paramerter::ProtoRobotHandguidingParameter &ProtoHandguidingParameter, aubo_robot_namespace::RobotHandguidingParameters &handguidingParameters);

    static  bool RobotHandguidingParameter_AuboToProto(const aubo_robot_namespace::RobotHandguidingParameters &handguidingParameters, aubo::robot::paramerter::ProtoRobotHandguidingParameter &ProtoHandguidingParameter);

    static  bool RobotKinematicsParameter_ProtoToAubo(const aubo::robot::paramerter::ProtoRobotKinematicsParameter &protoRobotKinematicsParameter, aubo_robot_namespace::RobotKinematicsParameters &kinematicsParameter);
    static bool RobotKinematicsParameter_ProtoToAubo_Legacy(const aubo::robot::paramerter::ProtoRobotKinematicsParameter_legacy &protoRobotKinematicsParameter, aubo_robot_namespace::RobotKinematicsParameters &kinematicsParameter);
    
    static  bool RobotKinematicsParameter_AuboToProto(const aubo_robot_namespace::RobotKinematicsParameters &kinematicsParameter, aubo::robot::paramerter::ProtoRobotKinematicsParameter &protoRobotKinematicsParameter );
    static  bool RobotKinematicsParameter_AuboToProto_Legacy(const aubo_robot_namespace::RobotKinematicsParameters &kinematicsParameter, aubo::robot::paramerter::ProtoRobotKinematicsParameter_legacy &protoRobotKinematicsParameter );
   
    static  bool RobotFrictionParameter_ProtoToAubo(const aubo::robot::paramerter::ProtoRobotFrictionParameter &ProtoRobotFrictionParameter, aubo_robot_namespace::RobotFrictionParameters &frictionParameters);

    static  bool RobotFrictionParameter_AuboToProto(const aubo_robot_namespace::RobotFrictionParameters &frictionParameters, aubo::robot::paramerter::ProtoRobotFrictionParameter &ProtoRobotFrictionParameter);

    static  bool RobotBaseParameter_ProtoToAubo(const aubo::robot::paramerter::ProtoRobotBaseParameter &protoRobotBaseParameter, aubo_robot_namespace::RobotBaseParameters &robotBaseParameter);
    static  bool RobotBaseParameter_ProtoToAubo_Legacy(const aubo::robot::paramerter::ProtoRobotBaseParameter_legacy &protoRobotBaseParameter, aubo_robot_namespace::RobotBaseParameters &robotBaseParameter);
   
    static  bool RobotBaseParameter_AuboToProto(const aubo_robot_namespace::RobotBaseParameters &robotBaseParameter, aubo::robot::paramerter::ProtoRobotBaseParameter &protoRobotBaseParameter );
    static  bool RobotBaseParameter_AuboToProto_Legacy(const aubo_robot_namespace::RobotBaseParameters &robotBaseParameter, aubo::robot::paramerter::ProtoRobotBaseParameter_legacy &protoRobotBaseParameter );
   
    static  bool RobotJointsParameter_ProtoToAubo(const aubo::robot::paramerter::ProtoRobotJointsParameter &protoRobotJointsParameter, aubo_robot_namespace::RobotJointsParameter &jointsParameter);

    static  bool RobotJointsParameter_AuboToProto(const aubo_robot_namespace::RobotJointsParameter &jointsParameter, aubo::robot::paramerter::ProtoRobotJointsParameter &protoRobotJointsParameter );

    static  bool resolveRequest_ProtoRobotBaseParameter(const char *buffer, int size, aubo_robot_namespace::RobotBaseParameters &robotBaseParameter);

    static  bool makeResponse_ProtoRobotBaseParameter(char **ptr, int *size, const aubo_robot_namespace::RobotBaseParameters &robotBaseParameter,int errorCode, const std::string &errorMsg);

    static  bool makeRequest_ProtoRobotBaseParameter(char **ptr, int *size, const aubo_robot_namespace::RobotBaseParameters &robotBaseParameter);

    static  bool ParseResponse_ProtoRobotBaseParameter(const char *ptr, int size, aubo_robot_namespace::RobotBaseParameters &robotBaseParameter, int &errorCode);

    static  bool resolveRequest_ProtoRobotJointsParameter(const char *buffer, int size, aubo_robot_namespace::RobotJointsParameter &robotJointsParameter);

    static  bool makeResponse_ProtoRobotJointsParameter(char **ptr, int *size, const aubo_robot_namespace::RobotJointsParameter &robotJointsParameter, int errorCode, const std::string &errorMsg);

    static  bool makeRequest_ProtoRobotJointsParameter(char **ptr, int *size, const aubo_robot_namespace::RobotJointsParameter &robotJointsParameter);

    static  bool ParseResponse_ProtoRobotJointsParameter(const char *ptr, int size, aubo_robot_namespace::RobotJointsParameter &robotJointsParameter, int &errorCode);
};


#endif // PROTOENCODEDECODE_H
