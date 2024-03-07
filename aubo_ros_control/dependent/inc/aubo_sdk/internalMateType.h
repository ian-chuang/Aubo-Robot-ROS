#ifndef INTERNALMATETYPE_H
#define INTERNALMATETYPE_H

#include "AuboRobotMetaType.h"

typedef enum{
    InterfaceBoardDICount = 0X33+1,
    InterfaceBoardDOCount = 0x2F+1,
    InterfaceBoardAICount = 0X03+1,
    InterfaceBoardAOCount = 0X03+1,
}InterfaceBoardIOCount;


enum {INTERFACE_BOARD_ERROR_BUFFER_SIZE = 0x10};   //接口板错误缓冲区大小
enum {INTERFACE_BOARD_IO_BUFFER_SIZE    = 0x20};   //接口板IO缓冲区大小


typedef struct
{
    //接口板IO数据
    uint8 data[INTERFACE_BOARD_IO_BUFFER_SIZE];
}InterfaceBoardIoData;


typedef struct
{
    //接口板错误
    uint8 boardError[INTERFACE_BOARD_ERROR_BUFFER_SIZE];
}InterfaceBoardError;


/**
 *  工具端IO状态
 **/
typedef struct PACKED
{
    aubo_robot_namespace::ToolPowerType  powerType;             // 电源类型

    float systemVoltage;                  // 系统电压

    aubo_robot_namespace::ToolDigitalStatus digitalIoStatus[4]; // 数字IO数据

    float aiData[2];                      // 模拟IO数据

    float systemTemperature;              // 系统温度

    uint8 errorStatus;                    // 错误状态

}RobotToolAllIOStatus;




/*****机械臂TCP参数*****/
typedef struct PACKED
{
    //TCP距离中心点距离X（单位，毫米）
    float positionX;
    //TCP距离中心点距离Y（单位，毫米）
    float positionY;
    //TCP距离中心点距离Z（单位，毫米）
    float positionZ;
    //TCP负载重量（单位，公斤）
    float payload;

}TcpParam;


typedef struct PACKED
{
    TcpParam paramAutorun;
    TcpParam paramManual;
}RobotTcpParam;




/** 坐标系标定 **/
typedef struct
{
    aubo_robot_namespace::coordinate_refer     coordType;

    aubo_robot_namespace::CoordCalibrateMathod methods;           //标定方法

    aubo_robot_namespace::wayPoint_S           wayPointArray[3];  //用于标定坐标系的３个点  对应于机械臂指的是工具末端的点

}CoordCalibrateByToolEndPoint;




/** 机械臂碰撞电流 **/
typedef struct PACKED
{
    uint16_t  JointCollisionCurrent[6][4];
    uint8_t   CollisionClass;

}RobotCollisionCurrent;


/** 服务器响应中的错误号　**/
typedef enum
{
    RobotResponseCode_SUCC,
    RobotResponseCode_RequestFormatError,
    RobotResponseCode_ProcessRequesFailed,
    RobotResponseCode_ConnectBreak,
    RobotResponseCode_RealRobotNoExist,
}RobotControlServerErrorCode;







/***************************机械臂运动属性***********************************/

//该结构体主要用于描述速度与加速度属性
typedef union
{
    double cartPara[2];
    double jointPara[aubo_robot_namespace::ARM_DOF];
}joint_cart_U;


typedef union
{
    aubo_robot_namespace::Ori orientation;
    double quaternionVector[4];
}cartesian_Ori_U;

enum arrival_ahead_state
{
    arrival_ahead_none,
    arrival_ahead_distance, //in meter
    arrival_ahead_time, //in second
    arrival_ahead_blend_distance,
    arrival_ahead_blend_time,
    arrival_ahead_done
};


/**
 * @brief 运动模式枚举
 **/
enum move_mode
{
    NO_MOVEMODE = 0,
    MODEJ,
    MODEL,
    MODEP
};


/**
 * @brief 运动结束原因枚举
 **/
enum MoveFinishFlag
{
    MoveFinishFlagSuccAtTrackTarget   = 0,
    MoveFinishFlagEnterStopState      = 1,
    MoveFinishFlagRobotControllerError= 2,

};


//Robot运动属性
typedef struct
{
    move_mode moveMode;
    aubo_robot_namespace::move_track subMoveMode;
    aubo_robot_namespace::teach_mode teachMode;
    bool enableIterIk;//是否可以使用迭代近似逆解
    bool toolTrack;
    aubo_robot_namespace::cartesianPos_U toolInEndPosition;
    aubo_robot_namespace::Ori toolInEndOrientation;
    //MoveConditionClass::coordinate_refer userCoord;

    struct
    {
        bool ena;
        float relativePosition[3];
        cartesian_Ori_U relativeOrientation; // TODO: relative orientation.
    }relativeMove;

    //joint(no motor): radian/s radian/s2
    //cartesian(no motor): m/s m/s2
    joint_cart_U maxVelc;
    joint_cart_U maxAcc;

    struct
    {
        arrival_ahead_state arrivalAheadStat;
        double arrivalAheadThr;   //米或者秒
    }arrivalAhead;

    float blendRadius;
    int circularLoopTimes;
    double jerkAccRatio;
}RobotMoveProfile;



// 错误码类型结构体
typedef struct PACKED{

    /*
     * 错误码，16bit
     * 不同分类的错误存在不同错误码
     */
    uint32_t err_code:16;

    /*
     * 错误节点，8bit
     * 底座 - 关节 - 工具端，每个节点占用1bit
     * 0xff代表整个机械臂
     */
    uint32_t err_node:8;

    /*
     * 错误分类，8bit
     *
     */
    uint32_t err_class:8;

    /*
     * 错误类型，2bit
     * 0x01 - 警告
     * 0x02 - 错误
     *
     */
    uint32_t err_type:4;

    /*
     * 停机方式，2bit
     * 0x00 - 0类停机
     * 0x01 - 1类停机
     * 0x02 - 2类停机
     *
     */
    uint32_t stop_model:4;

    /* 保留10bit */

    uint32_t :20;

    /*
     * 错误来源，4bit
     * 0x01 - A芯片
     * 0x02 - B芯片
     *
     */
    uint32_t src:4;

}RobotErrorInfo;


#endif // INTERNALMATETYPE_H
