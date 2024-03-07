#ifndef ROBOTMOVESERVICE_H
#define ROBOTMOVESERVICE_H

#include <vector>
#include "internalMateType.h"
#include "communicationmatetype.h"


class RobotControlServices;
class RobotMoveService
{
    typedef struct
    {
        bool isEnable;                                                             //是否使能偏移
        aubo_robot_namespace::CoordCalibrateByJointAngleAndTool  coordinateSystem; //暂时没用　如果有需要用全局偏移平面的时候使用
        float relativePosition[3];                                                 // x,y,z
        cartesian_Ori_U relativeOrientation;                                       //暂时没用　目前不支持
    }MoveRelativeParam_t;        //运动属性的偏移量参数类型

    typedef struct
    {
        arrival_ahead_state arrivalAheadStat;      //提前到位的模式
        double  arrivalAheadThr;                   //米或者秒
    }MoveArrivalAhead_t;


    enum RobotMoveClass
    {
        ROBOT_MOVE_CLASS_GENARAL,                  //常规运动类型
        ROBOT_MOVE_CLASS_OFFLINE_TRACK,            //离线轨迹运动类型
        ROBOT_MOVE_CLASS_OFFLINE_TRACK_RECOGNITION,//离线轨迹辨识
    };

    typedef struct
    {
        std::string trackName;
        int type;
        int subType;
    }offlineTrackRecognition;

public:

    RobotMoveService(RobotControlServices *p);  //构造函数


private:
    /** 初始化类的属性 **/
    int   initClassProfile();

    /** 发送关于运动的请求(moveProfile＋wayPointVector)　**/
    int   moveProfileAndWaypointsFormatCommunication(RobotCommandType cmdType, const RobotMoveProfile &moveProfile, const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);

    /** Move 服务(除了示教运动其余的运动都是基于该函数实现的)　**/
    int   moveBaseService(RobotMoveClass MoveType, const RobotMoveProfile &moveProfile,
                          const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector,
                          int &moveEndEventType, bool IsBolck);



public:  /** 运动控制　**/

    /** 运动控制接口 **/
    int   robotMoveControl(aubo_robot_namespace::RobotMoveControlCommand cmd);

    /** 急速停止的运动接口 **/
    int   robotMoveFastStop();

    /** 缓停的运动接口 **/
    int   robotMoveSlowStop(bool isBoardStopEvent = true);

    /** 缓停的运动接口 **/
    int   setReducePara(const double jerkRatio, const double acc[], int size);


public:
    /** 一个通用接口　**/
    int   robotMoveService(const RobotMoveProfile &moveProfile, const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector, int &moveEndEventType, bool IsBolck);


    /** 示教运动停止的接口 **/
    int  robotTeachStop();

    /** 示教运动接口　**/
    int  robotTeachMove (const RobotMoveProfile &moveProfile, const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem);


    /** 离线轨迹　**/
    int   offlineTrackStartupBaseService();

    int   offlineTrackMoveStartupService(bool isBolck);    //离线轨迹运动

    int   offlineTrackMoveStop();

    int   offlineTrackWaypointClear ();

    int   offlineTrackWaypointAppend(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);

    int   startupOfflineExcitTraj(const char *trackFile, aubo_robot_namespace::Robot_Dyn_identify_traj type, int subtype, bool isBolck);


    /** TCP转CAN透传 **/
    int   enterTcp2CanbusMode();

    int   leaveTcp2CanbusMode();

    int   setRobotPosData2Canbus(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);


    /** 缩减模式 **/
    int   enterRobotReduceMode();

    int   exitRobotReduceMode();


    /** servoj **/
    int startServoj();

    int stopServoj();

    int servoj(const double q[6], double duration, double smooth_scale, double delay_sacle, bool is_last_point);

    /** moveGroup **/
    int startMoveGroup();

    int setEndOfMoveGroup();

    int waitMoveGroupFinished();

public:    /** 关于属性设置　**/

    /** 初始化运动属性　**/
    int     initMoveProfile();

    /** 设置和获取关节型运动的最大速度和加速度　**/
    int     setMoveJointMaxAcc(const aubo_robot_namespace::JointVelcAccParam  &moveMaxAcc);

    int     setMoveJointMaxVelc(const aubo_robot_namespace::JointVelcAccParam  &moveMaxVelc);

    void    getMoveJointMaxAcc (aubo_robot_namespace::JointVelcAccParam  &moveMaxAcc);

    void    getMoveJointMaxVelc(aubo_robot_namespace::JointVelcAccParam  &moveMaxVelc);


    /** 设置和获取末端型运动的最大速度和加速度　包含:直线运动，轨迹运动中的圆轨迹，轨迹运动中的MoveP轨迹　**/
    int     setMoveEndMaxLineAcc  (double  moveMaxAcc);

    int     setMoveEndMaxLineVelc (double  moveMaxVelc);

    void    getMoveEndMaxLineAcc  (double  &moveMaxAcc);

    void    getMoveEndMaxLineVelc (double  &moveMaxVelc);

    int     setMoveEndMaxAngleAcc (double  moveMaxAcc);

    int     setMoveEndMaxAngleVelc(double  moveMaxVelc);

    void    getMoveEndMaxAngleAcc (double  &moveMaxAcc);

    void    getMoveEndMaxAngleVelc(double  &moveMaxVelc);

    int     setJerkAccRatio(double acc);

    void    getJerkAccRatio(double &acc);

    /**  运动属性中的路点设置与获取　**/
    void    clearWayPointVector();

    int     addWayPoint(const aubo_robot_namespace::wayPoint_S &wayPoint);          //基于基座标系

    void    getWayPointVector(std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);


    float   getBlendRadius();

    int     setBlendRadius(float value);

    int     getCircularLoopTimes();

    void    setCircularLoopTimes(int times);

    double  getTrackPlaybackCycle();

    void    setTrackPlaybackCycle(double second);


    void    setEnableIterIk(bool value);

    int     setMoveProfileRelativeParam(const aubo_robot_namespace::MoveRelative &relativeMoveOnBase);            //基于基座标系

    int     setMoveProfileRelativeParam(const aubo_robot_namespace::MoveRelative &relativeMoveOnUserCoord,
                                        const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord);  //基于自定义坐标系

    int     setTeachCoordinateSystem(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coordSystem); //设置示教坐标系


    /** 工具参数的设置 **/
    int   setMoveProfileToolParamIsNone();        //设置工具参数为无工具

    int   setMoveProfileToolParam(const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc);   //设置工具参数

    int   getMoveProfileToolParam(aubo_robot_namespace::ToolInEndDesc &toolInEndDesc);         //获取工具参数


    /** 提前到位　**/
    int   setNoArrivalAhead();

    int   setArrivalAheadDistanceMode(double distance /*米*/);

    int   setArrivalAheadTimeMode(double second /*秒*/);

     int  setArrivalAheadBlendDistanceMode(double distance /*米*/);


public:
    /**
     * 基本运动之关节运动
     * 批注:
     * 直线运动　no_teach模式下: 能实现平移＋旋转，但是只有旋转是不能保证tool末端点位置不变；当既有有平移又有旋转的情况下可以保证tool;
     *         teach_move（MOV_X,MOV_Y,MOV_Z任一）模式下：　能实现平移或者平移＋旋转    可以保证tool
     *         teach_rot( ROT_X,ROT_Y,ROT_Z)模式下：   　 能实现只旋转，并且可以保证tool的末端点。
     */
    int robotJointMove(aubo_robot_namespace::wayPoint_S  &wayPoint,  bool IsBolck = false);

    int robotJointMove(aubo_robot_namespace::MoveProfile_t  &moveProfile, aubo_robot_namespace::wayPoint_S  &wayPoint,  bool IsBolck = false);

    /** 跟随模式下的关节运动 **/
    int robotFollowModeJointMove(aubo_robot_namespace::wayPoint_S  &wayPoint);

    /** 基本运动之直线运动 **/
    int robotLineMove(aubo_robot_namespace::wayPoint_S   &wayPoint,  bool IsBolck = false);

    int robotLineMove(aubo_robot_namespace::MoveProfile_t  &moveProfile, aubo_robot_namespace::wayPoint_S  &wayPoint,  bool IsBolck = false);

    /** 基本运动之旋转运动 **/
    int robotLineRotateMove(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord, const double rotateAxisOnUserCoord[3]/*转轴*/, double rotateAngle, bool IsBolck = false);
    /** 旋转至目标路点 **/
    int robotLineRotateMove(const aubo_robot_namespace::wayPoint_S &targetWayPointOnBaseCoord,bool IsBolck = false);
    /** 根据起始姿态以及基坐标系下描述的旋转轴、旋转角，获取目标路点 **/
    int robotGetRotateTargetWaypiont(const aubo_robot_namespace::wayPoint_S &originateWayPointOnBaseCoord,const double rotateAxisOnBaseCoord[], double rotateAngle, aubo_robot_namespace::wayPoint_S &targetWayPointOnBaseCoord);

    /**
     * @brief robotGetRotateAxisUserToBase 将用户坐标系描述的旋转轴变换到基坐标系下描述
     * @param oriOnUserCoord 用户坐标系姿态
     * @param rotateAxisOnUserCoord 用户坐标系下描述的旋转轴
     * @param rotateAxisOnBaseCoord 基坐标系下描述的旋转轴（传出参数）
     * @return
     */
    int robotGetRotateAxisUserToBase(const aubo_robot_namespace::Ori &oriOnUserCoord,const double rotateAxisOnUserCoord[], double rotateAxisOnBaseCoord[]);

    /** 基本运动之轨迹运动 **/
    int robotTrackMove(aubo_robot_namespace::move_track  subMoveMode, bool IsBolck = false);

    /**
     * @brief robotTeachStart  启动示教
     *        示教是通过运动模式的跟随模式实现的。　　所以subMoveMode＝TRACKING，跟随模式下调用stop,机械臂停不下来
     * @param teachMode
     * @param direction
     * @return
     */
    int robotTeachStart(aubo_robot_namespace::teach_mode teachMode, bool direction);
    int robotGetTrack(int mode, double cycle, const double joint_start[], const double joint_end[], std::vector<double> &doubleTypeValueVector);


    /** 扩展运动:保持当前位姿运行到目标位置 **/
    int robotMoveToTargetPositionByRelative(move_mode             moveMode,
                                            const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem,
                                            const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc,
                                            const aubo_robot_namespace::MoveRelative    &relativeOnUser,
                                            bool IsBolck = false);

    int getJointAngleByTargetPositionKeepCurrentOri(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool  &userCoord,
                                      const aubo_robot_namespace::Pos             &positionOnUserCoord,
                                      const aubo_robot_namespace::ToolInEndDesc   &toolInEndDesc,     //相对于用户坐标系的目标位置
                                      aubo_robot_namespace::wayPoint_S            &targetWayPointOnBaseCoord);


    /** 扩展运动:保持当前位姿运行到目标位置  **/
    int robotMoveToTargetPositionByPosition(move_mode             moveMode,
                                            const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool  &userCoordSystem,  //用户平面标定参数
                                            const aubo_robot_namespace::Pos             &position,
                                            const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc,
                                            bool IsBolck = false);



     int setWeaveMoveParameters(const aubo_robot_namespace::WeaveMove &weaveMove);


     int setJointRangeOfMotion(const aubo_robot_namespace::JointRangeOfMotion &rangeOfMotion);

     int getJointRangeOfMotion(aubo_robot_namespace::JointRangeOfMotion &rangeOfMotion);
     int getJointPositionLimit(aubo_robot_namespace::JointRangeOfMotion &rangeOfMotion);


public:  //调速模式
     //获取调速模式配置
     int getRegulateSpeedModeConfig(aubo_robot_namespace::RegulateSpeedModeParamConfig_t &regulateSpeedModeParam);
     //设置调速模式配置
     int setRegulateSpeedModeConfig(const aubo_robot_namespace::RegulateSpeedModeParamConfig_t &regulateSpeedModeParam);

     //使能/失能调速模式
     int enableRegulateSpeedMode(bool enbaleFlag);

public: //力控模式
    //获取力控导纳参数
    int getForceControlModeAdmittancePatam(aubo_robot_namespace::AdmittancePatam_t &admittancePatam);
    //设置力控导纳参数
    int setForceControlModeAdmittancePatam(const aubo_robot_namespace::AdmittancePatam_t &admittancePatam);
    //获取"主动探寻力"参数
    int getForceControlModeExploreForceParam(double &forceLimit, double &distLimit);
    //设置"主动探寻力"参数
    int setForceControlModeExploreForceParam(double forceLimit, double distLimit);
    //使能/失能力控模式
    int enableForceControlModeService(bool enbaleFlag);

    int getRealtimeForceDataService(double forceData[6]);


public:

    /**
     * @brief getTargetWaypointByRelative      获取基于基座标下的目标路点通过相对用户坐标系的偏移，目标路点保持起点姿态
     * @param sourceWayPointOnBaseCoord　　　　　起始路点（基于基坐标系）
     * @param userCoord　　　　　　　　　　　　　　 用户坐标系
     * @param planeCalibrate　　　　　　　　　　　 用户坐标系标定参数
     * @param relative　　　　　　　　　　　　　　　偏移量（基于用户坐标系的偏移量）
     * @param targetWayPointOnBaseCoord　　　　　目标路点（基于基坐标系）
     * @return
     */
    int getTargetWaypointByRelative(const aubo_robot_namespace::wayPoint_S   &sourceWayPointOnBaseCoord, //基于基座标下的起始路点
                                    const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem,
                                    const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc,
                                    const aubo_robot_namespace::MoveRelative  &relativeOnUser,
                                    aubo_robot_namespace::wayPoint_S &targetWayPointOnBaseCoord);


    /**
     * @brief getTargetWaypointByPosition   　　获取基于基座标下的目标路点通过基于用户坐标系的位置，目标路点保持起点姿态
     * @param sourceWayPointOnBaseCoord　　　　　起始路点（基于基坐标系）
     * @param userCoord　　　　　　　　　　　　　　 用户坐标系
     * @param planeCalibrate　　　　　　　　　　　 用户坐标系标定参数
     * @param position　　　　　　　　　　　　　　　基于用户坐标系的目标位置
     * @param targetWayPointOnBaseCoord　　　　 目标路点（基于基坐标系）
     * @return
     */
    static int getTargetWaypointByPosition(const aubo_robot_namespace::wayPoint_S       &sourceWayPointOnBaseCoord,       //基于基座标下的起始点A
                                           const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem,
                                           const aubo_robot_namespace::Pos              &toolEndPosition,
                                           const aubo_robot_namespace::ToolInEndDesc    &toolInEndDesc,
                                           aubo_robot_namespace::wayPoint_S             &targetWayPointOnBaseCoord);





    //获取当前路点信息
    int   getCurrnetRoadPoint(aubo_robot_namespace::wayPoint_S &wayPoint);     //获取当前的路点信息

    int   getForceSensorData(aubo_robot_namespace::ForceSensorData &data);

    int   getRoadPointFromController(aubo_robot_namespace::wayPoint_S &wayPoint);     //获取当前的路点信息

    int   getTargetRoadPointFromController(aubo_robot_namespace::wayPoint_S &wayPoint);

private:
    //初始化运动属性
    void  initDefaultMoveProfile(RobotMoveProfile &moveProfile);               //初始化运动属性结构体为默认值

    //检查轨迹是否为关节型运动
    bool  checkIsJointTypeTrack(const move_mode  moveMode, const aubo_robot_namespace::move_track subMoveMode);

    //获取当前的运动属性除去偏移量属性  根据已设置的全局属性初始化运动属性结构体   TODO:待优化
    void  getCurrentMoveProfileRemoveOffset(const move_mode  moveMode,  const aubo_robot_namespace::move_track subMoveMode,
                                           const aubo_robot_namespace::teach_mode teachMode, RobotMoveProfile &moveProfile);

    //根据参考路点和全局属性修改运动属性
    int   modifyOffsetProfileByWaypointAndGlobalOffset(aubo_robot_namespace::wayPoint_S offset_reference_waypoint, RobotMoveProfile &moveProfile);

    //将基于用户坐标系的偏移量 转换成 基于基座坐标系的偏移量
    int  userCoordRelativeToBaseCoordRelative(aubo_robot_namespace::wayPoint_S &offset_reference_waypoint,
                                              const aubo_robot_namespace::MoveRelative   &relativeOnUserCoord,
                                              const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord,
                                              aubo_robot_namespace::MoveRelative &relativeOnBase
                                              );

    //类型转换  只支持 轴动和直线
    void  MoveProfileConvert(const move_mode  moveMode, aubo_robot_namespace::wayPoint_S &offset_reference_waypoint,
                             aubo_robot_namespace::MoveProfile_t  &sourceMoveProfile, RobotMoveProfile &targetMoveProfile);


private: //调试使用
    static void printWaypoint(aubo_robot_namespace::wayPoint_S &wayPoint);   //打印路点信息


private:
    RobotControlServices                                  *m_robotBaseService;               //机械臂基础服务


private:

    double                                         s_moveProfileEndMaxLineAcc;       //运动属性之末端型运动的最大线加速度

    double                                         s_moveProfileEndMaxLineVelc;      //运动属性之末端型运动的最大线速度

    double                                         s_moveProfileEndMaxAngleAcc;      //运动属性之末端型运动的最大角加速度

    double                                         s_moveProfileEndMaxAngleVelc;     //运动属性之末端型运动的最大角速度

    aubo_robot_namespace::JointVelcAccParam        s_moveProfileJointMaxAcc;         //运动属性之关节型运动的最大加速度

    aubo_robot_namespace::JointVelcAccParam        s_moveProfileJointMaxVelc;        //运动属性之关节型运动的最大速度

    double                                         s_moveProfileJerkAccRatio;        //运动属性之加加速度

    std::vector<aubo_robot_namespace::wayPoint_S>  s_moveWayPointVector;             //运动轨迹之的点的容器

    std::vector<aubo_robot_namespace::wayPoint_S>  s_moveWayPointVectorMoveP;        //运动轨迹之movep的点的容器

    aubo_robot_namespace::CoordCalibrateByJointAngleAndTool   s_teachMoveCoordinateSystem;     //运动属性之示教坐标系属性

    MoveRelativeParam_t                            s_moveProfileRelativeParam;       //运动属性之的偏移属性

    double                                         s_moveProfileBlendRadius;         //运动属性之的交融半径

    double                                         s_moveProfileTrackPlaybackCycle;  //运动属性之的轨迹回放周期

    int                                            s_moveProfileCircularLoopTimes;   //运动属性之的圆轨迹的运动圈数

    bool                                           s_moveProfileEnableIterIk;        //运动属性之的是否可以使用迭代近似逆解

    aubo_robot_namespace::ToolInEndDesc            s_moveProfileToolInEndDesc;       //运动属性之的工具描述

    bool                                           s_moveProfileToolTrack;           //运动属性之是否使用工具  配合s_moveProfileToolInEndDesc来使用

    MoveArrivalAhead_t                             s_moveArrivalAhead;               //运动模式至跟随模式提前到位

    bool                                           s_moveProfileIsInited;            //运动属性是否被初始化

    offlineTrackRecognition                        s_offlineTrackRecognition;

    //edit 20220408 wpy
    aubo_robot_namespace::move_track s_movetrack_type;

};


#endif // ROBOTMOVESERVICE_H
