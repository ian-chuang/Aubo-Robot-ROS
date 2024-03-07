#ifndef ROBOTUTILSERVICE_H
#define ROBOTUTILSERVICE_H

#include <vector>
#include "internalMateType.h"

class RobotUtilService
{
private:
    RobotUtilService();

public:
    /**
     * @brief 正解　　　　　此函数为正解函数，根据已知关节角　解得　对应的位置和姿态。
     * @param jointAngle  六个关节的关节角   输入参数   单位:弧度(rad)
     * @param size        关节角数组长度    规定为6个
     * @param wayPoint    正解结果    　　　输出参数
     * @return
     */
    static int robotFk(const double *jointAngle, int size, aubo_robot_namespace::wayPoint_S &wayPoint);


    /**
     * @brief robotIk
     *          机器人运动学方程的逆解，也称机器人的逆运动学问题，或间接位置求解。
     *          逆运动学问题：对某个机器人，当给出机器人手部在基座标系中所处的位置和姿态时，求出其对应的关节角信息。
     *          逆解可能有多个解，本函数根据起点位置，求得最优解。
     * @param startPointJointAngle   起点关节角
     * @param position　　　目标位置(基于基座标系法兰盘中心的位置)   　　　单位:米   输入参数
     * @param ori          目标姿态
     * @param wayPoint　　　逆解结果
     * @return
     */
    static int robotIk(const double *startPointJointAngle, const aubo_robot_namespace::Pos &position, const aubo_robot_namespace::Ori &ori, aubo_robot_namespace::wayPoint_S &wayPoint);

    /**
     * @brief 计算在给定位姿情况下的机械臂所有逆解，关节的求解范围是(-pi, pi]
     * @param position: 给定位姿对应的位置
     * @param ori: 给定位姿对应的姿态
     * @param wayPointVector: 逆解集合
     * @return
     */
    static int robotIk(const aubo_robot_namespace::Pos &position, const aubo_robot_namespace::Ori &ori, std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);

    //基于工具末端的位置和姿态(四元素)做逆解运算
    static int robotServiceIkBasedOnTcp(const double *referPointJointAngle,                 //参考点
                                 const aubo_robot_namespace::Pos &toolEndPositionOnBase,    //基于基座坐标系工具末端位置
                                 const aubo_robot_namespace::Ori &toolEndQuaternionOnBase,  //基于基座坐标系工具末端姿态
                                 const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc,  //工具参数
                                 aubo_robot_namespace::wayPoint_S &flangeWaypointOnBase);   //逆解结果

    //基于工具末端的位置和姿态（欧拉角）做逆解运算
    static int robotServiceIkBasedOnTcp(const double *referPointJointAngle,                 //参考点
                                 const aubo_robot_namespace::Pos &toolEndPositionOnBase,    //基于基座坐标系工具末端位置
                                 const aubo_robot_namespace::Rpy &toolEndRpyOnBase,         //基于基座坐标系工具末端姿态
                                 const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc,  //工具参数
                                 aubo_robot_namespace::wayPoint_S &flangeWaypointOnBase);   //逆解结果


    static int setJoint6Rot360(bool enable);

    static int setJoint1Rot360(bool enable);

    static int setRobotSystemCalibParam(const aubo_robot_namespace::RobotKinematicsParameters &param);


    /**
     * @brief toolCalibration         工具标定　　该函数只能标定工具的位置信息
     * @param wayPointPosCalibVector  提供４个或４个以上的点
     * @param poseCalibMethod         标定方法
     * @param toolInEndDesc           标定的结果
     * @return
     */
    static int toolCalibration(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointPosCalibVector,
                               char poseCalibMethod,
                               aubo_robot_namespace::ToolInEndDesc &toolInEndDesc);


    //工具标定
    /**
     * @brief toolCalibration         　工具标定　　该函数能标定出工具的位置信息和姿态信息
     * @param wayPointPosCalibVector　　提供４个或４个以上的点用于位置标定
     * @param wayPointOriCalibVector　　提供两个点用于姿态标定
     * @param poseCalibMethod　　　　　　标定方法
     * @param toolInEndDesc　　　　　　　标定的结果
     * @return
     */
    static int toolCalibration(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointPosCalibVector,
                               const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointOriCalibVector,
                               aubo_robot_namespace::ToolKinematicsOriCalibrateMathod poseCalibMethod,
                               aubo_robot_namespace::ToolInEndDesc &toolInEndDesc);




    //用户平面标定
    static int coordinateSystemCalibration(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coordCalibrate, double bInWPos[3], double bInWOri[9], double wInBPos[3]);


    //检查提供的参数是否能够标定平面
    static int checkCoordinateSystemCalibration(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coordCalibrate);



    /**
     * @brief base2UserCoordinate              基座标系转用户坐标系　　　　（基转用户加工具）
     * 由基座标系下点A位置和姿态  转成　　用户坐标系下工具末端点B位置和姿态
     * 当工具端信息为０００时，相当与基座标系点转用户坐标系点　可以不提供姿态。　　　只要工具存在必须有正确的姿态信息
     * @param flangeCenterPositionOnBase        基于基座标系下法兰盘中心的路点信息之位置信息
     * @param flangeCenterOrientationOnBase　　　基于基座标系下法兰盘中心的路点信息之姿态信息
     * @param userCoordSystem                   用户坐标系
     * @param toolInEndDesc                     工具描述
     * @param toolEndPositionOnUserCoord        转换结果:基于用户座标系的工具末端位置信息
     * @param toolEndOrientationOnUserCoord     转换结果:基于用户座标系的工具末端姿态信息
     * @return
     */
    static int base2UserCoordinate(const aubo_robot_namespace::Pos            &flangeCenterPositionOnBase,         //基于基座标系的法兰盘中心位置信息
                                   const aubo_robot_namespace::Ori            &flangeCenterOrientationOnBase,      //基于基座标系的姿态信息
                                   const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem, //用户坐标系
                                   const aubo_robot_namespace::ToolInEndDesc  &toolInEndDesc,                      //工具信息
                                   aubo_robot_namespace::Pos                  &toolEndPositionOnUserCoord,         //基于用户座标系的工具末端位置信息
                                   aubo_robot_namespace::Ori                  &toolEndOrientationOnUserCoord       //基于用户座标系的工具末端姿态信息
                                   );



    /**
     * @brief base2BaseAdditionalTool    基座标系的法兰盘中心点Ａ转基座标系的工具末端点A'    (基转基加工具)
     * 　　　　　　　　　　　　　　　　　　　　 根据用户提供的工具参数，将基座标系下的法兰盘中心点转成基于基座标的工具末端点。
     *
     * @param flangeCenterPointOnBase　　法兰盘中心基于基座标的点
     * @param toolInEndDesc             工具参数
     * @param toolEndPointOnBase　　　　　基于基座标的工具末端点
     * @return
     */
    static int base2BaseAdditionalTool(const aubo_robot_namespace::wayPoint_S    &flangeCenterPointOnBase,
                                       const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc,
                                       aubo_robot_namespace::wayPoint_S          &toolEndPointOnBase);

    static int base2BaseAdditionalTool(const aubo_robot_namespace::Pos &flangeCenterPositionOnBase,
                                       const aubo_robot_namespace::Ori &flangeCenterOrientationOnBase,
                                       const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc,
                                       aubo_robot_namespace::Pos  &toolEndPositionOnBase,
                                       aubo_robot_namespace::Ori  &toolEndOrientationOnBase);


    /**
     * @brief user2BaseCoordinate                 将用户坐标系下的工具末端点A的位置和姿态　转换成　基座标下法兰盘中心点A的位置和姿态   (用户转基去工具)
     * @param toolEndPositionOnUserCoord
     * @param toolEndOrientationOnUserCoord
     * @param userCoord
     * @param toolInEndDesc
     * @param flangeCenterPositionOnBase
     * @param flangeCenterOrientationOnBase
     * @return
     */
    static int user2BaseCoordinate( const aubo_robot_namespace::Pos            &toolEndPositionOnUserCoord,    //基于用户座标系的工具末端位置信息
                                    const aubo_robot_namespace::Ori            &toolEndOrientationOnUserCoord, //基于用户座标系的工具末端姿态信息
                                    const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord,  //用户坐标系
                                    const aubo_robot_namespace::ToolInEndDesc  &toolInEndDesc,                 //工具信息
                                    aubo_robot_namespace::Pos                  &flangeCenterPositionOnBase,    //基于基座标系的法兰盘中心位置信息
                                    aubo_robot_namespace::Ori                  &flangeCenterOrientationOnBase  //基于基座标系的姿态信息
                                    );



    //点在坐标系下的转换  ===> 将用户坐标系下的点A转换成基座标系下点A‘　　单纯的点转换不涉及工具等。
    static int userCoordPoint2BasePoint(const aubo_robot_namespace::Pos &userCoordPoint,
                                        const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem,
                                        aubo_robot_namespace::Pos &basePoint);


    //偏移向量坐标转换：将基于用户坐标系下的偏移转成基于基坐标下的偏移
    static int offsetVectorUserCoord2Base(const double vectorOnUserCoord[3],
                                          const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoordSystem,
                                          double vectorOnBaseCoord[3]);

    //TODO: 基于END示教的时候用到了
    static int endPosition2BasePosition(const aubo_robot_namespace::wayPoint_S &flangeCenterPointOnBase,
                                        const aubo_robot_namespace::Pos &endPosition, aubo_robot_namespace::Pos &basePosition, aubo_robot_namespace::Ori toolOriInEnd);

    //法兰盘姿态转成工具姿态
    static int endOrientation2ToolOrientation(aubo_robot_namespace::Ori &tcpOriInEnd, const aubo_robot_namespace::Ori &endOri,  aubo_robot_namespace::Ori &toolOri);

    //工具姿态转成法兰盘姿态
    static int toolOrientation2EndOrientation(aubo_robot_namespace::Ori &tcpOriInEnd, const aubo_robot_namespace::Ori &toolOri, aubo_robot_namespace::Ori &endOri);



    //姿态结构体初始化
    static  void initOrientation(aubo_robot_namespace::Ori &orientation);


    //4元素转欧拉角
    static int  quaternionToRPY(const aubo_robot_namespace::Ori &ori, aubo_robot_namespace::Rpy &rpy);

    //欧拉角转4元素
    static int  RPYToQuaternion(const aubo_robot_namespace::Rpy &rpy, aubo_robot_namespace::Ori &ori);

    static bool relativeMoveAndRot(aubo_robot_namespace::wayPoint_S &firstFlangeWaypointInMove, const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coord, const aubo_robot_namespace::ToolInEndDesc realToolInEnd,
                                   const aubo_robot_namespace::Pos &sourceRelativePositionOnUser, const aubo_robot_namespace::Ori &sourceRelativeOriOnUser,
                                   aubo_robot_namespace::Pos &targetRelativePositionOnBase, aubo_robot_namespace::Ori &targetRelativeOriOnBase);



    static int oriMatrixToQuaternion(double eerot[], aubo_robot_namespace::Ori &result);





public:
    //判断一个工具参数是否为空
    static bool isNoneTool(const aubo_robot_namespace::ToolInEndDesc &toolParam);


public:
    static void initPosDataType(aubo_robot_namespace::Pos &postion);

    static void initOriDataType(aubo_robot_namespace::Ori &ori);

    static void initMoveRelativeDataType(aubo_robot_namespace::MoveRelative &moveRelative);

    static void initWayPointDataType(aubo_robot_namespace::wayPoint_S &wayPoint);

    static void initToolInEndDescDataType(aubo_robot_namespace::ToolInEndDesc &toolInEndDesc);

    static void initCoordCalibrateByJointAngleAndToolDataType(aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coord);

    static void initToolInertiaDataType(aubo_robot_namespace::ToolInertia &toolInertia);

    static void initToolDynamicsParamDataType(aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);






public:
    static bool jointStringToWaypoint(const std::string &jointString, aubo_robot_namespace::wayPoint_S &waypoint);


    static void setRobotDhParam(int robotType, const aubo_robot_namespace::RobotDhPara &robotDhPara);




private:

    //用户平面标定
    static int coordinateSystemCalibration(const CoordCalibrateByToolEndPoint &coordCalibrate,
                                           double bInWPos[3], double bInWOri[9], double wInBPos[3]);

    //坐标转换   基座标系转用户坐标系
    static int base2UserCoordinatePrivate(const aubo_robot_namespace::wayPoint_S     &flangeCenterPointOnBase,       //基于基座标系的法兰盘中心路点信息
                                          const CoordCalibrateByToolEndPoint &userCoordSystem,                       //用户坐标系
                                          const aubo_robot_namespace::ToolInEndDesc  &toolInEndDesc,                 //工具信息
                                          aubo_robot_namespace::wayPoint_S           &toolEndPointOnUserCoord);      //得到结果为基于用户坐标系的工具末端的路点信息

    //将用户坐标系下的工具末端点A　转换成　基座标下法兰盘中心点A'
    static int user2BaseCoordinatePrivate(const aubo_robot_namespace::wayPoint_S     &toolEndPointOnUserCoord,     //用户坐标系下的工具末端点A
                                          const CoordCalibrateByToolEndPoint         &userCoordSystem,             //用户坐标系
                                          const aubo_robot_namespace::ToolInEndDesc  &toolInEndDesc,               //工具信息
                                          aubo_robot_namespace::wayPoint_S           &flangeCenterPointOnBase);    //基座标下法兰盘中心点A'


public:
    /**
     * @brief CoordCalibrateTypeConvert  坐标系标定类型转换  将用户提供的法兰盘中心点转成基于基座标的工具末端点
     * @param source
     * @param target
     * @return
     */
    static int  CoordCalibrateTypeConvert(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &source,
                                          CoordCalibrateByToolEndPoint &target);
};

#endif // ROBOTUTILSERVICE_H
