#ifndef IKFUNC_H
#define IKFUNC_H

#include <stdlib.h>
#include <vector>
#include "our_alg_i5p.h"
#ifndef WIN_NO_ARAL
#include "aral/robot_library_interface.hpp"
#endif


namespace MoveConditionClass
{
    enum coordinate_refer
    {
        BaseCoordinate = 0,
        EndCoordinate,
        WorldCoordinate
    };
}

typedef struct
{
    float dA[6];
    float dD[6];
    float dAlpha[6];
    float dTheta[6];
    float dBeta[6];
//    float dZeroOff[6];
}RobotSystemParamCalib_S;

typedef struct
{
    double minValue;
    double maxValue;
}RangeOfMotion;

/**
 * 关节运动范围
 */

typedef struct
{
    bool   enable;                        // 是否使能偏移
    RangeOfMotion rangeValues[ARM_DOF];    //运动范围 弧度
}JointRangeOfMotion;

#define DH_PARA_COMP_THR 0.000005*0.000005
#define DH_PARA_COMP_ORI_THR 0.00001*0.00001
#define DH_PARA_COMP_ERR_THR 0.001
#define DH_PARA_COMP_ERR_THR_JOINT 0.05
#define DH_PARA_COMP_MAX_COUNT 15
#define JOINT_MAX_POS  3.05345319099008

class Ikfunc
{
public:
    explicit Ikfunc();
    static void setJointLimits(const double* upper_bound, const double* lower_bound);
    //! 通过设置机械臂名称来判断是否为I系列机械臂
    static bool SetRobotType(const std::string& modelName, const std::string& config = "");
    static bool SetDhParaLen(aubo_robot_type robotType);
    //! 设置机械臂DH参数误差到aral中
    static bool SetDhError(const RobotSystemParamCalib_S& err);

    //2  (inverse)kinematics
    static bool ArmIkProtect(RoadPoint &res, double *last_jointpos, bool iterIK = false, bool relMove = false);
    static bool ArmFk(Pos &arm_pos, Ori &arm_ori, double joint_solve[ARM_DOF], bool enableCom = false); //for user
    static bool ArmFk(RoadPoint &arm_pos);
    static void ArmFk(const double joints[ARM_DOF], double *eetrans, double *eerot);
    static int  ComputeIk_new(RoadPoint &cur_stat, IKREAL_TYPE jointResults[ARM_DOF][8]);
    static int  ComputeIk_new_F(RoadPoint &cur_stat, IKREAL_TYPE jointResults[ARM_DOF][64]);
    static bool ArmIk(RoadPoint &cur_stat, double *joint = NULL, bool iterIK = false, bool relMove = false);
    static void extendToAllConfigurations(const int solution_num, const IKREAL_TYPE q_sols[ARM_DOF][8], std::vector<std::vector<double> > & q_out);
    static void extendToAllConfigurations_F(const int solution_num, const IKREAL_TYPE q_sols[ARM_DOF][64], std::vector<std::vector<double> > & q_out);

    static void waypoint2Pose(Pose_S *posePtr, RoadPoint *wpPtr, int size);
    static void pose2Waypoint(RoadPoint *wpPtr, Pose_S *posePtr, int size);
    static void QuaternionToOriMatrix(Ori q, IKREAL_TYPE eerot[]);
    static void posOri2homoArr(cartesianPos_U position, Ori quat, IKREAL_TYPE eerot[], IKREAL_TYPE eetrans[]);
    static bool userCoordinateCalib(Pose_S *waypointptr,char methods, IkReal *bInWPos = NULL, IkReal *bInWOri = NULL, IkReal *wInBPos = NULL);
    static bool toolCoordinateCalib(Pose_S *waypointptr,int posWpNum=4,bool waypoint_check=false,double *toolInEndPos=NULL,
                                    char poseCalibMethod=-1, Pose_S *wpOriCalib=NULL, Ori *toolInEndOri=NULL);

    static bool api_arm_ik(Pos arm_pos, Ori arm_ori, double joint_solve[ARM_DOF]);
    static bool api_user_coord_calib(Pose_S waypointptr[3],char methods, IkReal bInWPos[3], IkReal bInWOri[9], IkReal wInBPos[3]);
    static bool api_tool_coord_calib(Pose_S *wpPosCalib, unsigned int wpPosCalibSize, Pose_S *wpOriCalib, char poseCalibMethod, Pose_S &toolInEnd);
    static double SIGN(double x);
    static double NORM(double a, double b, double c, double d);
    static void setDhParameters(aubo_robot_type robotType, double A3, double A4, double D1, double D2, double D5, double D6);

    static bool OriMatrixToQuaternion(double eerot[], Ori &result);
    static void RPYToQuaternion(const double* rpy, Ori& res);
    static void RPYToRotation(const double* rpy, double* eerot);
    static void QuaternionToRPY(const Ori& res, double* rpy);


    static void hMatrixMultiply(IkReal eerot[], IkReal *eetrans, IkReal eerot1[], IkReal *eetrans1, RoadPoint *pos_transferred, IkReal *new_rot = NULL, IkReal *new_pos = NULL);
    static void hMatrixMultiply(IkReal eerot[], IkReal *eetrans, IkReal eerot1[], IkReal *eetrans1, Pose_S *pos_transferred, IkReal *new_rot = NULL);
    static void QuaternionMultply(Ori &end_ori,Ori left_ori, bool rightMul = false);
    static void tr2Delta(const IkReal eerot1[], const IkReal *eetrans1, const IkReal eerot2[], const IkReal eetrans2[], IkReal twist[]);
    static void getJacobian(const double joints[ARM_DOF], double jac[]);
    static void multiplyJacobianInv(const double tw[ARM_DOF], const double jac[], double joint[ARM_DOF]);
    static void rotationMultiply(const IkReal itrot[], const IkReal toolRot[], IkReal *ori);

private:
    static void QuaternionInversion(Ori &cur_ori);
    static void hMatrixVectorProduct(bool inv, IkReal *pOrg, IkReal *hRArr, bool relPos, IkReal *posIn, IkReal *posOut);
    static bool coordinateTransform(Pose_S &pos_transferred, Pose_S pos, double *toolOrgInEndPos, Ori *ToolOriInEnd, MoveConditionClass::coordinate_refer coord, RoadPoint *planWp = NULL, char methods = 0);//coordinate switch (for display for example).

public:
    static void toolPosition2EndPosition(Pose_S tool_stat, cartesianPos_U &end_position, double *toolPosInEnd = NULL);
    static void endPosition2ToolPosition(Pose_S &end_stat, double *toolPosInEnd, IKREAL_TYPE *toolPosition = NULL);
    static void endOrientation2ToolOrientation(Ori tcpOriInE, Ori endOri, Ori &toolOri);
    static void toolOrientation2EndOrientation(Ori tcpOriInE, Ori toolOri, Ori &endOri);

public:

    /**
     * @brief waypointDisplay    基座标　转　任意坐标
     * @param target_wp
     * @param source_wp
     * @param refCoord
     * @param ToolOrgInEndPos
     * @param planWp
     * @param methods
     */
    static bool waypointDisplay(Pose_S &target_wp,          //目标
                                const Pose_S source_wp,     //BASE下无工具  源
                                MoveConditionClass::coordinate_refer refCoord = MoveConditionClass::BaseCoordinate,
                                double *ToolOrgInEndPos = NULL,  //非空　　　提供工具的坐标X,Y,Z   [3]
                                Ori *ToolOriInEnd = NULL,
                                RoadPoint *planWp = NULL,        //标定平面的３个点
                                char methods = 0);               //标定方法




    /**
     * @brief robotCoordConvert   任意坐标系 转  任意坐标系
     * @param target_point
     * @param source_point
     * @param userPlaneWP
     * @param method
     * @param tcpPosInEnd
     * @param targetCoord
     * @param sourceCoord
     * @return
     */
    static bool robotCoordConvert(Pose_S &target_point,
                                  Pose_S source_point,
                                  Pose_S userPlaneWP[3],
                                  char method,
                                  double *tcpPosInEnd,
								  Ori *tcpOriInEnd,
                                  MoveConditionClass::coordinate_refer targetCoord,
                                  MoveConditionClass::coordinate_refer sourceCoord);

    static void quaternionToRPY(Ori ori, float *rpy);
    static void RPYToQuaternion(Ori &ori, float *rpy);

    static bool joint6Rot360;
    static bool joint1Rot360;
    static bool dh_real;    // compensate dh parameter error.
    static double dh_vector[4*ARM_DOF];
    static RobotSystemParamCalib_S RobotSystemParamCalib;
    static JointRangeOfMotion jointLimit;
    static bool i_series_robot;
#ifndef WIN_NO_ARAL
    static ARAL::interface::ARALIntfacePtr rl_interface;
#endif

    /**
    当源坐标系是用户坐标系时：source_point　里面的姿态也必须是基于用户坐标系。

    任意坐标转任意坐标　这个函数永远是“减去TCP”
    1: 源带Tcp  　　Tcp参数为正===>  目标不带Tcp
       应用:MoveLto 因为用户一般提供基于用户平面TCP末端点的位置信息　　而我们最后需要基于基座标法兰盘中心的点

    2: 源不带Tcp    Tcp参数为负===>　目标带Tcp  与waypointDisplay函数的功能一样
       应用:示教界面显示的时候，需要将基于基座标系法兰盘中心的点转成基于用户坐标系Tcp末端点

    3: 任意坐标系下的点　不提供Tcp参数　===>  任意坐标系下的点
       应用：基于用户平面的偏移转成基于基座标系的偏移

    注：如果提供ＴＣＰ参数，源点的姿态必须提供，否则算法无法真正确定末端位置
    **/
};
#endif
