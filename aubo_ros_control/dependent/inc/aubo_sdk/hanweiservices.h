#ifndef HANWEISERVICES_H
#define HANWEISERVICES_H

#include <vector>
#include "AuboRobotMetaType.h"

class RobotControlServices;
class HanWeiServices
{
public:
    typedef struct
    {
        aubo_robot_namespace::wayPoint_S flangeWayPoint;

        aubo_robot_namespace::Pos        endPosition;

        aubo_robot_namespace::Ori        endOrientation;

    }FlangeAndToolInfo;


public:
    HanWeiServices(RobotControlServices *p);

    int  safeLimitParamInit();

    int  getMovePJointRangeService(double minJointLimit[6], double maxJointLimit[6]);

    int  setMovePJointRangeService(const double minJointLimit[6], const double maxJointLimit[6]);

public:
    //设置关节范围限制
    void setJointRangeLimit(const double *jointMinLimit, const double *jointMaxLimit);

    //是否满足关节范围限制
    bool isMeetJointRangeLimit(const double *jointData);

    //判断两组关节角间是否满足  关节角差值限制
    bool isMeetJointAngleDiffLimit(const double *jointGroup1, const double *jointGroup2);

    //判断两路点间是否满足  距离差值限制
    bool isMeetDistDiffLimit(const double *jointGroup1, const double *jointGroup2);

    //判断两路点间是否满足  距离差值限制
    bool isMeetDistDiffLimit(const aubo_robot_namespace::Pos &position1, const aubo_robot_namespace::Pos &position2);

    //安全围栏限制
    bool safetyFenceLimit(const aubo_robot_namespace::Pos &position1);

    //判断是否满足限制
    bool isMeetLimit(const std::vector<FlangeAndToolInfo> &flangeAndToolInfoVector);

    //获取路点容器
    const std::vector<FlangeAndToolInfo> &getFlangeAndToolInfoVector();

    //获取分割索引容器
    std::vector<int> &getSplitIndexVector();

public:
    //基础方法:转换位姿文件为路点集合
    int  parseFileAsRoadpointSet(const char *filePath, aubo_robot_namespace::POSITION_ORIENTATION_TYPE poseType,
                                 const double *referPointJointAngle, const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc,
                                 std::vector<FlangeAndToolInfo> &roadpointInfoVector);

    //基础方法:将位姿列表转换成路点集合
    int  parsePoseListToRoadpointSet(const std::vector<aubo_robot_namespace::PositionAndQuaternion> &toolEndPoseVector,            //位姿集合
                                     const double *referPointJointAngle, const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc, //工具参数
                                     std::vector<FlangeAndToolInfo> &roadpointInfoVector);
    //滤波处理
    void trackFilter(std::vector<FlangeAndToolInfo> &flangeAndToolInfoVector);

    //轨迹分割处理
    void trackSplit(const std::vector<HanWeiServices::FlangeAndToolInfo> &flangeAndToolInfoVector, std::vector<int> &split_start_index);

    //轨迹平滑和安全限制检查
    int trackSmoothAndSafeCheck(std::vector<FlangeAndToolInfo> &wayPointVector, bool isSafeCheck, bool isSmoothFilter, bool isSplit, std::vector<int> &splitPointIndex);

    //生成轨迹并进行平滑处理
    int makeTrackByFileAndSmoothHandle(const char *filePath, aubo_robot_namespace::POSITION_ORIENTATION_TYPE poseType, const double *referPointJointAngle,
                        const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc, std::vector<FlangeAndToolInfo> &wayPointVector);

    //生成轨迹并进行平滑处理
    int makeTrackByFileAndSmoothSplitHandle(const char *filePath, aubo_robot_namespace::POSITION_ORIENTATION_TYPE poseType, const double *referPointJointAngle,
                        aubo_robot_namespace::ToolInEndDesc &toolInEndDesc, aubo_robot_namespace::wayPoint_S &firstWayPoint);

    int makeTrackByPoseList(const double *referPointJointAngle,                                                 //参考点
                            const aubo_robot_namespace::ToolInEndDesc &toolInEndDesc,                           //工具参数
                            const std::vector<aubo_robot_namespace::PositionAndQuaternion> &toolEndPoseVector,  //位姿集合
                            std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);

public:
    //参考网址：http://www.360doc.com/content/19/0324/19/7378868_823866124.shtml

    // 1.绕Z轴旋转γ角  将空间点绕Z轴旋转
    // 公式表示: x′=cosγ⋅x−sinγ⋅y   y′=sinγ⋅x+coγ⋅y  z′=z
    // 输入参数x y为空间点原始x y坐标; thetaz为空间点绕Z轴旋转多少度，角度制范围在-180到180;   outx outy为旋转后的结果坐标
    void codeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);

    // 2.绕Y轴旋转β角 将空间点绕Y轴旋转
    // 公式表示： x′=cosβ⋅x+sinβ⋅z   y′=y    z′=−sinβ⋅x+cosβ⋅z
    // 输入参数x z为空间点原始x z坐标; thetay为空间点绕Y轴旋转多少度，角度制范围在-180到180;  outx outz为旋转后的结果坐标
    void codeRotateByY(double x, double z, double thetay, double& outx, double& outz);

    // 3.绕X轴旋转α角 将空间点绕X轴旋转
    // 公式表示： x′=x  y′=cosα⋅y−sinα⋅z   z′=sinα⋅y+sinα⋅z
    //将空间点绕X轴旋转  输入参数 y z为空间点原始y z坐标;   thetax为空间点绕X轴旋转多少度，角度制范围在-180到180;   outy outz为旋转后的结果坐标
    void codeRotateByX(double y, double z, double thetax, double& outy, double& outz);

    //将字符串转换成 位置+欧拉角  按照空格分割
    bool StringToPositionAndRpy(const std::string &str, char splitSymbol, aubo_robot_namespace::Pos &position, aubo_robot_namespace::Rpy &rpy);

    //将字符串转换成 位置+姿态  按照逗号分割
    bool StringToPositionAndQuaternion(const std::string &str, char splitSymbol, aubo_robot_namespace::Pos &position, aubo_robot_namespace::Ori &quaternion);

private:
    RobotControlServices   *m_robotBaseService;    //机械臂基础服务

private:

    //关节范围  最小值限制
    double m_jointRangeMinLimit[6];

    //关节范围  最大值限制
    double m_jointRangeMaxLimit[6];

    //路点之间 最大距离限制  单位:毫米 mm
    double m_maxDistDiffLimit;

    //路点之间 最大关节角限制  单位:弧度
    double m_maxjointAngleDiffLimit[6];

    //分割点索引容器
    std::vector<int> m_splitStartIndex;

    //路点容器
    std::vector<FlangeAndToolInfo> m_wayPointVector;

public:
    //static std::string s_version;

    static std::string s_loginName;
};

#endif // HANWEISERVICES_H







/**
  路点文件处理步骤：
    step: 转换位姿文件为路点容器  convertPoseFileToVector
    step: 对逆解结果进行  异常判断
    step: 滤波 trackFilter
    step: 对滤波后的结果进行 异常判断
    step: 轨迹分割处理 trackSplit

  运动步骤

 1. 进行关节范围限制
 2. 对路点之间  进行关节差值限制  和  距离限制    避免逆解选解问题

 注：这样的操作  滤波前做一次  滤波后做一次


    Min joint angle : 0  -0.585678    -33.5568
    Max joint angle : 0  0.497190      28.4868       (-45  40)

    Min joint angle : 1  -0.232465     -13.3192      (-25  50)
    Max joint angle : 1  0.691401      39.61435

    Min joint angle : 2  -2.491911     -142.7759     (-155  -100)
    Min joint angle : 2  -1.991516     -114.1054

    Min joint angle : 3  -2.968009     -170.0538     (-172  -40)
    Max joint angle : 3  0.480152      27.5106

    Min joint angle : 4  -2.036965     -116.70949    (-130  -60)
    Min joint angle : 4  -1.299234     -74.44062

    Min joint angle : 5  -1.151031     -65.949218    (-90    90)
    Max joint angle : 5  0.935766      53.6154

*/
