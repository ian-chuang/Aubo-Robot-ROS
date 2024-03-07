#ifndef FORCECONTROL_H
#define FORCECONTROL_H


#include "AuboRobotMetaType.h"

typedef struct
{
    double force[3];    //力
    double torque[3];   //力矩
}WrenchParam;

typedef struct
{
    WrenchParam  offset;       //力传感器六个维度数据(力+力矩)
    double       mass;         //质量
    double       centroid[3];  //质心
    double       angle[2];     //安装角度
}FtSensorCalResult;


class RobotControlServices;
class ForceControl
{
public:
    ForceControl(RobotControlServices *p);

    int  getForceSensorData(WrenchParam &data);

    int  CalibToolAndSensor(aubo_robot_namespace::JointParam JointParamGroup[3], WrenchParam wrenchParamGroup[3], FtSensorCalResult &result);

    int  EnableForceControlPlan(bool enableFlag);

    int  SetForceControlAttributes(int type, double data[6]);

    int  SetForceDeviation(double data[6]);

    int  SetForceMaxValue(double data[6]);

    int  SetForceControlStiffness(double data[6]);

    int  SetForceControlDamp(double data[6]);

    int  SetForceControlMass(double data[6]);

    int  forceControlCalibrationZero();

private:
    RobotControlServices    *m_robotBaseService;               //机械臂基础服务

};

#endif // FORCECONTROL_H
