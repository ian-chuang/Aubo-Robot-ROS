#ifndef ROBOTCONVEYORTRACK_H
#define ROBOTCONVEYORTRACK_H

#include "robotcontrolservices.h"

class RobotConveyorTrack
{
public:
    RobotConveyorTrack(RobotControlServices *p);

public:
    int setConveyorEncoderReset(void);

    /**
     * @brief 启动传送带
     * @return
     */
    int setConveyorStartup(void);

    /**
     * @brief 停止传送带
     * @return
     */
    int setConveyorStop(void);

    /**
     * @brief 设置传送带方向
     * @param dir
     * @return
     */
    int setConveyorDir(int dir);


    /**
     * @brief 设置手眼标定结果关系
     * @param robotCameraCalib
     */
    int setRobotCameraCalib(const aubo_robot_namespace::RobotCameraCalib &robotCameraCalib);


    /**
     * @brief 设置传送带线速度
     * @param conveyorVelc (米/秒）
     */
    int setConveyorVelc(const double &conveyorVelc);


    /**
     * @brief 设置编码器距离关系
     * @param encoderValPerMeter 编码器距离关系（编码器脉冲个数/米)
     */
    int setEncoderValPerMeter(const uint32_t &encoderValPerMeter);


    /**
     * @brief 设置传送带起始窗口上限
     * @param startWindowUpstream　单位：米
     */
    int setStartWindowUpstream(double startWindowUpstream);


    /**
     * @brief 设置传送带起始窗口下限
     * @param startWindowDownstream 单位：米
     */
    int setStartWindowDownstream(double startWindowDownstream);

    /**
     * @brief 设置传送带跟踪轨迹下限 单位：米
     * @param trackDownstream
     */
    int setConveyorTrackDownstream(double trackDownstream);

    /**
     * @brief 设置是否允许启动传送带跟踪
     * @param enableConveyorTrack
     */
    int enableConveyorTrack();

    /**
     * @brief  获取传送带编码器数值
     * @return 传送带编码器数值
     */
    int getConveyorEncoderVal(int &value);

    /**
     * @brief 添加物体对象到传送带跟踪队列
     * @param objectPos　　物体位置
     * @param objectOri　　物体姿态
     * @param cameraProcTimeStamp　相机处理时间戳
     * @return
     */
    int appendObject2ConveyorTrackQueue(const aubo_robot_namespace::Pos &objectPos, const aubo_robot_namespace::Ori &objectOri, uint32_t timestamp);


    int setRobotConveyorTrackMaxVelc(double robotConveyorTrackMaxVelc);

    int setRobotConveyorTrackMaxAcc(double robotConveyorTrackMaxAcc);

    int setRobotConveyorSystemDelay(double robotConveyorSystemDelay);

    int setRobotTool(const aubo_robot_namespace::ToolInEndDesc &robotTool);

private:
    RobotControlServices          *m_robotBaseService;


};

#endif // ROBOTCONVEYORTRACK_H
