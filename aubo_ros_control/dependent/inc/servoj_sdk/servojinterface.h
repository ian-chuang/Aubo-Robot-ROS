#ifndef SERVOJINTERFACE_H
#define SERVOJINTERFACE_H

#include <memory>

class ServojInterfaceImpl;
class ServojInterface
{
public:
    ServojInterface();
    ~ServojInterface();

    /**
    * @brief 登录到 servoj 服务器，端口默认为 6000
    * @param
    * @return 0 成功 其他失败
    */
    int login(const char *ip, int port, const char *username, const char *password);

    /**
    * @brief 登出 servoj 服务器
    * @return 0 成功 其他失败
    */
    int logout();

    /**
    * @brief 进入 servoj 模式
    * 注意：进入 servoj 模式同时也会进入 tcp2can 透传模式，此时无法通过其他方式控制机器人运动，比如示教器上的示教按钮
    * servojclient 一旦断开与 servoj 服务器的连接，会自动退出 servoj 模式
    * @return 0 成功 其他失败
    */
    int startServoj();

    /**
    * @brief 退出 servoj 模式
    * @return 0 成功 其他失败
    */
    int stopServoj();

    /**
    * @brief 下发 servoj 目标点
    * @param q : 目标点，为6个关节角，单位弧度
    * @param duration : 目标点的运动时间间隔, 且必须大于插补周期(5ms)
    * @param smooth_scale : 运动平滑程度, 设置范围(0, 1], 1表示平滑性最好
    * @param delay_sacle : 运动延迟程度, 设置范围(0, 1], 1表示跟踪响应最快
    * @param is_last_point : false 跟踪模式
    *						 true  如果当前目标点速度加速度不为0,则会自动规划减速到0的运动(如果在减速过程中用户提供了新的目标点,
    *							   则会结束减速,继续跟踪新的目标点).

    * @return 0: 成功 ； 其他：失败
    */
    int servoj(const double q[6], double duration, double smooth_scale, double delay_sacle, bool is_last_point = false);

    /**
    * @brief 下发 servoj 目标点
    * @param q : 目标点，为6个关节角，单位弧度
    * @param v : 关节最大速度限制，为6个关节角，单位弧度，如果为0，表示以关节自身的最大速度作为限定值
    * @param a : 关节最大加速度限制，为6个关节角，单位弧度，如果为0，表示以关节自身的最大加速度作为限定值
    * @param duration : 目标点的运动时间间隔, 且必须大于插补周期(5ms)
    * @param smooth_scale : 运动平滑程度, 设置范围(0, 1], 1表示平滑性最好
    * @param delay_sacle : 运动延迟程度, 设置范围(0, 1], 1表示跟踪响应最快
    * @param is_last_point : false 跟踪模式
    *						 true  如果当前目标点速度加速度不为0,则会自动规划减速到0的运动(如果在减速过程中用户提供了新的目标点,
    *							   则会结束减速,继续跟踪新的目标点).

    * @return 0: 成功 ； 其他：失败
    */
    int servoj(const double q[6], const double v[6], const double a[6], double duration, double smooth_scale, double delay_sacle, bool is_last_point = false);


    /**
    * @brief 下发 servoj 目标点
    * @param q : 目标点，为6个关节角，单位弧度
    * @param v : 关节最大速度限制，为6个关节角，单位弧度，如果为0，表示以关节自身的最大速度作为限定值
    * @param a : 关节最大加速度限制，为6个关节角，单位弧度，如果为0，表示以关节自身的最大加速度作为限定值
    * @param duration : 目标点的运动时间间隔, 且必须大于插补周期(5ms)
    * @param smooth_scale : 运动平滑程度, 设置范围(0, 1], 1表示平滑性最好
    * @param delay_sacle : 运动延迟程度, 设置范围(0, 1], 1表示跟踪响应最快
    * @param total_delay_time : 传出参数，返回 servoj 实际总延迟，可用于控制发送频率
    * @param is_last_point : false 跟踪模式
    *						 true  如果当前目标点速度加速度不为0,则会自动规划减速到0的运动(如果在减速过程中用户提供了新的目标点,
    *							   则会结束减速,继续跟踪新的目标点).

    * @return 0: 成功 ； 其他：失败
    */
    int servoj(const double q[6], const double v[6], const double a[6], double duration, double smooth_scale, double delay_sacle, double &total_delay_time, bool is_last_point = false);

private:
    std::shared_ptr<ServojInterfaceImpl> impl_;
};

#endif // SERVOJINTERFACE_H
