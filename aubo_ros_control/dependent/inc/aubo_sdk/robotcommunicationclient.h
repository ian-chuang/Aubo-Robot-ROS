#ifndef ROBOTCOMMUNICATIONCLIENT_H
#define ROBOTCOMMUNICATIONCLIENT_H

#include <stdlib.h>
#include <iostream>
#include <pthread.h>
#include <vector>

#include "communicationmatetype.h"


class RobotCommunicationClient
{
    enum {RECV_BUFFER_SIZE = 2*1024*1024};                    //接收缓冲区的大小   1M=6000waypoint
    enum {SEND_BUFFER_SIZE = RECV_BUFFER_SIZE-256};           //发送缓冲区的大小

    enum {SIGNLE_PACKAGE_MESSAGE_LEN = 1*1024};               //单包数据的最大

    enum {ROBOT_COMMAND_SOF_BYTE = 4};                        //数据报协议帧头段所占字节数
    enum {ROBOT_COMMAND_LEN_BYTE = 4};                        //数据报描述数据长度段所占字节数
    enum {ROBOT_COMMAND_CRC_BYTE = 4};                        //数据报校验段所占字节数
    enum {ROBOT_COMMAND_END_BYTE = 4};                        //数据报帧尾段所占字节数
    enum {ROBOT_COMMAND_MIN_BYTE = ROBOT_COMMAND_SOF_BYTE+ROBOT_COMMAND_LEN_BYTE+ROBOT_COMMAND_CRC_BYTE+ROBOT_COMMAND_END_BYTE};  //一个数据报的最小长度


public:
    RobotCommunicationClient();
    ~RobotCommunicationClient();


private:
    void         initClassProfile();

protected:

    void         setServerIP(std::string host);

    void         setServerPort(int port);

    int          connectRobotServer();               //与Robot建立连接并创建线程去接收消息

    int          disconnectRobotServer();            //断开与Robot的连接

    bool         encodeAndSendCommunicationPackage(const CommunicationRequest &request);    //组装报文并发送

public:
    bool         getCurrentConnectStatus();          //获取当前连接状态

private:

    int          connectCommunicationServer();

    int          disconnectCommunicationServer();

    ssize_t      recvMessageBaseMethed(int sockfd, void *buff, size_t len, int flags);

    ssize_t      recvPeek    (int sockfd, void *buff, size_t len);

    ssize_t      recvCommunicationData (int sockfd, void *buff, size_t len);

    ssize_t      sendCommunicationData(int sockfd, const char *buff, int len);                   //向客户端发送消息


private:
    ssize_t      recvRobotMessageHead(int sockfd);                                      //接收消息帧头

    bool         checkMessageFormat  (const void *buff, int length);                    //检查包尾是否正确

    ssize_t      recvRobotCommunicationPackage(int sockfd, char *recvBuffer, int bufferSize); //从socket中读取一包数据

    bool         encodeAndSendCommunicationPackage(int fd, const CommunicationRequest &request);    //组装报文并发送

    bool         makeDisconnectEventCommunicationPackage(char *buff, int bufferSize, int *buffValidSize);

    void         recvCommunicationPackageLoop();                                              //循环接收Robot响应消息


private:

    static void   *eventLoopThread (void *args);                               //接收服务器响应的线程

    static void   *communicationHeartbeatThread(void *args);

    virtual       void responseProcess(int fd, const char *buff, int size) = 0;   //响应处理

    virtual       void disconnectProcess() = 0;   //断开连接处理

    virtual       int  heartbeatService() = 0;   //通信心跳服务



private:
    pthread_t           m_threadId;                   //线程ID

    pthread_t           m_communicationHeartbeatThreadId;   //通信心跳线程ID

    std::string         m_serverIP;                   //服务器IP地址

    int                 m_serverport;                 //服务器端口

    int                 m_socketFd;                   //与服务器建立连接的Socket

    char               *m_recvBuffer;                 //接收消息缓冲区,用于存放来自服务器的消息

    char               *m_sendBuffer;                 //发送消息缓冲区

    bool                m_connectStatus;              //连接状态

    pthread_mutex_t     m_sendRequestMutex;           //多线程同时发送消息的互斥锁

private:
    static  unsigned char ROBOT_COMMAND_SOF[ROBOT_COMMAND_SOF_BYTE];   //协议帧头
    static  unsigned char ROBOT_COMMAND_CRC[ROBOT_COMMAND_CRC_BYTE];   //默认校验字段
    static  unsigned char ROBOT_COMMAND_END[ROBOT_COMMAND_END_BYTE];   //协议帧尾

    static  bool s_dataRecvThreadRunningFlag;
    static  bool s_communicationHeartbeatThreadRunningFlag;
};

#endif // ROBOTCOMMUNICATIONCLIENT_H
