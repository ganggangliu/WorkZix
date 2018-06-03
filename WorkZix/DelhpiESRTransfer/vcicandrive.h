// 文件名: VciCanDrive.h
// 说明: 周立功CAN数据发送和读取
// 作者: 崔鹏、陈波
// 创建日期: 2016-06-01
//


#ifndef VCI_CAN_DRIVE_H
#define VCI_CAN_DRIVE_H

#ifdef WIN32
    #include <Windows.h>
#endif

#include "cancommon.h"




// 单次发送和接收8字节数据时的最大个数
#define VCI_MAX_BUFF_CNT 16

class VciCanDrive
{
public:
    typedef struct VCI_CAN_CONFIG_
    {
        unsigned long	AccCode;    // 验收码
        unsigned long	AccMask;    // 屏蔽码
        unsigned long	Reserved;   // 保留
        unsigned char	Filter;     // 滤波方式，1表示单滤波，0表示双滤波
        // Timing0和Timing1用来设置CAN波特率, 00 1C - 500Kbps
        unsigned char	Timing0;    // 定时器0
        unsigned char	Timing1;    // 定时器1
        unsigned char	Mode;       // 模式，0表示正常模式，1表示只听模式
    } VCI_CAN_CONFIG;

public:
	VciCanDrive();
	~VciCanDrive();

    static bool OpenDevice(unsigned long deviceType_,
                           unsigned long deviceInd_);
    static bool CloseDevice(unsigned long deviceType_,
                            unsigned long deviceInd_);
    bool Connect(unsigned long deviceType_,
                 unsigned long deviceInd_,
                 unsigned long canId_,
                 const CAN_CONFIG& config_);
    bool Start(void);
    bool Stop(void);
    bool Reset(void);
    unsigned long Send(CAN_OBJ& data, unsigned long count);
    unsigned long GetRecvNum(void);
    unsigned long Receive(CAN_OBJ& data, unsigned long count, int waitTime=-1);

public:
	// 设备类型号, ie. USBCAN1 3, USBCAN2 4
    unsigned long m_deviceType;
	// 设备索引号，当只有一个USBCAN时，索引号为0，有两个时可以为0或1。
    unsigned long m_deviceInd;
	// 第几路CAN
    unsigned long m_canInd;
	// 初始化参数结构
    VCI_CAN_CONFIG m_canConfig;
};

#endif // VCI_CAN_DRIVE_H
