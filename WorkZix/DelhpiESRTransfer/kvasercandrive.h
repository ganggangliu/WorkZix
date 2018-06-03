// 文件名: kvasercandrive.h
// 说明: Kvaser CAN写与读数据
// 作者: 陈波
// 创建日期: 2016-06-01
//

#ifndef KVASER_CAN_DRIVE_H
#define KVASER_CAN_DRIVE_H

#include "transplant.h"
#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    #include "canlib.h"
#endif
#include "cancommon.h"



class KvaserCanDrive
{
public:
    typedef struct KVASER_CAN_CONFIG_
    {
        unsigned long	acc_code;//AccCode;    // 验收码
        unsigned long	acc_mask;//AccMask;    // 屏蔽码
        unsigned long	reserved;//Reserved;   // 保留
        unsigned char	filter;//Filter;     // 滤波方式，1表示单滤波，0表示双滤波
        // Timing0和Timing1用来设置CAN波特率, 00 1C - 500Kbps
        unsigned int       baud_rate; // BaudRate; unit is bps
        unsigned char	mode;//Mode;       // 模式，0表示正常模式，1表示只听模式
    } KVASER_CAN_CONFIG;

public:
    KvaserCanDrive();
    ~KvaserCanDrive();

    bool OpenDevice(unsigned long deviceType_,
                           unsigned long deviceInd_);
    bool CloseDevice(unsigned long deviceType_,
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

private:
    // 获取发送给Kvaser设备的波特率代号
    // freq 频率，单位是bps
    long GetBaudRate(unsigned int uFreq);
    // 设备类型号, ie. USBCAN1 3, USBCAN2 4
    unsigned long m_deviceType;
    // 设备索引号，当只有一个USBCAN时，索引号为0，有两个时可以为0或1。
    unsigned long m_deviceInd;
    // 第几路CAN
    unsigned long m_canInd;
    // 初始化参数结构
    KVASER_CAN_CONFIG m_canConfig;

    // 打开通道的方式
    int m_iOpenFlag;

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    CanHandle m_canHandle;
#endif
};


#endif // KVASER_CAN_DRIVE_H


