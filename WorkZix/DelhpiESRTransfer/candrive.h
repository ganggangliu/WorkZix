// 文件名: candrive.h
// 说明: 为了统一周立功CAN读取和Kavser读取
//          而添加的头文件。
// 作者: 陈波
// 创建日期: 2016-06-01
//

#ifndef CAN_DRIVE_H
#define CAN_DRIVE_H

#ifdef _MSC_VER
    #include <Windows.h>
#endif

#include "transplant.h"

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    #include "vcicandrive.h"
#elif (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    #include "kvasercandrive.h"
#elif (CAN_DEV_TYPE == CAN_DEV_TYPE_VECTOR)
#endif


class CanDrive
{
public:
    CanDrive();
    ~CanDrive();

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

public:
    // 设备类型号, ie. USBCAN1 3, USBCAN2 4
    unsigned long m_deviceType;
    // 设备索引号，当只有一个USBCAN时，索引号为0，有两个时可以为0或1。
    unsigned long m_deviceInd;
    // 第几路CAN
    unsigned long m_canInd;
    // 初始化参数结构
    CAN_CONFIG m_canConfig;


private:
#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    VciCanDrive m_hInstance;
#elif (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    KvaserCanDrive m_hInstance;
#elif (CAN_DEV_TYPE == CAN_DEV_TYPE_VECTOR)
#endif
};


#endif // CAN_DRIVE_H


