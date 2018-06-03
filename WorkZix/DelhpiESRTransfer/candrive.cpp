// 文件名: candrive.cpp
// 说明: CAN数据发送和读取
// 作者: 陈波
// 创建日期: 2016-06-01
//

#include "candrive.h"
#include "debugfunction.h"


CanDrive::CanDrive()
{
    m_deviceType = 4/*16*/;	// 设备类型号(USBCAN2)
    m_deviceInd = 0;	// 设备索引号(只有一个USBCAN)
    m_canInd = 1;		// 第几路CAN
    m_canConfig.acc_code = 0x00000000;	// 验收码
    m_canConfig.acc_mask = 0xFFFFFFFF;	// 屏蔽码
    m_canConfig.reserved = 0;
    m_canConfig.filter = 1;				// 滤波方式，1表示单滤波，0表示双滤波
    // Timing0和Timing1用来设置CAN波特率, 00 1C - 500Kbps
    m_canConfig.baud_rate = 500 * 1000;
    m_canConfig.mode = 0;				// 模式，0表示正常模式，1表示只听模式
}

CanDrive::~CanDrive()
{
    // close can di
    // this->close();
}

bool CanDrive::OpenDevice(
                unsigned long deviceType_,
                unsigned long deviceInd_)
{
    return (m_hInstance.OpenDevice(deviceType_, deviceInd_));
}

bool CanDrive::CloseDevice(
            unsigned long deviceType_,
            unsigned long deviceInd_)
{
    return (m_hInstance.CloseDevice(deviceType_, deviceInd_));
}

bool CanDrive::Connect(
             unsigned long deviceType_,
             unsigned long deviceInd_,
             unsigned long canId_,
             const CAN_CONFIG& config_)
{
    m_deviceType = deviceType_;
    m_deviceInd = deviceInd_;
    m_canInd = canId_;
    m_canConfig = config_;

    COM_INFO(3, ("%s\n", __FUNCTION__));
    return (m_hInstance.Connect(deviceType_, deviceInd_, canId_, m_canConfig));
}

bool CanDrive::Start(void)
{
    return (m_hInstance.Start());
}

bool CanDrive::Stop(void)
{
    return (m_hInstance.Stop());
}

bool CanDrive::Reset(void)
{
    return (m_hInstance.Reset());
}

unsigned long CanDrive::Send(CAN_OBJ& data, unsigned long count)
{
    return (m_hInstance.Send(data, count));
}

unsigned long CanDrive::GetRecvNum(void)
{
    return (m_hInstance.GetRecvNum());
}

unsigned long CanDrive::Receive(CAN_OBJ& data, unsigned long count, int waitTime)
{
    return (m_hInstance.Receive(data, count, waitTime));
}

