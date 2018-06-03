// 文件名: kvasercandrive.cpp
// 说明: Kvaser CAN数据发送和读取
// 作者: 陈波
// 创建日期: 2016-06-01
//

#include "kvasercandrive.h"
#include "debugfunction.h"

KvaserCanDrive::KvaserCanDrive()
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

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    m_iOpenFlag = canOPEN_EXCLUSIVE;
#else
    m_iOpenFlag = 0;
#endif
}

KvaserCanDrive::~KvaserCanDrive()
{
    // close can di
    // this->close();
}

bool KvaserCanDrive::OpenDevice(
                unsigned long deviceType_,
                unsigned long deviceInd_)
{
    COM_INFO(3, ("%s, Kvaser_OpenDevice(%d, %d)\n", __FUNCTION__,
                 deviceInd_, m_iOpenFlag));

    (void)deviceType_;

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    int iChanCount = 0;
    canInitializeLibrary();
    unsigned short iCanVersion = canGetVersion();
    canStatus stat = canGetNumberOfChannels(&iChanCount);
    OutputInfo("Kvaser CAN version: %u.%u\n",
               iCanVersion >> 8, iCanVersion % 256);
    if (stat != canOK)
    {
        COM_ERR("%s, Kvaser get channels num err, err code = 0x%08x\n",
                __FUNCTION__, stat);
      return (false);
    }
    else if (iChanCount <= 0)
    {
        COM_ERR("%s, Kvaser not enough channel, num = %d\n",
                __FUNCTION__, iChanCount);
        return (false);
    }
    else
    {
        OutputInfo("Kvaser CAN channel num = %d\n", iChanCount);
    }

    m_canHandle = canOpenChannel(deviceInd_, m_iOpenFlag);
    COM_INFO(1, ("Open can status: %d\n", m_canHandle));
    if (m_canHandle < 0)
    {
        COM_ERR("%s, Kvaser_OpenDevice err, err code = 0x%08x\n",
                __FUNCTION__, m_canHandle);
        return (false);
    }
#endif // (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)

    return (true);
}

bool KvaserCanDrive::CloseDevice(
            unsigned long deviceType_,
            unsigned long deviceInd_)
{
    (void)deviceType_;
    (void)deviceInd_;

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    COM_INFO(3, ("%s, Kvaser_canClose(%d)", __FUNCTION__, m_canHandle));

    canStatus opStatus = canClose(m_canHandle);
    if (canOK != opStatus)
    {
        COM_ERR("%s, Kvaser_canClose err, err code = 0x%08x\n",
                __FUNCTION__, opStatus);

        return (false);
    }
#endif // (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)

    return (true);
}

bool KvaserCanDrive::Connect(
             unsigned long deviceType_,
             unsigned long deviceInd_,
             unsigned long canId_,
             const CAN_CONFIG& config_)
{
    m_deviceType = deviceType_;
    m_deviceInd = deviceInd_;
    m_canInd = canId_;
    m_canConfig.acc_code = config_.acc_code;
    m_canConfig.acc_mask = config_.acc_mask;
    m_canConfig.reserved = config_.reserved;
    m_canConfig.filter = config_.filter;
    m_canConfig.baud_rate = config_.baud_rate;
    m_canConfig.mode = config_.mode;

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    long freq = GetBaudRate(m_canConfig.baud_rate);

    COM_INFO(3, ("%s, canSetBusParams(%d, %ld)\n", __FUNCTION__,
                 m_canHandle, freq));
    canStatus opStatus = canSetBusParams(m_canHandle, freq, 0, 0, 0, 0, 0);
    if (canOK != opStatus)
    {
        COM_ERR("%s, canSetBusParams err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        return (false);
    }
#endif // (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)

    return (true);
}

bool KvaserCanDrive::Start(void)
{
    COM_INFO(3, ("%s, Kvaser_canBusOn()\n", __FUNCTION__));

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    canStatus opStatus = canBusOn(m_canHandle);
    if (canOK != opStatus)
    {
        COM_ERR("%s, canBusOn err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        return (false);
    }
#endif // (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)

    return (true);
}

bool KvaserCanDrive::Stop(void)
{
    COM_INFO(3, ("%s, Kvaser_canBusOff()\n", __FUNCTION__));

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    canStatus opStatus = canBusOff(m_canHandle);
    if (canOK != opStatus)
    {
        COM_ERR("%s, canBusOff err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        return (false);
    }
#endif // (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)

    return (true);
}

bool KvaserCanDrive::Reset(void)
{
#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    COM_INFO(3, ("%s, Kvaser_ResetCAN(%d)\n", __FUNCTION__, m_canHandle));

    canStatus opStatus = canResetBus(m_canHandle);
    if (canOK != opStatus)
    {
        COM_ERR("%s, Kvaser_ResetCAN err, err code = 0x%08x\n",
                __FUNCTION__, opStatus);
        return (false);
    }
#endif // (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)

    return (true);
}

unsigned long KvaserCanDrive::Send(CAN_OBJ& data, unsigned long count)
{

    unsigned long i = 0;

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    unsigned int flag = 0;
    canStatus opStatus = canOK;

    for (i = 0; i < count; i++)
    {
        if ((&data)[i].send_type == 0)
        {
            // 标准帧
            flag = canMSG_STD;
        }

        if ((&data)[i].remote_flag != 0)
        {
            flag = canMSG_RTR;
        }
        else if ((&data)[i].extern_flag != 0)
        {
            flag |= canMSG_EXT;
        }

        opStatus = canWrite(m_canHandle, (&data)[i].id, (&data)[i].data,
                            (&data)[i].data_len, flag);
        if (opStatus != canOK)
        {
            break;
        }
    }

    if (opStatus == canOK)
    {
            opStatus = canWriteSync(m_canHandle, 100);
    }
#else
    (void)data;
    (void)count;
#endif // (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)

    return (i);
}

unsigned long KvaserCanDrive::GetRecvNum(void)
{
    // Kvaser CAN驱动没有对应的函数
    return (0);
}

// count 最多接收的个数，超过了的话，就只读这么多。
// waitTime 没有数据时的等待时间（ms）
unsigned long KvaserCanDrive::Receive(CAN_OBJ& data, unsigned long count,
                                      int waitTime)
{
    unsigned long nReceiveCount = 0;

    (void)count;
    (void)waitTime;

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    canStatus opStatus = canOK;
    long lId;
    unsigned int uDlc;
    unsigned int uFlag;
    unsigned long ulTime;
    //unsigned long ulTimeout = waitTime;

    //COM_INFO(6, ("%s, VCI_Receive(%d, %d, %d, data, %d)", __FUNCTION__, m_deviceType, m_deviceInd, m_canInd, count));

    //opStatus = canReadWait(m_canHandle, &lId, data.data,
    //                       &uDlc, &uFlag, &ulTime, ulTimeout);
    opStatus = canRead(m_canHandle, &lId, data.data,
                           &uDlc, &uFlag, &ulTime);
    if (opStatus != canOK)
    {
        // 注意：如果没有读到数据则必须调用此函数来读取出当前的错误码，
        // 千万不能省略这一步（即使你可能不想知道错误码是什么）
        canStatus flushStatus = canFlushReceiveQueue(m_canHandle);
        if (canOK != flushStatus)
        {
            COM_ERR("%s, canFlushReceiveQueue err, err code = 0x%08x\n",
                    __FUNCTION__, flushStatus);
        }
    }
    else
    {
        nReceiveCount++;
    }
#else
    (void)data;
#endif // (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)

    return (nReceiveCount);
}

// 获取发送给Kvaser设备的波特率代号
// 输入参数：
//        freq 频率，单位是bps
long KvaserCanDrive::GetBaudRate(unsigned int uFreq)
{
#if (CAN_DEV_TYPE == CAN_DEV_TYPE_KVASER)
    long lReturn = BAUD_500K;

    switch(uFreq)
    {
        case 1000000:
        {
            lReturn = BAUD_1M;
            break;
        }
        case 500000:
        {
            lReturn = BAUD_500K;
            break;
        }
        case 250000:
        {
          lReturn = BAUD_250K;
          break;
        }
        case 125000:
        {
          lReturn = BAUD_125K;
          break;
        }
        case 100000:
        {
          lReturn = BAUD_100K;
          break;
        }
        case 62500:
        {
          lReturn = BAUD_62K;
          break;
        }
        case 50000:
        {
          lReturn = BAUD_50K;
          break;
        }
        default:
        {
          printf("Baudrate set to 500 kbit/s. \n");
          lReturn = BAUD_500K;
          break;
        }
    }

    return lReturn;
#else
    (void)uFreq;
    return (0);
#endif
}


