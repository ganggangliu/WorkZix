// 文件名: VciCanDrive.cpp
// 说明: 周立功CAN数据发送和读取
// 作者: 崔鹏、陈波
// 创建日期: 2016-06-01
//

#include "vcicandrive.h"
#include "debugfunction.h"
#include "transplant.h"
#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    #include "ControlCAN.h"
#endif

VciCanDrive::VciCanDrive()
{
	m_deviceType = 4/*16*/;	// 设备类型号(USBCAN2)
	m_deviceInd = 0;	// 设备索引号(只有一个USBCAN)
	m_canInd = 1;		// 第几路CAN
	m_canConfig.AccCode = 0x00000000;	// 验收码
	m_canConfig.AccMask = 0xFFFFFFFF;	// 屏蔽码
	m_canConfig.Reserved = 0;
	m_canConfig.Filter = 1;				// 滤波方式，1表示单滤波，0表示双滤波
	// Timing0和Timing1用来设置CAN波特率, 00 1C - 500Kbps
	m_canConfig.Timing0 = 0x00;			// 定时器0
	m_canConfig.Timing1 = 0x1C;			// 定时器1
	m_canConfig.Mode = 0;				// 模式，0表示正常模式，1表示只听模式
}

VciCanDrive::~VciCanDrive()
{
	// close can di
    // this->close();
}

bool VciCanDrive::OpenDevice(
                unsigned long deviceType_,
                unsigned long deviceInd_)
{
    COM_INFO(3, ("%s, VCI_OpenDevice(%d, %d, 0)\n", __FUNCTION__, deviceType_, deviceInd_));

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    DWORD opStatus = STATUS_ERR;
    opStatus = VCI_OpenDevice(deviceType_, deviceInd_, 0);
    COM_INFO(1, ("Open can status: %d\n", opStatus));
    if((ERR_DEVICEOPENED != opStatus) && (STATUS_OK != opStatus)) {
        COM_ERR("%s, VCI_OpenDevice err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        return (false);
    }
#endif

    return (true);
}

bool VciCanDrive::CloseDevice(
            unsigned long deviceType_,
            unsigned long deviceInd_)
{
    COM_INFO(3, ("%s, VCI_CloseDevice(%d, %d)", __FUNCTION__, deviceType_, deviceInd_));

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    //Sleep(500);
    DWORD opStatus = VCI_CloseDevice(deviceType_, deviceInd_);
    if (STATUS_OK != opStatus) {
        COM_ERR("%s, VCI_CloseDevice err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        return (false);
    }
#endif

    return (true);
}

bool VciCanDrive::Connect(
             unsigned long deviceType_,
             unsigned long deviceInd_,
             unsigned long canId_,
             const CAN_CONFIG& config_)
{
    m_deviceType = deviceType_;
    m_deviceInd = deviceInd_;
    m_canInd = canId_;
    m_canConfig.AccCode = config_.acc_code;
    m_canConfig.AccMask = config_.acc_mask;
    m_canConfig.Reserved = config_.reserved;
    m_canConfig.Filter = config_.filter;
    if (config_.baud_rate == 500000)
    {
        m_canConfig.Timing0 = 0x00;			// 定时器0
        m_canConfig.Timing1 = 0x1C;			// 定时器1
    }
    m_canConfig.Mode = config_.mode;

    COM_INFO(3, ("%s, VCI_InitCAN(%d, %d, %d, config)\n", __FUNCTION__,
                 m_deviceType, m_deviceInd, m_canInd));

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    VCI_INIT_CONFIG canCofig;
    canCofig.AccCode = m_canConfig.AccCode;
    canCofig.AccMask = m_canConfig.AccMask;
    canCofig.Reserved = m_canConfig.Reserved;
    canCofig.Filter = m_canConfig.Filter;
    canCofig.Timing0 = m_canConfig.Timing0;
    canCofig.Timing1 = m_canConfig.Timing1;
    canCofig.Mode = m_canConfig.Mode;

    DWORD opStatus = VCI_InitCAN(m_deviceType, m_deviceInd, m_canInd, &canCofig);
    if(STATUS_OK != opStatus) {
        COM_ERR("%s, VCI_InitCAN err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        //VCI_CloseDevice(m_deviceType, m_deviceInd);
        return (false);
    }
#endif

    return (true);
}

bool VciCanDrive::Start(void)
{
    COM_INFO(3, ("%s, VCI_StartCAN(%d, %d, %d)", __FUNCTION__, m_deviceType, m_deviceInd, m_canInd));

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    DWORD opStatus = VCI_StartCAN(m_deviceType, m_deviceInd, m_canInd);
    if (STATUS_OK != opStatus) {
        COM_ERR("%s, VCI_StartCAN err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        return (false);
    }
    opStatus = VCI_ClearBuffer(m_deviceType, m_deviceInd, m_canInd);
    if (STATUS_OK != opStatus) {
        COM_ERR("%s, VCI_ClearBuffer err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        return (false);
    }
#endif

    return (true);
}

bool VciCanDrive::Stop(void)
{
    COM_INFO(3, ("%s, VCI_StopCAN(%d, %d, %d)", __FUNCTION__,
                 m_deviceType, m_deviceInd, m_canInd));

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    // VCI_StopCAN called?
//    DWORD opStatus = VCI_StopCAN(m_deviceType, m_deviceInd, m_canInd);
//    if (STATUS_OK != opStatus) {
//        COM_ERR("%s, VCI_StartCAN err, err code = 0x%08x\n", __FUNCTION__, opStatus);
//        return (false);
//    }
//    opStatus = VCI_ClearBuffer(m_deviceType, m_deviceInd, m_canInd);
//    if (STATUS_OK != opStatus) {
//        COM_ERR("%s, VCI_ClearBuffer err, err code = 0x%08x\n", __FUNCTION__, opStatus);
//        return (false);
//    }
#endif

    return (true);
}

bool VciCanDrive::Reset(void)
{
    COM_INFO(3, ("%s, VCI_ResetCAN(%d, %d, %d)", __FUNCTION__, m_deviceType, m_deviceInd, m_canInd));

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    DWORD opStatus = VCI_ResetCAN(m_deviceType, m_deviceInd, m_canInd);
    if (STATUS_OK != opStatus) {
        COM_ERR("%s, VCI_ResetCAN err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        return (false);
    }
#endif

    return (true);
}

unsigned long VciCanDrive::Send(CAN_OBJ& data, unsigned long count)
{
    //VEH_DEBUG(6, ("%s, VCI_Transmit(%d, %d, %d, data, %d)", __FUNCTION__, m_deviceType, m_deviceInd, m_canInd, count));

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    const int iMaxBuffCnt = VCI_MAX_BUFF_CNT;
    int iBuffCnt = (count > iMaxBuffCnt) ? iMaxBuffCnt : count;
    VCI_CAN_OBJ objBuff[iMaxBuffCnt];
    int j = 0;

    for (j = 0; j < iBuffCnt; j++)
    {
        VCI_CAN_OBJ& vci_data = objBuff[j];
        int i = 0;

        vci_data.ID = (&data)[j].id;
        vci_data.TimeStamp = (&data)[j].time_stamp;
        vci_data.TimeFlag = (&data)[j].time_flag;
        vci_data.SendType = (&data)[j].send_type;
        vci_data.RemoteFlag = (&data)[j].remote_flag;
        vci_data.ExternFlag = (&data)[j].extern_flag;
        vci_data.DataLen = (&data)[j].data_len;
        for (i = 0; i < 8; i++)
        {
            vci_data.Data[i] = (&data)[j].data[i];
        }
        for (i = 0; i < 3; i++)
        {
            vci_data.Reserved[i] = (&data)[j].reserved[i];
        }
    }

    return (VCI_Transmit(m_deviceType, m_deviceInd, m_canInd,
                         &objBuff[0], iBuffCnt));
#else
    (void)data;
    (void)count;

    return (0);
#endif
}

unsigned long VciCanDrive::GetRecvNum(void)
{
#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    return (VCI_GetReceiveNum(m_deviceType, m_deviceInd, m_canInd));
#else
    return (0);
#endif
}

unsigned long VciCanDrive::Receive(CAN_OBJ& data,
                                   unsigned long count,
                                   int waitTime)
{
	ULONG nReceiveCount = 0;

#if (CAN_DEV_TYPE == CAN_DEV_TYPE_ZLG)
    const int iMaxBuffCnt = VCI_MAX_BUFF_CNT;
    int iBuffCnt = (count > iMaxBuffCnt) ? iMaxBuffCnt : count;
    VCI_CAN_OBJ objBuff[iMaxBuffCnt];

    //COM_INFO(6, ("%s, VCI_Receive(%d, %d, %d, data, %d)", __FUNCTION__, m_deviceType, m_deviceInd, m_canInd, count));
    nReceiveCount = VCI_Receive(m_deviceType, m_deviceInd, m_canInd, &objBuff[0],
                                iBuffCnt, waitTime);
    if (nReceiveCount <= 0)
    {
        // 注意：如果没有读到数据则必须调用此函数来读取出当前的错误码，
        // 千万不能省略这一步（即使你可能不想知道错误码是什么）
        VCI_ERR_INFO errinfo;
        VCI_ReadErrInfo(m_deviceType, m_deviceInd, m_canInd, &errinfo);
        COM_ERR("%s, VCI_Receive err, ErrCode(%08x) Passive_ErrData(%02x %02x %02x) ArLost_ErrData(%02x)\n", __FUNCTION__,
            errinfo.ErrCode, errinfo.Passive_ErrData[0], errinfo.Passive_ErrData[1], errinfo.Passive_ErrData[2],
            errinfo.ArLost_ErrData);
        DWORD opStatus = VCI_ClearBuffer(m_deviceType, m_deviceInd, m_canInd);
        if (STATUS_OK != opStatus) {
            COM_ERR("%s, VCI_ClearBuffer err, err code = 0x%08x\n", __FUNCTION__, opStatus);
        }
    }
    else
    {
        // 将接收到的数据复制到输出里面
        int i;

        for (i = 0; i < (int)nReceiveCount && i < iBuffCnt; i++)
        {
            int j = 0;
            CAN_OBJ &iter = (&data)[i]; // iterator

            iter.id = objBuff[i].ID;
            iter.time_stamp = objBuff[i].TimeStamp;
            iter.time_flag = objBuff[i].TimeFlag;
            iter.send_type = objBuff[i].SendType;
            iter.remote_flag = objBuff[i].RemoteFlag;
            iter.extern_flag = objBuff[i].ExternFlag;
            iter.data_len = objBuff[i].DataLen;
            for (j = 0; j < 8; j++)
            {
                iter.data[j] = objBuff[i].Data[j];
            }
            for (j = 0; j < 3; j++)
            {
                iter.reserved[j] = objBuff[i].Reserved[j];
            }
        }
    }
#else
    (void)data;
    (void)count;
    (void)waitTime;
#endif

	return (nReceiveCount);
}
