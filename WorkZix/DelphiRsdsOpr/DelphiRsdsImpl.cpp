#include "DelphiRsdsImpl.h"

void CDelphiRsdsImpl::BinDataCallBackL(unsigned char* pData, int nLen, void* pUser)
{
	LIDAR_RADAR_INFO RsdsData;
	long nCont = TcpIpParse(pData, &RsdsData);
	RsdsData.nLidarRadarType = 0;
	m_mutexL.lock();
	m_DataL = RsdsData;
	m_nDataStateL = 0;
	m_mutexL.unlock();
	if (m_pCallBackL)
	{
		m_pCallBackL(&RsdsData, m_pUserL);
	}
}

void CDelphiRsdsImpl::BinDataCallBackR(unsigned char* pData, int nLen, void* pUser)
{
	LIDAR_RADAR_INFO RsdsData;
	long nCont = TcpIpParse(pData, &RsdsData);
	RsdsData.nLidarRadarType = 1;
	m_mutexR.lock();
	m_DataR = RsdsData;
	m_nDataStateR = 0;
	m_mutexR.unlock();
	if (m_pCallBackR)
	{
		m_pCallBackR(&RsdsData, m_pUserR);
	}
}

void CDelphiRsdsImpl::TcpIpDataCallBackL(unsigned char* pData, int nLen, void* pUser)
{
	m_BinDataOprL.FeedData(pData, nLen);
}

void CDelphiRsdsImpl::TcpIpDataCallBackR(unsigned char* pData, int nLen, void* pUser)
{
	m_BinDataOprR.FeedData(pData, nLen);
}

CDelphiRsdsImpl::CDelphiRsdsImpl()
{
	m_pUserL = 0;
	m_pUserR = 0;
	m_nDataStateL = -1;
	m_nDataStateR = -1;

	m_TcpIpOprL.SetCallBack(boost::bind(&CDelphiRsdsImpl::TcpIpDataCallBackL,this,_1,_2,_3),0);
	m_TcpIpOprR.SetCallBack(boost::bind(&CDelphiRsdsImpl::TcpIpDataCallBackR,this,_1,_2,_3),0);

	m_BinDataOprL.SetCallBack(boost::bind(&CDelphiRsdsImpl::BinDataCallBackL,this,_1,_2,_3),m_pUserL);
	m_BinDataOprR.SetCallBack(boost::bind(&CDelphiRsdsImpl::BinDataCallBackR,this,_1,_2,_3),m_pUserR);
	unsigned char szHead[] = {0xAA, 0X55, 0x33, 0XCC, 0x00, 0XFF, 0x11, 0XEE, 0x22, 0XDD, 0x44, 0XBB, 0x01, 0X12, 0x45, 0X67};
	m_BinDataOprL.SetDataPattern(szHead, 16, 964-16);
	m_BinDataOprR.SetDataPattern(szHead, 16, 964-16);

	return;
}

int CDelphiRsdsImpl::Init(CDelphiRsdsImplParam& Param)
{
	m_Param = Param;
	m_XCanOpr.Init(Param.XCanParam);
	m_PCanOpr.Init(Param.PCanParam);

	return 1;
}

void CDelphiRsdsImpl::SetCallBackL(FunctionRsdsData  hCallBack, void* pUser)
{
	m_pCallBackL = hCallBack;
	m_pUserL = pUser;
}

void CDelphiRsdsImpl::SetCallBackR(FunctionRsdsData  hCallBack, void* pUser)
{
	m_pCallBackR = hCallBack;
	m_pUserR = pUser;
}

int CDelphiRsdsImpl::GetDataL(LIDAR_RADAR_INFO& Data)
{
	m_mutexL.lock();
	if (m_nDataStateL < 0)
	{
		m_mutexL.unlock();
		return m_nDataStateL;
	}
	Data = m_DataL;
	m_nDataStateL++;
	m_mutexL.unlock();
	return m_nDataStateL;
}

int CDelphiRsdsImpl::GetDataR(LIDAR_RADAR_INFO& Data)
{
	m_mutexR.lock();
	if (m_nDataStateR < 0)
	{
		m_mutexR.unlock();
		return m_nDataStateR;
	}
	Data = m_DataR;
	m_nDataStateR++;
	m_mutexR.unlock();
	return m_nDataStateR;
}

int CDelphiRsdsImpl::Start()
{
	m_nDataStateL = -1;
	m_nDataStateR = -1;

	int nRt = PCanInit();
	if (nRt <= 0)
		printf("PCAN init failed!\n");
	else
		printf("PCAN init finish!\n");

	nRt = XCanInit();
	if (nRt <= 0)
		printf("XCAN init failed!\n");
	else
		printf("XCAN init finish!\n");

	nRt = m_TcpIpOprL.Connect(55555, "169.254.145.71", (char*)m_Param.szIpLeft.c_str());
	if (nRt <= 0)
		printf("TcpIp OprL Start failed!\n");
	else
		printf("TcpIp OprL Start finish!\n");
	m_TcpIpOprL.Start();

	nRt = m_TcpIpOprR.Connect(55555, "169.254.145.72", (char*)m_Param.szIpRight.c_str());
	if (nRt <= 0)
		printf("TcpIp OprR Start failed!\n");
	else
		printf("TcpIp OprR Start finish!\n");
	m_TcpIpOprR.Start();

	return 1;
}

int CDelphiRsdsImpl::Stop()
{
//	m_XCanOpr.Stop();
//	m_PCanOpr.Stop();
	m_TcpIpOprL.Close();
	m_TcpIpOprR.Close();

	return 1;
}

int CDelphiRsdsImpl::PCanInit()
{
	//Wait for PcanData
	int nRt =  m_PCanOpr.Start();
	if (nRt <= 0)
	{
		printf("PCAN open failed!\n");
		return 0;
	}
	printf("PCAN open finish!\n");

	//Check if there is data
	nRt = 0;
	int nCheckTime = 100;
	while(1)
	{
		CCanMsgData Data;
		int nState = m_PCanOpr.GetMsg(Data);
		if (nState <= 0)
		{
			boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(10));

			nCheckTime--;
		}
		else
		{
			if (Data.id == 1280)
			{
				nRt = 1;
				break;
			}
		}
		if (nCheckTime <= 0)
		{
			nRt = 0;
			break;
		}
	}
	if (nRt <= 0)
	{
		printf("PCAN no data found!\n");
		return 0;
	}
	else
	{
		printf("PCAN radar data(1280) found!\n");
	}

	//Enable NetData
	CCanMsgData DataSend;
	DataSend.channelInd = m_Param.PCanParam.sChannel[0].nChannleInd;

	boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(100));
	DataSend.id = 0x762; 
	unsigned char buffer0[8] = {0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
	memcpy(DataSend.msg, buffer0, 8);
	nRt = m_PCanOpr.SendMsg(DataSend);

	boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(100));
	DataSend.id = 0x762; 
//	unsigned char buffer1[8] = {0xf1,0x03,0x02,0x00,0x00,0x00,0x00,0x00}; 
	unsigned char buffer1[8] = {0xf1,0x03,0x01,0x00,0x00,0x00,0x00,0x00}; 
	memcpy(DataSend.msg, buffer1, 8);
	nRt = m_PCanOpr.SendMsg(DataSend);

	boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(100));
	DataSend.id = 0x764; 
	unsigned char buffer2[8] = {0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
	memcpy(DataSend.msg, buffer2, 8);
	nRt = m_PCanOpr.SendMsg(DataSend);

	boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(100));
	DataSend.id = 0x764; 
//	unsigned char buffer3[8] = {0xf1,0x03,0x02,0x00,0x00,0x00,0x00,0x00}; 
	unsigned char buffer3[8] = {0xf1,0x03,0x01,0x00,0x00,0x00,0x00,0x00}; 
	memcpy(DataSend.msg, buffer3, 8);
	nRt = m_PCanOpr.SendMsg(DataSend);

	m_PCanOpr.Stop();
}

int CDelphiRsdsImpl::XCanInit()
{
	//Input vehicle data
	int nRt =  m_XCanOpr.Start();
	if (nRt <= 0)
	{
		printf("XCAN open failed!\n");
		return 0;
	}
	printf("XCAN open finish!\n");

	for (unsigned int i = 0; i < 5; i++)
	{
		CCanMsgData DataSend;
		DataSend.channelInd = m_Param.XCanParam.sChannel[0].nChannleInd;

		boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(10));
		DataSend.id = 0x20; 
		unsigned char buffer0[8] = {0x00,0x08,0x00,0x00,0x3b,0x0a,0x8c,0x70}; 
		memcpy(DataSend.msg, buffer0, 8);
		nRt = m_XCanOpr.SendMsg(DataSend);

		boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(10));
		DataSend.id = 0x65;
		unsigned char buffer1[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
		memcpy(DataSend.msg, buffer1, 8);
		nRt = m_XCanOpr.SendMsg(DataSend);

		boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(10));
		DataSend.id = 0x100;
		unsigned char buffer2[8] = {0x80,0x20,0x00,0xa7,0x50,0x80,0xdf,0xfe};
		memcpy(DataSend.msg, buffer2, 8);
		nRt = m_XCanOpr.SendMsg(DataSend);

		boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(10));
		DataSend.id = 0x145;
		unsigned char buffer3[8] = {0x16,0x00,0x80,0x00,0x00,0x00,0x37,0xff}; 
		memcpy(DataSend.msg, buffer3, 8);
		nRt = m_XCanOpr.SendMsg(DataSend);

		boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(10));
		DataSend.id = 0x400;
		unsigned char buffer4[8] = {0x01,0x61,0x00,0x00,0x00,0x00,0x00,0x00};
		memcpy(DataSend.msg, buffer4, 8);
		nRt = m_XCanOpr.SendMsg(DataSend);

		boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(10));
		DataSend.id = 0x405;
		unsigned char buffer5[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x09,0xc6}; 
		memcpy(DataSend.msg, buffer5, 8);
		nRt = m_XCanOpr.SendMsg(DataSend);

		boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(10));
		DataSend.id = 0x500;
		unsigned char buffer6[8] = {0x66,0x02,0x00,0x01,0x00,0x00,0x01,0x00}; 
		memcpy(DataSend.msg, buffer6, 8);
		nRt = m_XCanOpr.SendMsg(DataSend);
	}

	boost::thread::sleep(boost::get_system_time() + boost::posix_time::millisec(10));
	m_XCanOpr.Stop();

	return 1;
}

int CDelphiRsdsImpl::TcpIpParse(unsigned char* psz, LIDAR_RADAR_INFO* pRsdsData)
{
	short T;
	unsigned char Tc;
	uint16_t uInt16T;
	uint32_t uInt32T;
	uint64_t uInt64T;

	memcpy(&uInt32T, psz + 16, sizeof(uint32_t));
	uint32_t nScanIndex = uInt32T;
	memcpy(&uInt32T, psz + 20, sizeof(uint32_t));
	uint32_t nTimeStamp = uInt32T;
	memcpy(&uInt16T, psz + 24, sizeof(uint16_t));
	uint16_t nTcpPackageSize = uInt16T;
	memcpy(&Tc, psz + 26, sizeof(unsigned char));
	uint8_t nTcpMajorVersion = Tc;
	memcpy(&Tc, psz + 27, sizeof(unsigned char));
	uint8_t nTcpMinorVersion = Tc;
	memcpy(&uInt32T, psz + 28, sizeof(uint32_t));
	uint32_t nArmSwVer = uInt32T;
	memcpy(&uInt32T, psz + 32, sizeof(uint32_t));
	uint32_t nDspSwVer = uInt32T;
	memcpy(&uInt32T, psz + 36, sizeof(uint32_t));
	uint32_t nHstSwVer = uInt32T;
	memcpy(&uInt64T, psz + 40, sizeof(uint64_t));
	uint64_t nIdleThread = uInt64T;
	memcpy(&uInt32T, psz + 48, sizeof(uint32_t));
	uint32_t nScanIndexDsp = uInt32T;
	memcpy(&uInt32T, psz + 52, sizeof(uint32_t));
	uint32_t nTimeStampDsp = uInt32T;

	int nValid = 0;
	for (int i = 0; i < 64; i++)
	{
		LIDAR_RADAR_OBJECT_INFO& ObjInfo = pRsdsData->Objects[i];
		long nStart = 68 + 14*i;

		memcpy(&T, psz + nStart + 0, sizeof(short));
		double dLonPos = (double)T / 128.f;
		memcpy(&T, psz + nStart + 2, sizeof(short));
		double dLonVel = (double)T / 128.f;
		memcpy(&T, psz + nStart + 4, sizeof(short));
		double dLonAcc = (double)T / 1024.f;
		memcpy(&T, psz + nStart + 6, sizeof(short));
		double dLatPos = (double)T / 128.f;
		memcpy(&T, psz + nStart + 8, sizeof(short));
		double dLatVel = (double)T / 128.f;
		memcpy(&T, psz + nStart + 10, sizeof(short));
		double dLatAcc = (double)T / 1024.f;
		memcpy(&Tc, psz + nStart + 12, sizeof(unsigned char));
		unsigned int Status = (unsigned int)Tc;
		memcpy(&Tc, psz + nStart + 13, sizeof(unsigned char));
		bool Stationary = (bool)Tc;

		ObjInfo.nObjectID = i;
		ObjInfo.fObjLocX = dLatPos;
		ObjInfo.fObjLocY = dLonPos;
		if (Status == 0 || Status == 1)
			ObjInfo.bValid = 0;
		else
		{
			ObjInfo.bValid = 1;
			nValid++;
		}

	}

	//	printf("%d\n",nValid);

	return nValid;
}
