#include "VPL16Opr.h"

static DWORD WINAPI VPL16PackDataThread(LPVOID pParam)
{
	CVPL16Opr* pVPL16Opr = (CVPL16Opr*)pParam;
	pVPL16Opr->runPackData2RoundData();
	return 0;
}

static DWORD WINAPI VPL16RoundDataThread(LPVOID pParam)
{
	CVPL16Opr* pVPL16Opr  = (CVPL16Opr*)pParam;
	pVPL16Opr->runProcessRoundData();
	return 0;
}

void __stdcall OnBinData(void* pData, void* pUser)
{
	char* pData_ = (char*)pData;
	CVPL16Opr* pVPL16Opr = (CVPL16Opr*)pUser;
	int nRt = pVPL16Opr->ParsePackData(pData_,pVPL16Opr->m_CurData);
	if (nRt <= 0)
	{
		printf("Data parse errer!\n");
		return;
	}

	nRt = pVPL16Opr->m_PackBuf.AddData(pVPL16Opr->m_CurData);
	if (nRt == 0)
	{
		printf("Pack data buffer drop data : %d\n",pVPL16Opr->m_PackBuf.GetDropCont());
	}

	return;
}

CVPL16Opr::CVPL16Opr()
{
	char szTemp[] = {0xff,0xff,0xff,0xff,0xff,0xff,0x60,0x76,0x88,0x00,0x00,0x00,0x08,0x00,0x45,0x00,0x04,0xd2,0x00,0x00,0x40,0x00,0xff,0x11,0xb4,0xaa,0xc0,0xa8,0x01,0xc8,0xff,0xff,0xff,0xff,0x09,0x40,0x09,0x40,0x04,0xbe,0x00,0x00};
	m_pszHead = new char[42];
	memcpy(m_pszHead,&szTemp,42);
	m_nHeadLen = 42;
	m_nDataLen = 1248;
	m_nRecLen = 1248*4;
	m_nPort = 2368;
	m_DataCallBack = NULL;
	m_pUser = NULL;
	m_dStartAngle = 0.0;
	m_dAngleBefor = 0.0;

	double Ticks_[] = {-15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0, -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0};
	m_Ticks.resize(16);
	memcpy(m_Ticks.data(),Ticks_,16*sizeof(double));

	m_OpenGL = GetLibGuiOpenGL();
	m_OpenGL->SetCameraMode(AUTO_FREE);
	m_OpenGL->SetScale(1.0);

	m_PackBuf.Init(10);
	m_RoundBuf.Init(10);

	m_idPack = NULL;
	m_idRound = NULL;
}
CVPL16Opr::~CVPL16Opr()
{

}

void CVPL16Opr::Init(char* pszAddr, int nPort/* = 2368*/)
{
	m_UDPOpr.Connect(nPort,pszAddr);
}

void CVPL16Opr::SetCallBack(lpVPL16DataCallBack DataCallBack, void* pUser)
{
	m_DataCallBack = DataCallBack;
	m_pUser = pUser;

	m_UDPOpr.SetCallBack((lpTcpIpDataRecFunc)OnBinData,this,m_pszHead,m_nHeadLen,m_nDataLen,m_nRecLen);
}

int CVPL16Opr::Start()
{
	if (m_idPack != 0)
	{
		printf("m_hThread != 0\n");
		return 0;
	}
	m_idPack = CreateThread(NULL,0,VPL16PackDataThread,this,0,NULL);
	if (m_idPack == 0)
	{
		printf("CreateThread Failed \n");
		return 0;
	}

	if (m_idRound != 0)
	{
		printf("m_hThread != 0\n");
		return 0;
	}
	m_idRound = CreateThread(NULL,0,VPL16RoundDataThread,this,0,NULL);
	if (m_idRound == 0)
	{
		printf("CreateThread Failed \n");
		return 0;
	}

	return m_UDPOpr.Start();
}

int CVPL16Opr::ParsePackData(char* pszData, VPL16DataPack& Data)
{
	double dAzimuthN = Bytes2Float(pszData+42+2,2,0.01);
	double dAzimuthN2 = Bytes2Float(pszData+42+100+2,2,0.01);
	double dInterval = dAzimuthN2-dAzimuthN;
	if (dInterval<0)
		dInterval+=360.0;
	dInterval /= 2.0;

	for (int i = 0; i < 12; i++)
	{
		VPL16DataBlock& BlockN = Data.DataBlock[2*i];
		BlockN.fAzimuth = Bytes2Float(pszData+42+i*100+2,2,0.01);
		for (int j = 0; j < 16; j++)
		{
			BlockN.fDist[j] = Bytes2Float(pszData+42+i*100+2+2+j*3,2,0.002);
			BlockN.dReflect[j] = *((unsigned char*)(pszData+42+i*100+2+2+j*3+2));
		}

		VPL16DataBlock& BlockN1 = Data.DataBlock[2*i+1];
		BlockN1.fAzimuth = BlockN.fAzimuth+dInterval;
		if (BlockN1.fAzimuth>=360.0)
			BlockN1.fAzimuth -= 360.0;
		for (int j = 0; j < 16; j++)
		{
			BlockN1.fDist[j] = Bytes2Float(pszData+42+i*100+2+2+16*3+j*3,2,0.002);
			BlockN1.dReflect[j] = *((unsigned char*)(pszData+42+i*100+2+2+16*3+j*3+2));
		}
	}

	return 1;
}

double CVPL16Opr::Bytes2Float(char* pBytes, int nLen, double dScale)
{
	unsigned long temp;
	char* ptemp = (char*)(&temp);
	memset(&temp,0,sizeof(unsigned long));
	memcpy(ptemp,pBytes,nLen);
	double fOut = (double)temp*dScale;
	return fOut;
}

void CVPL16Opr::OnRoundData(VPL16DataRound& DataRound)
{
	VPL16DataPointsCloud PointsCloud;

	for (int i = 0; i < DataRound.VecBlock.size(); i++)
	{
		VPL16DataBlock& Block = DataRound.VecBlock[i];
		for (int j = 0; j < 16; j++)
		{
			double dAngleW = m_Ticks[j]/180.0*CV_PI;
			double dAngleA = Block.fAzimuth/180.0*CV_PI;
			double dDist = Block.fDist[j];
			unsigned char Reflect = Block.dReflect[j];
			if (dDist<=1.0)
			{
				continue;
			}
			Point3d pt;
			pt.x = dDist*cos(dAngleW)*sin(dAngleA);
			pt.y = dDist*cos(dAngleW)*cos(dAngleA);
			pt.z = dDist*sin(dAngleW);
			PointsCloud.CloudPoints.push_back(pt);
			PointsCloud.Reflect.push_back(Reflect);
		}
	}

	Mat Cpt(PointsCloud.CloudPoints.size(),3,CV_64F,PointsCloud.CloudPoints.data());
	Mat Cpt_ = Cpt.clone();
//	printf("Points cont:%d",PointsCloud.CloudPoints.size());
	m_OpenGL->ClearPoints();
	m_OpenGL->AddPointsRelative(Cpt_);

	if (m_DataCallBack)
	{
		(*m_DataCallBack)(&Cpt_,m_pUser);
	}

	long nnn = 0;
	nnn = 1;

}

void CVPL16Opr::runPackData2RoundData()
{
	VPL16DataRound CurDataRound;
	VPL16DataPack CurDataPack;
	while(1)
	{
		vector<VPL16DataPack> VecPack;
		int nRt = m_PackBuf.GetDatas(VecPack);
		if (nRt == 0)
		{
			Sleep(1);
		}

		for (int i = 0; i < VecPack.size(); i++)
		{
			VPL16DataPack& PackT = VecPack[i];
			for (int j = 0; j < 24; j++)
			{
				long nCont = CurDataRound.VecBlock.size();
				VPL16DataBlock& CurBlock = PackT.DataBlock[j];
				if (nCont <= 0)
				{
					CurDataRound.VecBlock.push_back(CurBlock);
					continue;
				}

				bool bIsLoop = false;
				VPL16DataBlock& LastBlock = CurDataRound.VecBlock[nCont-1];
				if ((CurBlock.fAzimuth < LastBlock.fAzimuth) && 
					(m_dStartAngle == 0.0))
				{
					bIsLoop = true;
				}
				if ((CurBlock.fAzimuth > m_dStartAngle) && 
					(LastBlock.fAzimuth <= m_dStartAngle) &&
					(m_dStartAngle != 0.0))
				{
					bIsLoop = true;
				}

				if (bIsLoop)
				{
					printf("Loop detect!!! include pack cont:%d\n",CurDataRound.VecBlock.size());
					int nRt = m_RoundBuf.AddData(CurDataRound);
					if (nRt == 0)
						printf("Round data buffer drop data : %d\n",m_RoundBuf.GetDropCont());
					CurDataRound.VecBlock.clear();
				}

				CurDataRound.VecBlock.push_back(CurBlock);
			}
		}
	}
}

void CVPL16Opr::runProcessRoundData()
{
	long nCont = 0;
	while(1)
	{
		vector<VPL16DataRound> VecRoundData;
		int nRt = m_RoundBuf.GetDatas(VecRoundData);
		if (nRt == 0)
		{
			Sleep(1);
		}
		for (int i = 0; i < VecRoundData.size(); i++)
		{
			OnRoundData(VecRoundData[i]);
			printf("Ind:%d\n",nCont++);
		}
	}

}