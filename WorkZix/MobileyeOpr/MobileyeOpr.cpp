#include "MobileyeOpr.h"

#if _DEBUG
#pragma comment(lib,"CanOprD.lib")
#else
#pragma comment(lib,"CanOpr.lib")
#endif

void __stdcall CanParseOprDataCallBack(void* pData, void* pUser)
{
	vector<CCanParseItem>* pData_ = (vector<CCanParseItem>*)pData;
	CMobileyeOpr* pUser_ = (CMobileyeOpr*)pUser;

	pUser_->TransData(*pData_, pUser_->m_CurData);
	pUser_->m_nFrameInd++;
	pUser_->m_CurData.nFrameId = pUser_->m_nFrameInd;

	if (pUser_->m_pMobileyeCallBack)
	{
		(*(pUser_->m_pMobileyeCallBack))(&(pUser_->m_CurData), (pUser_->m_pUser));
	}
}

CMobileyeOpr::CMobileyeOpr()
{
	m_pMobileyeCallBack = NULL;
	m_pUser = NULL;
	return;
}

CMobileyeOpr::~CMobileyeOpr()
{
	return;
}

int CMobileyeOpr::Init(CMobileyeParam& Param)
{
	m_Param = Param;
	CCanParam CanParam;
	CanParam.nDevType = m_Param.nDevType;
	CanParam.nDevInd = m_Param.nDevInd;
	CanParam.sChannel.resize(1);
	CanParam.sChannel[0].nChannleInd = m_Param.nChannelInd;
	CanParam.sChannel[0].nBaudRate = m_Param.nBaudRate;
	m_CanParseOpr.Init(CanParam);
	m_CanParseOpr.SetCallBack(CanParseOprDataCallBack, this);

	SetParsePattern();

	m_nFrameInd = 0;

	return 1;
}

void CMobileyeOpr::SetParsePattern()
{
	vector<CCanParseItem> ParsePattern;
	CCanParseItem ItemT;

	//////////////////////////////////////////////////////////////////////////
	//Obstacle status
	//////////////////////////////////////////////////////////////////////////
	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.nNumObstacles");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 0;
	ItemT.nBitWidth = 8;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.nTimeStamp");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 8;
	ItemT.nBitWidth = 8;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.nAppVersion");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 16;
	ItemT.nBitWidth = 8;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.nActVerNumSec");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 24;
	ItemT.nBitWidth = 2;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.bLeftCloseRangCutIn");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 26;
	ItemT.nBitWidth = 1;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.bRightCloseRangCutIn");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 27;
	ItemT.nBitWidth = 1;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.nGo");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 28;
	ItemT.nBitWidth = 4;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.nProtocolVersion");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 32;
	ItemT.nBitWidth = 8;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.bCloseCar");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 40;
	ItemT.nBitWidth = 1;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	strcpy(ItemT.szName, "ObstacleInfo.nFailSafe");
	ItemT.nMsgInd = 0x738;
	ItemT.nStartBit = 41;
	ItemT.nBitWidth = 4;
	ParsePattern.push_back(ItemT);

	//////////////////////////////////////////////////////////////////////////
	//Obstacle Details
	//////////////////////////////////////////////////////////////////////////
	for (unsigned int i = 0; i < 15; i++)
	{
		//////////////////////////////////////////////////////////////////////////
		//Obstacle[i] detail A
		//////////////////////////////////////////////////////////////////////////
		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nObstacleId", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 0;
		ItemT.nBitWidth = 8;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fPosX", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 8;
		ItemT.nBitWidth = 12;
		ItemT.dScale = 0.0625;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fPosY", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 24;
		ItemT.nBitWidth = 9;
		ItemT.dScale = 0.0625;
		ItemT.bIsSigned = true;
		ItemT.nSignedBit = 33;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nBlinkerInfo", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 34;
		ItemT.nBitWidth = 3;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nCutInOrOut", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 37;
		ItemT.nBitWidth = 3;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fRelVelX", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 40;
		ItemT.nBitWidth = 11;
		ItemT.dScale = 0.0625;
		ItemT.bIsSigned = true;
		ItemT.nSignedBit = 51;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nObstacleType", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 52;
		ItemT.nBitWidth = 3;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nObstacleStatus", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 56;
		ItemT.nBitWidth = 3;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].bObstacleBrakeLights", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 59;
		ItemT.nBitWidth = 1;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nObstacleValid", i);
		ItemT.nMsgInd = 0x739 + i*3 + 0;
		ItemT.nStartBit = 62;
		ItemT.nBitWidth = 2;
		ParsePattern.push_back(ItemT);

		//////////////////////////////////////////////////////////////////////////
		//Obstacle[i] detail B
		//////////////////////////////////////////////////////////////////////////
		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fLength", i);
		ItemT.nMsgInd = 0x739 + i*3 + 1;
		ItemT.nStartBit = 0;
		ItemT.nBitWidth = 8;
		ItemT.dScale = 0.5;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fWidth", i);
		ItemT.nMsgInd = 0x739 + i*3 + 1;
		ItemT.nStartBit = 8;
		ItemT.nBitWidth = 8;
		ItemT.dScale = 0.05;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nAge", i);
		ItemT.nMsgInd = 0x739 + i*3 + 1;
		ItemT.nStartBit = 16;
		ItemT.nBitWidth = 8;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nLane", i);
		ItemT.nMsgInd = 0x739 + i*3 + 1;
		ItemT.nStartBit = 24;
		ItemT.nBitWidth = 2;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].bCIPV", i);
		ItemT.nMsgInd = 0x739 + i*3 + 1;
		ItemT.nStartBit = 26;
		ItemT.nBitWidth = 1;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fRadarPosX", i);
		ItemT.nMsgInd = 0x739 + i*3 + 1;
		ItemT.nStartBit = 28;
		ItemT.nBitWidth = 12;
		ItemT.dScale = 0.0625;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fRadarVelX", i);
		ItemT.nMsgInd = 0x739 + i*3 + 1;
		ItemT.nStartBit = 40;
		ItemT.nBitWidth = 11;
		ItemT.dScale = 0.0625;
		ItemT.bIsSigned = true;
		ItemT.nSignedBit = 51;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nRadarMatchConfidence", i);
		ItemT.nMsgInd = 0x739 + i*3 + 1;
		ItemT.nStartBit = 52;
		ItemT.nBitWidth = 3;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].nMatchedRadarId", i);
		ItemT.nMsgInd = 0x739 + i*3 + 1;
		ItemT.nStartBit = 56;
		ItemT.nBitWidth = 7;
		ParsePattern.push_back(ItemT);

		//////////////////////////////////////////////////////////////////////////
		//Obstacle[i] detail C
		//////////////////////////////////////////////////////////////////////////
		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fAngleRate", i);
		ItemT.nMsgInd = 0x739 + i*3 + 2;
		ItemT.nStartBit = 0;
		ItemT.nBitWidth = 15;
		ItemT.dScale = 0.01;
		ItemT.bIsSigned = true;
		ItemT.nSignedBit = 15;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fScaleChange", i);
		ItemT.nMsgInd = 0x739 + i*3 + 2;
		ItemT.nStartBit = 16;
		ItemT.nBitWidth = 15;
		ItemT.dScale = 0.0002;
		ItemT.bIsSigned = true;
		ItemT.nSignedBit = 31;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fAccelX", i);
		ItemT.nMsgInd = 0x739 + i*3 + 2;
		ItemT.nStartBit = 32;
		ItemT.nBitWidth = 9;
		ItemT.dScale = 0.03;
		ItemT.bIsSigned = true;
		ItemT.nSignedBit = 41;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].bReplaced", i);
		ItemT.nMsgInd = 0x739 + i*3 + 2;
		ItemT.nStartBit = 44;
		ItemT.nBitWidth = 1;
		ParsePattern.push_back(ItemT);

		ItemT = CCanParseItem();
		sprintf(ItemT.szName, "Obstalces[%d].fAngle", i);
		ItemT.nMsgInd = 0x739 + i*3 + 2;
		ItemT.nStartBit = 48;
		ItemT.nBitWidth = 15;
		ItemT.dScale = 0.01;
		ItemT.bIsSigned = true;
		ItemT.nSignedBit = 63;
		ParsePattern.push_back(ItemT);
	}

	//////////////////////////////////////////////////////////////////////////
	//LKA_LEFT
	//////////////////////////////////////////////////////////////////////////
	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.nLaneType");
	ItemT.nMsgInd = 0x766;
	ItemT.nStartBit = 0;
	ItemT.nBitWidth = 4;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.nQuality");
	ItemT.nMsgInd = 0x766;
	ItemT.nStartBit = 4;
	ItemT.nBitWidth = 2;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.nModelDegree");
	ItemT.nMsgInd = 0x766;
	ItemT.nStartBit = 6;
	ItemT.nBitWidth = 2;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.fPositionParamC0");
	ItemT.nMsgInd = 0x766;
	ItemT.nStartBit = 8;
	ItemT.nBitWidth = 16;
	ItemT.bIsSigned = true;
	ItemT.dScale = 1.0/256.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.fCurvatureParamC2");
	ItemT.nMsgInd = 0x766;
	ItemT.nStartBit = 24;
	ItemT.nBitWidth = 16;
	ItemT.dScale = 1.0/1024.0/1000.0;
	ItemT.dBias = -0.032;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.fCurvatureParamC3");
	ItemT.nMsgInd = 0x766;
	ItemT.nStartBit = 40;
	ItemT.nBitWidth = 16;
	ItemT.dScale = 1.0/pow(2.0,28);
	ItemT.dBias = -0.0001220703125;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.fWidthOfMarking");
	ItemT.nMsgInd = 0x766;
	ItemT.nStartBit = 56;
	ItemT.nBitWidth = 8;
	ItemT.dScale = 0.01;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.fHedingAngleParameterC1");
	ItemT.nMsgInd = 0x767;
	ItemT.nStartBit = 0;
	ItemT.nBitWidth = 16;
	ItemT.dScale = 1.0/1024.0;
	ItemT.dBias = -32.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.fViewRange");
	ItemT.nMsgInd = 0x767;
	ItemT.nStartBit = 16;
	ItemT.nBitWidth = 15;
	ItemT.dScale = 1.0/256.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Left.bViewRangeAvailability");
	ItemT.nMsgInd = 0x767;
	ItemT.nStartBit = 31;
	ItemT.nBitWidth = 1;
	ParsePattern.push_back(ItemT);

	//////////////////////////////////////////////////////////////////////////
	//LKA_RIGHT
	//////////////////////////////////////////////////////////////////////////
	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.nLaneType");
	ItemT.nMsgInd = 0x768;
	ItemT.nStartBit = 0;
	ItemT.nBitWidth = 4;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.nQuality");
	ItemT.nMsgInd = 0x768;
	ItemT.nStartBit = 4;
	ItemT.nBitWidth = 2;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.nModelDegree");
	ItemT.nMsgInd = 0x768;
	ItemT.nStartBit = 6;
	ItemT.nBitWidth = 2;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.fPositionParamC0");
	ItemT.nMsgInd = 0x768;
	ItemT.nStartBit = 8;
	ItemT.nBitWidth = 16;
	ItemT.bIsSigned = true;
	ItemT.dScale = 1.0/256.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.fCurvatureParamC2");
	ItemT.nMsgInd = 0x768;
	ItemT.nStartBit = 24;
	ItemT.nBitWidth = 16;
	ItemT.dScale = 1.0/1024.0/1000.0;
	ItemT.dBias = -0.032;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.fCurvatureParamC3");
	ItemT.nMsgInd = 0x768;
	ItemT.nStartBit = 40;
	ItemT.nBitWidth = 16;
	ItemT.dScale = 1.0/pow(2.0,28);
	ItemT.dBias = -0.0001220703125;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.fWidthOfMarking");
	ItemT.nMsgInd = 0x768;
	ItemT.nStartBit = 56;
	ItemT.nBitWidth = 8;
	ItemT.dScale = 0.01;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.fHedingAngleParameterC1");
	ItemT.nMsgInd = 0x769;
	ItemT.nStartBit = 0;
	ItemT.nBitWidth = 16;
	ItemT.dScale = 1.0/1024.0;
	ItemT.dBias = -32.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.fViewRange");
	ItemT.nMsgInd = 0x769;
	ItemT.nStartBit = 16;
	ItemT.nBitWidth = 15;
	ItemT.dScale = 1.0/256.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "LKA_Right.bViewRangeAvailability");
	ItemT.nMsgInd = 0x769;
	ItemT.nStartBit = 31;
	ItemT.nBitWidth = 1;
	ParsePattern.push_back(ItemT);

	//////////////////////////////////////////////////////////////////////////
	//RefPoints
	//////////////////////////////////////////////////////////////////////////
	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "RefPoints.fRefPoint1Position");
	ItemT.nMsgInd = 0x76a;
	ItemT.nStartBit = 0;
	ItemT.nBitWidth = 16;
	ItemT.dScale = 1.0/256.0;
	ItemT.dBias = -128.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "RefPoints.fRefPoint1Distance");
	ItemT.nMsgInd = 0x76a;
	ItemT.nStartBit = 16;
	ItemT.nBitWidth = 15;
	ItemT.dScale = 1.0/256.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "RefPoints.bRefPoint1Validity");
	ItemT.nMsgInd = 0x76a;
	ItemT.nStartBit = 31;
	ItemT.nBitWidth = 1;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "RefPoints.fRefPoint2Position");
	ItemT.nMsgInd = 0x76a;
	ItemT.nStartBit = 32;
	ItemT.nBitWidth = 16;
	ItemT.dScale = 1.0/256.0;
	ItemT.dBias = -128.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "RefPoints.fRefPoint2Distance");
	ItemT.nMsgInd = 0x76a;
	ItemT.nStartBit = 48;
	ItemT.nBitWidth = 15;
	ItemT.dScale = 1.0/256.0;
	ParsePattern.push_back(ItemT);

	ItemT = CCanParseItem();
	sprintf(ItemT.szName, "RefPoints.bRefPoint2Validity");
	ItemT.nMsgInd = 0x76a;
	ItemT.nStartBit = 63;
	ItemT.nBitWidth = 1;
	ParsePattern.push_back(ItemT);


	//////////////////////////////////////////////////////////////////////////
	m_CanParseOpr.SetParsePattern(ParsePattern);
}

void CMobileyeOpr::SetCallBack(lpMobileyeDataCallBack hCallBack, void* pUser)
{
	m_pMobileyeCallBack = hCallBack;
	m_pUser = pUser;
	return;
}

int CMobileyeOpr::Start()
{
	return m_CanParseOpr.Start();
}

int CMobileyeOpr::GetData(CMobileyeMsg* Msg)
{
	return 1;
}

int CMobileyeOpr::TransData(vector<CCanParseItem>& Data_in, CMobileyeMsg& Data_out)
{
	CMobileyeMsg Data_outT;
	unsigned int nInd = 0;

	Data_outT.ObstacleInfo.nNumObstacles = Data_in[nInd++].fValue;
	Data_outT.ObstacleInfo.nTimeStamp = Data_in[nInd++].fValue;
	Data_outT.ObstacleInfo.nAppVersion = Data_in[nInd++].fValue;
	Data_outT.ObstacleInfo.nActVerNumSec = Data_in[nInd++].fValue;
	Data_outT.ObstacleInfo.bLeftCloseRangCutIn = Data_in[nInd++].fValue;
	Data_outT.ObstacleInfo.bRightCloseRangCutIn = Data_in[nInd++].fValue;
	Data_outT.ObstacleInfo.nGo = Data_in[nInd++].fValue;
	Data_outT.ObstacleInfo.nProtocolVersion = Data_in[nInd++].fValue;
	Data_outT.ObstacleInfo.bCloseCar = Data_in[nInd++].fValue;
	Data_outT.ObstacleInfo.nFailSafe = Data_in[nInd++].fValue;
	
	for (unsigned int i = 0; i < 15; i++)
	{
		CObstacleDetails ObsDetailT;
		bool bIsFilled = Data_in[nInd].bIsProcessed;
		ObsDetailT.nObstacleId = Data_in[nInd++].fValue;
		ObsDetailT.fPosX = Data_in[nInd++].fValue;
		ObsDetailT.fPosY = Data_in[nInd++].fValue;
		ObsDetailT.nBlinkerInfo = Data_in[nInd++].fValue;
		ObsDetailT.nCutInOrOut = Data_in[nInd++].fValue;
		ObsDetailT.fRelVelX = Data_in[nInd++].fValue;
		ObsDetailT.nObstacleType = Data_in[nInd++].fValue;
		ObsDetailT.nObstacleStatus = Data_in[nInd++].fValue;
		ObsDetailT.bObstacleBrakeLights = Data_in[nInd++].fValue;
		ObsDetailT.nObstacleValid = Data_in[nInd++].fValue;
		ObsDetailT.fLength = Data_in[nInd++].fValue;
		ObsDetailT.fWidth = Data_in[nInd++].fValue;
		ObsDetailT.nAge = Data_in[nInd++].fValue;
		ObsDetailT.nLane = Data_in[nInd++].fValue;
		ObsDetailT.bCIPV = Data_in[nInd++].fValue;
		ObsDetailT.fRadarPosX = Data_in[nInd++].fValue;
		ObsDetailT.fRadarVelX = Data_in[nInd++].fValue;
		ObsDetailT.nRadarMatchConfidence = Data_in[nInd++].fValue;
		ObsDetailT.nMatchedRadarId = Data_in[nInd++].fValue;
		ObsDetailT.fAngleRate = Data_in[nInd++].fValue;
		ObsDetailT.fScaleChange = Data_in[nInd++].fValue;
		ObsDetailT.fAccelX = Data_in[nInd++].fValue;
		ObsDetailT.bReplaced = Data_in[nInd++].fValue;
		ObsDetailT.fAngle = Data_in[nInd++].fValue;
		if (bIsFilled)
		{
			Data_outT.Obstalces.push_back(ObsDetailT);
		}
	}

	if (Data_in[nInd].bIsProcessed)
	{
		Data_outT.LKA_Left.bIsValid = true;
	}
	Data_outT.LKA_Left.nLaneType = Data_in[nInd++].fValue;
	Data_outT.LKA_Left.nQuality = Data_in[nInd++].fValue;
	Data_outT.LKA_Left.nModelDegree = Data_in[nInd++].fValue;
	Data_outT.LKA_Left.dPositionParamC0 = Data_in[nInd++].fValue;
	Data_outT.LKA_Left.dCurvatureParamC2 = Data_in[nInd++].fValue;
	Data_outT.LKA_Left.dCurvatureDerivativeParamC3 = Data_in[nInd++].fValue;
	Data_outT.LKA_Left.dWidthOfMarking = Data_in[nInd++].fValue;
	Data_outT.LKA_Left.dHedingAngleParameterC1 = Data_in[nInd++].fValue;
	Data_outT.LKA_Left.dViewRange = Data_in[nInd++].fValue;
	Data_outT.LKA_Left.bViewRangeAvailability = Data_in[nInd++].fValue;

	if (Data_in[nInd].bIsProcessed)
	{
		Data_outT.LKA_Right.bIsValid = true;
	}
	Data_outT.LKA_Right.nLaneType = Data_in[nInd++].fValue;
	Data_outT.LKA_Right.nQuality = Data_in[nInd++].fValue;
	Data_outT.LKA_Right.nModelDegree = Data_in[nInd++].fValue;
	Data_outT.LKA_Right.dPositionParamC0 = Data_in[nInd++].fValue;
	Data_outT.LKA_Right.dCurvatureParamC2 = Data_in[nInd++].fValue;
	Data_outT.LKA_Right.dCurvatureDerivativeParamC3 = Data_in[nInd++].fValue;
	Data_outT.LKA_Right.dWidthOfMarking = Data_in[nInd++].fValue;
	Data_outT.LKA_Right.dHedingAngleParameterC1 = Data_in[nInd++].fValue;
	Data_outT.LKA_Right.dViewRange = Data_in[nInd++].fValue;
	Data_outT.LKA_Right.bViewRangeAvailability = Data_in[nInd++].fValue;

	if (Data_in[nInd].bIsProcessed)
	{
		Data_outT.RefPoints.bIsValid= true;
	}
	Data_outT.RefPoints.dRefPoint1Position = Data_in[nInd++].fValue;
	Data_outT.RefPoints.dRefPoint1Distance = Data_in[nInd++].fValue;
	Data_outT.RefPoints.bRefPoint1Validity = Data_in[nInd++].fValue;
	Data_outT.RefPoints.dRefPoint2Position = Data_in[nInd++].fValue;
	Data_outT.RefPoints.dRefPoint2Distance = Data_in[nInd++].fValue;
	Data_outT.RefPoints.bRefPoint2Validity = Data_in[nInd++].fValue;


	Data_out = Data_outT;

	return 1;
}