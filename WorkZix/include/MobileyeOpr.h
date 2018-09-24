#ifndef MOBILEYE_OPR_H
#define MOBILEYE_OPR_H

#ifdef MOBILEYEOPR_EXPORTS
#define MOBILEYEOPR_API __declspec(dllexport)
#else
#define MOBILEYEOPR_API __declspec(dllimport)
#endif

#include <Windows.h>
#include "CanOpr.h"

typedef void(__stdcall *lpMobileyeDataCallBack)(void*, void*);

//////////////////////////////////////////////////////////////////////////
//Mobileye param
//////////////////////////////////////////////////////////////////////////
typedef struct tag_MobileyeParam
{
	CanDeviceType nDevType;			//CAN Device type
	int nDevInd;					//CAN Device index on the OS
	int nChannelInd;				//CAN channel index on the device
	CanBaudRate nBaudRate;			//CAN channel baudrate
	tag_MobileyeParam()
	{
		nDevType = CAN_KVASER;
		nDevInd = 0;
		nChannelInd = 0;
		nBaudRate = CAN_BAUDRATE_500K;
	}
}CMobileyeParam;

//////////////////////////////////////////////////////////////////////////
//Mobileye message
//////////////////////////////////////////////////////////////////////////
typedef struct tag_LaneDetail_669
{
	//0:bad 3:best
	unsigned char nLaneConfidenceLeft;
	bool bLDW_AvailableLeft;
	//0:dashed 1:solid 2:none 3:road edge 4:double lane mark 5:bott's dots 6:invalid
	unsigned char nLaneTypeLeft;
	//m
	float fDistance2LeftLane;
	unsigned char nLaneConfidenceRight;
	bool bLDW_AvailableRight;
	unsigned char nLaneTypeRight;
	float fDistance2RightLane;

}CLaneDetail;

typedef struct tag_ObstacleStatus_738
{
	unsigned char nNumObstacles;
	unsigned char nTimeStamp;		//ms
	unsigned char nAppVersion;
	unsigned char nActVerNumSec;
	bool bLeftCloseRangCutIn;
	bool bRightCloseRangCutIn;
	unsigned char nGo;				//0:stop 1:go 2:undecided 3:driver decision is required [4-14]:unused 15:not calculated
	unsigned char nProtocolVersion;
	bool bCloseCar;
	unsigned char nFailSafe;	//0000:no failsafe; 0001:low sun; 0010:blur image; 0100:unused
}CObstacleStatus;

typedef struct tag_ObstacleDetails_739_765
{
	unsigned char nObstacleId;
	float fPosX;
	float fPosY;
	float fRelVelX;
	//000:Vehicle 001:Truck 010:Bike 011:Ped 100:Bicycle
	unsigned char nObstacleType;
	//0:Undefined 1:Standing 2:Stop 3:Moving 4:Oncoming 5:Parking
	unsigned char nObstacleStatus;
	bool bObstacleBrakeLights;
	//0:undefined 1:in_host_line 2:out_host_line 3:cut_in 4:cut_out
	unsigned char nCutInOrOut;
	//0:unavailable 1:off 2:left 3:right 4:both
	unsigned char nBlinkerInfo;
	//1:new valid 2:older valid
	unsigned char nObstacleValid;
	float fLength;
	float fWidth;
	unsigned char nAge;
	//0:undefined 1:ego lane 2:next lane 3:invalid
	unsigned char nLane;
	bool bCIPV;
	float fRadarPosX;
	float fRadarVelX;
	//0:no match 1:not good 2-4:good 5:very good
	unsigned char nRadarMatchConfidence;
	unsigned char nMatchedRadarId;
	//degree/sec clockwise
	float fAngleRate;
	//pixel/sec
	float fScaleChange;
	//m/s^2
	float fAccelX;
	bool bReplaced;
	//degree
	float fAngle;
}CObstacleDetails;

typedef struct tag_LKA_Lane_766_769
{
	bool bIsValid;
	//refer to tag_LaneDetail_669
	unsigned char nLaneType;
	//0,1:not good;  2,3:good
	unsigned char nQuality;
	//1:linear model 2:parabolic model 3:3thd-degree model
	unsigned char nModelDegree;
	//unit:meter meaning:physical distance between lane mark and camera on the lateral position 
	double dPositionParamC0;
	//To extract the road radius(r) from curvature(C2):r=1/(2*C2);
	double dCurvatureParamC2;
	double dCurvatureDerivativeParamC3;
	//
	double dWidthOfMarking;

	//unit:radians meaning:physical slope of the lane mark
	double dHedingAngleParameterC1;
	//unit:meter physical view range of the lane mark
	double dViewRange;
	bool bViewRangeAvailability;
	tag_LKA_Lane_766_769()
	{
		bIsValid = false;
	}
}CLKA_Lane;

typedef struct tag_LKA_Lane_76A
{
	bool bIsValid;
	//unit:meter, physical distance between camera and reference point1 on lateral axis
	double dRefPoint1Position;
	//unit:meter, physical distance between camera and reference point1 on longitude axis
	double dRefPoint1Distance;
	bool bRefPoint1Validity;
	double dRefPoint2Position;
	double dRefPoint2Distance;
	bool bRefPoint2Validity;
	tag_LKA_Lane_76A()
	{
		bIsValid = false;
	}
}CReferencePoints;

typedef struct tag_NumberOfNextLane_76B
{
	unsigned int nNumberOfNextLaneMarkerReported;
}CNumberOfNextLane;

typedef struct tag_LKA_NextLane_76C_77B
{
	//1: solid 2:undecided
	unsigned char nLaneType;
	//Not yet implemented for next lanes
	unsigned char nQuality;
	//1:linear model 2:parabolic model 3:3thd-degree model
	unsigned char nModelDegree;
	//unit:meter meaning:physical distance between lane mark and camera on the lateral position 
	double dPositionParamC0;
	//To extract the road radius(r) from curvature(C2):r=1/(2*C2);
	double dCurvatureParamC2;
	double dCurvatureDerivativeParamC3;
	//
	double dWidthOfMarking;

	//unit:radians meaning:physical slope of the lane mark
	double dHedingAngleParameterC1;
	//unit:meter physical view range of the lane mark
	double dViewRange;
	bool bViewRangeAvailability;
}CLKA_NextLane;

typedef struct tag_MobileyeMsg
{
	unsigned int nFrameId;
	CLaneDetail LaneDetail;				//669
	CObstacleStatus ObstacleInfo;		//738
	vector<CObstacleDetails> Obstalces;	//739-765
	CLKA_Lane LKA_Left;					//766-767
	CLKA_Lane LKA_Right;				//768-769
	CReferencePoints RefPoints;			//76A
	CNumberOfNextLane NextLaneCont;		//76B
	vector<CLKA_Lane> NextLanes;		//76C-77B
	tag_MobileyeMsg()
	{
		nFrameId = 0;
		Obstalces.clear();
	}
}CMobileyeMsg;

class MOBILEYEOPR_API CMobileyeOpr 
{
public:
	CMobileyeOpr();
	~CMobileyeOpr();

	int Init(CMobileyeParam& Param);
	void SetCallBack(lpMobileyeDataCallBack hCallBack, void* pUser);
	int Start();
	int GetData(CMobileyeMsg* Msg);

friend void __stdcall CanParseOprDataCallBack(void* pData, void* pUser);

private:
	void SetParsePattern();
	int TransData(vector<CCanParseItem>& Data_in, CMobileyeMsg& Data_out);
	CMobileyeParam m_Param;
	CCanParseOpr m_CanParseOpr;
	lpMobileyeDataCallBack m_pMobileyeCallBack;
	void* m_pUser;
	CMobileyeMsg m_CurData;
	unsigned int m_nFrameInd;
};

#endif
