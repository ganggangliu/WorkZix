#include "MobileyeOpr.h"
#include "OpenCVInc.h"
#include <math.h>
#include "GpsManager.h"

#include "LcmReceiver.h"
#include <LCM_SENSOR_FUSION_PACKAGE.hpp>

#if _DEBUG
#pragma comment(lib,"MobileyeOprD.lib")
#else
#pragma comment(lib,"MobileyeOpr.lib")
#endif

int g_nGridFront = 150000;
int g_nGridSide = 30000;
int g_nGridSize = 250;
Mat Draw(CMobileyeMsg& Msg);
Point2d CarXY2ImageUV(Point2d CarXY);
void DrawLane(Mat Img, CLKA_Lane& Lane);

CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE> g_LcmSend(string("LCM_SENSOR_FUSION_PACKAGE"));
CLcmRevicer<LCM_MOBILEYE_INFO> g_MobileyeLcmSend(string("LCM_MOBILEYE_INFO"));
void SendLcmPackage(CMobileyeMsg* pData_);
void Send(CMobileyeMsg* pData_);

CGpsManager g_PosData;

void __stdcall MobileyeDataCallBack(void* pData, void* pUser)
{
	CMobileyeMsg* pData_ = (CMobileyeMsg*)pData;

	Mat Img = Draw(*pData_);
	cvNamedWindow("Img",1);
	imshow("Img",Img);
	waitKey(1);

	Send(pData_);
	return;

	SendLcmPackage(pData_);

	system("cls");
	printf("Frame:%d\n", pData_->nFrameId);
	printf("Left line: %d %d\n", pData_->LKA_Left.bIsValid, pData_->LKA_Left.nQuality);
	printf("Right line: %d %d\n", pData_->LKA_Right.bIsValid, pData_->LKA_Right.nQuality);
	printf("Obstacle cont:%d/%d\n", pData_->ObstacleInfo.nNumObstacles, 
		pData_->Obstalces.size());
	printf("RefPoint:[%.2f, %.2f] %d\n", pData_->RefPoints.dRefPoint1Position, 
		pData_->RefPoints.dRefPoint1Distance, pData_->RefPoints.bRefPoint1Validity);
	for (int i = 0; i < pData_->Obstalces.size(); i++)
	{
		CObstacleDetails& ObsT = pData_->Obstalces[i];
		printf("++++++++++++++++++++++++++++++++++++++++++++++++++\n");
		printf("Id:%d\t",ObsT.nObstacleId);
		printf("Pos:[%.2fm,%.2fm]\t", ObsT.fPosX, ObsT.fPosY);
		printf("RelVelX:%.2fm/s\t", ObsT.fRelVelX);
		if (ObsT.nObstacleType == 0x000)
			printf("Type:Vehicle");
		else if (ObsT.nObstacleType == 0x001)
			printf("Type:Truck");
		else if (ObsT.nObstacleType == 0x010)
			printf("Type:Bike");
		else if (ObsT.nObstacleType == 0x011)
			printf("Type:Ped");
		else if (ObsT.nObstacleType == 0x100)
			printf("Type:Bicycle");
		else
			printf("Type:Unknow");
		printf("%d\n",ObsT.nObstacleType);

		printf("Size:[%.2fm,%.2fm]\t", ObsT.fLength, ObsT.fWidth);
		printf("Age:%d\t", ObsT.nAge);
		if (ObsT.nObstacleValid == 1)
			printf("New");
		else if (ObsT.nObstacleValid == 2)
			printf("Old");
		printf("\t");

		if (ObsT.nLane == 1)
			printf("Lane:Ego");
		else if(ObsT.nLane == 2)
			printf("Lane:Next");
		else
			printf("Lane:Unknow");
		printf("%d\t", ObsT.nLane );

		printf("CIPV:%d\t", ObsT.bCIPV);

		printf("\n");
	}

// 	Mat Img = Draw(*pData_);
// 	cvNamedWindow("Img",1);
// 	imshow("Img",Img);
// 	waitKey(1);

	Sleep(1);
}

int main(int argc, char* argv[])
{
	g_PosData.Init(CGpsManager::GPS_MANAGER_TYPE_POS);
	g_PosData.Start();

	CMobileyeOpr Mobileye;
	CMobileyeParam Param;
	Param.nDevType = CAN_KVASER;
	Param.nDevInd = 0;
	Param.nChannelInd = 0;
	if (argc > 1)
	{
		Param.nChannelInd = atoi(argv[1]);
	}
	Param.nBaudRate = CAN_BAUDRATE_500K;
	Mobileye.Init(Param);
	Mobileye.SetCallBack(MobileyeDataCallBack,NULL);
	Mobileye.Start();

	Sleep(INFINITE);

	return 0;
}

Mat Draw(CMobileyeMsg& Msg)
{
	Mat Img = Mat::zeros(g_nGridFront/g_nGridSize+1,g_nGridSide*2/g_nGridSize+1,CV_8UC3);

	//Draw Lanes
	DrawLane(Img, Msg.LKA_Left);
	DrawLane(Img, Msg.LKA_Right);

	//Draw Objects
	for (int i = 0; i < Msg.Obstalces.size(); i++)
	{
		CObstacleDetails& ObsT = Msg.Obstalces[i];
		CvScalar color = CV_RGB(255,255,255);
		if (ObsT.bCIPV)
		{
			color = CV_RGB(255,0,0);
		}
		else if (ObsT.nLane == 1)
		{
			color = CV_RGB(255,255,0);
		}
		else
		{
			color = CV_RGB(255,255,255);
		}
		double X = -1000.0 * ObsT.fPosY;
		double Y = 1000.0 * ObsT.fPosX;
		Point2f center(X, Y);
		double dObjLength = 0.1;
		if (ObsT.fLength <= 30.0)
		{
			dObjLength = ObsT.fLength;
		}
		Point2f size(1000.0 * ObsT.fWidth, 1000.0 * dObjLength);
		RotatedRect rectOrient(center, size, 0.0);
		Point2f vertices[4];
		rectOrient.points(vertices);
		Point2d vertices_[4];
		for (int j = 0 ; j < 4; j++)
		{
			vertices_[j] = CarXY2ImageUV(vertices[j]);
		}
		for (int j = 0; j < 4; j++)
		{
			line(Img, vertices_[j], vertices_[(j+1)%4], color);
		}

		//Draw relative vel
		double dTimeDelta = 3.0;
		double dXDelta = 0.0;
		double dYDelta = dTimeDelta * 1000.0 * ObsT.fRelVelX;
		Point2d ptStart(-1000.0*ObsT.fPosY, 1000.0*ObsT.fPosX);
		Point2d ptEnd(ptStart.x + dXDelta, ptStart.y + dYDelta);
		Point2i ptStart_ = CarXY2ImageUV(ptStart);
		Point2i ptEnd_ = CarXY2ImageUV(ptEnd);
		cv::line(Img,ptStart_,ptEnd_,color);

		//Classify
		string szClassify;
		if (ObsT.nObstacleType == 0x000)
			szClassify = "Vehicle";
		else if (ObsT.nObstacleType == 0x001)
			szClassify = "Truck";
		else if (ObsT.nObstacleType == 0x010)
			szClassify = "Bike";
		else if (ObsT.nObstacleType == 0x011)
			szClassify = "Ped";
		else if (ObsT.nObstacleType == 0x100)
			szClassify = "Bicycle";
		else
			szClassify = "Unknow";
		putText(Img, szClassify, ptStart_, 0, 0.5, color);
	}

	//Draw ref point
	if (Msg.RefPoints.bIsValid)
	{
		Point2d RefPoint(Msg.RefPoints.dRefPoint1Position * 1000.0, Msg.RefPoints.dRefPoint1Distance * 1000.0);
		circle(Img,CarXY2ImageUV(RefPoint),3,CV_RGB(255,255,255),-1);
	}
	
	return Img;
}

void DrawLane(Mat Img, CLKA_Lane& Lane)
{
	CvScalar color;
	if (Lane.nQuality <= 1)
	{
		color = CV_RGB(255,0,0);
	}
	else
	{
		color = CV_RGB(255,255,255);
	}
	if (Lane.bIsValid/* && Lane.nQuality >= 2*/)
	{
		double dC0 = Lane.dPositionParamC0;
		double dC1 = Lane.dHedingAngleParameterC1;
		double dC2 = Lane.dCurvatureParamC2;
		double dC3 = Lane.dCurvatureDerivativeParamC3;
		vector<Point2d> LanePts;
		for (double i = 0.0; i < Lane.dViewRange; i++)
		{
			Point2d pt(dC3*pow(i,3)+dC2*pow(i,2)+dC1*i+dC0,i);
			pt *= 1000.0;
			LanePts.push_back(pt);
		}
		for (unsigned int i = 1; i < LanePts.size(); i++)
		{
			Point2d pt0 = CarXY2ImageUV(LanePts[i-1]);
			Point2d pt1 = CarXY2ImageUV(LanePts[i]);
			if (Lane.nLaneType == 0)
			{
				if (i%2 == 0)
				{
					line(Img,pt0,pt1,color);
				}
			}
			else
			{
				line(Img,pt0,pt1,color);
			}
		}
	}
}

Point2d CarXY2ImageUV(Point2d CarXY)
{
	Point2d out;
	double X = CarXY.x;
	double Y = CarXY.y;
	X = g_nGridSide + X;
	Y = g_nGridFront - Y;
	out.x = floor(X/g_nGridSize);
	out.y = floor(Y/g_nGridSize);

	return out;
}

void Send(CMobileyeMsg* pData_)
{
	LCM_MOBILEYE_INFO MobileyeData;
	MobileyeData.FrameId = pData_->nFrameId;

	MobileyeData.LeftLine.Valid =				pData_->LKA_Left.bIsValid;
	MobileyeData.LeftLine.Type =				pData_->LKA_Left.nLaneType;
	MobileyeData.LeftLine.Quality =				pData_->LKA_Left.nQuality;
	MobileyeData.LeftLine.nModelDegree =		pData_->LKA_Left.nModelDegree;
	MobileyeData.LeftLine.C0 =					pData_->LKA_Left.dPositionParamC0;
	MobileyeData.LeftLine.C2 =					pData_->LKA_Left.dCurvatureParamC2;
	MobileyeData.LeftLine.C3 =					pData_->LKA_Left.dCurvatureDerivativeParamC3;
	MobileyeData.LeftLine.Width =				pData_->LKA_Left.dWidthOfMarking;
	MobileyeData.LeftLine.C1 =					pData_->LKA_Left.dHedingAngleParameterC1;
	MobileyeData.LeftLine.Range=				pData_->LKA_Left.dViewRange;
	MobileyeData.LeftLine.RangeAvailability =	pData_->LKA_Left.bViewRangeAvailability;

	MobileyeData.RightLine.Valid =				pData_->LKA_Right.bIsValid;
	MobileyeData.RightLine.Type =				pData_->LKA_Right.nLaneType;
	MobileyeData.RightLine.Quality =			pData_->LKA_Right.nQuality;
	MobileyeData.RightLine.nModelDegree =		pData_->LKA_Right.nModelDegree;
	MobileyeData.RightLine.C0 =					pData_->LKA_Right.dPositionParamC0;
	MobileyeData.RightLine.C2 =					pData_->LKA_Right.dCurvatureParamC2;
	MobileyeData.RightLine.C3 =					pData_->LKA_Right.dCurvatureDerivativeParamC3;
	MobileyeData.RightLine.Width =				pData_->LKA_Right.dWidthOfMarking;
	MobileyeData.RightLine.C1 =					pData_->LKA_Right.dHedingAngleParameterC1;
	MobileyeData.RightLine.Range=				pData_->LKA_Right.dViewRange;
	MobileyeData.RightLine.RangeAvailability =	pData_->LKA_Right.bViewRangeAvailability;

	for (unsigned int i = 0; i < pData_->Obstalces.size(); i++)
	{
		LCM_MOBILEYE_OBJECT ObjT;
		ObjT.Id =				pData_->Obstalces[i].nObstacleId;
		ObjT.PosX =				pData_->Obstalces[i].fPosX;
		ObjT.PosY =				pData_->Obstalces[i].fPosY;
		ObjT.RelVelX =			pData_->Obstalces[i].fRelVelX;
		ObjT.Type=				pData_->Obstalces[i].nObstacleType;
		ObjT.Status =			pData_->Obstalces[i].nObstacleStatus;
		ObjT.BrakeLights =		pData_->Obstalces[i].bObstacleBrakeLights;
		ObjT.CutInOrOut =		pData_->Obstalces[i].nCutInOrOut;
		ObjT.BlinkerInfo=		pData_->Obstalces[i].nBlinkerInfo;
		ObjT.Valid =			pData_->Obstalces[i].nObstacleValid;
		ObjT.Length =			pData_->Obstalces[i].fLength;
		ObjT.Width =			pData_->Obstalces[i].fWidth;
		ObjT.Age =				pData_->Obstalces[i].nAge;
		ObjT.Lane =				pData_->Obstalces[i].nLane;
		ObjT.bCIPV =			pData_->Obstalces[i].bCIPV;
		ObjT.fRadarPosX =		pData_->Obstalces[i].fRadarPosX;
		ObjT.fRadarVelX =		pData_->Obstalces[i].fRadarVelX;
		ObjT.nRadarMatchConfidence = pData_->Obstalces[i].nRadarMatchConfidence;
		ObjT.nMatchedRadarId =	pData_->Obstalces[i].nMatchedRadarId;
		ObjT.fAngleRate =		pData_->Obstalces[i].fAngleRate;
		ObjT.fScaleChange =		pData_->Obstalces[i].fScaleChange;
		ObjT.fAccelX =			pData_->Obstalces[i].fAccelX;
		ObjT.bReplaced =		pData_->Obstalces[i].bReplaced;
		ObjT.fAngle =			pData_->Obstalces[i].fAngle;
		MobileyeData.ObjList.push_back(ObjT);
	}
	MobileyeData.nContObj = MobileyeData.ObjList.size();

	g_MobileyeLcmSend.Send("LCM_MOBILEYE_INFO", MobileyeData);

	printf("++++++++++++++++++++++++++++++++++++++++++++++++\n");
	printf("Frame:%d\n", MobileyeData.FrameId);
	printf("Object cont:%d\n", MobileyeData.ObjList.size());
	printf("Left Line:%d   Right Line:%d\n", MobileyeData.LeftLine.Quality,
		MobileyeData.RightLine.Quality);
}

void SendLcmPackage(CMobileyeMsg* pData_)
{
	LCM_SENSOR_FUSION_PACKAGE Pack;

	LCM_GPS_DATA Gps;
	int nRt = g_PosData.GetData(Gps);
	if (nRt != 1)
	{
		return;
	}

	Pack.Gps.GPS_VE = Gps.GPS_VE;
	Pack.Gps.GPS_VN = Gps.GPS_VN;

	if (pData_->LKA_Left.bIsValid &&
		pData_->LKA_Right.bIsValid &&
		pData_->LKA_Left.nQuality >= 2 &&
		pData_->LKA_Right.nQuality >= 2)
	{
		double dC0L = pData_->LKA_Left.dPositionParamC0;
		double dC1L = pData_->LKA_Left.dHedingAngleParameterC1;
		double dC2L = pData_->LKA_Left.dCurvatureParamC2;
		double dC3L = pData_->LKA_Left.dCurvatureDerivativeParamC3;
		vector<Point2d> LanePtsL;
		for (double i = -20.0; i < 100.0/*pData_->LKA_Left.dViewRange*/; i++)
		{
			Point2d pt(dC3L*pow(i,3)+dC2L*pow(i,2)+dC1L*i+dC0L,i);
			pt;
			LanePtsL.push_back(pt);
		}

		double dC0R = pData_->LKA_Right.dPositionParamC0;
		double dC1R = pData_->LKA_Right.dHedingAngleParameterC1;
		double dC2R = pData_->LKA_Right.dCurvatureParamC2;
		double dC3R = pData_->LKA_Right.dCurvatureDerivativeParamC3;
		vector<Point2d> LanePtsR;
		for (double i = -20.0; i < 100.0/*pData_->LKA_Right.dViewRange*/; i++)
		{
			Point2d pt(dC3R*pow(i,3)+dC2R*pow(i,2)+dC1R*i+dC0R,i);
			pt;
			LanePtsR.push_back(pt);
		}

		LCM_NAVI_LINE LineL;
		for (unsigned int i = 0; i < LanePtsL.size(); i++)
		{
			LCM_POINT2D_F pt;
			pt.x = LanePtsL[i].x;
			pt.y = LanePtsL[i].y;
			LineL.Line.push_back(pt);
		}
		LineL.ContLinePoint = LineL.Line.size();

		LCM_NAVI_LINE LineR;
		for (unsigned int i = 0; i < LanePtsR.size(); i++)
		{
			LCM_POINT2D_F pt;
			pt.x = LanePtsR[i].x;
			pt.y = LanePtsR[i].y;
			LineR.Line.push_back(pt);
		}
		LineR.ContLinePoint = LineR.Line.size();

		Pack.NaviInfo.Lines.push_back(LineL);
		Pack.NaviInfo.Lines.push_back(LineR);
		Pack.NaviInfo.ContLine = Pack.NaviInfo.Lines.size();

		LCM_NAVI_PATH Path;
		for (unsigned int i = 0; i < LanePtsL.size() && i < LanePtsR.size(); i++)
		{
			LCM_POINT2D_F pt;
			pt.x = (LanePtsL[i].x + LanePtsR[i].x)/2.0;
			pt.y = (LanePtsL[i].y + LanePtsR[i].y)/2.0;
			Path.Path.push_back(pt);
		}
		Path.ContPathPoint = Path.Path.size();
		Path.PathId = 0;
		Path.PathType = 1;
		
		Pack.NaviInfo.Paths.push_back(Path);
		Pack.NaviInfo.ContPath = Pack.NaviInfo.Paths.size();
	}
	
	for (int i = 0; i < pData_->Obstalces.size(); i++)
	{
		CObstacleDetails& ObsT = pData_->Obstalces[i];
		LCM_IBEO_OBJECT ObjT;

		double X = 1000.0 * ObsT.fPosX;
		double Y = 1000.0 * ObsT.fPosY;
		Point2f center(X, Y);
		double dObjLength = 0.1;
		if (ObsT.fLength <= 30.0)
		{
			dObjLength = ObsT.fLength;
		}
		Point2f size(1000.0 * dObjLength, 1000.0 * ObsT.fWidth);
		RotatedRect rectOrient(center, size, 0.0);
		Point2f vertices[4];
		rectOrient.points(vertices);
		for (int j = 0 ; j < 4; j++)
		{
			LCM_POINT2D_F pt;
			pt.x = vertices[j].x;
			pt.y = vertices[j].y;
			ObjT.ContourPts.push_back(pt);
		}
		ObjT.ContContourPt = ObjT.ContourPts.size();

		ObjT.Age = 100;
		ObjT.ObjExtMeasurement = 1.0;
		ObjT.PredictAge = 0;
		ObjT.AbsVelocity.x = 0.0;
		ObjT.AbsVelocity.y = 0.0;
		ObjT.RefPoint.x = X;
		ObjT.RefPoint.y = Y;
		ObjT.Classification = 0;
		ObjT.Id = ObsT.nObstacleId;
		ObjT.ObjBoxCenter.x = X;
		ObjT.ObjBoxCenter.y = Y;
		ObjT.ObjBoxSize.x = size.x;
		ObjT.ObjBoxSize.y = size.y;
		ObjT.RelativeVelocity.x = ObsT.fRelVelX*1000.0;
		ObjT.RelativeVelocity.y = 0.0;
		ObjT.ObjOrientation = 0.0;

		Pack.IbeoObjList.IbeoObjects.push_back(ObjT);
	}

	Pack.IbeoObjList.ContObjects = Pack.IbeoObjList.IbeoObjects.size();

	g_LcmSend.Send("LCM_SENSOR_FUSION_PACKAGE", Pack);

}