#include <fstream>
#include "TrackProcess .h"
#include "GisTransform.h"
#include "Other.h"

using namespace std;
using namespace cv;

CTrackProcess::CTrackProcess()
{
	Init();
}

int CTrackProcess::Init(/*CTrackProcessParam Param*/)
{
//	m_Param = Param;
// 	m_ParamL.dAngleOffSet = 9.4/*9.3*/;
// 	m_ParamL.PlaceOffSet = Point2d(-0.75, 4.03);
// 	m_ParamL.dFocus = 0.00097/*0.00096*//3;
// 
// 	m_ParamR.dAngleOffSet = -12.6/*-12.7*/;
// 	m_ParamR.PlaceOffSet = Point2d(0.75, 4.03/*3.65*/);
// 	m_ParamR.dFocus = 0.00097/*0.00096*//3;

	m_ParamL.dAngleOffSet = 8.2/*9.4*//*9.3*/;
	m_ParamL.PlaceOffSet = Point2d(-0.75/*-0.385*/, 4.03);
	m_ParamL.dFocus = 0.00097/*0.00096*//3;

	m_ParamR.dAngleOffSet = -12.6/*-12.6*//*-12.7*/;
	m_ParamR.PlaceOffSet = Point2d(0.75/*0.385*/, 4.03/*3.65*/);
	m_ParamR.dFocus = 0.00099/*0.00096*//3;

	m_UbloxPlaceOffSet = Point2d(0.0, 2.0);
	m_dUbloxHeadingOffSet = -2.0;

	return 1;
}

int CTrackProcess::AlignData(std::vector<CTrackData>& MotionL, 
	std::vector<CTrackData>& MotionR,
	std::vector<CUbloxData>& UbloxInfo)
{
	if (MotionL.size() <= 0 || MotionR.size() <= 0 || UbloxInfo.size() <= 0)
	{
		printf("MotionL.size() <= 0 || MotionR.size() <= 0 || UbloxInfo.size() <= 0\n");
		return -1;
	}

	int64_t nStartTimeTrackL = MotionL.front().nSysTime;
	int64_t nStartTimeTrackR = MotionR.front().nSysTime;
	int64_t nStartTime = -1;
	m_nStartIndUblox = -1;
	for (unsigned int i = 0;i < UbloxInfo.size(); i++)
	{
		if (UbloxInfo[i].nSystemTime >= nStartTimeTrackL &&
			UbloxInfo[i].nSystemTime >= nStartTimeTrackR)
		{
			nStartTime = UbloxInfo[i].nSystemTime;
			m_nStartIndUblox = i;
			break;
		}
	}
	if (m_nStartIndUblox < 0)
	{
		return -1;
	}

	m_nStartIndTrackL = -1;
	for (unsigned int i = 0; i < MotionL.size(); i++)
	{
		if (MotionL[i].nSysTime >= nStartTime)
		{
			m_nStartIndTrackL = i;
			break;
		}
	}
	if (m_nStartIndTrackL < 0)
	{
		return -1;
	}

	m_nStartIndTrackR = -1;
	for (unsigned int i = 0; i < MotionR.size(); i++)
	{
		if (MotionR[i].nSysTime >= nStartTime)
		{
			m_nStartIndTrackR = i;
			break;
		}
	}
	if (m_nStartIndTrackR < 0)
	{
		return -1;
	}

	Point2d UbloxStartPt(UbloxInfo[m_nStartIndUblox].nLatitude*0.0000001,
		UbloxInfo[m_nStartIndUblox].nLongitude*0.0000001);
	double dUbloxStartHeanding = UbloxInfo[m_nStartIndUblox].nHeadMot*0.00001 + 
		m_dUbloxHeadingOffSet;
	m_UbloxInCart.push_back(m_UbloxPlaceOffSet);
	for (unsigned int i = m_nStartIndUblox+1; i < UbloxInfo.size(); i++)
	{
		Point2d UbloxPt(UbloxInfo[i].nLatitude*0.0000001,
			UbloxInfo[i].nLongitude*0.0000001);
		Point2d DeltaPt = GisWgsPoint2PointRotate(UbloxStartPt, 
			UbloxPt, dUbloxStartHeanding);
		m_UbloxInCart.push_back(m_UbloxPlaceOffSet+DeltaPt);
	}

	return 1;
}

int CTrackProcess::Process(std::string szPath)
{
	m_szPath = szPath;
	if (m_szPath.back() != '/' && m_szPath.back() != '\\')
	{
		m_szPath += "/";
	}

	m_UbloxOpr.ReadLog(m_szPath+"ublox");
	m_UbloxOpr.Ublox2KmlFile(m_szPath+"result/ublox.kml", CV_RGB(255,0,0));

	ReadMotion(m_szPath+"result/left.txt", m_MotionL);
	ReadMotion(m_szPath+"result/right.txt", m_MotionR);

	AlignData(m_MotionL, m_MotionR, m_UbloxOpr.GetUbloxInfo());

	Motion2Track(m_MotionL, m_TrackL, m_ParamL, m_nStartIndTrackL);
	CompareTrackAndUblox(m_MotionL, m_UbloxOpr.GetUbloxInfo(), m_ParamL, m_nStartIndTrackL);
	WriteResult("left", m_MotionL, CV_RGB(0,255,0));
	
	Motion2Track(m_MotionR, m_TrackR, m_ParamR, m_nStartIndTrackR);
	CompareTrackAndUblox(m_MotionR, m_UbloxOpr.GetUbloxInfo(), m_ParamR, m_nStartIndTrackR);
	WriteResult("right", m_MotionR, CV_RGB(0,0,255));

	Mition2TrackStereoEx2(m_MotionL, m_ParamL, m_MotionR, m_ParamR, 
		m_UbloxOpr.GetUbloxInfo(), m_MotionStereo);
	CompareTrackAndUblox(m_MotionStereo, m_UbloxOpr.GetUbloxInfo(), m_ParamL, 0);
	WriteResult("stereo", m_MotionStereo, CV_RGB(255,0,255));

	return 1;
}

int CTrackProcess::Motion2Track(std::vector<CTrackData>& Motion,
	std::vector<cv::Point3d>& Track,
	CTrackProcessParam Param,
	int64_t nStartInd)
{
	double dYc = Param.PlaceOffSet.y;
	double dXc = Param.PlaceOffSet.x;
	for (unsigned int i = 0; i < Motion.size(); i++)
	{
		Point2f move(-1.0*Motion[i].fPixelY, -1.0*Motion[i].fPixelX);
		Motion[i].dMoveX = move.x*Param.dFocus;
		Motion[i].dMoveY = move.y*Param.dFocus;
		double dAlpha_rad = atan2(move.y, move.x);//rad
		double dCamHeading_rad = Param.dAngleOffSet/180.0*CV_PI + dAlpha_rad;//rad
		double dCamNormal_rad = dCamHeading_rad - CV_PI/2;
		if (dCamNormal_rad >= -1e-10 && dCamNormal_rad <= 0.0)
			dCamNormal_rad = -1e-10;
		if (dCamNormal_rad <= 1e-10 && dCamNormal_rad > 0.0)
			dCamNormal_rad = 1e-10;
		double dTanCam = tan(dCamNormal_rad);
		double dOriCar = dXc-dYc/dTanCam;
		double dRadiuCam = sqrt(pow(dYc,2) + pow(dXc-dOriCar,2));//m
		double dMoveDistCam = sqrt(pow(move.x,2) + pow(move.y,2))*Param.dFocus;//m
		double dDeltaAngleCam = atan2(dMoveDistCam, dRadiuCam);//rad
		if (dOriCar >= 0.0)
		{
			dDeltaAngleCam *= -1.0;
		}
		Motion[i].dAngleDelta = dDeltaAngleCam/CV_PI*180.0;
		if (i <= nStartInd)
		{
			Motion[i].dCamAngleCart = 90.0 + Param.dAngleOffSet;
			Motion[i].dCartX = Param.PlaceOffSet.x;
			Motion[i].dCartY = Param.PlaceOffSet.y;
		}
		else
		{
			Motion[i].dCamAngleCart = Motion[i-1].dCamAngleCart + Motion[i].dAngleDelta;
			Point2d pt0(Motion[i-1].dCartX, Motion[i-1].dCartY);
			Point2d pt1(Motion[i].dMoveX, Motion[i].dMoveY);
			Point2d out = Vehicle2Cart(pt0, Motion[i].dCamAngleCart, pt1);
			Motion[i].dCartX = out.x;
			Motion[i].dCartY = out.y;
		}
	}

	return 1;
}

int CTrackProcess::ReadMotion(std::string szLog, std::vector<CTrackData>& Motion)
{
	Motion.clear();
	ifstream fs(szLog);
	if (!fs.is_open())
	{
		return -1;
	}

	string szLine;
	while(getline(fs, szLine))
	{
		CTrackData temp;
		char aa[256] = {0};
		char bb[256] = {0};
		int nRt = sscanf(szLine.c_str(), "%lld,%lld,%f,%f",
			&temp.nInd, &temp.nSysTime, &temp.fPixelX, &temp.fPixelY);
		if (nRt < 4)
		{
			break;
		}
		Motion.push_back(temp);
	}

	return Motion.size();
}

int CTrackProcess::CompareTrackAndUblox(std::vector<CTrackData>& Motion,
	std::vector<CUbloxData>& UbloxInfo, CTrackProcessParam& Param,
	int64_t nStartInd)
{
	if (Motion.size() <= 0 || UbloxInfo.size() <= 0)
	{
		printf("Motion.size() <= 0 || UbloxInfo.size() <= 0\n");
		return -1;
	}

// 	int64_t nStartTimeTrack = Motion.front().nSysTime;
// 	int64_t nStartTime = -1;
// 	int64_t nStartIndUblox = -1;
// 	for (unsigned int i = 0;i < UbloxInfo.size(); i++)
// 	{
// 		if (UbloxInfo[i].nSystemTime >= nStartTimeTrack)
// 		{
// 			nStartTime = UbloxInfo[i].nSystemTime;
// 			nStartIndUblox = i;
// 			break;
// 		}
// 	}
// 	if (nStartIndUblox < 0)
// 	{
// 		return -1;
// 	}
// 
// 	Point2d UbloxStartPt(UbloxInfo[nStartIndUblox].nLatitude*0.0000001,
// 		UbloxInfo[nStartIndUblox].nLongitude*0.0000001);
// 	double dUbloxStartHeanding = UbloxInfo[nStartIndUblox].nHeadMot*0.00001 + 
// 		m_dUbloxHeadingOffSet;
// 	m_UbloxInCart.push_back(m_UbloxPlaceOffSet);
// 	for (unsigned int i = nStartIndUblox+1; i < UbloxInfo.size(); i++)
// 	{
// 		Point2d UbloxPt(UbloxInfo[i].nLatitude*0.0000001,
// 			UbloxInfo[i].nLongitude*0.0000001);
// 		Point2d DeltaPt = GisWgsPoint2PointRotate(UbloxStartPt, 
// 			UbloxPt, dUbloxStartHeanding);
// 		m_UbloxInCart.push_back(DeltaPt+m_UbloxInCart.back());
// 	}
// 
// 	int64_t nStartIndTrack = -1;
// 	for (unsigned int i = 0; i < Motion.size(); i++)
// 	{
// 		if (Motion[i].nSysTime == nStartTime)
// 		{
// 			nStartIndTrack = i;
// 			break;
// 		}
// 	}
// 	if (nStartIndTrack < 0)
// 	{
// 		return -1;
// 	}

	Point2d ptUblox(UbloxInfo[m_nStartIndUblox].nLatitude*0.0000001,
					UbloxInfo[m_nStartIndUblox].nLongitude*0.0000001);
	Point2d ptCam = Point2GisWgsPointRotate(ptUblox, Param.PlaceOffSet - m_UbloxPlaceOffSet,
					-1.0*UbloxInfo[m_nStartIndUblox].nHeadMot*0.00001);
	double dHeadingCam = UbloxInfo[m_nStartIndUblox].nHeadMot*0.00001 - 
						Param.dAngleOffSet + m_dUbloxHeadingOffSet;

	for (unsigned int i = 0; i <= nStartInd; i++)
	{
		Motion[i].dLatitude = ptCam.x;
//			UbloxInfo[nStartIndUblox].nLatitude*0.0000001;
		Motion[i].dLongitude= ptCam.y;
//			UbloxInfo[nStartIndUblox].nLongitude*0.0000001;
		Motion[i].dHeading = dHeadingCam;
//			UbloxInfo[nStartIndUblox].nHeadMot*0.00001 - Param.dAngleOffSet;//WGS坐标系与平面坐标系角度是相反的，所以是减号
	}
	for (unsigned int i = nStartInd+1; i< Motion.size(); i++)
	{
		Point2d pt0(Motion[i-1].dLatitude, Motion[i-1].dLongitude);
		Point2d pt1(Motion[i-1].dMoveX, Motion[i-1].dMoveY);
		Point2d pt2 = Point2GisWgsPointRotate(pt0, pt1, -1.0*Motion[i-1].dHeading);
		Motion[i].dLatitude = pt2.x;
		Motion[i].dLongitude = pt2.y;
		Motion[i].dHeading = Motion[i-1].dHeading - Motion[i-1].dAngleDelta;
	}

	return 1;
}

int CTrackProcess::WriteResult(std::string szName, std::vector<CTrackData>& Motion, 
	cv::Scalar color)
{
	vector<Point3d> Line;
	for (unsigned int i = 0; i < Motion.size(); i++)
	{
		Line.push_back(Point3d(Motion[i].dLongitude,
			Motion[i].dLatitude, 0.0));
	}
	WriteKmlFile(m_szPath+"result/" + szName + ".kml", szName, Line, color);

	return Motion.size();
}

int CTrackProcess::Mition2TrackStereo(std::vector<CTrackData>& MotionL,
	CTrackProcessParam ParamL,
	std::vector<CTrackData>& MotionR,
	CTrackProcessParam ParamR,
	std::vector<CUbloxData>& UbloxInfo,
	std::vector<CTrackData>& MotionStereo)
{
	Point2d ptRight2LeftNotRot(m_ParamR.PlaceOffSet - m_ParamL.PlaceOffSet);
	double dRotAng = 90 + (/*m_ParamR.dAngleOffSet*/ - m_ParamL.dAngleOffSet);
	Point2d ptRight2LeftRot = Vehicle2Cart(Point2d(0.0,0.0),
		dRotAng, ptRight2LeftNotRot);
	double dAngR2L = m_ParamR.dAngleOffSet - m_ParamL.dAngleOffSet;
	for (unsigned int i = 0; i < MotionL.size() && i < m_MotionR.size(); i++)
	{
		if (i >= m_nStartIndTrackL && i >= m_nStartIndTrackR)
		{
			unsigned int nIndL = i - m_nStartIndTrackL;
			unsigned int nIndR = i - m_nStartIndTrackR;
			CTrackData Data = m_MotionL[nIndL];
			CTrackData& DataL = m_MotionL[nIndL];
			CTrackData& DataR = m_MotionR[nIndR];

//			Point2d pt0 = ptRight2LeftRot;
			Point2d pt1(DataR.dMoveX, DataR.dMoveY);
			Point2d pt2 = Vehicle2Cart(Point2d(0.0,0.0), 90 + dAngR2L, pt1);
			Point2d MoveLeft(DataL.dMoveX, DataL.dMoveY);
			Point2d RotMoveRight2Left = pt2 - MoveLeft;
			double dDist = sqrt(pow(ptRight2LeftRot.x,2) + pow(ptRight2LeftRot.y,2));
			double dDot = ptRight2LeftRot.dot(RotMoveRight2Left)/dDist;
			double dCross = ptRight2LeftRot.cross(RotMoveRight2Left)/dDist;
			Data.dAngleDelta = atan2(dCross, dDist)/CV_PI*180.0;
			Data.dStereoErr = dDot;
			if (MotionStereo.size() <= 0)
			{
				Data.dCamAngleCart = 90.0 + ParamL.dAngleOffSet;
				Data.dCartX = ParamL.PlaceOffSet.x/* - m_UbloxPlaceOffSet.x*/;
				Data.dCartY = ParamL.PlaceOffSet.y/* - m_UbloxPlaceOffSet.y*/;
			}
			else
			{
				Data.dCamAngleCart = MotionStereo.back().dCamAngleCart + Data.dAngleDelta;
				Point2d pt0(MotionStereo.back().dCartX, MotionStereo.back().dCartY);
				Point2d pt1(Data.dMoveX, Data.dMoveY);
				Point2d out = Vehicle2Cart(pt0, Data.dCamAngleCart, pt1);
				Data.dCartX = out.x;
				Data.dCartY = out.y;
			}
			MotionStereo.push_back(Data);
		}
	}

	return 1;
}

int CTrackProcess::Mition2TrackStereoEx(std::vector<CTrackData>& MotionL,
	CTrackProcessParam ParamL,
	std::vector<CTrackData>& MotionR,
	CTrackProcessParam ParamR,
	std::vector<CUbloxData>& UbloxInfo,
	std::vector<CTrackData>& MotionStereo)
{
	ofstream fs("MotionStereo.txt");
	Point2d ptRight2LeftNotRot(m_ParamR.PlaceOffSet - m_ParamL.PlaceOffSet);
	double dRotAng = 90 + (/*m_ParamR.dAngleOffSet*/ - m_ParamL.dAngleOffSet);
	Point2d ptRight2LeftRot = Vehicle2Cart(Point2d(0.0,0.0),
		dRotAng, ptRight2LeftNotRot);
	double dAngR2L = m_ParamR.dAngleOffSet - m_ParamL.dAngleOffSet;
	for (unsigned int i = 0; i+m_nStartIndTrackL < MotionL.size() && 
		(i+m_nStartIndTrackR < m_MotionR.size()); i++)
	{
		unsigned int nIndL = i + m_nStartIndTrackL;
		unsigned int nIndR = i + m_nStartIndTrackR;
		CTrackData Data = m_MotionL[nIndL];
		CTrackData& DataL = m_MotionL[nIndL];
		CTrackData& DataR = m_MotionR[nIndR];

		Point2d VecLCam(DataL.dMoveX, DataL.dMoveY);
		Point2d VecLVeh = Vehicle2Cart(Point2d(0.0,0.0), 90 + m_ParamL.dAngleOffSet, VecLCam);
		double dAngLCar = atan2(VecLVeh.y, VecLVeh.x)/CV_PI*180.0;
		Point2d VecRCam(DataR.dMoveX, DataR.dMoveY);
		Point2d VecRVeh = Vehicle2Cart(Point2d(0.0,0.0), 90 + m_ParamR.dAngleOffSet, VecRCam);
		double dAngRCar = atan2(VecRVeh.y, VecRVeh.x)/CV_PI*180.0;
		double dAngLBia = dAngLCar-90;
		double dAngRBia = dAngRCar-90;
		char szLine[1024] = {0};
		sprintf(szLine, "%d,dAngLBia:%.2f, dAngRBia:%.2f\n",nIndL, dAngLBia, dAngRBia);
		fs << szLine;
		fs.flush();
		if (dAngLBia*dAngRBia <= 0.0 ||
			(dAngLBia > 0 && dAngRBia > 0 && abs(dAngLBia) < abs(dAngRBia)) ||
			(dAngLBia < 0 && dAngRBia < 0 && abs(dAngLBia) > abs(dAngRBia)))
		{
			Data.dAngleDelta = 0.0;
		}
		else
		{
			Point2d v0 = VecLVeh;
			Point2d v1 = VecRVeh;
			double dTemp = v1.y*v0.x-v0.y*v1.x;
			if (abs(dTemp) <= 0.0000000001)
			{
				Data.dAngleDelta = 0.0;
			}
			else
			{
				Point2d Center;
				Center.y = ptRight2LeftNotRot.x*v1.x/dTemp*v0.x;
				Center.x = ptRight2LeftNotRot.x*v1.x/dTemp*(-1.0*v0.y);
				double dDist = sqrt(pow(Center.x,2) + pow(Center.y,2));
				if (dDist <= 5.0)
				{
					Data.dAngleDelta = 0.0;
				}
				else
				{
					double dMove = sqrt(pow(v0.x,2) + pow(v0.y,2));
					Data.dAngleDelta = atan2(dMove, dDist)/CV_PI*180.0;
					if (Center.x >= 0)
					{
						Data.dAngleDelta *= -1.0;
					}
				}
			}
		}
// 		if (abs(Data.dAngleDelta) >= 0.5)
// 		{
// 			Data.dAngleDelta = 0.0;
// 		}

// 		Point2d pt1(DataR.dMoveX, DataR.dMoveY);
// 		Point2d pt2 = Vehicle2Cart(Point2d(0.0,0.0), 90 + dAngR2L, pt1);
// 
// 		Point2d v0(DataL.dMoveX, DataL.dMoveY);
// 		Point2d v1 = pt2;
// 		Point2d p1 = ptRight2LeftRot;
// 		Point2d ptCenter;
// 		double aa = p1.x*v1.x+p1.y*v1.y;//p1和v1垂直性
// 		double bb = v1.y*v0.x-v0.y*v1.x;//v1和v0平行性
// 		if (abs(aa) <= 1e-4/* || *//*abs(bb) <= 1e-3*/)
// 		{
// 			Data.dAngleDelta = 0.0;
// 		}
// 		else
// 		{
// 			double dTemp = (aa)/(bb);
// 			ptCenter.y = dTemp*v0.x;
// 			ptCenter.x = dTemp*-1.0*v0.y;
// 			double dDist = sqrt(pow(ptCenter.x,2) + pow(ptCenter.y,2));
// 			double dAngDist = atan2(ptCenter.y, ptCenter.x)*180.0/CV_PI;
// 			double dMove = sqrt(pow(v0.x,2) + pow(v0.y,2));
// 			double dAngMove = atan2(v0.y, v0.x)*180.0/CV_PI;
// 			double dDiffAng = dAngMove - dAngDist;
// 			if (dDiffAng < 0.0)
// 				dDiffAng += 360.0;
// 			Data.dAngleDelta = atan2(dMove, dDist)/CV_PI*180.0;
// 			if (dDiffAng <= 180.0)
// 			{
// 				Data.dAngleDelta *= -1.0;
// 			}
// 		}
		if (MotionStereo.size() <= 0)
		{
			Data.dCamAngleCart = 90.0 + ParamL.dAngleOffSet;
			Data.dCartX = ParamL.PlaceOffSet.x;
			Data.dCartY = ParamL.PlaceOffSet.y;
		}
		else
		{
			Data.dCamAngleCart = MotionStereo.back().dCamAngleCart + Data.dAngleDelta;
			Point2d pt0(MotionStereo.back().dCartX, MotionStereo.back().dCartY);
			Point2d pt1(Data.dMoveX, Data.dMoveY);
			Point2d out = Vehicle2Cart(pt0, Data.dCamAngleCart, pt1);
			Data.dCartX = out.x;
			Data.dCartY = out.y;
		}
		MotionStereo.push_back(Data);
		m_DeltaHead.push_back(Data.dAngleDelta);
	}

	return 1;
}

int CTrackProcess::Mition2TrackStereoEx1(std::vector<CTrackData>& MotionL,
	CTrackProcessParam ParamL,
	std::vector<CTrackData>& MotionR,
	CTrackProcessParam ParamR,
	std::vector<CUbloxData>& UbloxInfo,
	std::vector<CTrackData>& MotionStereo)
{
	Point2d ptRight2LeftNotRot(m_ParamR.PlaceOffSet - m_ParamL.PlaceOffSet);
	double dRotAng = 90 + (/*m_ParamR.dAngleOffSet*/ - m_ParamL.dAngleOffSet);
	Point2d ptRight2LeftRot = Vehicle2Cart(Point2d(0.0,0.0),
		dRotAng, ptRight2LeftNotRot);
	double dAngR2L = m_ParamR.dAngleOffSet - m_ParamL.dAngleOffSet;
	ofstream fs("MotionStereo.txt");
	for (unsigned int i = 0; i+m_nStartIndTrackL < MotionL.size() && 
		(i+m_nStartIndTrackR < m_MotionR.size()); i++)
	{
		unsigned int nIndL = i + m_nStartIndTrackL;
		unsigned int nIndR = i + m_nStartIndTrackR;
		CTrackData Data = m_MotionL[nIndL];
		CTrackData& DataL = m_MotionL[nIndL];
		CTrackData& DataR = m_MotionR[nIndR];

		double dSpeedL = sqrt(pow(DataL.dMoveX,2) + pow(DataL.dMoveY,2));
		double dSpeedR = sqrt(pow(DataR.dMoveX,2) + pow(DataR.dMoveY,2));
		double dSpeedDiff = dSpeedR - dSpeedL;
		double dAngDiff = dSpeedDiff/1.3/*ptRight2LeftNotRot.x*//CV_PI*180.0;
		Data.dAngleDelta = dAngDiff;
// 		if (dSpeedDiff >= 0)
// 		{
// 			Data.dAngleDelta *= -1.0;
// 		}

		if (MotionStereo.size() <= 0)
		{
			Data.dCamAngleCart = 90.0 + ParamL.dAngleOffSet;
			Data.dCartX = ParamL.PlaceOffSet.x;
			Data.dCartY = ParamL.PlaceOffSet.y;
		}
		else
		{
			Data.dCamAngleCart = MotionStereo.back().dCamAngleCart + Data.dAngleDelta;
			Point2d pt0(MotionStereo.back().dCartX, MotionStereo.back().dCartY);
			Point2d pt1(Data.dMoveX, Data.dMoveY);
			Point2d out = Vehicle2Cart(pt0, Data.dCamAngleCart, pt1);
			Data.dCartX = out.x;
			Data.dCartY = out.y;
		}
		fs << Data.dAngleDelta << endl;
		MotionStereo.push_back(Data);
		m_DeltaHead.push_back(Data.dAngleDelta);
	}

	return 1;
}

int CTrackProcess::Mition2TrackStereoEx2(std::vector<CTrackData>& MotionL,
	CTrackProcessParam ParamL,
	std::vector<CTrackData>& MotionR,
	CTrackProcessParam ParamR,
	std::vector<CUbloxData>& UbloxInfo,
	std::vector<CTrackData>& MotionStereo)
{
	Point2d ptRight2LeftNotRot(m_ParamR.PlaceOffSet - m_ParamL.PlaceOffSet);
	double dRotAng = 90 + (/*m_ParamR.dAngleOffSet*/ - m_ParamL.dAngleOffSet);
	Point2d ptRight2LeftRot = Vehicle2Cart(Point2d(0.0,0.0),
		dRotAng, ptRight2LeftNotRot);
	double dAngR2L = m_ParamR.dAngleOffSet - m_ParamL.dAngleOffSet;
	ofstream fs("MotionStereo.txt");
	for (unsigned int i = 0; i+m_nStartIndTrackL < MotionL.size() && 
		(i+m_nStartIndTrackR < m_MotionR.size()); i++)
	{
		unsigned int nIndL = i + m_nStartIndTrackL;
		unsigned int nIndR = i + m_nStartIndTrackR;
		CTrackData Data = m_MotionL[nIndL];
		CTrackData& DataL = m_MotionL[nIndL];
		CTrackData& DataR = m_MotionR[nIndR];

		Point2d VecLCam(DataL.dMoveX, DataL.dMoveY);
		Point2d VecLVeh = Vehicle2Cart(Point2d(0.0,0.0), 90 + m_ParamL.dAngleOffSet, VecLCam);
		double dAngLCar = atan2(VecLVeh.y, VecLVeh.x)/CV_PI*180.0;
		double dSpeedL = sqrt(pow(DataL.dMoveX,2)+pow(DataL.dMoveY,2));
		double dSpeedR = sqrt(pow(DataR.dMoveX,2)+pow(DataR.dMoveY,2));
		double dAngBia = dAngLCar - 90.0;
		Point2d center(-1000,0);
		if (dSpeedL == 0 || dSpeedR == 0)
		{
			Data.dAngleDelta = 0.0;
		}
		else if (dAngBia >= 0.0 && dSpeedL >= dSpeedR)
		{
			Data.dAngleDelta = 0.0;
		}
		else if (dAngBia <= 0.0 && dSpeedL <= dSpeedR)
		{
			Data.dAngleDelta = 0.0;
		}
		else
		{
			double L = ParamR.PlaceOffSet.x - ParamL.PlaceOffSet.x;
			if (dAngBia >= 0.0)
			{
				double dAngL = dAngBia;
				double dAngR = asin(dSpeedL/dSpeedR*sin(dAngL/180.0*CV_PI))/CV_PI*180.0;
				Point2d ptCenter(0,0);
				ptCenter.x = L/(1.0 - tan(dAngL/180.0*CV_PI)/tan(dAngR/180.0*CV_PI));
				ptCenter.y = tan((dAngLCar+90.0)/180.0*CV_PI)*ptCenter.x;
				center = ptCenter;
				double dDist = sqrt(pow(ptCenter.x,2)+pow(ptCenter.y,2));
				double dMove = sqrt(pow(DataL.dMoveX,2)+pow(DataL.dMoveY,2));
				Data.dAngleDelta = atan2(dMove, dDist)/CV_PI*180.0;
			}
			else
			{
				double dAngL = abs(dAngBia);
				double dAngR = asin(dSpeedL/dSpeedR*sin(dAngL/180.0*CV_PI))/CV_PI*180.0;
				Point2d ptCenter(0,0);
				ptCenter.x = L/(tan(dAngR/180.0*CV_PI)/tan(dAngL/180.0*CV_PI) - 1.0) + L;
				ptCenter.y = tan((dAngLCar-90.0)/180.0*CV_PI)*ptCenter.x;
				center = ptCenter;
				double dDist = sqrt(pow(ptCenter.x,2)+pow(ptCenter.y,2));
				double dMove = sqrt(pow(DataL.dMoveX,2)+pow(DataL.dMoveY,2));
				Data.dAngleDelta = -1.0*atan2(dMove, dDist)/CV_PI*180.0;
			}
		}
		fs << Data.dAngleDelta << ","<< center.x << ","<< center.y << endl;

		if (MotionStereo.size() <= 0)
		{
			Data.dCamAngleCart = 90.0 + ParamL.dAngleOffSet;
			Data.dCartX = ParamL.PlaceOffSet.x;
			Data.dCartY = ParamL.PlaceOffSet.y;
		}
		else
		{
			Data.dCamAngleCart = MotionStereo.back().dCamAngleCart + Data.dAngleDelta;
			Point2d pt0(MotionStereo.back().dCartX, MotionStereo.back().dCartY);
			Point2d pt1(Data.dMoveX, Data.dMoveY);
			Point2d out = Vehicle2Cart(pt0, Data.dCamAngleCart, pt1);
			Data.dCartX = out.x;
			Data.dCartY = out.y;
		}
		MotionStereo.push_back(Data);
		m_DeltaHead.push_back(Data.dAngleDelta);
	}

	return 1;
}