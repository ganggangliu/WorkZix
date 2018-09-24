#include "VirtualNavi.h"
#include <math.h>

CVirtualLane::CVirtualLane()
{
	m_nNearIndStart = 0;
	m_nNearIndEnd = 0;
	m_nNearInd = 0;
	m_dMaxTrackDist = 10.0;
	m_dDistLine2Path = 2.0;
}

CVirtualLane::~CVirtualLane()
{

}

long CVirtualLane::Init(char* pszPath, CGisTransform* pGisOprMap)
{
	m_pGisOprMap = pGisOprMap;
	ReadTrackLog(pszPath);

	return 1;
}

LCM_NAVI_TO_SENSE_INFO CVirtualLane::TransData(LCM_NAVI_REQUIRE_INFO GpsRequir)
{
	LCM_NAVI_TO_SENSE_INFO out;
	LCM_NAVI_REQUIRE_INFO GpsRequirT = GpsRequir;
	Point2d ptT = m_pGisOprMap->Wgs2Cart(Point2d(GpsRequir.Gps.GPS_LATITUDE, GpsRequir.Gps.GPS_LONGITUDE));
	GpsRequirT.Gps.GPS_LATITUDE = ptT.x;
	GpsRequirT.Gps.GPS_LONGITUDE = ptT.y;
	CGisTransform GisLocal;
	GisLocal.SetBaseLocatin(GpsRequirT.Gps.GPS_LATITUDE, GpsRequirT.Gps.GPS_LONGITUDE, GpsRequirT.Gps.GPS_HEADING);

	out.MsgInd = GpsRequirT.MsgInd;
	out.RefLocale.x = ptT.x;
	out.RefLocale.y = ptT.y;
	out.RefHeading = GpsRequirT.Gps.GPS_HEADING;

	//calc dist of next 100 track points to the vehicle, find the nearest one;
	double dNearDist = DBL_MAX;
	long nNearInd = m_nNearInd;
	for (int i = 0; i < 10; i++)
	{
		long nIndT0 = m_nNearInd + i;
		while (nIndT0 >= VecGps.size())
			nIndT0 -= VecGps.size();
	
		//long nIndT1 = m_nNearInd + i + 1;
		//while (nIndT1 >= m_VecGps.size())
		//	nIndT1 -= m_VecGps.size();
		double dDist0 = sqrt(pow((GpsRequirT.Gps.GPS_LATITUDE - VecGps.at(nIndT0).GPS_LATITUDE), 2) + pow((GpsRequirT.Gps.GPS_LONGITUDE - VecGps.at(nIndT0).GPS_LONGITUDE), 2));
//		double dDist1 = sqrt(pow((GpsRequirT.Gps.GPS_LATITUDE - m_VecGps.at(nIndT1).GPS_LATITUDE), 2) + pow((GpsRequirT.Gps.GPS_LONGITUDE - m_VecGps.at(nIndT1).GPS_LONGITUDE), 2));
		if (/*dDist0 < dDist1 && */dDist0 <= dNearDist)
		{
			nNearInd = nIndT0;
			dNearDist = dDist0;
//			break;
		}
	}
	m_nNearInd = nNearInd;

	//if the distance of knee point to the vehicle is great then MAX_TRACK_DISTANCE, 
	//continue search for another nearest track point in all track points
	//meanwhile its dist must be less then MAX_TRACK_DISTANCE
	if (dNearDist > m_dMaxTrackDist)
	{
		printf("Over range:%.2f\n", dNearDist);
		for (int i = 0; i < VecGps.size(); i++)
		{
			long nIndT = m_nNearInd + i;
			while (nIndT >= VecGps.size())
				nIndT -= VecGps.size();
//			double dDist = GetDistance(GpsRequirT.Gps.GPS_LATITUDE, m_VecGps.at(nIndT).GPS_LATITUDE, GpsRequirT.Gps.GPS_LONGITUDE, m_VecGps.at(nIndT).GPS_LONGITUDE);
			double dDist = sqrt(pow((GpsRequirT.Gps.GPS_LATITUDE - VecGps.at(nIndT).GPS_LATITUDE), 2) + pow((GpsRequirT.Gps.GPS_LONGITUDE - VecGps.at(nIndT).GPS_LONGITUDE), 2));
			if (dDist <= m_dMaxTrackDist)
			{
				printf("Get new track:%.2f\n", dDist);
				m_nNearInd = nIndT;
				dNearDist = dDist;
				break;
			}
		}
		//if no track point fit, no track data output
		if (dNearDist > m_dMaxTrackDist)
		{
//			printf("Get new track faild:%.2f\n", dNearDist);
			printf("Get new track faild!!!  dNearDist:%.2f ,m_nNearInd:%d, m_nNearIndStart:%d, m_nNearIndEnd:%d\n", dNearDist,m_nNearInd,m_nNearIndStart,m_nNearIndEnd);
			out.Paths.clear();
			out.ContPath = 0;
			return out;
		}
	}

	double dAcc = 0.0;
	for (int i = 0; i < 100; i++)
	{
		long nIndT0 = m_nNearInd - i;
		while (nIndT0 < 0)
			nIndT0 += VecGps.size();
		long nIndT1 = m_nNearInd - i - 1;
		while (nIndT1 < 0)
			nIndT1 += VecGps.size();
//		double dDist = GetDistance(m_VecGps.at(nIndT0).GPS_LATITUDE, m_VecGps.at(nIndT1).GPS_LATITUDE, m_VecGps.at(nIndT0).GPS_LONGITUDE, m_VecGps.at(nIndT1).GPS_LONGITUDE);
		double dDist = sqrt(pow((VecGps.at(nIndT0).GPS_LATITUDE - VecGps.at(nIndT1).GPS_LATITUDE), 2) + pow((VecGps.at(nIndT0).GPS_LONGITUDE - VecGps.at(nIndT1).GPS_LONGITUDE), 2));
		dAcc += dDist;
		if (dAcc >= 30.0)
		{
			m_nNearIndStart = -1*i;
			break;
		}
	}

	dAcc = 0.0;
	for (int i = 0; i < 100; i++)
	{
		long nIndT0 = m_nNearInd + i;
		while (nIndT0 >= VecGps.size())
			nIndT0 -= VecGps.size();
		long nIndT1 = m_nNearInd + i + 1;
		while (nIndT1 >= VecGps.size())
			nIndT1 -= VecGps.size();
//		double dDist = GetDistance(m_VecGps.at(nIndT0).GPS_LATITUDE, m_VecGps.at(nIndT1).GPS_LATITUDE, m_VecGps.at(nIndT0).GPS_LONGITUDE, m_VecGps.at(nIndT1).GPS_LONGITUDE);
		double dDist = sqrt(pow((VecGps.at(nIndT0).GPS_LATITUDE - VecGps.at(nIndT1).GPS_LATITUDE), 2) + pow((VecGps.at(nIndT0).GPS_LONGITUDE - VecGps.at(nIndT1).GPS_LONGITUDE), 2));
		dAcc += dDist;
		if (dAcc >= 100.0)
		{
			m_nNearIndEnd = i;
			break;
		}
	}

	out.Paths.clear();
	LCM_NAVI_PATH PathT;
	LCM_NAVI_LINE LineL, LineR;
	for (int i = m_nNearIndStart; i <= m_nNearIndEnd; i++)
	{
		long nIndT = m_nNearInd + i;
		while (nIndT < 0)
			nIndT += VecGps.size();
		while (nIndT >= VecGps.size())
			nIndT -= VecGps.size();
	
		LCM_POINT2D_F pt0;
		Point2d pt0_(VecGps[nIndT].GPS_LATITUDE, VecGps[nIndT].GPS_LONGITUDE);
		Point2d pt1_ = GisLocal.CartRotate(pt0_);
		pt0.x = pt1_.x;
		pt0.y = pt1_.y;
		PathT.Path.push_back(pt0);

		pt0_ = Point2d(VecLineLeft[nIndT].GPS_LATITUDE, VecLineLeft[nIndT].GPS_LONGITUDE);
		pt1_ = GisLocal.CartRotate(pt0_);
		pt0.x = pt1_.x;
		pt0.y = pt1_.y;
		LineL.Line.push_back(pt0);

		pt0_ = Point2d(VecLineRight[nIndT].GPS_LATITUDE, VecLineRight[nIndT].GPS_LONGITUDE);
		pt1_ = GisLocal.CartRotate(pt0_);
		pt0.x = pt1_.x;
		pt0.y = pt1_.y;
		LineR.Line.push_back(pt0);
	}
	LineL.ContLinePoint = LineL.Line.size();
	LineR.ContLinePoint = LineR.Line.size();
	PathT.ContPathPoint = PathT.Path.size();
	PathT.IndLeftLine = 1;
	PathT.IndRightLine = 2;
	out.Paths.push_back(PathT);
	out.ContPath = out.Paths.size();
	LineL.LineId = 1;
	out.Lines.push_back(LineL);
	LineR.LineId = 2;
	out.Lines.push_back(LineR);
	out.ContLine = out.Lines.size();

	return out;
}

LCM_NAVI_TO_SENSE_INFO CVirtualLane::TransDataNotLoop(LCM_NAVI_REQUIRE_INFO GpsRequir)
{
	LCM_NAVI_TO_SENSE_INFO out;
	LCM_NAVI_REQUIRE_INFO GpsRequirT = GpsRequir;
	Point2d ptT = m_pGisOprMap->Wgs2Cart(Point2d(GpsRequir.Gps.GPS_LATITUDE, GpsRequir.Gps.GPS_LONGITUDE));
	GpsRequirT.Gps.GPS_LATITUDE = ptT.x;
	GpsRequirT.Gps.GPS_LONGITUDE = ptT.y;
	CGisTransform GisLocal;
	GisLocal.SetBaseLocatin(GpsRequirT.Gps.GPS_LATITUDE, GpsRequirT.Gps.GPS_LONGITUDE, GpsRequirT.Gps.GPS_HEADING);

	out.MsgInd = GpsRequirT.MsgInd;
	out.RefLocale.x = ptT.x;
	out.RefLocale.y = ptT.y;
	out.RefHeading = GpsRequirT.Gps.GPS_HEADING;

	if (m_nNearInd >= VecGps.size()-1)
	{
		m_nNearInd = 0;
		out.Paths.clear();
		out.ContPath = 0;
		return out;
	}
	//calc dist of next 100 track points to the vehicle, find the nearest one;
	double dNearDist = DBL_MAX;
	long nNearInd = m_nNearInd;
	for (int i = 0; i < 10; i++)
	{
		long nIndT0 = m_nNearInd + i;
		if (nIndT0 >= VecGps.size())
		{
			nIndT0 = VecGps.size()-1;
		}
		double dDist0 = sqrt(pow((GpsRequirT.Gps.GPS_LATITUDE - VecGps.at(nIndT0).GPS_LATITUDE), 2) + pow((GpsRequirT.Gps.GPS_LONGITUDE - VecGps.at(nIndT0).GPS_LONGITUDE), 2));
		if (dDist0 <= dNearDist)
		{
			nNearInd = nIndT0;
			dNearDist = dDist0;
		}
	}
	m_nNearInd = nNearInd;

	//if the distance of knee point to the vehicle is great then MAX_TRACK_DISTANCE, 
	//continue search for another nearest track point in all track points
	//meanwhile its dist must be less then MAX_TRACK_DISTANCE
	if (dNearDist > m_dMaxTrackDist)
	{
		printf("Over range:%.2f\n", dNearDist);
		for (int i = 0; i < VecGps.size(); i++)
		{
			long nIndT = m_nNearInd + i;
			while (nIndT >= VecGps.size())
				nIndT -= VecGps.size();
			double dDist = sqrt(pow((GpsRequirT.Gps.GPS_LATITUDE - VecGps.at(nIndT).GPS_LATITUDE), 2) + pow((GpsRequirT.Gps.GPS_LONGITUDE - VecGps.at(nIndT).GPS_LONGITUDE), 2));
			if (dDist <= m_dMaxTrackDist)
			{
				printf("Get new track:%.2f\n", dDist);
				m_nNearInd = nIndT;
				dNearDist = dDist;
				break;
			}
		}
		//if no track point fit, no track data output
		if (dNearDist > m_dMaxTrackDist)
		{
			printf("Get new track faild:%.2f\n", dNearDist);
			out.Paths.clear();
			out.ContPath = 0;
			return out;
		}
	}

	double dAcc = 0.0;
	m_nNearIndStart = 0;
	for (int i = 0; i < 100; i++)
	{
		long nIndT0 = m_nNearInd - i;
		long nIndT1 = m_nNearInd - i - 1;
		if(nIndT0 < 0 || nIndT1 < 0)
		{
			m_nNearIndStart = -1*i;
			break;
		}
		double dDist = sqrt(pow((VecGps.at(nIndT0).GPS_LATITUDE - VecGps.at(nIndT1).GPS_LATITUDE), 2) + pow((VecGps.at(nIndT0).GPS_LONGITUDE - VecGps.at(nIndT1).GPS_LONGITUDE), 2));
		dAcc += dDist;
		if (dAcc >= 30.0)
		{
			m_nNearIndStart = -1*i;
			break;
		}
	}

	dAcc = 0.0;
	m_nNearIndEnd = 0;
	for (int i = 0; i < 100; i++)
	{
		long nIndT0 = m_nNearInd + i;
		long nIndT1 = m_nNearInd + i + 1;
		if(nIndT0 >= VecGps.size() || nIndT1 >= VecGps.size())
		{
			m_nNearIndEnd = i;
			break;
		}
		double dDist = sqrt(pow((VecGps.at(nIndT0).GPS_LATITUDE - VecGps.at(nIndT1).GPS_LATITUDE), 2) + pow((VecGps.at(nIndT0).GPS_LONGITUDE - VecGps.at(nIndT1).GPS_LONGITUDE), 2));
		dAcc += dDist;
		if (dAcc >= 100.0)
		{
			m_nNearIndEnd = i;
			break;
		}
	}

	out.Paths.clear();
	LCM_NAVI_PATH PathT;
	LCM_NAVI_LINE LineL, LineR;
	for (int i = m_nNearIndStart; i <= m_nNearIndEnd; i++)
	{
		long nIndT = m_nNearInd + i;
		if (nIndT < 0 || nIndT >= VecGps.size())
		{
			continue;
		}

		LCM_POINT2D_F pt0;
		Point2d pt0_(VecGps[nIndT].GPS_LATITUDE, VecGps[nIndT].GPS_LONGITUDE);
		Point2d pt1_ = GisLocal.CartRotate(pt0_);
		pt0.x = pt1_.x;
		pt0.y = pt1_.y;
		PathT.Path.push_back(pt0);

		pt0_ = Point2d(VecLineLeft[nIndT].GPS_LATITUDE, VecLineLeft[nIndT].GPS_LONGITUDE);
		pt1_ = GisLocal.CartRotate(pt0_);
		pt0.x = pt1_.x;
		pt0.y = pt1_.y;
		LineL.Line.push_back(pt0);

		pt0_ = Point2d(VecLineRight[nIndT].GPS_LATITUDE, VecLineRight[nIndT].GPS_LONGITUDE);
		pt1_ = GisLocal.CartRotate(pt0_);
		pt0.x = pt1_.x;
		pt0.y = pt1_.y;
		LineR.Line.push_back(pt0);
	}
	LineL.ContLinePoint = LineL.Line.size();
	LineR.ContLinePoint = LineR.Line.size();
	PathT.ContPathPoint = PathT.Path.size();
	PathT.IndLeftLine = 1;
	PathT.IndRightLine = 2;
	out.Paths.push_back(PathT);
	out.ContPath = out.Paths.size();
	LineL.LineId = 1;
	out.Lines.push_back(LineL);
	LineR.LineId = 2;
	out.Lines.push_back(LineR);
	out.ContLine = out.Lines.size();

	return out;
}

int CVirtualLane::GetCurInd()
{
	return m_nNearInd;
}

void CVirtualLane::ReadTrackLog(char* pszPath)
{
	VecGpsOri.clear();
	VecGps.clear();
	VecLineLeft.clear();
	VecLineRight.clear();

	vector<LCM_GPS_DATA> Imu0;
	std::ifstream fCloudPt;
	fCloudPt.open(pszPath);
	char szLine[1024] = {0};
	while(fCloudPt.getline(szLine,1024))
	{
		LCM_GPS_DATA gpsT;
		ParseImuData(szLine,&gpsT);
		Imu0.push_back(gpsT);
	}

	VecGpsOri = Imu0;

	vector<LCM_GPS_DATA> Imu1;
	FittingCurvesWithBSplineOrder2(Imu0,Imu1);

	vector<LCM_GPS_DATA> Imu2;
	double dMileStone = 0;
	Imu2.push_back(Imu1[0]);
	for (int i = 1; i < Imu1.size(); i++)
	{
		double dDist = GetDistance(Imu1[i-1].GPS_LATITUDE, Imu1[i].GPS_LATITUDE,
			Imu1[i-1].GPS_LONGITUDE, Imu1[i].GPS_LONGITUDE);
		dMileStone+=dDist;
		if (dMileStone>=1.0)
		{
			Imu2.push_back(Imu1[i]);
			dMileStone = 0.0;
		}
	}

	long nNearInd = 0;
	double dNearDist = 0;
	//for (int i = (int)Imu2.size() - 2; i >= 0; i--)
	//{
	//	double dDist0 = GetDistance(Imu2[0].GPS_LATITUDE, Imu2[i].GPS_LATITUDE, Imu2[0].GPS_LONGITUDE, Imu2[i].GPS_LONGITUDE);
	//	double dDist1 = GetDistance(Imu2[0].GPS_LATITUDE, Imu2[i+1].GPS_LATITUDE, Imu2[0].GPS_LONGITUDE, Imu2[i+1].GPS_LONGITUDE);
	//	if (dDist1 < dDist0)
	//	{
	//		nNearInd = i;
	//		dNearDist = dDist1;
	//		break;
	//	}
	//}
	nNearInd = Imu2.size() - 1;
	for (int i = 0; i <= nNearInd; i++)
	{
		VecGps.push_back(Imu2[i]);
	}

	printf("Track point count:%d, nearest dist:%.2fm\n", nNearInd, dNearDist);

	m_pGisOprMap->SetBaseLocatin(VecGps[0].GPS_LATITUDE, VecGps[0].GPS_LONGITUDE, VecGps[0].GPS_HEADING);
	for (int i = 0 ; i < VecGps.size(); i++)
	{
		Point2d pt0(VecGps[i].GPS_LATITUDE, VecGps[i].GPS_LONGITUDE);
		Point2d pt1 = m_pGisOprMap->Wgs2Cart(pt0);
		VecGps[i].GPS_LATITUDE = pt1.x;
		VecGps[i].GPS_LONGITUDE = pt1.y;
	}

	for (int i = 0; i < VecGps.size(); i++)
	{
		long nInd0 = i;
		long nInd1 = i + 1;
		if (nInd1 >= VecGps.size())
			nInd1 = 0;
		Point2d pt0(VecGps[nInd0].GPS_LATITUDE, VecGps[nInd0].GPS_LONGITUDE);
		Point2d pt1(VecGps[nInd1].GPS_LATITUDE, VecGps[nInd1].GPS_LONGITUDE);
		Point2d pt0_1 = pt1 - pt0;
		Point2d ptT;
		double Agl = 90.0/180.0*3.141592654;
		ptT.x = pt0_1.x * cos(Agl) - pt0_1.y * sin(Agl);
		ptT.y = pt0_1.x * sin(Agl) + pt0_1.y * cos(Agl);
		double dLen = sqrt(pow(ptT.x, 2) + pow(ptT.y, 2));
		Point2d ptT_ = ptT * (1/dLen) * m_dDistLine2Path;
		Point2d ptL = pt0 + ptT_;
		Point2d ptR = pt0 - ptT_;
		LCM_GPS_DATA GpsT;
		GpsT = VecGps[i];
		GpsT.GPS_LATITUDE = ptL.x;
		GpsT.GPS_LONGITUDE = ptL.y;
		VecLineLeft.push_back(GpsT);
		GpsT.GPS_LATITUDE = ptR.x;
		GpsT.GPS_LONGITUDE = ptR.y;
		VecLineRight.push_back(GpsT);
	}

}

long CVirtualLane::ParseImuData(char* pBuffer, LCM_GPS_DATA* pImu)
{
	char* pszLocal = strchr(pBuffer,',') + 1;

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_TIME = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_HEADING = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_PITCH = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_ROLL = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_LATITUDE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_LONGITUDE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_ALTITUDE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_VE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_VN = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_VU = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_BASELINE = atof(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_NSV1 = atoi(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_NSV2 = atoi(pszLocal);

	pszLocal = strchr(pszLocal,',') + 1;
	pImu->GPS_STATE = atoi(pszLocal);

	return 0;
}

long CVirtualLane::FittingCurvesWithBSplineOrder2(vector<LCM_GPS_DATA>& Track_in, vector<LCM_GPS_DATA>& Track_out)
{
	int seg_num = 3;
	if (seg_num < 1) 
	{
		return 0;
	}

	vector<Point2d> m_trajectorySmooth;
	vector<Point2d> m_trajectorySeg;
	for (int i = 0; i < Track_in.size(); i++)
	{
		Point2d ptT;
		ptT.x = Track_in[i].GPS_LONGITUDE;
		ptT.y = Track_in[i].GPS_LATITUDE;
		m_trajectorySeg.push_back(ptT);
	}

	m_trajectorySmooth.clear();
	int count = m_trajectorySeg.size();
	if (count < 3) {
		for (int i = 0; i < count; ++i) {
			m_trajectorySmooth.push_back(m_trajectorySeg[i]);
		}
		return 1;
	}
	// 移动到第一个点
	// 边界处理
	Point2d p_start;
	Point2d p_end;
	p_start.x = m_trajectorySeg[0].x - (m_trajectorySeg[1].x - m_trajectorySeg[0].x);
	p_start.y = m_trajectorySeg[0].y - (m_trajectorySeg[1].y - m_trajectorySeg[0].y);
	p_end.x = m_trajectorySeg[count - 1].x - (m_trajectorySeg[count - 2].x - m_trajectorySeg[count - 1].x);
	p_end.y = m_trajectorySeg[count - 1].y - (m_trajectorySeg[count - 2].y - m_trajectorySeg[count - 1].y);

	double a0 = 0.5 * (p_start.x + m_trajectorySeg[1].x);
	double a1 = m_trajectorySeg[1].x - p_start.x;
	double a2 = 0.5 * (p_start.x - 2.0 * m_trajectorySeg[1].x + m_trajectorySeg[2].x);
	double b0 = 0.5 * (p_start.y + m_trajectorySeg[1].y);
	double b1 = m_trajectorySeg[1].y - p_start.y;
	double b2 = 0.5 * (p_start.y - 2.0 * m_trajectorySeg[1].y + m_trajectorySeg[2].y);

	Point2d out;
	int out_count = 0;
	double t_delta = 1.0 / (double)seg_num;
	double t_loop = 0.0;
	for (int i = 0; i < seg_num; ++i) {
		out.x = a0 + a1 * t_loop + a2 * t_loop * t_loop;
		out.y = b0 + b1 * t_loop + b2 * t_loop * t_loop;

		t_loop += t_delta;
		out_count++;
		m_trajectorySmooth.push_back(out);
	}
	for (int i = 1; i < count-3; ++i) {
		// 计算第K段分量式参数方程的系数
		a0 = 0.5 * (m_trajectorySeg[i].x + m_trajectorySeg[i+1].x);
		a1 = m_trajectorySeg[i+1].x - m_trajectorySeg[i].x;
		a2 = 0.5 * (m_trajectorySeg[i].x - 2.0 * m_trajectorySeg[i+1].x + m_trajectorySeg[i+2].x);
		b0 = 0.5 * (m_trajectorySeg[i].y + m_trajectorySeg[i+1].y);
		b1 = m_trajectorySeg[i+1].y - m_trajectorySeg[i].y;
		b2 = 0.5 * (m_trajectorySeg[i].y - 2.0 * m_trajectorySeg[i+1].y + m_trajectorySeg[i+2].y);
		// 段内按步长画线
		t_loop = 0;
		for (int i = 0; i < seg_num; ++i) {
			out.x = a0 + a1 * t_loop + a2 * t_loop * t_loop;
			out.y = b0 + b1 * t_loop + b2 * t_loop * t_loop;

			t_loop += t_delta;
			out_count++;
			m_trajectorySmooth.push_back(out);
		}
	}

	if (count > 3) {
		a0 = 0.5 * (m_trajectorySeg[count - 3].x + m_trajectorySeg[count - 2].x);
		a1 = m_trajectorySeg[count - 2].x - m_trajectorySeg[count - 3].x;
		a2 = 0.5 * (m_trajectorySeg[count - 3].x - 2.0 * m_trajectorySeg[count - 2].x + p_end.x);
		b0 = 0.5 * (m_trajectorySeg[count - 3].y + m_trajectorySeg[count - 2].y);
		b1 = m_trajectorySeg[count - 2].y - m_trajectorySeg[count - 3].y;
		b2 = 0.5 * (m_trajectorySeg[count - 3].y - 2.0 * m_trajectorySeg[count - 2].y + p_end.y);

		t_loop = 0;
		for (int i = 0; i < seg_num; ++i) {
			out.x = a0 + a1 * t_loop + a2 * t_loop * t_loop;
			out.y = b0 + b1 * t_loop + b2 * t_loop * t_loop;

			t_loop += t_delta;
			out_count++;
			m_trajectorySmooth.push_back(out);
		}
	}

	out.x = m_trajectorySeg[count - 1].x;
	out.y = m_trajectorySeg[count - 1].y;
	m_trajectorySmooth.push_back(out);

	for (int i = 0; i < m_trajectorySmooth.size(); i++)
	{
		LCM_GPS_DATA gpsT;
		gpsT.GPS_LONGITUDE = m_trajectorySmooth[i].x;
		gpsT.GPS_LATITUDE = m_trajectorySmooth[i].y;
		Track_out.push_back(gpsT);
	}

	return 1;
}

double CVirtualLane::GetDistance(double dLat0, double dLat1, double dLon0, double dLon1 )
{
	double dZ = 111319.55*(dLat1 - dLat0);
	double dX = 111319.55*(dLon1 - dLon0)*cos((dLat0)/180.f*3.141592654);
	return (sqrt(pow(dZ,2)+pow(dX,2)));
}

CVirtualNavi::CVirtualNavi():
m_TrafficLightReqSend(string("LCM_TRAFIC_LIGHT_REQUIRE"))
{

}

CVirtualNavi::~CVirtualNavi()
{

}

void CVirtualNavi::Init(char* pszPath)
{
	m_MainLane.Init(pszPath,&m_GisOprMap);
}
long CVirtualNavi::AddObjects(char* pszPath)
{
	m_ObjectList.Init(pszPath, &m_GisOprMap);
	return 1;
}
long CVirtualNavi::AddBranch(char* pszPath, int nLaneSide)
{
	CVirtualLane Branch;
	Branch.Init(pszPath,&m_GisOprMap);

	int nNearInd = 0;
	double dNearDist = DBL_MAX;
	LCM_GPS_DATA RefPt = Branch.VecGps.front();
	for (int i = 0; i < m_MainLane.VecGps.size(); i++)
	{
		double dDist = sqrt(pow(RefPt.GPS_LATITUDE-m_MainLane.VecGps[i].GPS_LATITUDE,2)+pow(RefPt.GPS_LONGITUDE-m_MainLane.VecGps[i].GPS_LONGITUDE,2));
		if (dDist <= dNearDist)
		{
			dNearDist = dDist;
			nNearInd = i;
		}
	}
	Branch.nBranchStart = nNearInd;

	nNearInd = 0;
	dNearDist = DBL_MAX;
	RefPt = Branch.VecGps.back();
	for (int i = 0; i < m_MainLane.VecGps.size(); i++)
	{
		double dDist = sqrt(pow(RefPt.GPS_LATITUDE-m_MainLane.VecGps[i].GPS_LATITUDE,2)+pow(RefPt.GPS_LONGITUDE-m_MainLane.VecGps[i].GPS_LONGITUDE,2));
		if (dDist <= dNearDist)
		{
			dNearDist = dDist;
			nNearInd = i;
		}
	}
	Branch.nBranchEnd = nNearInd;

	if (Branch.nBranchStart >= Branch.nBranchEnd)
	{
		printf("Warning: Branch.nBranchStart >= Branch.nBranchEnd\n");
	}

	Branch.Flag = nLaneSide;

	m_Branches.push_back(Branch);

	return Branch.VecGps.size();
}

LCM_NAVI_TO_SENSE_INFO CVirtualNavi::TransData(LCM_NAVI_REQUIRE_INFO GpsRequir)
{
	LCM_NAVI_TO_SENSE_INFO Result = m_MainLane.TransData(GpsRequir);

	if (Result.Paths.size() == 0)
	{
		return Result;
	}

	m_ObjectList.TransData(GpsRequir, Result.Objs);
	Result.ContObj = Result.Objs.size();

	Result.Paths[0].PathId = 0;
	Result.Paths[0].PathType = 0;
	Result.Paths[0].IndLeftLine = 0;
	Result.Paths[0].IndRightLine = 1;

	for (int i = 0; i < m_Branches.size(); i++)
	{
		if (m_MainLane.GetCurInd() < m_Branches[i].nBranchStart ||
			m_MainLane.GetCurInd() > m_Branches[i].nBranchEnd)
		{
			continue;
		}
		LCM_NAVI_TO_SENSE_INFO ResultT = m_Branches[i].TransDataNotLoop(GpsRequir);
		if (ResultT.Paths.size() == 0)
		{
			continue;
		}
		ResultT.Paths[0].PathId = i+1;
		ResultT.Paths[0].PathType = m_Branches[i].Flag;
		ResultT.Paths[0].IndLeftLine = Result.Lines.size();
		ResultT.Paths[0].IndRightLine = Result.Lines.size()+1;
		Result.Paths.push_back(ResultT.Paths[0]);
		Result.Lines.push_back(ResultT.Lines[0]);
		Result.Lines.push_back(ResultT.Lines[1]);
	}

	Result.ContLine = Result.Lines.size();
	Result.ContPath = Result.Paths.size();

	LCM_NAVI_TO_SENSE_INFO ResultSend = Result;
	for (unsigned int i = 0; i < ResultSend.Paths.size(); i++)
	{
		for (unsigned int j = i+1 ; j < ResultSend.Paths.size(); j++)
		{
			if (ResultSend.Paths[j].PathType <= ResultSend.Paths[i].PathType)
			{
				LCM_NAVI_PATH PathT = ResultSend.Paths[j];
				ResultSend.Paths[j] = ResultSend.Paths[i];
				ResultSend.Paths[i] = PathT;
			}
		}
	}

	for (unsigned int i = 0; i <ResultSend.Paths.size(); i++)
	{
		ResultSend.Paths[i].PathId = i;
		if (ResultSend.Paths[i].PathType != 0)
		{
			ResultSend.Paths[i].PathType = 0;
		}
		else
		{
			ResultSend.Paths[i].PathType = 1;
		}
	}

	return ResultSend;
}

void CVirtualNavi::TrafficLightRequire(LCM_NAVI_TO_SENSE_INFO& NaviInfo)
{
	int nNearestStopLineInd = -1;
	double dDistMin = DBL_MAX;
	for (unsigned int i = 0; i < NaviInfo.Objs.size(); i++)
	{
		LCM_NAVI_OBJECT& ObjT = NaviInfo.Objs[i];
		if (ObjT.nMajorType != 7)
			continue;
// 		if (ObjT.points[0].y <= 0.0 || ObjT.points[0].y >= 100.0)
// 			continue;
		double dDist = sqrt(pow(ObjT.points[0].x,2)+pow(ObjT.points[0].y,2));
		if (dDist <= dDistMin)
		{
			dDistMin = dDist;
			nNearestStopLineInd = i;
		}
	}
	if (nNearestStopLineInd == -1)
	{
		printf("No stop line(traffic light) ahead!\n");
		return;
	}

	int nTrafficLightInd = -1;
	for (unsigned int i = 0; i < NaviInfo.Objs.size(); i++)
	{
		LCM_NAVI_OBJECT& ObjT = NaviInfo.Objs[i];
		if (ObjT.nValue == NaviInfo.Objs[nNearestStopLineInd].nMinorType)
		{
			nTrafficLightInd = i;
			break;
		}
	}
	if (nTrafficLightInd == -1)
	{
		printf("No traffic light matched!\n");
		return;
	}

	LCM_TRAFIC_LIGHT_REQUIRE Req;
	Req.MsgInd = NaviInfo.MsgInd;
	Req.TraficLightId = NaviInfo.Objs[nTrafficLightInd].nMinorType;
	Req.Type = 0;
	Req.TraficLightLocate = NaviInfo.Objs[nTrafficLightInd].points[0];
	Req.StopLineLocate = NaviInfo.Objs[nNearestStopLineInd].points[0];
	m_TrafficLightReqSend.Send("LCM_TRAFIC_LIGHT_REQUIRE", Req);
	printf("Traffic Light:No:%d (%.2f,%.2f,%.2f), Stop Line:(%.2f,%.2f,%.2f)\n",
		Req.TraficLightId,
		Req.TraficLightLocate.x,
		Req.TraficLightLocate.y,
		Req.TraficLightLocate.z,
		Req.StopLineLocate.x,
		Req.StopLineLocate.y,
		Req.StopLineLocate.z);
}

CVirtualObjectList::CVirtualObjectList()
{

}
CVirtualObjectList::~CVirtualObjectList()
{

}

long CVirtualObjectList::Init(char* pszPath, CGisTransform* pGisOprMap)
{
	m_pGisOprMap = pGisOprMap;
	FileStorage fs(pszPath, CV_STORAGE_READ);
	if (!fs.isOpened())
	{
		printf("Navi Objectlist file open failed!\n");
		return 0;
	}
	fs["NaviObjectList"] >> m_ObjList;
	fs.release();
	cout << "NaviObjectList:" << endl << m_ObjList << endl;

	vector<Vec<double,7> > VecItem;
	for (int i = 0; i < m_ObjList.rows; i++)
	{
		Vec<double,7> ItemT;
		ItemT.val[0] = m_ObjList.at<double>(i,0);
		Point2d pt = m_pGisOprMap->Wgs2Cart(Point2d(m_ObjList.at<double>(i,1),m_ObjList.at<double>(i,2)));
		ItemT.val[1] = pt.x;
		ItemT.val[2] = pt.y;
		ItemT.val[3] = m_ObjList.at<double>(i,3);
		ItemT.val[4] = m_ObjList.at<double>(i,4);
		ItemT.val[5] = m_ObjList.at<double>(i,5);
		ItemT.val[6] = m_ObjList.at<double>(i,6);
		VecItem.push_back(ItemT);
	}
	Mat matT(m_ObjList.rows, m_ObjList.cols, CV_64F, VecItem.data());
	matT.copyTo(m_ObjListTrans);

	return 1;
}

int CVirtualObjectList::TransData(LCM_NAVI_REQUIRE_INFO GpsRequir, vector<LCM_NAVI_OBJECT>& Objs)
{
	Objs.clear();
	CGisTransform Trans;
	Trans.SetBaseLocatin(GpsRequir.Gps.GPS_LATITUDE, GpsRequir.Gps.GPS_LONGITUDE, GpsRequir.Gps.GPS_HEADING);
	for (int i = 0; i < m_ObjList.rows; i++)
	{
		LCM_NAVI_OBJECT ObjT;
		Point2d ptIn(m_ObjList.at<double>(i,1), m_ObjList.at<double>(i,2));
		Point2d ptOut = Trans.Wgs2CartRotate(ptIn);
		LCM_POINT3D_F ptObj;
		ptObj.x = ptOut.x;
		ptObj.y = ptOut.y;
		ptObj.z = m_ObjList.at<double>(i,3);
		ObjT.points.push_back(ptObj);
		ObjT.nPtCont = 1;
		ObjT.nValue = m_ObjList.at<double>(i,5);
		ObjT.nPlaceType = 0;
		ObjT.fOffSet = 0;
		ObjT.nMajorType = m_ObjList.at<double>(i,4);
		ObjT.nMinorType = m_ObjList.at<double>(i,0);
		if (ObjT.nMajorType == 6)
		{
			ObjT.nPlaceType = 2;
		}
		double dDist = sqrt(pow(ObjT.points[0].x, 2) + pow(ObjT.points[0].y, 2));
		if (dDist <= 200.0 && ObjT.points[0].y >= -30.0)
		{
			Objs.push_back(ObjT);
		}
	}

	return Objs.size();
}