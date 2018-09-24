#include <time.h>
#include "GpsManager.h"
#include "LcmReceiver.h"
#include "LCM_IBEO_CLOUD_POINTS.hpp"
#include "ProjectOpr.h"
#include <fstream>
#include "OpenCVInc.h"

long g_nDataSource = 0;		//Data source: 0:Sensor 1:Log Data
bool g_bWriteLog = false;	//Is write log data:
char g_szPath[MAX_PATH] = {0};	//Path for log data and track data
long g_nFrameIndex = 0;
CLidarDataOpr LidarDataOpr("");
vector<LCM_GPS_DATA> g_VecGps;


#define MAX_TRACK_DISTANCE	10.0
long g_nNearIndStart = 0;
long g_nNearIndEnd = 0;
long g_nNearInd = 0;
double g_dNearDist = DBL_MAX;

std::string szChannleName = "LIDAR_CLOUD_POINTS_WITH_GPS";
CLcmRevicer<LCM_IBEO_CLOUD_POINTS> LcmLidarCloudPointsWithGps(szChannleName);

int mainFromLog();
int mainFromSensor();
long ReadTrackLog(char* pszPath, vector<LCM_GPS_DATA>* pImu);
long ParseImuData(char* pBuffer, LCM_GPS_DATA* pImu);
long GetTrack(LCM_GPS_DATA* pImuInfo, vector<LCM_GPS_DATA>* pImu ,LCM_IBEO_CLOUD_POINTS* pTrack);
double GetDistance(double dLat0, double dLat1, double dLon0, double dLon1 );
long FittingCurvesWithBSplineOrder2(vector<LCM_GPS_DATA>& Track_in, vector<LCM_GPS_DATA>& Track_out);

void WINAPI CloudPointsCallBack(void* pData,void* pUser)
{
	LCM_IBEO_CLOUD_POINTS* pLidarPoints = (LCM_IBEO_CLOUD_POINTS*)pData;
	CGpsManager* pGpsMgr = (CGpsManager*)pUser;
	LCM_GPS_DATA ImuInfo;
	pGpsMgr->GetData(ImuInfo);
	if (g_bWriteLog)
	{
		LidarDataOpr.WriteLog(pLidarPoints,&ImuInfo);
	}

	LCM_IBEO_CLOUD_POINTS LidarPointsWithGps;
	LidarPointsWithGps = *pLidarPoints;
	LidarPointsWithGps.GpsData = ImuInfo;

	GetTrack(&ImuInfo,&g_VecGps,&LidarPointsWithGps);

	LcmLidarCloudPointsWithGps.Send(szChannleName,LidarPointsWithGps);

	printf("%d\n",LidarPointsWithGps.FrameInd);

	return;
}

int main(int argc, char* argv[])
{
	if (argc != 4)
	{
		printf("3 Params need!\n");
		printf("[Data source] [Is write log] [Path for log]\n");
		printf("[Data source]: 0: data from sensor  1:data from log\n");
		printf("[Is write log]: 0:don't  1:do , this not used when Data source is 1\n");
		printf("[Path for log]: path for log data and track\n");
		getchar();
	}
	g_nDataSource = atoi(argv[1]);
	g_bWriteLog = atoi(argv[2]);
	strcpy(g_szPath,argv[3]);
	if (g_nDataSource == 0)			//从传感器获取数据
	{
		mainFromSensor();
	}
	else						//从log获取数据
	{
		mainFromLog();
	}

	return 1;
}

int mainFromSensor()
{
	if (g_bWriteLog)
	{
		LidarDataOpr.CreateDir();
	}
	//GPS Triger
	char szPath[MAX_PATH] = {0};
	strcpy(szPath,g_szPath);
	char szPathTrace[MAX_PATH] = {0};
	strcpy(szPathTrace,szPath);
	strcat(szPathTrace,"Track.txt");
	ReadTrackLog(szPathTrace,&g_VecGps);
	printf("Track data size: %d\n",g_VecGps.size());

	CGpsManager GpsMgr;//(CGpsManager::GPS_MANAGER_TYPE_IMU);
	GpsMgr.Init(CGpsManager::GPS_MANAGER_TYPE_IMU);
	GpsMgr.Start();

	CLcmRevicer<LCM_IBEO_CLOUD_POINTS> LcmLidarPoints(string("LIDAR_CLOUD_POINTS"));
	LcmLidarPoints.SetCallBack(CloudPointsCallBack,&GpsMgr);
	LcmLidarPoints.Start();

	Sleep(INFINITE);
	return 0;
}

int mainFromLog()
{
	char szPath[MAX_PATH] = {0};
	strcpy(szPath,g_szPath);
	CLidarDataOpr Lidar(szPath);
	LCM_IBEO_CLOUD_POINTS Cpt;
	LCM_GPS_DATA Imu;

	char szPathTrace[MAX_PATH] = {0};
	strcpy(szPathTrace,szPath);
	strcat(szPathTrace,"Track.txt");
	ReadTrackLog(szPathTrace,&g_VecGps);
	printf("Track data size: %d\n",g_VecGps.size());

	int nStartInd = 0;
	std::cin >> nStartInd;
	Lidar.SetInd(nStartInd);
	while(1)
	{
		Sleep(60);
//		getchar();
		long nnn = Lidar.ReadLog(&Cpt,&Imu);
		if (nnn == -1)
		{
			//g_nNearIndStart = 0;
			//g_nNearIndEnd = 0;
			Lidar.SetInd(nStartInd);
		}
		printf("%d\n",nnn);

		LCM_IBEO_CLOUD_POINTS LidarCloudGps;
		LidarCloudGps = Cpt;
		LidarCloudGps.GpsData = Imu;

		GetTrack(&Imu,&g_VecGps,&LidarCloudGps);

		LcmLidarCloudPointsWithGps.Send(szChannleName,LidarCloudGps);
	}
}


long ReadTrackLog(char* pszPath, vector<LCM_GPS_DATA>* pImu)
{
	pImu->clear();
	vector<LCM_GPS_DATA> Imu0;
	std::ifstream g_fCloudPt;
	g_fCloudPt.open(pszPath);
	char szLine[1024] = {0};
	while(g_fCloudPt.getline(szLine,1024))
	{
		LCM_GPS_DATA gpsT;
		ParseImuData(szLine,&gpsT);
		Imu0.push_back(gpsT);
	}

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
	for (int i = Imu2.size() - 2; i >= 0; i--)
	{
		double dDist0 = GetDistance(Imu2[0].GPS_LATITUDE, Imu2[i].GPS_LATITUDE, Imu2[0].GPS_LONGITUDE, Imu2[i].GPS_LONGITUDE);
		double dDist1 = GetDistance(Imu2[0].GPS_LATITUDE, Imu2[i+1].GPS_LATITUDE, Imu2[0].GPS_LONGITUDE, Imu2[i+1].GPS_LONGITUDE);
		if (dDist1 < dDist0)
		{
			nNearInd = i;
			dNearDist = dDist1;
		}
	}
	pImu->clear();
	for (int i = 0; i <= nNearInd; i++)
	{
		//LCM_GPS_DATA GpsDataT;
		//GpsDataT = Imu2[i];
		//GpsDataT.GPS_LATITUDE = 111319.55*(Imu2[i].GPS_LATITUDE - Imu2[0].GPS_LATITUDE);
		//GpsDataT.GPS_LONGITUDE = 111319.55*(Imu2[i].GPS_LONGITUDE - Imu2[0].GPS_LONGITUDE)*cos((Imu2[0].GPS_LATITUDE)/180.f*3.141592654);
		pImu->push_back(Imu2[i]);
	}

	printf("Track point count:%d, nearest dist:%.2fm\n", nNearInd, dNearDist);

	return pImu->size();
}

long ParseImuData(char* pBuffer, LCM_GPS_DATA* pImu)
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

long GetTrack(LCM_GPS_DATA* pImuInfo, vector<LCM_GPS_DATA>* pImu ,LCM_IBEO_CLOUD_POINTS* pTrack)
{
	//calc dist of next 100 track points to the vehicle, record the first knee point(rising edge);
	for (int i = 0; i < 100; i++)
	{
		long nIndT0 = g_nNearInd + i;
		while (nIndT0 >= pImu->size())
			nIndT0 -= pImu->size();
		long nIndT1 = g_nNearInd + i + 1;
		while (nIndT1 >= pImu->size())
			nIndT1 -= pImu->size();
//		double dDist0 = sqrt(pow((pImuInfo->GPS_LATITUDE - pImu->at(nIndT0).GPS_LATITUDE),2) + pow((pImuInfo->GPS_LONGITUDE - pImu->at(nIndT0).GPS_LONGITUDE),2));
//		double dDist1 = sqrt(pow((pImuInfo->GPS_LATITUDE - pImu->at(nIndT1).GPS_LATITUDE),2) + pow((pImuInfo->GPS_LONGITUDE - pImu->at(nIndT1).GPS_LONGITUDE),2));
		double dDist0 = GetDistance(pImuInfo->GPS_LATITUDE, pImu->at(nIndT0).GPS_LATITUDE, pImuInfo->GPS_LONGITUDE, pImu->at(nIndT0).GPS_LONGITUDE);
		double dDist1 = GetDistance(pImuInfo->GPS_LATITUDE, pImu->at(nIndT1).GPS_LATITUDE, pImuInfo->GPS_LONGITUDE, pImu->at(nIndT1).GPS_LONGITUDE);
		if (dDist0 < dDist1 && dDist0 <= g_dNearDist)
		{
			g_nNearInd = nIndT0;
			g_dNearDist = dDist0;
			break;
		}
	}

	//if the distance of knee point to the vehicle is great then MAX_TRACK_DISTANCE, 
	//continue search for another nearest track point in all track points
	//meanwhile its dist must be less then MAX_TRACK_DISTANCE
	if (g_dNearDist > MAX_TRACK_DISTANCE)
	{
		for (int i = 0; i < pImu->size(); i++)
		{
			long nIndT = g_nNearInd + i;
			while (nIndT >= pImu->size())
				nIndT -= pImu->size();
//			double dDist = sqrt(pow((pImuInfo->GPS_LATITUDE - pImu->at(nIndT).GPS_LATITUDE),2) + pow((pImuInfo->GPS_LONGITUDE - pImu->at(nIndT).GPS_LONGITUDE),2));
		double dDist = GetDistance(pImuInfo->GPS_LATITUDE, pImu->at(nIndT).GPS_LATITUDE, pImuInfo->GPS_LONGITUDE, pImu->at(nIndT).GPS_LONGITUDE);
		if (dDist <= MAX_TRACK_DISTANCE)
			{
				g_nNearInd = nIndT;
				g_dNearDist = dDist;
				break;
			}
		}
		//if no track point fit, no track data output
		if (g_dNearDist > MAX_TRACK_DISTANCE)
		{
			pTrack->TrackPoints.clear();
			pTrack->ContTrack = 0;
			return 0;
		}
	}

	double dAcc = 0.0;
	for (int i = 0; i < 100; i++)
	{
		long nIndT0 = g_nNearInd - i;
		while (nIndT0 < 0)
			nIndT0 += pImu->size();
		long nIndT1 = g_nNearInd - i - 1;
		while (nIndT1 < 0)
			nIndT1 += pImu->size();
		double dDist = GetDistance(pImu->at(nIndT0).GPS_LATITUDE, pImu->at(nIndT1).GPS_LATITUDE, pImu->at(nIndT0).GPS_LONGITUDE, pImu->at(nIndT1).GPS_LONGITUDE);
//		double dDist = sqrt(pow((pImu->at(nIndT0).GPS_LATITUDE - pImu->at(nIndT1).GPS_LATITUDE),2) + pow((pImu->at(nIndT0).GPS_LONGITUDE - pImu->at(nIndT1).GPS_LONGITUDE),2));
		dAcc += dDist;
		if (dAcc >= 20.0)
		{
			g_nNearIndStart = -1*i;
			break;
		}
	}

	dAcc = 0.0;
	for (int i = 0; i < 100; i++)
	{
		long nIndT0 = g_nNearInd + i;
		while (nIndT0 >= pImu->size())
			nIndT0 -= pImu->size();
		long nIndT1 = g_nNearInd + i + 1;
		while (nIndT1 >= pImu->size())
			nIndT1 -= pImu->size();
		double dDist = GetDistance(pImu->at(nIndT0).GPS_LATITUDE, pImu->at(nIndT1).GPS_LATITUDE, pImu->at(nIndT0).GPS_LONGITUDE, pImu->at(nIndT1).GPS_LONGITUDE);
//		double dDist = sqrt(pow((pImu->at(nIndT0).GPS_LATITUDE - pImu->at(nIndT1).GPS_LATITUDE),2) + pow((pImu->at(nIndT0).GPS_LONGITUDE - pImu->at(nIndT1).GPS_LONGITUDE),2));
		dAcc += dDist;
		if (dAcc >= 50.0)
		{
			g_nNearIndEnd = i;
			break;
		}
	}

	pTrack->TrackPoints.clear();
	for (int i = g_nNearIndStart; i <= g_nNearIndEnd; i++)
	{
		long nIndT = g_nNearInd + i;
		while (nIndT < 0)
			nIndT += pImu->size();
		while (nIndT >= pImu->size())
			nIndT -= pImu->size();

		LCM_POINT2D_F pt0;
		pt0.y = 111319.55*(pImu->at(nIndT).GPS_LATITUDE - pImuInfo->GPS_LATITUDE);
		pt0.x = 111319.55*(pImu->at(nIndT).GPS_LONGITUDE - pImuInfo->GPS_LONGITUDE)*cos((pImuInfo->GPS_LATITUDE)/180.f*3.141592654);
		double dAngle = pImuInfo->GPS_HEADING/180.f*3.141592654;
		pt0.x = cos(dAngle)*pt0.x - sin(dAngle)*pt0.y;
		pt0.y = sin(dAngle)*pt0.x + cos(dAngle)*pt0.y;
		pTrack->TrackPoints.push_back(pt0);
	}
	pTrack->ContTrack = pTrack->TrackPoints.size();

	return 0;
}

double GetDistance(double dLat0, double dLat1, double dLon0, double dLon1 )
{
	double dZ = 111319.55*(dLat1 - dLat0);
	double dX = 111319.55*(dLon1 - dLon0)*cos((dLat0)/180.f*3.141592654);
	return (sqrt(pow(dZ,2)+pow(dX,2)));
}

long FittingCurvesWithBSplineOrder2(vector<LCM_GPS_DATA>& Track_in, vector<LCM_GPS_DATA>& Track_out)
{
	int seg_num = 3;
	if (seg_num < 1) {
		return (false);
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
		return (true);
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

	return (true);
}
