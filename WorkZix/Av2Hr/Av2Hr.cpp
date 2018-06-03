#include "IDataStore.h"
#include "IMessageCodec.h"
#include "IMessageObserver.h"
#include "IMessageProvider.h"
#include "adm_kotei_api.h"
#include "LibGuiOpenGL.h"
#include "GisTransform.h"
#include "ADM_VEHICLE_STATE.hpp"
#include "Av2Hr2Sensor.h"


#if _DEBUG
#pragma comment(lib,"LibGuiOpenGLD.lib")
#else
#pragma comment(lib,"LibGuiOpenGL.lib")
#endif

LIBGUIOPENGL_HANDLE g_Opengl = GetLibGuiOpenGL();

long Wgs2Cart4Av2HR(Av2HR_Package_Data& Pack_in, Av2HR_Package_Data& Pack_out);
int SampleForADM_KOTEI_API();
int DrawOpengl(Av2HR_Package_Data& AvData);
Av2HR_ADM_POINT TransPoint(CGisTransform& GisOpr, Av2HR_ADM_POINT& pt);

Av2HR_Package_Data g_AvData;

CAv2Hr2Sensor g_Trans;

int main(int argc, char* argv[])
{
	g_Trans.Start();
	SampleForADM_KOTEI_API();

	return 0;
}

int SampleForADM_KOTEI_API()
{
	g_Opengl->SetCameraMode(1);
	g_Opengl->SetScale(0.001);
	Mat RefPoint = Mat::eye(4,4,CV_64F);
	g_Opengl->AddRefPoints(RefPoint);
	//初始化ADM模块
	if (0 != ADM_KOTEI_API_Initialize())
	{
		return -1;
	}

	int nRt = 0;
	long nCont = 0;
	while(1)
	{
		Av2HR_Package_Data AvData;

		Sleep(1000);
		if (Av2HR_getLaneSectionList(&AvData.LaneSectionList) < 0)
		{
			continue;
		}
		if (Av2HR_getPOIList(&AvData.PoiList) < 0)
		{
			printf("Av2HR_getPOIList error!\n");
			continue;
		}
		if (Av2HR_getLaneLineProfile(&AvData.ProfileString) < 0)
		{
			printf("Av2HR_getLaneLineProfile error!\n");
			continue;
		}
		if (Av2HR_getVehicleState(&AvData.VS) < 0)
		{
			printf("Av2HR_getVehicleState error!\n");
			continue;
		}

		nCont++;

// 		double dDistDelta = GisWgsPoint2PointDist(
// 			Point2d(g_AvData.VS.m_iLatitude * 0.0000001, g_AvData.VS.m_iLongitude * 0.0000001),
// 			Point2d(AvData.VS.m_iLatitude * 0.0000001, AvData.VS.m_iLongitude * 0.0000001));
// 		if (dDistDelta <= 30.0)
// 		{
// 			continue;
// 		}
		
// 		if (nCont == 1)
// 		{
// 			g_Trans.UpdataNaviData(AvData);
// 		}
// 		else
// 		{
// 			g_Trans.UpdataNaviData(g_AvData);
// 		}
		g_Trans.UpdataNaviData(g_AvData);
//		printf("Local navi data updata!\n");

		g_AvData = AvData;

		g_AvData.VS.m_iLatitude += g_Trans.m_Param.dLatitudeOffSet*10000000;
		g_AvData.VS.m_iLongitude += g_Trans.m_Param.dLongitudeOffSet*10000000;
		Av2HR_Package_Data AvDataT;
		Wgs2Cart4Av2HR(g_AvData, AvDataT);

		DrawOpengl(AvDataT);

// 		if (nCont >= 2)
// 		{
// 			getchar();
// 		}
	}

	

	return 1;
}

int DrawOpengl(Av2HR_Package_Data& AvData)
{
	g_Opengl->ClearLines();
	g_Opengl->ClearBreakLines();
	g_Opengl->ClearSphere();
//	std::cout << "LaneSectionList" << AvData.LaneSectionList.m_iPathIndex << std::endl;
//	std::cout << "ProfileString" << AvData.ProfileString.m_iPathIndex << std::endl;
	for (int i = 0; i < AvData.ProfileString.m_iLaneLineProfileNum; i++)
	{
		Av2HR_ADM_LANE_LINE_PROFILE& ProFileT = AvData.ProfileString.m_laneLineProfileString[i];
		for (int j = 0; j < ProFileT.m_iLaneLineNum; j++)
		{
			Av2HR_ADM_OUTLINE& OutLine = ProFileT.m_outlineString[j];
			vector<Vec3d> VecLine;
			for (int k = 0; k < OutLine.m_usPointNum; k++)
			{
				Av2HR_ADM_POINT& PtT = OutLine.m_pointString[k];
				Vec3d pt(PtT.m_ulLatitude,PtT.m_ulLongitude,0);
				VecLine.push_back(pt);
			}
			Mat Lines(VecLine.size(),3,CV_64F,VecLine.data());
			if (OutLine.m_iOutlineType == 1)
			{
				g_Opengl->AddBreakLinesRelative(Lines.clone());
			}
			else
			{
				g_Opengl->AddLinesRelative(Lines.clone());
			}
		}
	}

	for (int i = 0; i < AvData.LaneSectionList.m_iLaneSecionNum; i++)
	{
		Av2HR_ADM_LANE_SECTION& SectionT = AvData.LaneSectionList.m_laneSectionList[i];
		for (int j = 0; j < SectionT.m_iLaneNum; j++)
		{
			Mat color;
			Av2HR_ADM_LANE& LaneT = SectionT.m_laneList[j];
			if (LaneT.m_bIsCalculatedRoute == 0)
			{
				color = (Mat_<uchar>(1,3)<<0,255,0);
			}
			else
			{
				color = (Mat_<uchar>(1,3)<<0,0,255);
			}
			vector<Vec3d> VecLine;
			for (int k = 0; k < LaneT.m_iWaypointNum; k++)
			{
				Av2HR_ADM_WAYPOINT& WayPoint = LaneT.m_waypointList[k];
				Vec3d pt(WayPoint.m_crd.m_ulLatitude,WayPoint.m_crd.m_ulLongitude,0);
				VecLine.push_back(pt);
			}
			Mat Lines(VecLine.size(),3,CV_64F,VecLine.data());
			//g_Opengl->AddBreakLinesRelative(Lines.clone(),Mat(),(Mat_<uchar>(1,3)<<0,255,0));
			g_Opengl->AddLinesRelative(Lines.clone(),Mat(),color.clone());
		}
	}
	//红绿灯
	for (int i = 0; i < AvData.PoiList.m_poiList.size(); i++)
	{
		Av2HR_ADM_POI& poi = AvData.PoiList.m_poiList[i];
		if (poi.m_iMajorType == 6)
		{
			float sz = 1000.0;
			Point3d pt(poi.m_crd.m_ulLatitude, poi.m_crd.m_ulLongitude, poi.m_iValue*10.0);	
			if (poi.m_iMinorType == 1)
			{
				g_Opengl->AddSphereRelative(pt, sz, Mat(), (Mat_<uchar>(1,3)<<255,0,0));
			}
			else
			{
				g_Opengl->AddSphereRelative(pt, sz, Mat(), (Mat_<uchar>(1,3)<<255,255,0));
			}
		}
		if (poi.m_iMajorType == 7 && poi.m_iPlaceType == 2 && i < AvData.PoiList.m_poiList.size()-1)
		{
			Av2HR_ADM_POI& poi_ = AvData.PoiList.m_poiList[i+1];
			if (poi_.m_iMajorType == 7 && poi_.m_iPlaceType == 3)
			{
				Mat Line = Mat::zeros(2,3,CV_64F);
				Line.at<double>(0,0) = poi.m_crd.m_ulLatitude;
				Line.at<double>(0,1) = poi.m_crd.m_ulLongitude;
				Line.at<double>(0,2) = poi.m_crd.m_ulAltitude;
				Line.at<double>(1,0) = poi_.m_crd.m_ulLatitude;
				Line.at<double>(1,1) = poi_.m_crd.m_ulLongitude;
				Line.at<double>(1,2) = poi_.m_crd.m_ulAltitude;
				g_Opengl->AddLinesRelative(Line, Mat(), (Mat_<uchar>(1,3)<<255,0,0));
			}
		}
		
	}
	//停止线
	for (int i = 0; i < AvData.PoiList.m_poiList.size(); i++)
	{
		Av2HR_ADM_POI& poi = AvData.PoiList.m_poiList[i];
		if (poi.m_iMajorType == 8 && poi.m_iPlaceType == 2 && i < AvData.PoiList.m_poiList.size()-1)
		{
			Av2HR_ADM_POI& poi_ = AvData.PoiList.m_poiList[i+1];
			if (poi_.m_iMajorType == 8 && poi_.m_iPlaceType == 3)
			{
				Mat Line = Mat::zeros(2,3,CV_64F);
				Line.at<double>(0,0) = poi.m_crd.m_ulLatitude;
				Line.at<double>(0,1) = poi.m_crd.m_ulLongitude;
				Line.at<double>(0,2) = poi.m_crd.m_ulAltitude;
				Line.at<double>(1,0) = poi_.m_crd.m_ulLatitude;
				Line.at<double>(1,1) = poi_.m_crd.m_ulLongitude;
				Line.at<double>(1,2) = poi_.m_crd.m_ulAltitude;
				g_Opengl->AddLinesRelative(Line, Mat(), (Mat_<uchar>(1,3)<<255,255,0));
			}
		}
		
	}
	//路口
	for (int i = 0; i < AvData.PoiList.m_poiList.size(); i++)
	{
		Av2HR_ADM_POI& poi = AvData.PoiList.m_poiList[i];
		if (poi.m_iMajorType == 5 && poi.m_iMinorType == 11 && poi.m_iPlaceType == 2 && i < AvData.PoiList.m_poiList.size()-1)
		{
			for (int j = i+1; j < AvData.PoiList.m_poiList.size(); j++)
			{
				Av2HR_ADM_POI& poi_ = AvData.PoiList.m_poiList[j];
				if (poi_.m_iMajorType == 5 && poi_.m_iMinorType == 11 && poi_.m_iPlaceType == 3)
				{
					Mat Line = Mat::zeros(2,3,CV_64F);
					Line.at<double>(0,0) = poi.m_crd.m_ulLatitude;
					Line.at<double>(0,1) = poi.m_crd.m_ulLongitude;
					Line.at<double>(0,2) = poi.m_crd.m_ulAltitude;
					Line.at<double>(1,0) = poi_.m_crd.m_ulLatitude;
					Line.at<double>(1,1) = poi_.m_crd.m_ulLongitude;
					Line.at<double>(1,2) = poi_.m_crd.m_ulAltitude;
					g_Opengl->AddLinesRelative(Line, Mat(), (Mat_<uchar>(1,3)<<255,255,0));
					break;
				}
			}
		}

	}

	CGisTransform GisTrans;
	GisTrans.SetBaseLocatin(AvData.VS.m_iLatitude * 0.0000001, AvData.VS.m_iLongitude * 0.0000001, AvData.VS.m_iHeading * 0.1);
	//画objects
	for (unsigned int i = 0; i < g_Trans.m_ObjOpr.m_ObjList.rows; i++)
	{
		Point2d pt_in(g_Trans.m_ObjOpr.m_ObjList.at<double>(i,1),g_Trans.m_ObjOpr.m_ObjList.at<double>(i,2));
		Point2d pt_out = GisTrans.Wgs2Cart(pt_in) * 1000.0;
		float sz = 5000.0;
		Point3d pt(pt_out.x, pt_out.y, 0.0);	
		g_Opengl->AddSphereRelative(pt, sz, Mat(), (Mat_<uchar>(1,3)<<255,255,255));
	}


	return 1;
}

Av2HR_ADM_POINT TransPoint(CGisTransform& GisOpr, Av2HR_ADM_POINT& pt)
{
	Point2d pt_in(pt.m_ulLatitude * 0.0000001, pt.m_ulLongitude * 0.0000001);
	Point2d pt_out = GisOpr.Wgs2Cart(pt_in);
	Av2HR_ADM_POINT pt_;
	pt_.m_ulLatitude = pt_out.x * 1000.0;
	pt_.m_ulLongitude = pt_out.y * 1000.0;
	pt_.m_ulAltitude = pt.m_ulAltitude;
	return pt_;
}

long Wgs2Cart4Av2HR(Av2HR_Package_Data& Pack_in, Av2HR_Package_Data& Pack_out)
{
	Pack_out = Pack_in;

	CGisTransform GisTrans;
	GisTrans.SetBaseLocatin(Pack_in.VS.m_iLatitude * 0.0000001, Pack_in.VS.m_iLongitude * 0.0000001, Pack_in.VS.m_iHeading * 0.1);

	for (int i = 0; i < Pack_in.ProfileString.m_iLaneLineProfileNum; i++)
	{
		Av2HR_ADM_LANE_LINE_PROFILE& ProFileT = Pack_in.ProfileString.m_laneLineProfileString[i];
		Av2HR_ADM_LANE_LINE_PROFILE& ProFileT_ = Pack_out.ProfileString.m_laneLineProfileString[i];
		for (int j = 0; j < ProFileT.m_iLaneLineNum; j++)
		{
			Av2HR_ADM_OUTLINE& OutLine = ProFileT.m_outlineString[j];
			Av2HR_ADM_OUTLINE& OutLine_ = ProFileT_.m_outlineString[j];
			vector<Vec3d> VecLine;
			for (int k = 0; k < OutLine.m_usPointNum; k++)
			{
				Av2HR_ADM_POINT& PtT = OutLine.m_pointString[k];
				Av2HR_ADM_POINT& PtT_ = OutLine_.m_pointString[k];
				PtT_ = TransPoint(GisTrans,PtT);
			}
		}
	}

	for (int i = 0; i < Pack_in.PoiList.m_iPOINum; i++)
	{
		Av2HR_ADM_POI& PoiT = Pack_in.PoiList.m_poiList[i];
		Av2HR_ADM_POI& PoiT_ = Pack_out.PoiList.m_poiList[i];
		PoiT_.m_crd = TransPoint(GisTrans,PoiT.m_crd);
	}

	for (int i = 0; i < Pack_in.LaneSectionList.m_iLaneSecionNum; i++)
	{
		Av2HR_ADM_LANE_SECTION& LaneSectionT = Pack_in.LaneSectionList.m_laneSectionList[i];
		Av2HR_ADM_LANE_SECTION& LaneSectionT_ = Pack_out.LaneSectionList.m_laneSectionList[i];
		for (int j = 0; j < LaneSectionT.m_iLaneNum; j++)
		{
			Av2HR_ADM_LANE& LaneT = LaneSectionT.m_laneList[j];
			Av2HR_ADM_LANE& LaneT_ = LaneSectionT_.m_laneList[j];
			for (int k = 0; k < LaneT.m_iWaypointNum; k++)
			{
				Av2HR_ADM_WAYPOINT& WapPointT = LaneT.m_waypointList[k];
				Av2HR_ADM_WAYPOINT& WapPointT_ = LaneT_.m_waypointList[k];
				WapPointT_.m_crd = TransPoint(GisTrans,WapPointT.m_crd);
			}
		}
	}

	return 1;
}