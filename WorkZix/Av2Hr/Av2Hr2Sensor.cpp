#include "Av2Hr2Sensor.h"
#include <math.h>

#include <iostream>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/typeof/typeof.hpp>
using namespace boost::property_tree;

void WINAPI NaviReqCallBack(void* pData, void* pUser)
{
	LCM_NAVI_REQUIRE_INFO* pReq = (LCM_NAVI_REQUIRE_INFO*) pData;
	CAv2Hr2Sensor* pUser_ = (CAv2Hr2Sensor*) pUser;
	pReq->Gps.GPS_LONGITUDE += pUser_->m_Param.dLongitudeOffSet;
	pReq->Gps.GPS_LATITUDE += pUser_->m_Param.dLatitudeOffSet;
//	printf("Req recieved: %d\n",pReq->MsgInd);
	if (pUser_->m_bIsBusy)
	{
		printf("Virtual navy is busy! quit");
		return;
	}
	pUser_->m_bIsBusy = true;

	Av2HR_Package_Data Pack;
	EnterCriticalSection(&pUser_->m_cs);
	Pack = pUser_->m_Pack;
	LeaveCriticalSection(&pUser_->m_cs);
	LCM_NAVI_TO_SENSE_INFO NaviSendData;
	pUser_->AddObjects(*pReq,NaviSendData);
	pUser_->FillDataEx(Pack,*pReq,NaviSendData);
	pUser_->m_NaviSend.Send(string("LCM_NAVI_TO_SENSE_INFO"),NaviSendData);
//	printf("Navi data send: %d!\n",NaviSendData.MsgInd);

	pUser_->m_bIsBusy = false;
}

int CAv2HrParam::ReadParam()
{
	char szFilePath[MAX_PATH] = {0};
	GetModuleFileNameA(NULL,szFilePath,MAX_PATH);
	char* p = strrchr(szFilePath,'\\');
	*p = 0x00;
	strcat(szFilePath,"\\Av2HrSensor.xml");

	if (boost::filesystem::exists(szFilePath))
	{
		ptree pt;
		read_xml(szFilePath, pt);

		dLongitudeOffSet = pt.get<double>("CAv2HrParam.dLongitudeOffSet", 0.0);
		dLatitudeOffSet = pt.get<double>("CAv2HrParam.dLatitudeOffSet", 0.0);
		bIsLaneChangeEnable = pt.get<bool>("CAv2HrParam.bIsLaneChangeEnable", false);
		bTLEnable = pt.get<bool>("CAv2HrParam.bTLEnable", false);
		dTLMaxDist = pt.get<double>("CAv2HrParam.dTLMaxDist", 200.0);
		dTLMinHeight = pt.get<double>("CAv2HrParam.dTLMinHeight", 5.0);
		dTLMaxAngle = pt.get<double>("CAv2HrParam.dTLMaxAngle", 20.0);
		dSLMaxDist = pt.get<double>("CAv2HrParam.dSLMaxDist", 100.0);
		dSLMaxLateralDist = pt.get<double>("CAv2HrParam.dSLMaxLateralDist", 5.0);
		dSLOffSet = pt.get<double>("CAv2HrParam.dSLOffSet", 3.5);
		dChangeLaneShortenDist = pt.get<double>("CAv2HrParam.dChangeLaneShortenDist", 0.0);

		return 1;
	}
	else
	{
		printf("%s NOT exists!\n", szFilePath);

		return 0;
	}

	return 0;
}

int CAv2HrParam::WriteParam()
{
	char szFilePath[MAX_PATH] = {0};
	GetModuleFileNameA(NULL,szFilePath,MAX_PATH);
	char* p = strrchr(szFilePath,'\\');
	*p = 0x00;
	strcat(szFilePath,"\\Av2HrSensor.xml");

	ptree pt;
	pt.put<double>("CAv2HrParam.dLongitudeOffSet", dLongitudeOffSet);
	pt.put<double>("CAv2HrParam.dLatitudeOffSet", dLatitudeOffSet);
	pt.put<bool>("CAv2HrParam.bIsLaneChangeEnable", bIsLaneChangeEnable);
	pt.put<bool>("CAv2HrParam.bTLEnable", bTLEnable);
	pt.put<double>("CAv2HrParam.dTLMaxDist", dTLMaxDist);
	pt.put<double>("CAv2HrParam.dTLMinHeight", dTLMinHeight);
	pt.put<double>("CAv2HrParam.dTLMaxAngle", dTLMaxAngle);
	pt.put<double>("CAv2HrParam.dSLMaxDist", dSLMaxDist);
	pt.put<double>("CAv2HrParam.dSLMaxLateralDist", dSLMaxLateralDist);
	pt.put<double>("CAv2HrParam.dSLOffSet", dSLOffSet);
	pt.put<double>("CAv2HrParam.dChangeLaneShortenDist", dChangeLaneShortenDist);

	write_xml(szFilePath, pt);

	return 1;
}

void CAv2HrParam::PrintParam()
{
	printf("CAv2HrParam.dLongitudeOffSet = %.7f degree\n", dLongitudeOffSet);
	printf("CAv2HrParam.dLatitudeOffSet = %.7f degree\n", dLatitudeOffSet);
	printf("CAv2HrParam.bIsLaneChangeEnable = %d\n", bIsLaneChangeEnable);
	printf("CAv2HrParam.bTLEnable = %d\n", bTLEnable);
	printf("CAv2HrParam.dTLMaxDist = %.3f meter\n", dTLMaxDist);
	printf("CAv2HrParam.dTLMinHeight = %.3f meter\n", dTLMinHeight);
	printf("CAv2HrParam.dTLMaxAngle = %.3f degree\n", dTLMaxAngle);
	printf("CAv2HrParam.dSLMaxDist = %.3f meter\n", dSLMaxDist);
	printf("CAv2HrParam.dSLMaxLateralDist = %.3f meter\n", dSLMaxLateralDist);
	printf("CAv2HrParam.dSLOffSet = %.3f meter\n", dSLOffSet);
	printf("CAv2HrParam.dChangeLaneShortenDist = %.3f meter\n", dChangeLaneShortenDist);
}

CAv2Hr2Sensor::CAv2Hr2Sensor():
m_Req(string("LCM_NAVI_REQUIRE_INFO")),
m_NaviSend(string("LCM_NAVI_TO_SENSE_INFO")),
m_TrafficLightReqSend(string("LCM_TRAFIC_LIGHT_REQUIRE"))
{
	m_bIsBusy = false;
	InitializeCriticalSection(&m_cs);
	m_dRangeDistMin = 50.0;
	m_dRangeDistMax = 100.0;

	printf("Loading param...\n");
	m_Param.ReadParam();
	printf("Loading param finish\n");
	m_Param.PrintParam();
//	m_Param.WriteParam();

// 	m_Opengl->SetCameraMode(1);
// 	m_Opengl->SetScale(0.001);
// 	Mat RefPoint = Mat::eye(4,4,CV_64F);
// 	m_Opengl->AddRefPoints(RefPoint);
	m_ObjOpr.Init("Av2HrObjList.yml");
}

CAv2Hr2Sensor::~CAv2Hr2Sensor()
{
	DeleteCriticalSection(&m_cs);
}

int CAv2Hr2Sensor::Start()
{
	m_Req.SetCallBack(NaviReqCallBack,this);
	m_Req.Start();

	return 1;
}

int CAv2Hr2Sensor::UpdataNaviData(Av2HR_Package_Data& Pack_in)
{
	EnterCriticalSection(&m_cs);
	m_Pack = Pack_in;
	LeaveCriticalSection(&m_cs);
	return 1;
}

int CAv2Hr2Sensor::FillDataEx(Av2HR_Package_Data& Pack_in, LCM_NAVI_REQUIRE_INFO& Req_in, LCM_NAVI_TO_SENSE_INFO& Pack_out)
{
	printf("+++++++++++++++++++++++++++++++++++++++++\n");
	Pack_out.MsgInd = Req_in.MsgInd;

	m_GlobalTran.SetBaseLocatin(Req_in.Gps.GPS_LATITUDE, Req_in.Gps.GPS_LONGITUDE, Req_in.Gps.GPS_HEADING);
	Point2d pt_in(Req_in.Gps.GPS_LATITUDE, Req_in.Gps.GPS_LONGITUDE);
	Point2d pt_out = m_GlobalTran.Wgs2Cart(pt_in);
	Pack_out.MsgInd = Req_in.MsgInd;
	Pack_out.RefLocale.x = pt_out.x;
	Pack_out.RefLocale.y = pt_out.y;
	Pack_out.RefHeading = Req_in.Gps.GPS_HEADING;

	CGisTransform LocalTrans;
	LocalTrans.SetBaseLocatin(Req_in.Gps.GPS_LATITUDE, Req_in.Gps.GPS_LONGITUDE, Req_in.Gps.GPS_HEADING);

	for (int i = 0; i < Pack_in.ProfileString.m_iLaneLineProfileNum; i++)
	{
		Av2HR_ADM_LANE_LINE_PROFILE& ProFileT = Pack_in.ProfileString.m_laneLineProfileString[i];
		for (int j = 0; j < ProFileT.m_iLaneLineNum; j++)
		{
			bool bIsInRange = false;
			LCM_NAVI_LINE Line_;
			Av2HR_ADM_OUTLINE& OutLine = ProFileT.m_outlineString[j];
			for (int k = 0; k < OutLine.m_usPointNum; k++)
			{
				Av2HR_ADM_POINT& PtT = OutLine.m_pointString[k];
				LCM_POINT2D_F pt = TransPoint(LocalTrans,PtT);
				Line_.Line.push_back(pt);
				double dDist = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
				if (dDist <= 50.0)
				{
					bIsInRange = true;
				}
			}
			Line_.LineId = 1;
			Line_.LineType = OutLine.m_iOutlineType;
			Line_.ContLinePoint = Line_.Line.size();
			if (bIsInRange)
			{
				Pack_out.Lines.push_back(Line_);
			}
		}
	}
	Pack_out.ContLine = Pack_out.Lines.size();
	printf("Line cont:%d\n", Pack_out.ContLine);

	printf("Pack_in.LaneSectionList.m_iLaneSecionNum = %d\n", Pack_in.LaneSectionList.m_iLaneSecionNum);
	if (Pack_in.LaneSectionList.m_iLaneSecionNum <= 0)
	{
		return 0;
	}

	int nPathMinInd = -1;
	for (int i = 0; i < Pack_in.LaneSectionList.m_iLaneSecionNum; i++)
	{
		Av2HR_ADM_LANE_SECTION& LaneSectionT = Pack_in.LaneSectionList.m_laneSectionList[i];
		for (int j = 0; j < LaneSectionT.m_iLaneNum; j++)
		{
			Av2HR_ADM_LANE& Lane_ = LaneSectionT.m_laneList[j];
			if (Lane_.m_bIsCalculatedRoute == 0)
			{
				continue;
			}
			for (int k = 0; k < Lane_.m_iWaypointNum; k++)
			{
				Av2HR_ADM_WAYPOINT& WapPointT = Lane_.m_waypointList[k];
				LCM_POINT2D_F pt = TransPoint(LocalTrans,WapPointT.m_crd);
				double dDist = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
				if (dDist < m_dRangeDistMin)
				{
					nPathMinInd = i;
					break;
				}
			}
			if (nPathMinInd != -1)
			{
				break;
			}
		}
		if (nPathMinInd != -1)
		{
			break;
		}
	}
	printf("PathMinInd = %d\n", nPathMinInd);
	if (nPathMinInd == -1)
	{
		return 0;
	}

	int nPathMaxInd = -1;
	for (int i = nPathMinInd; i < Pack_in.LaneSectionList.m_iLaneSecionNum; i++)
	{
		Av2HR_ADM_LANE_SECTION& LaneSectionT = Pack_in.LaneSectionList.m_laneSectionList[i];
		for (int j = 0; j < LaneSectionT.m_iLaneNum; j++)
		{
			Av2HR_ADM_LANE& Lane_ = LaneSectionT.m_laneList[j];
			if (Lane_.m_bIsCalculatedRoute == 0)
			{
				continue;
			}
			Av2HR_ADM_WAYPOINT& WapPointT = Lane_.m_waypointList[0];
			LCM_POINT2D_F pt = TransPoint(LocalTrans,WapPointT.m_crd);
			double dDist = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
			if (dDist > m_dRangeDistMax)
			{
				nPathMaxInd = i;
				break;
			}
		}
		if (nPathMaxInd != -1)
		{
			break;
		}
	}
	if (nPathMaxInd == -1)
	{
		nPathMaxInd = Pack_in.LaneSectionList.m_iLaneSecionNum - 1;
	}
	printf("\n nPathMinInd = %d, nPathMaxInd = %d \n", nPathMinInd, nPathMaxInd);
	if (nPathMinInd > nPathMaxInd)
	{
		return 0;
	}

	int nPathNearInd = -1;
	double dNearestDist = DBL_MAX;
	for (int i = nPathMinInd; i <= nPathMaxInd; i++)
	{
		Av2HR_ADM_LANE_SECTION& LaneSectionT = Pack_in.LaneSectionList.m_laneSectionList[i];
		for (int j = 0; j < LaneSectionT.m_iLaneNum; j++)
		{
			Av2HR_ADM_LANE& Lane_ = LaneSectionT.m_laneList[j];
			if (Lane_.m_bIsCalculatedRoute == 0)
			{
				continue;
			}
			for (int k = 0; k < Lane_.m_iWaypointNum; k++)
			{
				Av2HR_ADM_WAYPOINT& WapPointT = Lane_.m_waypointList[k];
				LCM_POINT2D_F pt = TransPoint(LocalTrans,WapPointT.m_crd);
				double dDist = sqrt(pow(pt.x, 2) + pow(pt.y, 2));
				if (dDist < dNearestDist)
				{
					nPathNearInd = i;
					dNearestDist = dDist;
				}
			}
		}
	}
	if (nPathNearInd == -1)
	{
		return 0;
	}
	printf("\n nPathNearInd == %d \n", nPathNearInd);

	CPathJoint PathJoint(m_Param);
	printf("FeedBase:%d,", nPathNearInd);
	PathJoint.FeedBase(Pack_in.LaneSectionList.m_laneSectionList[nPathNearInd]);
	for (int i = nPathNearInd+1; i <= nPathMaxInd; i++)
	{
		Av2HR_ADM_LANE_SECTION& LaneSectionT = Pack_in.LaneSectionList.m_laneSectionList[i];
		PathJoint.FeedDataFront(LaneSectionT);
		printf("%d,", i);
	}
	for (int i = nPathNearInd-1; i >= nPathMinInd; i--)
	{
		Av2HR_ADM_LANE_SECTION& LaneSectionT = Pack_in.LaneSectionList.m_laneSectionList[i];
		PathJoint.FeedDataBack(LaneSectionT);
		printf("%d,", i);
	}
	printf("\n");
	printf("FeedData from %d to %d\n", nPathMinInd, nPathMaxInd);

	Av2HR_ADM_LANE_SECTION FinalPathSec = PathJoint.GetFinalLaneSection(Req_in);
	printf("GetFinalLaneSection finish\n");

	vector< LCM_NAVI_PATH > PathsTemp;
	for (int i = 0; i < FinalPathSec.m_laneList.size(); i++)
	{
		Av2HR_ADM_LANE& LaneT = FinalPathSec.m_laneList[i];
		LCM_NAVI_PATH Lcm_Path;
		Lcm_Path.PathType = LaneT.m_bIsCalculatedRoute;
		for (int j = 0; j < LaneT.m_waypointList.size(); j++)
		{
			Av2HR_ADM_WAYPOINT& WapPointT = LaneT.m_waypointList[j];
			LCM_POINT2D_F pt = TransPoint(LocalTrans,WapPointT.m_crd);
			Lcm_Path.Path.push_back(pt);
		}
		Lcm_Path.ContPathPoint = Lcm_Path.Path.size();
		PathsTemp.push_back(Lcm_Path);
	}
	printf("Step1\n");
	
	vector< LCM_NAVI_PATH > PathsTemp0;
	for (unsigned int i = 0; i < PathsTemp.size(); i++)
	{
		if (PathsTemp[i].PathType == 1 || m_Param.bIsLaneChangeEnable)
		{
			PathsTemp0.push_back(PathsTemp[i]);
		}
	}
	printf("Step2: %d Lanes\n", PathsTemp0.size());
	/*
	vector< LCM_NAVI_PATH > PathsTemp0;
	if (m_Param.bIsLaneChangeEnable)
	{
		//去掉较短的路径（<10米）
		for (unsigned int i = 0; i < PathsTemp.size(); i++)
		{
			LCM_NAVI_PATH& Path_ = PathsTemp[i];
			LCM_POINT2D_F& pt0 = Path_.Path.front();
			LCM_POINT2D_F& pt1 = Path_.Path.back();
			double dDist = sqrt(pow(pt0.x - pt1.x, 2) + pow(pt0.y - pt1.y, 2));
			//保存10米以上的路径和主路径
			if (dDist >= 10.0 || Path_.PathType == 1)
			{
				PathsTemp0.push_back(Path_);
			}
		}
	}
	else
	{
		//只保留主路径
		for (unsigned int i = 0; i < PathsTemp.size(); i++)
		{
			LCM_NAVI_PATH& Path_ = PathsTemp[i];
			//保存10米以上的路径和主路径
			if (Path_.PathType == 1)
			{
				PathsTemp0.push_back(Path_);
			}
		}
	}
	printf("Step2\n");
	*/

	//轨迹点插值
// 	vector< LCM_NAVI_PATH > PathsTemp1 = PathsTemp0;
// 	for (unsigned int i = 0; i < PathsTemp0.size(); i++)
// 	{
// 		PathsTemp1[i].ContPathPoint = 0;
// 		PathsTemp1[i].Path.clear();
// 		for (unsigned j = 0; j < PathsTemp0[i].Path.size()-1; j++)
// 		{
// 			PathsTemp1[i].Path.push_back(PathsTemp0[i].Path[j]);
// 			LCM_POINT2D_F pt;
// 			pt.x = (PathsTemp0[i].Path[j].x + PathsTemp0[i].Path[j+1].x)/2.0;
// 			pt.y = (PathsTemp0[i].Path[j].y + PathsTemp0[i].Path[j+1].y)/2.0;
// 			PathsTemp1[i].Path.push_back(pt);
// 		}
// 		PathsTemp1[i].ContPathPoint = PathsTemp1[i].Path.size();
// 	}
// 	PathsTemp0 = PathsTemp1;

	//轨迹0.5米一个点，后30米到前100米
	double dPathDense = 0.5;
	vector<LCM_NAVI_PATH> PathsTemp0_;
	for (unsigned int i = 0; i < PathsTemp0.size(); i++)
	{
		LCM_NAVI_PATH PathT = PathsTemp0[i];
		LCM_NAVI_PATH Path_ = PathT;
		Path_.Path.clear();
		Path_.ContPathPoint = 0;
		double dDistAcc = 0.0;
		bool bIsStart = false;
		bool bIsFinish = false;
		for (unsigned int j = 1; j < PathT.Path.size(); j++)
		{
			LCM_POINT2D_F& pt0 = PathT.Path[j-1];
			LCM_POINT2D_F& pt1 = PathT.Path[j];
			double dDist_ = sqrt(pow(pt0.x,2) + pow(pt0.y,2));
			if (dDist_ <= 50.0 && bIsStart == false)
				bIsStart = true;
			if (dDist_ >= 100.0 && bIsStart == true)
				bIsFinish = true;
			if (!bIsStart)
				continue;
			if (bIsFinish)
				break;

			double dDist = sqrt(pow((pt0.x - pt1.x),2) + pow((pt0.y - pt1.y),2));
			dDistAcc+=dDist;
			if (dDistAcc >= dPathDense)
			{
				Path_.Path.push_back(pt0);
				dDistAcc = 0;
			}
		}
		Path_.ContPathPoint = Path_.Path.size();
		if (Path_.Path.size() > 1)
		{
			PathsTemp0_.push_back(Path_);
		}
	}
	PathsTemp0 = PathsTemp0_;
	printf("Step3\n");

	//整理type
	for (int i = 0; i < PathsTemp0.size(); i++)
	{
		PathsTemp0[i].PathId = i;
	}
	printf("Step4\n");

	//添加红绿灯
	double dAngleMinTL = 10000000.0;
	int nNearIndTL = -1;
	for (unsigned int i = 0; i < Pack_in.PoiList.m_poiList.size(); i++)
	{
		Av2HR_ADM_POI& Obj = Pack_in.PoiList.m_poiList[i];
		LCM_POINT2D_F pt = TransPoint(LocalTrans, Obj.m_crd);
		if (Obj.m_iMajorType != 6)
			continue;
		if (Obj.m_iMinorType != 1)
			continue;
 		if (pt.y <= 0.0)//过滤后面的
 			continue;
		if (sqrt(pow(pt.x,2)+pow(pt.y,2)) >= m_Param.dTLMaxDist)//过滤太远的
			continue;
		if (Obj.m_iValue/100.0 < m_Param.dTLMinHeight)//过滤掉低高度的
			continue;
		double dAngle = abs(atan2(pt.x,pt.y))/CV_PI*180.0;
		if (dAngle >= /*20.0*/m_Param.dTLMaxAngle)	//过滤掉侧向
			continue;
		if (dAngle <= dAngleMinTL)
		{
			dAngleMinTL = dAngle;
			nNearIndTL = i;
		}
	}
	vector<Point2d> VecSl;
	for (unsigned int i = 0; i+1 < Pack_in.PoiList.m_poiList.size(); i++)
	{
		Av2HR_ADM_POI Obj = Pack_in.PoiList.m_poiList[i];
		if (Obj.m_iMajorType != 7)
			continue;
		if (Obj.m_iPlaceType != 2)
			continue;
		LCM_POINT2D_F ptStart = TransPoint(LocalTrans, Pack_in.PoiList.m_poiList[i].m_crd);
		LCM_POINT2D_F ptEnd = TransPoint(LocalTrans, Pack_in.PoiList.m_poiList[i+1].m_crd);
		double dLen = sqrt(pow(ptStart.x - ptEnd.x, 2) + pow(ptStart.y - ptEnd.y, 2));
		Point2d per = Point2d((ptEnd.x - ptStart.x)/dLen, (ptEnd.y - ptStart.y)/dLen);
		for (double j = 0.0; j < dLen; j += 1.0)
		{
			Point2d pt = Point2d(per.x * j + ptStart.x, per.y * j + ptStart.y);
			VecSl.push_back(pt);
		}
		VecSl.push_back(Point2d(ptEnd.x, ptEnd.y));
	}
	double dDistMinSL = 100000000.0;
	double dAngMinSL = 100000000.0;
	int nNearIndSL = -1;
	for (unsigned int i = 0; i < VecSl.size(); i++)
	{
		Point2d pt = VecSl[i];
		pt.y -= m_Param.dSLOffSet;
// 		if (pt.y <= 0.0)//过滤后面的
// 			continue;
		if (abs(pt.x) > /*5.0*/m_Param.dSLMaxLateralDist)//过滤侧面的
			continue;
		double dDist = sqrt(pow(pt.x,2)+pow(pt.y,2));
		if (dDist >= /*100.0*/m_Param.dSLMaxDist)//过滤太远的
			continue;
		double dAngle = abs(atan2(pt.x,pt.y))/CV_PI*180.0;
		if (dAngle <= dAngMinSL)
		{
			dAngMinSL = dAngle;
			nNearIndSL = i;
		}
	}
	if (nNearIndTL != -1 && nNearIndSL != -1 && m_Param.bTLEnable)
	{
		Av2HR_ADM_POI& ObjTL = Pack_in.PoiList.m_poiList[nNearIndTL];
		LCM_POINT2D_F ptTL = TransPoint(LocalTrans, ObjTL.m_crd);
		Point2d ptSL = VecSl[nNearIndSL];
		ptSL.y -= m_Param.dSLOffSet;
		
		LCM_NAVI_OBJECT ObjT;
		LCM_POINT3D_F ptObj;
		ptObj.x = ptTL.x;
		ptObj.y = ptTL.y;
		ptObj.z = ObjTL.m_iValue/100.0;
		ObjT.points.clear();
		ObjT.points.push_back(ptObj);
		ObjT.nPtCont = 1;
		ObjT.nValue = Pack_out.Objs.size()+1;
		ObjT.nPlaceType = 2;
		ObjT.fOffSet = 0;
		ObjT.nMajorType = 6;
		ObjT.nMinorType = Pack_out.Objs.size();
		Pack_out.Objs.push_back(ObjT);

		ptObj.x = ptSL.x;
		ptObj.y = ptSL.y;
		ptObj.z = 0.0;
		ObjT.points.clear();
		ObjT.points.push_back(ptObj);
		ObjT.nPtCont = 1;
		ObjT.nValue = 0;
		ObjT.nPlaceType = 0;
		ObjT.fOffSet = 0;
		ObjT.nMajorType = 7;
		ObjT.nMinorType = Pack_out.Objs.size();
		Pack_out.Objs.push_back(ObjT);
	}

	//添加路口信息
	for (unsigned int i = 0; i < Pack_in.PoiList.m_poiList.size(); i++)
	{
		Av2HR_ADM_POI& poi = Pack_in.PoiList.m_poiList[i];
		if (poi.m_iMajorType == 5 && poi.m_iMinorType == 11)
		{

			LCM_POINT2D_F pt0 = TransPoint(LocalTrans, poi.m_crd);
			LCM_NAVI_OBJECT ObjT;
			LCM_POINT3D_F ptObj;
			ptObj.x = pt0.x;
			ptObj.y = pt0.y;
			ptObj.z = 0.0;
			ObjT.points.push_back(ptObj);
			ObjT.nPtCont = ObjT.points.size();

			ObjT.nValue = poi.m_iValue;
			ObjT.nPlaceType = poi.m_iPlaceType;
			ObjT.fOffSet = poi.m_iOffset;
			ObjT.nMajorType = 11;
			ObjT.nMinorType = Pack_out.Objs.size();
			Pack_out.Objs.push_back(ObjT);

		}

	}

	Pack_out.ContObj = Pack_out.Objs.size();
	printf("Step5\n");

	//发送红绿灯检测请求给相机检测程序
	TrafficLightRequire(Pack_out);
	printf("Step6\n");

	

	Pack_out.Paths = PathsTemp0;
	Pack_out.ContPath = Pack_out.Paths.size();
	//
	

	return 1;

}

int CAv2Hr2Sensor::AddObjects(LCM_NAVI_REQUIRE_INFO& Req_in, LCM_NAVI_TO_SENSE_INFO& Pack)
{
	vector<LCM_NAVI_OBJECT> ObjList;
	m_ObjOpr.TransData(Req_in, ObjList);
	for (unsigned int i = 0; i < ObjList.size(); i++)
	{
		Pack.Objs.push_back(ObjList[i]);
	}
	Pack.ContObj = Pack.Objs.size();

	return 1;
}

void CAv2Hr2Sensor::LaneChangeDetect(LCM_NAVI_TO_SENSE_INFO& NaviInfo)
{
	for (unsigned int i = 0; i < NaviInfo.Paths.size(); i++)
	{
		LCM_NAVI_PATH& PathT = NaviInfo.Paths[i];
		if (PathT.PathType != 0)
			continue;
		int nStartInd = -1;
		for (unsigned int j = 2; j < PathT.Path.size(); j++)
		{
			LCM_POINT2D_F pt0 = PathT.Path[j-2];
			LCM_POINT2D_F pt1 = PathT.Path[j-1];
			LCM_POINT2D_F pt2 = PathT.Path[j];
			Point2d vec1(pt1.x- pt0.x, pt1.y- pt0.y);
			Point2d vec2(pt2.x- pt1.x, pt2.y- pt1.y);
			double dsin = vec1.x * vec2.y - vec2.x * vec1.y;
			double dcos = vec1.x * vec2.x + vec1.y * vec2.y;
			double dAngle = abs(atan2(dsin, dcos)/CV_PI*180.0);
			if (dAngle >= 70)
			{
				nStartInd = j;
				break;
			}
		}
		if (nStartInd == -1)
			continue;
		int nEndInd = -1;
		double dAcc = 0.0;
		for (unsigned int j = nStartInd+1; j < PathT.Path.size(); j++)
		{
			LCM_POINT2D_F pt0 = PathT.Path[j-1];
			LCM_POINT2D_F pt1 = PathT.Path[j];
			double dDist = sqrt(pow(pt0.x-pt1.x,2)+pow(pt0.y-pt1.y,2));
			dAcc += dDist;
			if (dAcc >= 30)
			{
				nEndInd = j;
				break;
			}
		}
		if (nEndInd == -1)
			continue;



	}
}

void CAv2Hr2Sensor::TrafficLightRequire(LCM_NAVI_TO_SENSE_INFO& NaviInfo)
{
	int nNearestStopLineInd = -1;
	double dDistMin = DBL_MAX;
	for (unsigned int i = 0; i < NaviInfo.Objs.size(); i++)
	{
		LCM_NAVI_OBJECT& ObjT = NaviInfo.Objs[i];
		if (ObjT.nMajorType != 7)
			continue;
// 		if (ObjT.points[0].y <= -50.0 || ObjT.points[0].y >= 50.0)
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
// 		if (NaviInfo.Objs[i].nMajorType != 6)
// 			continue;
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

LCM_POINT2D_F CAv2Hr2Sensor::TransPoint(CGisTransform& GisOpr, Av2HR_ADM_POINT& pt)
{
	Point2d pt_in(pt.m_ulLatitude * 0.0000001, pt.m_ulLongitude * 0.0000001);
	Point2d pt_out = GisOpr.Wgs2CartRotate(pt_in);
	LCM_POINT2D_F pt_;
	pt_.x = pt_out.x;
	pt_.y = pt_out.y;
	return pt_;
}

Av2HR_ADM_LANE_SECTION CPathJoint::GetFinalLaneSection(LCM_NAVI_REQUIRE_INFO& Req_in)
{
 	Av2HR_ADM_LANE_SECTION out;

	for (int i = 0; i < m_Paths.size(); i++)
	{
		CLaneJoint& PathRef = m_Paths[i];
		Av2HR_ADM_LANE LaneT;
		LaneT.m_bIsCalculatedRoute = PathRef.m_bIsCalculatedRoute;
		list<Av2HR_ADM_WAYPOINT>::iterator it;
		for (it = PathRef.m_waypointList.begin(); it != PathRef.m_waypointList.end(); it++)
		{
			LaneT.m_waypointList.push_back(*it);
		}
		LaneT.m_iWaypointNum = LaneT.m_waypointList.size();
		out.m_laneList.push_back(LaneT);
	}

	//副车道缩短一段距离
	for (int i = 0; i < out.m_laneList.size(); i++)
	{
		Av2HR_ADM_LANE& LaneRef = out.m_laneList[i];
		if (LaneRef.m_bIsCalculatedRoute == 1)
		{
			continue;
		}
		Av2HR_ADM_WAYPOINT EndPt = LaneRef.m_waypointList.back();
		Point2d pt0(EndPt.m_crd.m_ulLatitude *  0.0000001, EndPt.m_crd.m_ulLongitude *  0.0000001);
		vector<Av2HR_ADM_WAYPOINT>::reverse_iterator rit;
		for (rit = LaneRef.m_waypointList.rbegin(); rit != LaneRef.m_waypointList.rend(); )
		{
			Point2d pt1(rit->m_crd.m_ulLatitude * 0.0000001, rit->m_crd.m_ulLongitude * 0.0000001);
			double dDist = GisWgsPoint2PointDist(pt0, pt1);
			if (dDist <= m_Param.dChangeLaneShortenDist)
			{
				rit = vector<Av2HR_ADM_WAYPOINT>::reverse_iterator(LaneRef.m_waypointList.erase((++rit).base())); 
			}
			else
			{
				++rit;
			}
		}
	}
	//删除没有点的路径
	vector<Av2HR_ADM_LANE>::iterator it;
	for (it = out.m_laneList.begin(); it != out.m_laneList.end(); )
	{
		if (it->m_waypointList.size() < 2)
		{
			it = out.m_laneList.erase(it);
		}
		else
		{
			++it;
		}
	}



	out.m_iLaneNum = out.m_laneList.size();

	return out;
}

vector<CLaneJoint> CPathJoint::Av2HR_ADM_LANE_2_CLaneJoint(Av2HR_ADM_LANE_SECTION& LaneSection_in)
{
	vector<CLaneJoint> out;
	for (int i = 0; i < LaneSection_in.m_iLaneNum; i++)
	{
		Av2HR_ADM_LANE& LaneRef = LaneSection_in.m_laneList[i];
		CLaneJoint Lane;
		Lane.m_bIsCalculatedRoute = LaneRef.m_bIsCalculatedRoute;
		for (int j = 0; j < LaneRef.m_iWaypointNum; j++)
		{
			Lane.m_waypointList.push_back(LaneRef.m_waypointList[j]);
		}
		out.push_back(Lane);
	}

	//去掉少于1个点的轨迹
	vector<CLaneJoint>::iterator it;
	for (it = out.begin(); it != out.end();)
	{
		if (it->m_waypointList.size() < 1)
		{
			it = out.erase(it);
		}
		else
		{
			it++;
		}
	}

	return out;
}

vector<CLaneJoint> CPathJoint::GetParallelPaths(Av2HR_ADM_LANE_SECTION& LaneSection_in)
{
	vector<CLaneJoint> LaneSection = Av2HR_ADM_LANE_2_CLaneJoint(LaneSection_in);
	vector<CLaneJoint> out;

// 	//每条路径必须至少2个点
// 	for (int i = 0; i < LaneSection.size(); i++)
// 	{
// 		if (LaneSection[i].m_waypointList.size() < 2)
// 		{
// 			printf("LaneSection.m_laneList.size() < 2\n");
// 			getchar();
// 			return out;
// 		}
// 	}

	int nMainInd = -1;
	for (int i = 0; i < LaneSection.size(); i++)
	{
		if (LaneSection[i].m_bIsCalculatedRoute == 1)
		{
			nMainInd = i;
			break;
		}
	}
	if (nMainInd == -1)
	{
		return out;
	}

	//只保留与主路径平行的道路
	CLaneJoint MainPath = LaneSection[nMainInd];
	for (int i = 0; i < LaneSection.size(); i++)
	{
		CLaneJoint& PathT = LaneSection[i];
		if (PathT.m_bIsCalculatedRoute == 1)
		{
			out.push_back(PathT);
		}
		else
		{
			Av2HR_ADM_WAYPOINT ptStart0 = MainPath.m_waypointList.front();
			Av2HR_ADM_WAYPOINT ptStart1 = PathT.m_waypointList.front();
			Av2HR_ADM_WAYPOINT ptEnd0 = MainPath.m_waypointList.back();
			Av2HR_ADM_WAYPOINT ptEnd1 = PathT.m_waypointList.back();
			Point2d ptStart0_(ptStart0.m_crd.m_ulLatitude * 0.0000001, ptStart0.m_crd.m_ulLongitude * 0.0000001);
			Point2d ptStart1_(ptStart1.m_crd.m_ulLatitude * 0.0000001, ptStart1.m_crd.m_ulLongitude * 0.0000001);
			Point2d ptEnd0_(ptEnd0.m_crd.m_ulLatitude * 0.0000001, ptEnd0.m_crd.m_ulLongitude * 0.0000001);
			Point2d ptEnd1_(ptEnd1.m_crd.m_ulLatitude * 0.0000001, ptEnd1.m_crd.m_ulLongitude * 0.0000001);
			double dDistStart = GisWgsPoint2PointDist(ptStart0_,ptStart1_);
			double dDistEnd = GisWgsPoint2PointDist(ptEnd0_,ptEnd1_);
			if (dDistStart > 2.0 && dDistEnd > 2.0)
			{
				out.push_back(PathT);
			}
		}
	}

	return out;
}

void CPathJoint::FeedBase(Av2HR_ADM_LANE_SECTION& LaneSection_in)
{

	m_Paths = GetParallelPaths(LaneSection_in);
	printf("FeedData: init Lane cont: %d\n", m_Paths.size());

}

void CPathJoint::FeedDataFront(Av2HR_ADM_LANE_SECTION& LaneSection_in)
{
	vector<CLaneJoint> LaneSection = GetParallelPaths(LaneSection_in);
	bool bIsContinue = false;
	for (int i = 0; i < m_Paths.size(); i++)
	{
		for (int j = 0; j < LaneSection.size(); j++)
		{
			CLaneJoint& Lane0 = m_Paths[i];
			Av2HR_ADM_WAYPOINT& pt0 = Lane0.m_waypointList.back();
			CLaneJoint& Lane1 = LaneSection[j];
			Av2HR_ADM_WAYPOINT& pt1 = Lane1.m_waypointList.front();
			if (pt0.m_crd.m_ulLatitude == pt1.m_crd.m_ulLatitude &&
				pt0.m_crd.m_ulLongitude == pt1.m_crd.m_ulLongitude &&
				Lane0.m_bIsCalculatedRoute == 1 && 
				Lane1.m_bIsCalculatedRoute == 1)
			{
				bIsContinue = true;
				break;
			}
		}
		if (bIsContinue)
		{
			break;
		}
	}

	if (bIsContinue)
	{
		for (int i = 0; i < m_Paths.size(); i++)
		{
			for (int j = 0; j < LaneSection.size(); j++)
			{
				CLaneJoint& Lane0 = m_Paths[i];
				Av2HR_ADM_WAYPOINT& pt0 = Lane0.m_waypointList.back();
				CLaneJoint& Lane1 = LaneSection[j];
				Av2HR_ADM_WAYPOINT& pt1 = Lane1.m_waypointList.front();
				if (pt0.m_crd.m_ulLatitude == pt1.m_crd.m_ulLatitude &&
					pt0.m_crd.m_ulLongitude == pt1.m_crd.m_ulLongitude)
				{
					if (Lane0.m_bIsCalculatedRoute == 1 &&
						Lane1.m_bIsCalculatedRoute == 1)
					{
						list<Av2HR_ADM_WAYPOINT>::iterator it;
						for (it = Lane1.m_waypointList.begin(); it != Lane1.m_waypointList.end(); it++)
						{
							Lane0.m_waypointList.push_back(*it);
						}
					}
					if (Lane0.m_bIsCalculatedRoute != 1 &&
						Lane1.m_bIsCalculatedRoute != 1)
					{
						list<Av2HR_ADM_WAYPOINT>::iterator it;
						for (it = Lane1.m_waypointList.begin(); it != Lane1.m_waypointList.end(); it++)
						{
							Lane0.m_waypointList.push_back(*it);
						}
					}
				}
			}
		}
	}
	else
	{
		for (int i = 0; i < m_Paths.size(); i++)
		{
			for (int j = 0; j < LaneSection.size(); j++)
			{
				CLaneJoint& Lane0 = m_Paths[i];
				Av2HR_ADM_WAYPOINT& pt0 = Lane0.m_waypointList.back();
				CLaneJoint& Lane1 = LaneSection[j];
				Av2HR_ADM_WAYPOINT& pt1 = Lane1.m_waypointList.front();
				if (pt0.m_crd.m_ulLatitude == pt1.m_crd.m_ulLatitude &&
					pt0.m_crd.m_ulLongitude == pt1.m_crd.m_ulLongitude)
				{
					list<Av2HR_ADM_WAYPOINT>::iterator it;
					for (it = Lane1.m_waypointList.begin(); it != Lane1.m_waypointList.end(); it++)
					{
						Lane0.m_waypointList.push_back(*it);
					}
					Lane0.m_bIsCalculatedRoute = Lane1.m_bIsCalculatedRoute;
				}
			}
		}
	}

	return;
}

void CPathJoint::FeedDataBack(Av2HR_ADM_LANE_SECTION& LaneSection_in)
{
 	vector<CLaneJoint> LaneSection = GetParallelPaths(LaneSection_in);

	for (int i = 0; i < m_Paths.size(); i++)
	{
		for (int j = 0; j < LaneSection.size(); j++)
		{
			CLaneJoint& Lane0 = m_Paths[i];
			Av2HR_ADM_WAYPOINT& pt0 = Lane0.m_waypointList.front();
			CLaneJoint& Lane1 = LaneSection[j];
			Av2HR_ADM_WAYPOINT& pt1 = Lane1.m_waypointList.back();
			if (pt0.m_crd.m_ulLatitude == pt1.m_crd.m_ulLatitude &&
				pt0.m_crd.m_ulLongitude == pt1.m_crd.m_ulLongitude)
			{
				list<Av2HR_ADM_WAYPOINT>::reverse_iterator rit;
				for (rit = Lane1.m_waypointList.rbegin(); rit != Lane1.m_waypointList.rend(); rit++)
				{
					Lane0.m_waypointList.push_front(*rit);
				}
			}
		}
	}


	return;
}

int CAv2Hr2Sensor::DrawOpengl(Av2HR_Package_Data& AvData)
{
	m_Opengl->ClearLines();
	m_Opengl->ClearBreakLines();
	m_Opengl->ClearSphere();
	std::cout << "LaneSectionList" << AvData.LaneSectionList.m_iPathIndex << std::endl;
	std::cout << "ProfileString" << AvData.ProfileString.m_iPathIndex << std::endl;
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
				m_Opengl->AddBreakLinesRelative(Lines.clone());
			}
			else
			{
				m_Opengl->AddLinesRelative(Lines.clone());
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
			//m_Opengl->AddBreakLinesRelative(Lines.clone(),Mat(),(Mat_<uchar>(1,3)<<0,255,0));
			m_Opengl->AddLinesRelative(Lines.clone(),Mat(),color.clone());
		}
	}

	for (int i = 0; i < AvData.PoiList.m_poiList.size(); i++)
	{
		Av2HR_ADM_POI& poi = AvData.PoiList.m_poiList[i];
		if (poi.m_iMajorType == 6)
		{
			float sz = 1000.0;
			Point3d pt(poi.m_crd.m_ulLatitude, poi.m_crd.m_ulLongitude, 0);	
			m_Opengl->AddSphereRelative(pt, sz, Mat(), (Mat_<uchar>(1,3)<<255,0,0));
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
				m_Opengl->AddLinesRelative(Line, Mat(), (Mat_<uchar>(1,3)<<255,0,0));
			}
		}

	}


	return 1;
}

CVirtualObjectList::CVirtualObjectList()
{

}
CVirtualObjectList::~CVirtualObjectList()
{

}

long CVirtualObjectList::Init(char* pszPath)
{
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
		ItemT.val[1] = m_ObjList.at<double>(i,1);
		ItemT.val[2] = m_ObjList.at<double>(i,2);
		ItemT.val[3] = m_ObjList.at<double>(i,3);
		ItemT.val[4] = m_ObjList.at<double>(i,4);
		ItemT.val[5] = m_ObjList.at<double>(i,5);
		ItemT.val[6] = m_ObjList.at<double>(i,6);
		VecItem.push_back(ItemT);
	}
// 	Mat matT(m_ObjList.rows, m_ObjList.cols, CV_64F, VecItem.data());
// 	matT.copyTo(m_ObjListTrans);

	return 1;
}

int CVirtualObjectList::TransData(LCM_NAVI_REQUIRE_INFO GpsRequir, vector<LCM_NAVI_OBJECT>& Objs)
{
	Objs.clear();
	Point2d pt0(GpsRequir.Gps.GPS_LATITUDE, GpsRequir.Gps.GPS_LONGITUDE);
	double dHeading = GpsRequir.Gps.GPS_HEADING;
	for (int i = 0; i < m_ObjList.rows; i++)
	{
		LCM_NAVI_OBJECT ObjT;
		Point2d ptIn(m_ObjList.at<double>(i,1), m_ObjList.at<double>(i,2));
		Point2d ptOut = GisWgsPoint2PointRotate(pt0, ptIn, dHeading);
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
		if (dDist <= 200.0 && ObjT.points[0].y >= -50.0)
		{
			Objs.push_back(ObjT);
		}
	}

	return Objs.size();
}