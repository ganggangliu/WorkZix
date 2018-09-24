#include "LidarMappingOpr.h"
#include "OpenCVInc.h"
#include "LibGuiOpenGL.h"
#include "LcmReceiver.h"
#include "LCM_SENSOR_FUSION_PACKAGE.hpp"

#include <iostream>

//#include <boost/algorithm/string.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
//#include <boost/format.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/typeof/typeof.hpp>
using namespace std;  
using namespace boost::property_tree;

#include "GisTransform.h"

#if _DEBUG
#pragma comment(lib,"LibGuiOpenGLD.lib")
#else
#pragma comment(lib,"LibGuiOpenGL.lib")
#endif

int LinkObj2Path(LCM_NAVI_TO_SENSE_INFO& Navi, LCM_IBEO_OBJECT_LIST& Obj);

std::string szChannleNamePackData = "LCM_SENSOR_FUSION_PACKAGE";

void WINAPI PackDataCallBack(void* pFutionPack, void* pUser)
{
	CLidarMappingOpr* pLidarMappingOpr = (CLidarMappingOpr*)pUser;
	LCM_SENSOR_FUSION_PACKAGE* pPackData = (LCM_SENSOR_FUSION_PACKAGE*)pFutionPack;
	EnterCriticalSection(&pLidarMappingOpr->m_cs);
	pLidarMappingOpr->m_DataPackage.clear();
	if (1/*pPackData->IbeoObjList.IbeoObjects.size() > 0*/)	//Ibeo障碍物模式
	{
		pLidarMappingOpr->m_DataPackage.nPackType = 1;
		pLidarMappingOpr->LidarObjects(pPackData);
	}
	else												//点云模式
	{
		pLidarMappingOpr->m_DataPackage.nPackType = 0;
		pLidarMappingOpr->LidarCloudMapping(pPackData);
	}
	LeaveCriticalSection(&pLidarMappingOpr->m_cs);
	if (pLidarMappingOpr->m_LidarDataRecFunc)
	{
		(*(pLidarMappingOpr->m_LidarDataRecFunc))(&(pLidarMappingOpr->m_DataPackage));
	}
}

tag_LidarMappingParam::tag_LidarMappingParam()
{
	nFrameCont = 10;
	bIsMapping = false;
	dCalibLongOffset = 0.0;
	dCalibLatOffset = 0.0;
	dCalibHeadingOffset = 0.0;
	nGroundLine = 3;
	nSkyLine = 5;
	nValidCont = 5;
	nHistoryLen = 6;
	dMaxStd = 200;
	dFrameInterval = 1/12.5;
	dCollisionPredictTime = 3.0;
	dMinExtMeasurment = 1.f;

//	WriteParam();
};

int tag_LidarMappingParam::LoadParam()
{
	char szFilePath[MAX_PATH] = {0};
	GetModuleFileNameA(NULL,szFilePath,MAX_PATH);
	char* p = strrchr(szFilePath,'\\');
	*p = 0x00;
	strcat(szFilePath,"\\LidarMappingOpr.xml");

	if (boost::filesystem::exists(szFilePath))
	{
		ptree pt;
		read_xml(szFilePath, pt);
		nFrameCont = pt.get<int>("tag_LidarMappingParam.nFrameCont", 10);
		bIsMapping = pt.get<bool>("tag_LidarMappingParam.bIsMapping", false);
		dCalibLongOffset = pt.get<double>("tag_LidarMappingParam.dCalibLongOffset", 3500.0);
		dCalibLatOffset = pt.get<double>("tag_LidarMappingParam.dCalibLatOffset", 0.0);
		dCalibHeadingOffset = pt.get<double>("tag_LidarMappingParam.dCalibHeadingOffset", 0.0);
		nGroundLine = pt.get<int>("tag_LidarMappingParam.nGroundLine", 3);
		nSkyLine = pt.get<int>("tag_LidarMappingParam.nSkyLine", 5);
		nValidCont = pt.get<int>("tag_LidarMappingParam.nValidCont", 5);

		return 1;
	}
	else
	{
		printf("%s NOT exists!\n", szFilePath);

		return 0;
	}

	return 0;
}

int tag_LidarMappingParam::WriteParam()
{
	char szFilePath[MAX_PATH] = {0};
	GetModuleFileNameA(NULL,szFilePath,MAX_PATH);
	char* p = strrchr(szFilePath,'\\');
	*p = 0x00;
	strcat(szFilePath,"\\LidarMappingOpr.xml");

	ptree pt;
	pt.put<int>("tag_LidarMappingParam.nFrameCont", nFrameCont);
	pt.put<bool>("tag_LidarMappingParam.bIsMapping", bIsMapping);
	pt.put<double>("tag_LidarMappingParam.dCalibLongOffset", dCalibLongOffset);
	pt.put<double>("tag_LidarMappingParam.dCalibLatOffset", dCalibLatOffset);
	pt.put<double>("tag_LidarMappingParam.dCalibHeadingOffset", dCalibHeadingOffset);
	pt.put<int>("tag_LidarMappingParam.nGroundLine", nGroundLine);
	pt.put<int>("tag_LidarMappingParam.nSkyLine", nSkyLine);
	pt.put<int>("tag_LidarMappingParam.nValidCont", nValidCont);

	write_xml(szFilePath, pt);

	return 1;
}

CLidarMappingOpr::CLidarMappingOpr(CLidarMappingParam Param)
{
	m_pLcmLidarCloud = new CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE>(szChannleNamePackData);
//	m_Param = Param;
	m_Param.LoadParam();
	m_phHandle = NULL;
	if (m_Param.bIsMapping)
	{
		m_phHandle = GetLibGuiOpenGL();
		((ILibGuiOpenGL*)m_phHandle)->SetCameraMode(AUTO_ANGLE);
		((ILibGuiOpenGL*)m_phHandle)->SetScale(0.005);
	}

	m_LidarDataRecFunc = NULL;
	m_bIsOldData = true;
	m_nIndOut = 0;
	InitializeCriticalSection(&m_cs);

	m_nGridFront = 50000;
	m_nGridBack = 20000;
	m_nGridSide = 20000;
	m_nGridSize = 100;

	RNG rng;
	for (int i = 0; i < 1000; i++)
	{
		m_ColorList[i].val[0] = rng.uniform(0,256);
		m_ColorList[i].val[1] = rng.uniform(0,256);
		m_ColorList[i].val[2] = rng.uniform(0,256);
	}

	m_bIsTurning = false;

	m_bIsBreak = false;

	m_nCurLaneId = -1;
	m_nNearestPointOfCurLane = -1;
	m_dDistCar2NearLane = DBL_MAX;

	m_dMobileyeHeadingOffSet = 0.3;
	m_MobileyeTranslate = Point3d(200.0, 4000.0, 0.0);

	m_nPredictStart = -1;
	m_nPredictEnd = -1;

	return;
}

CLidarMappingOpr::~CLidarMappingOpr()
{
	if (m_pLcmLidarCloud)
	{
		delete (CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE>*)m_pLcmLidarCloud;
		m_pLcmLidarCloud = NULL;
	}

	if (m_phHandle)
	{
		((ILibGuiOpenGL*)m_phHandle)->UnInit();
	}

	DeleteCriticalSection(&m_cs);

}


long CLidarMappingOpr::LidarCloudMapping(void* pPackData)
{
	LCM_SENSOR_FUSION_PACKAGE* pPackData_ = (LCM_SENSOR_FUSION_PACKAGE*)pPackData;
	//////////////////////////////////////////////////////////////////////////
	m_bIsOldData = false;
	m_nIndOut++;

	m_DataPackage.ObjectList.objects.resize(pPackData_->RadarList.size());
	for (int i = 0; i < pPackData_->RadarList.size(); i++)
	{
		LIDAR_RADAR_OBJECT_INFO& IbeoObj = pPackData_->RadarList[i];
		SENSOR_OBJECT& ObjTemp = m_DataPackage.ObjectList.objects[i];
		ObjTemp.obj_id = IbeoObj.nObjectID;
		ObjTemp.valid = 1;
		ObjTemp.from = IbeoObj.bValid;
// 		ObjTemp.obj_loc_x = -1.0*IbeoObj.fObjLocY;
// 		ObjTemp.obj_loc_y = IbeoObj.fObjLocX;
		ObjTemp.obj_loc_x = IbeoObj.fObjLocX;
		ObjTemp.obj_loc_y = IbeoObj.fObjLocY;
		ObjTemp.obj_size_x = 2.0/*IbeoObj.ObjBoxSize.y*/;
		ObjTemp.obj_size_y = 2.0/*IbeoObj.ObjBoxSize.x*/;
	}

	m_DataPackage.ImuData.Heading = pPackData_->Gps.GPS_HEADING;
	m_DataPackage.ImuData.Lattitude = pPackData_->Gps.GPS_LATITUDE;
	m_DataPackage.ImuData.Longitude = pPackData_->Gps.GPS_LONGITUDE;
	m_DataPackage.ImuData.Ve = pPackData_->Gps.GPS_VE;
	m_DataPackage.ImuData.Vn = pPackData_->Gps.GPS_VN;
	
	m_DataPackage.Lanes.clear();
	m_DataPackage.Lines.clear();
	vector<LCM_NAVI_LINE>& LinesT = pPackData_->NaviInfo.Lines;
	vector<LCM_NAVI_PATH>& PathsT = pPackData_->NaviInfo.Paths;
	for (int i = 0; i < LinesT.size(); i++)
	{
		LINE_INFO LineT;
		LineT.nInd = LinesT[i].LineId;
		LineT.nType = LinesT[i].LineType;
		for (int j = 0; j < LinesT[i].Line.size(); j++)
		{
			Point2d pt;
			pt.x = LinesT[i].Line[j].x;
			pt.y = LinesT[i].Line[j].y;
			LineT.points.push_back(pt);
		}
		m_DataPackage.Lines.push_back(LineT);
	}

	//根据Path的type进行排列
	for (int i = 0; i < PathsT.size(); i++)
	{
		for (int j = i+1; j < PathsT.size(); j++)
		{
			if (PathsT[i].PathType > PathsT[j].PathType)
			{
				LCM_NAVI_PATH tt = PathsT[i];
				PathsT[i] = PathsT[j];
				PathsT[j] = tt;
			}
		}
	}

	for (int i = 0; i < PathsT.size(); i++)
	{
		LANE_INFO LaneT;
		LaneT.nInd = PathsT[i].PathId;
		LaneT.nType = PathsT[i].PathType;
		for (int j = 0; j < PathsT[i].Path.size(); j++)
		{
			Point2d pt;
			pt.x = PathsT[i].Path[j].x;
			pt.y = PathsT[i].Path[j].y;
			LaneT.points.push_back(pt);
		}
		m_DataPackage.Lanes.push_back(LaneT);
	}

	//add navi events
	AddNaviEvents(m_DataPackage, pPackData_);

	//////////////////////////////////////////////////////////////////////////
	Mat Pose = Mat::eye(4, 4, CV_64F);
	Pose.at<double>(0,3) = pPackData_->NaviInfo.RefLocale.x * 1000;
	Pose.at<double>(1,3) = pPackData_->NaviInfo.RefLocale.y * 1000;
	Mat R = (Mat_<double>(1,3) << 0.0, 0.0, -1.0 * pPackData_->NaviInfo.RefHeading/180.0*CV_PI);
	Mat R_;
	Rodrigues(R, R_);
	R_.copyTo(Pose(Range(0,3),Range(0,3)));
	m_ListPose.push_front(Pose);
//	cout << Pose << endl;

	LCM_IBEO_CLOUD_POINTS* pCloudPoints_ = &(pPackData_->IbeoCloudPoints);
	vector<Point3d> VecPt;
	VecPt.reserve(pCloudPoints_->IbeoPoints.size());
	for (int i = 0; i < pCloudPoints_->IbeoPoints.size(); i++)
	{
		if (pCloudPoints_->IbeoPoints[i].layer <= m_Param.nGroundLine ||
			pCloudPoints_->IbeoPoints[i].layer >= m_Param.nSkyLine ||
			pCloudPoints_->IbeoPoints[i].distance <= 1000.0 ||
			(pCloudPoints_->IbeoPoints[i].flag & 0x04) /*||
			(pCloudPoints_->IbeoPoints[i].flag & 0x01) ||
			(pCloudPoints_->IbeoPoints[i].flag & 0x02) ||
			(pCloudPoints_->IbeoPoints[i].flag & 0x08) ||
			pCloudPoints_->IbeoPoints[i].echo != 0*/)
		{
			continue;
		}
		Point3d ptT;
		double ang = (pCloudPoints_->IbeoPoints[i].angle/32.0 + 90.0 + m_Param.dCalibHeadingOffset)/180.0*CV_PI;
// 		if (ang >= 125.0/180.0*CV_PI || ang <= 40.0/180.0*CV_PI)
// 		{
// 			continue;
// 		}
		ptT.x = cos(ang)*pCloudPoints_->IbeoPoints[i].distance + m_Param.dCalibLatOffset;
		ptT.y = sin(ang)*pCloudPoints_->IbeoPoints[i].distance + m_Param.dCalibLongOffset;
		ptT.z = 0;
// 		if (ptT.y>50000)
// 		{
// 			continue;
// 		}
		VecPt.push_back(ptT);
	}

	Mat matT = Mat(VecPt.size(),3,CV_64F,VecPt.data());
	Mat mat1 = Mat::ones(VecPt.size(),4,CV_64F);
	if (VecPt.size()>0)								//
	{
		matT.copyTo(mat1(Range::all(),Range(0,3)));
	}
	
	m_ListMatCloudPt.push_front(mat1);

	Mat GroundPt = Pose * mat1.t();
	m_ListGroundPt.push_front(GroundPt);

	if (m_ListMatCloudPt.size()>m_Param.nFrameCont)
	{
		m_ListMatCloudPt.pop_back();
		m_ListGroundPt.pop_back();
		m_ListPose.pop_back();
	}

	if (m_Param.bIsMapping)
	{
		((ILibGuiOpenGL*)m_phHandle)->AddRefPoints(pPackData_->NaviInfo.RefLocale.x * 1000,
			pPackData_->NaviInfo.RefLocale.y * 1000,
			pPackData_->NaviInfo.RefHeading * -1.0);
		((ILibGuiOpenGL*)m_phHandle)->AddPointsRelative(pPackData_->NaviInfo.RefLocale.x * 1000,
			pPackData_->NaviInfo.RefLocale.y * 1000,
			pPackData_->NaviInfo.RefHeading * -1.0,
			matT.clone());
	}

	if (m_ListMatCloudPt.size()>=m_Param.nFrameCont)
	{
		m_DataPackage.LidarMapping = DrawLidarPoints();
		return 1;
	}
	else
	{
		m_DataPackage.LidarMapping.release();
		return 0;
	}


}

long CLidarMappingOpr::LidarObjects(void* pPackData)
{
	LCM_SENSOR_FUSION_PACKAGE* pPackData_ = (LCM_SENSOR_FUSION_PACKAGE*)pPackData;
	//////////////////////////////////////////////////////////////////////////
	m_bIsOldData = false;
	m_nIndOut++;

	//Process Ibeo data
	double dAngRot = (90.0 + m_Param.dCalibHeadingOffset)/180.0*CV_PI;
	m_DataPackage.ObjectList.frame_ind = pPackData_->IbeoObjList.FrameInd;
 	m_DataPackage.ObjectList.objects.clear();
	m_DataPackage.ObjectList.objects.reserve(pPackData_->IbeoObjList.IbeoObjects.size());
	m_DataPackage.NaviEvents.clear();
	for (int i = 0; i < pPackData_->IbeoObjList.IbeoObjects.size(); i++)
	{
		LCM_IBEO_OBJECT& IbeoObj = pPackData_->IbeoObjList.IbeoObjects[i];
		SENSOR_OBJECT ObjTemp;
		ObjTemp.from = 0;
		ObjTemp.obj_id = IbeoObj.Id;
		ObjTemp.valid = 1;
		ObjTemp.age = IbeoObj.Age;
		ObjTemp.ext_measurement = IbeoObj.ObjExtMeasurement;
		ObjTemp.predict_age = IbeoObj.PredictAge;
		ObjTemp.classify = IbeoObj.Classification;
		ObjTemp.obj_heading =		IbeoObj.ObjOrientation + m_Param.dCalibHeadingOffset;
		ObjTemp.ref_point.x = (IbeoObj.RefPoint.x*cos(dAngRot) - IbeoObj.RefPoint.y*sin(dAngRot) + m_Param.dCalibLatOffset);
		ObjTemp.ref_point.y = (IbeoObj.RefPoint.x*sin(dAngRot) + IbeoObj.RefPoint.y*cos(dAngRot) + m_Param.dCalibLongOffset);
		ObjTemp.obj_loc_x =	(IbeoObj.ObjBoxCenter.x*cos(dAngRot) - IbeoObj.ObjBoxCenter.y*sin(dAngRot) + m_Param.dCalibLatOffset);
		ObjTemp.obj_loc_y =	(IbeoObj.ObjBoxCenter.x*sin(dAngRot) + IbeoObj.ObjBoxCenter.y*cos(dAngRot) + m_Param.dCalibLongOffset);
		ObjTemp.obj_loc_z =	0.0;
		ObjTemp.obj_size_x =		IbeoObj.ObjBoxSize.y;
		ObjTemp.obj_size_y =		IbeoObj.ObjBoxSize.x;
		ObjTemp.obj_size_z = 1.0;
		ObjTemp.v_x =		(IbeoObj.RelativeVelocity.x*cos(dAngRot) - IbeoObj.RelativeVelocity.y*sin(dAngRot));
		ObjTemp.v_y =		(IbeoObj.RelativeVelocity.x*sin(dAngRot) + IbeoObj.RelativeVelocity.y*cos(dAngRot));
 		ObjTemp.v_x_abs_from_sensor =	(IbeoObj.AbsVelocity.x*cos(dAngRot) - IbeoObj.AbsVelocity.y*sin(dAngRot));
 		ObjTemp.v_y_abs_from_sensor =	(IbeoObj.AbsVelocity.x*sin(dAngRot) + IbeoObj.AbsVelocity.y*cos(dAngRot));
// 		float fHeadRate = (pPackData_->Gps.GPS_HEADING - m_DataBefor.ImuData.Heading)/180.0*CV_PI/(1/12.5);//rad/s
// 		float fHeading = pPackData_->Gps.GPS_HEADING/180.0*CV_PI;
// 		float fVelLongitude = sin(fHeading);
// 		ObjTemp.v_x_abs_from_sensor = pPackData_->Gps.GPS_VE * 1000.0 + sin(fHead_) * ObjTemp.v_y + cos(fHead_) * ObjTemp.v_x;
// 		ObjTemp.v_y_abs_from_sensor = pPackData_->Gps.GPS_VN * 1000.0 + cos(fHead_) * ObjTemp.v_y + sin(fHead_) * ObjTemp.v_x;
		float fSpeed = sqrt(pow(pPackData_->Gps.GPS_VE,2)+pow(pPackData_->Gps.GPS_VN,2)) * 1000.0;
		ObjTemp.v_x_abs = 0.0;
		ObjTemp.v_y_abs = fSpeed + ObjTemp.v_y;
		ObjTemp.contour.resize(IbeoObj.ContourPts.size());
		for (int j = 0; j < IbeoObj.ContourPts.size(); j++)
		{
			cv::Point2d pt;
			pt.x =  (IbeoObj.ContourPts[j].x*cos(dAngRot) - IbeoObj.ContourPts[j].y*sin(dAngRot) + m_Param.dCalibLatOffset);
			pt.y =  (IbeoObj.ContourPts[j].x*sin(dAngRot) + IbeoObj.ContourPts[j].y*cos(dAngRot) + m_Param.dCalibLongOffset);
			ObjTemp.contour[j] = pt;
		}

		double X = ObjTemp.obj_loc_x;
		double Y = ObjTemp.obj_loc_y;
		Point2f center(X, Y);
		Point2f size(ObjTemp.obj_size_x, ObjTemp.obj_size_y);
		RotatedRect rectOrient(center, size, ObjTemp.obj_heading);
		Point2f vertices_[4];
		rectOrient.points(vertices_);
		for (int i = 0; i < 4; i++)
		{
			ObjTemp.rotate_rect.push_back(vertices_[i]);
		}

//		StateCheck(ObjTemp);
		m_DataPackage.ObjectList.objects.push_back(ObjTemp);
	}

	//Process Delphi Radar data
	for (int i = 0; i < pPackData_->RadarList.size(); i++)
	{
		LIDAR_RADAR_OBJECT_INFO ObjLcm = pPackData_->RadarList[i];
		if (ObjLcm.bValid == 12 || ObjLcm.bValid == 13)
		{
			double dRotAng = 180.0/180.f*CV_PI;
			Point2d T = cvPoint2D32f(0,2.3);
			double X = ((ObjLcm.fObjLocX*cos(dRotAng) - ObjLcm.fObjLocY*sin(dRotAng)) + T.x);
			double Y = ((ObjLcm.fObjLocX*sin(dRotAng) + ObjLcm.fObjLocY*cos(dRotAng)) + T.y);
			double vx = (ObjLcm.fObjSizeX*cos(dRotAng) - ObjLcm.fObjSizeY*sin(dRotAng));
			double vy = (ObjLcm.fObjSizeX*sin(dRotAng) + ObjLcm.fObjSizeY*cos(dRotAng));
			ObjLcm.fObjLocX = X;
			ObjLcm.fObjLocY = Y;
			ObjLcm.fObjSizeX = vx;
			ObjLcm.fObjSizeY = vy;
			double dObjAng = atan2(ObjLcm.fObjLocY, ObjLcm.fObjLocX)/CV_PI*180.0;
			if (dObjAng >= 45 && dObjAng <= 135)
			{
				continue;
			}
		}
		if (ObjLcm.bValid == 10 || ObjLcm.bValid == 11)
		{
			double dRotAng = 0.0/180.f*CV_PI;
			Point2d T = cvPoint2D32f(0.0,-2.0);
			double X = ((ObjLcm.fObjLocX*cos(dRotAng) - ObjLcm.fObjLocY*sin(dRotAng)) + T.x);
			double Y = ((ObjLcm.fObjLocX*sin(dRotAng) + ObjLcm.fObjLocY*cos(dRotAng)) + T.y);
			ObjLcm.fObjLocX = X;
			ObjLcm.fObjLocY = Y;
			ObjLcm.fObjSizeX = 0.0;
			ObjLcm.fObjSizeY = 0.0;
			double dObjAng = atan2(ObjLcm.fObjLocY, ObjLcm.fObjLocX)/CV_PI*180.0;
			if (dObjAng > -110 && dObjAng < -70)
			{
				continue;
			}
			if (ObjLcm.fObjLocY >= 10.0)
			{
				continue;
			}
		}
		SENSOR_OBJECT ObjTemp;
		ObjTemp.obj_id = (int)ObjLcm.bValid * 1000 + ObjLcm.nObjectID;
		ObjTemp.valid = 1/*ObjLcm.bValid*/;
		ObjTemp.from = ObjLcm.bValid;
		ObjTemp.valid = 1;
		ObjTemp.age = 100;
		ObjTemp.classify = 0;
		ObjTemp.obj_heading = 0;
		ObjTemp.ref_point.x = ObjLcm.fObjLocX*1000.0;
		ObjTemp.ref_point.y = ObjLcm.fObjLocY*1000.0;
		ObjTemp.obj_loc_x =	ObjLcm.fObjLocX*1000.0;
		ObjTemp.obj_loc_y =	ObjLcm.fObjLocY*1000.0;
		ObjTemp.obj_loc_z =	0.0;
		ObjTemp.obj_size_x = 100.0;
		ObjTemp.obj_size_y = 100.0;
		ObjTemp.obj_size_z = 1.0;
		ObjTemp.v_x = ObjLcm.fObjSizeX*1000.0;
		ObjTemp.v_y = ObjLcm.fObjSizeY*1000.0;
		float fHead_ = pPackData_->Gps.GPS_HEADING/180.0*CV_PI;
		ObjTemp.v_x_abs = pPackData_->Gps.GPS_VE * 1000.0 + sin(fHead_) * ObjTemp.v_y + cos(fHead_) * ObjTemp.v_x;
		ObjTemp.v_y_abs = pPackData_->Gps.GPS_VN * 1000.0 + cos(fHead_) * ObjTemp.v_y + sin(fHead_) * ObjTemp.v_x;
		if (ObjLcm.bValid == 9 || ObjLcm.bValid == 10 || ObjLcm.bValid == 11 || ObjLcm.bValid == 12 || ObjLcm.bValid == 13)
		{
			ObjTemp.obj_loc_y += 6500;
			ObjTemp.v_x = 0;
			ObjTemp.v_y = 0;
			ObjTemp.v_x_abs = 0;
			ObjTemp.v_y_abs = 0;
		}
		Point2d pt(ObjTemp.obj_loc_x, ObjTemp.obj_loc_y);
		ObjTemp.contour.push_back(pt);

		double X = ObjTemp.obj_loc_x;
		double Y = ObjTemp.obj_loc_y;
		Point2f center(X, Y);
		Point2f size(ObjTemp.obj_size_x, ObjTemp.obj_size_y);
		RotatedRect rectOrient(center, size, 0.0);
		Point2f vertices_[4];
		rectOrient.points(vertices_);
		for (int i = 0; i < 4; i++)
		{
			ObjTemp.rotate_rect.push_back(vertices_[i]);
		}

		m_DataPackage.ObjectList.objects.push_back(ObjTemp);
	}

	//Process Mobileye Objects
	double dMobiAngRot = (90.0 + m_dMobileyeHeadingOffSet)/180.0*CV_PI;
	for (unsigned int i = 0; i < pPackData_->MobileyeInfo.ObjList.size(); i++)
	{
		LCM_MOBILEYE_OBJECT& ObjRef = pPackData_->MobileyeInfo.ObjList[i];
		SENSOR_OBJECT ObjTemp;

		double X = 1000.0*(ObjRef.PosX*cos(dMobiAngRot) - ObjRef.PosY*sin(dMobiAngRot)) + m_MobileyeTranslate.x;
		double Y = 1000.0*(ObjRef.PosX*sin(dMobiAngRot) + ObjRef.PosY*cos(dMobiAngRot)) + m_MobileyeTranslate.y;
		Point2f center(X, Y);
		double dObjLength = 0.1;
		Point2f size(1000.0 * ObjRef.Width, 1000.0 * dObjLength);
		RotatedRect rectOrient(center, size, 0.0);
		Point2f vertices[4];
		rectOrient.points(vertices);
		for (unsigned int j = 0; j < 4; j++)
		{
			Point2d pt;
			pt.x = vertices[j].x;
			pt.y = vertices[j].y;
			ObjTemp.contour.push_back(pt);
		}
		ObjTemp.from = 20;
		ObjTemp.obj_id = 5000+ObjRef.Id;
		ObjTemp.valid = 1;
		ObjTemp.age = ObjRef.Age;
		ObjTemp.ext_measurement = 1.0;
		ObjTemp.predict_age = 0;
		ObjTemp.classify = 0;
		ObjTemp.obj_heading = 0.0;
		ObjTemp.ref_point.x = X;
		ObjTemp.ref_point.y = Y;
		ObjTemp.obj_loc_x = X;
		ObjTemp.obj_loc_y = Y;
		ObjTemp.obj_loc_z = 0.0;
		ObjTemp.obj_size_x = size.x;
		ObjTemp.obj_size_y = size.y;
		ObjTemp.obj_size_z = 1.0;
		ObjTemp.v_x = 0.0;
		ObjTemp.v_y = 1000.0*ObjRef.RelVelX*sin(dAngRot);
		float fSpeed = sqrt(pow(pPackData_->Gps.GPS_VE,2)+pow(pPackData_->Gps.GPS_VN,2)) * 1000.0;
		ObjTemp.v_x_abs = 0.0;
		ObjTemp.v_y_abs = fSpeed + ObjTemp.v_y;

		ObjTemp.rotate_rect = ObjTemp.contour;
		
		bool bIsInLane = true;
		if (pPackData_->MobileyeInfo.LeftLine.Quality >= 2 &&
			pPackData_->MobileyeInfo.RightLine.Quality >= 2)
		{
			Point2d ptLeft(-1.0*ObjRef.PosY - ObjRef.Width/2.0, ObjRef.PosX);
			Point2d ptRight(-1.0*ObjRef.PosY + ObjRef.Width/2.0, ObjRef.PosX);
			double dC3 = pPackData_->MobileyeInfo.LeftLine.C3;
			double dC2 = pPackData_->MobileyeInfo.LeftLine.C2;
			double dC1 = pPackData_->MobileyeInfo.LeftLine.C1;
			double dC0 = pPackData_->MobileyeInfo.LeftLine.C0;
			double dDistLeft2Line = dC3*pow(ptRight.y,3)+dC2*pow(ptRight.y,2)+dC1*ptRight.y+dC0;
			if (ptRight.x < dDistLeft2Line - 0.5)
			{
				bIsInLane = false;
			}

			dC3 = pPackData_->MobileyeInfo.RightLine.C3;
			dC2 = pPackData_->MobileyeInfo.RightLine.C2;
			dC1 = pPackData_->MobileyeInfo.RightLine.C1;
			dC0 = pPackData_->MobileyeInfo.RightLine.C0;
			double dDistRight2Line = dC3*pow(ptLeft.y,3)+dC2*pow(ptLeft.y,2)+dC1*ptLeft.y+dC0;
			if (ptLeft.x > dDistRight2Line + 0.5)
			{
				bIsInLane = false;
			}
		}

		if (bIsInLane && ObjTemp.age >= 5 && abs(ObjTemp.v_y_abs) < 10.0*1000.0*1000.0/3600.0)
		{
			m_DataPackage.ObjectList.objects.push_back(ObjTemp);
		}
	}

	m_DataPackage.ImuData.GPSTime = pPackData_->Gps.GPS_TIME;
	m_DataPackage.ImuData.Heading = pPackData_->Gps.GPS_HEADING;
	m_DataPackage.ImuData.Lattitude = pPackData_->Gps.GPS_LATITUDE;
	m_DataPackage.ImuData.Longitude = pPackData_->Gps.GPS_LONGITUDE;
	m_DataPackage.ImuData.Ve = pPackData_->Gps.GPS_VE;
	m_DataPackage.ImuData.Vn = pPackData_->Gps.GPS_VN;
	m_DataPackage.ImuData.Status = pPackData_->Gps.GPS_STATE;

	m_DataPackage.Lanes.clear();
	m_DataPackage.Lines.clear();
	vector<LCM_NAVI_LINE>& LinesT = pPackData_->NaviInfo.Lines;
	vector<LCM_NAVI_PATH>& PathsT = pPackData_->NaviInfo.Paths;
	for (int i = 0; i < LinesT.size(); i++)
	{
		LINE_INFO LineT;
		LineT.nInd = LinesT[i].LineId;
		LineT.nType = LinesT[i].LineType;
		for (int j = 0; j < LinesT[i].Line.size(); j++)
		{
			Point2d pt;
			pt.x = LinesT[i].Line[j].x;
			pt.y = LinesT[i].Line[j].y;
			LineT.points.push_back(pt);
		}
		m_DataPackage.Lines.push_back(LineT);
	}

	//按照Path的type从小到大进行排列

	for (int i = 0; i < PathsT.size(); i++)
	{
		LANE_INFO LaneT;
		LaneT.nInd = PathsT[i].PathId;
		LaneT.nType = PathsT[i].PathType;
		for (int j = 0; j < PathsT[i].Path.size(); j++)
		{
			Point2d pt;
			pt.x = PathsT[i].Path[j].x;
			pt.y = PathsT[i].Path[j].y;
			LaneT.points.push_back(pt);
		}
		m_DataPackage.Lanes.push_back(LaneT);
	}

	m_DataPackage.nCurLaneId = -1;
	for (int i = 0; i < m_DataPackage.Lanes.size(); i++)
	{
		if (m_DataPackage.Lanes[i].nType == 1)
		{
			m_DataPackage.nCurLaneId = i;
			break;
		}
	}

	//获取mobileye信息
	Process_Mobileye(pPackData_);

	Process_LaneKeep(pPackData_);

	//确认当前车道
	GetCurLaneId();

	//加入路口信息
	Process_11(pPackData_);

	//加入禁止预测信息
	Process_30_31(pPackData_);

	//障碍物与车道关联
	LinkObj2Path(&m_DataPackage);

//	GetTurnningInfo();
	GetTurnningInfoFromNavi();

	//add navi events
	AddNaviEvents(m_DataPackage, pPackData_);
	
//	CollisionDetect(m_DataPackage);

	//障碍物预测
	GetPredictAbsVel();

	//过滤障碍物
//	FilterObj(&m_DataPackage);

	//根据红绿灯及地图弯道添加禁止变道标记
	Process_100(pPackData_);

	//根据yml文件补充的禁止变道标记
	Process_20(pPackData_);

//	OtherWork();

	if (m_dDistCar2NearLane >= 5.0)
	{
		m_DataPackage.Lanes.clear();

// 		LANE_INFO PathT;
// 		PathT.nInd = 0;
// 		PathT.nType = 1;
// 		for (double i = -20; i < 50.0 ;i++)
// 		{
// 			Point2d pt(0.0, i);
// 			PathT.points.push_back(pt);
// 		}
// 		m_DataPackage.Lanes.push_back(PathT);
// 		m_nCurLaneId = 0;
		m_nCurLaneId = -1;

		m_DataPackage.NaviEvents.clear();
		NAVI_EVENT Event;
		Event.nMajorType = 200;
		m_DataPackage.NaviEvents.push_back(Event);
	}

	m_DataBefor = m_DataPackage;

	return 1;
}

void CLidarMappingOpr::Process_LaneKeep(void* pPackData)
{
	LCM_SENSOR_FUSION_PACKAGE* pPackData_ = (LCM_SENSOR_FUSION_PACKAGE*) pPackData;

	m_GpsStateList.push_front(pPackData_->Gps.GPS_STATE);
	if (m_GpsStateList.size() > 12)
	{
		m_GpsStateList.pop_back();
	}
	list<int>::iterator it;
	int nGpsState = 0;
	for (it = m_GpsStateList.begin(); it != m_GpsStateList.end(); it++)
	{
		if (*it == 2 || *it == 1)
		{
			nGpsState = 2;
			break;
		}
	}

	if (nGpsState == 2)
		return;

	//信号不好先减速
	NAVI_EVENT EventSlow;
	EventSlow.nMajorType = 8;
	EventSlow.PointsDest.push_back(Point3f(0.0, 30.0, 0.0));
	m_DataPackage.NaviEvents.push_back(EventSlow);

//	return;
	
	NAVI_EVENT EventMobiLane;
	bool bIsLaneExist = false;
	for (unsigned int i = 0; i < m_DataPackage.NaviEvents.size(); i++)
	{
		if (m_DataPackage.NaviEvents[i].nMajorType == -22)
		{
			EventMobiLane = m_DataPackage.NaviEvents[i];
			bIsLaneExist = true;
			break;
		}
	}
	if (!bIsLaneExist)
		return;

	double dDist2MobiLane;
	for (unsigned int i = 0; i+1 < EventMobiLane.PointsSource.size(); i++)
	{
		if (EventMobiLane.PointsSource[i].y < 0 && EventMobiLane.PointsSource[i+1].y >= 0)
		{
			Point3f pt0 = EventMobiLane.PointsSource[i];
			Point3f pt1 = EventMobiLane.PointsSource[i+1];
			dDist2MobiLane = (0-pt0.y)/(pt1.y-pt0.y)*(pt1.x-pt0.x)+pt0.x;
			break;
		}
	}
	double dDeltaDist = 0.0;
	if (dDist2MobiLane <= -0.3)
		dDeltaDist = -0.3 - dDist2MobiLane;
	if (dDist2MobiLane >= 0.3)
		dDeltaDist = 0.3 - dDist2MobiLane;
	vector<Point3f> LaneFixed = EventMobiLane.PointsSource;
	for (unsigned int i = 0; i < LaneFixed.size(); i++)
	{
		LaneFixed[i].x += dDeltaDist;
	}

	m_DataPackage.Lanes.clear();
	m_DataPackage.Lines.clear();
	LANE_INFO Lane;
	Lane.nInd = 0;
	Lane.nType = 1;
	for (unsigned int i = 0; i < LaneFixed.size(); i++)
	{
		Lane.points.push_back(Point2d(LaneFixed[i].x, LaneFixed[i].y));
	}
	m_DataPackage.Lanes.push_back(Lane);
	m_DataPackage.nCurLaneId = 0;

}

void CLidarMappingOpr::Process_Mobileye(void* pPackData)
{
	LCM_SENSOR_FUSION_PACKAGE* pPackData_ = (LCM_SENSOR_FUSION_PACKAGE*) pPackData;

	NAVI_EVENT EventLeftLine;
	EventLeftLine.nMajorType = -20;
	EventLeftLine.nValue = pPackData_->MobileyeInfo.LeftLine.Quality;
	if (pPackData_->MobileyeInfo.LeftLine.Valid != 0 && 
		pPackData_->MobileyeInfo.LeftLine.Quality >= 2)
	{
		double dC0L = pPackData_->MobileyeInfo.LeftLine.C0;
		double dC1L = pPackData_->MobileyeInfo.LeftLine.C1;
		double dC2L = pPackData_->MobileyeInfo.LeftLine.C2;
		double dC3L = pPackData_->MobileyeInfo.LeftLine.C3;	
		for (double i = -50.0; i < 50.0/*pPackData_->MobileyeInfo.LeftLine.Range*/; i++)
		{
			Point3f pt(dC3L*pow(i,3)+dC2L*pow(i,2)+dC1L*i+dC0L, i, 0.0);
			EventLeftLine.PointsSource.push_back(pt);
		}
	}
	if (EventLeftLine.PointsSource.size() > 0)
	{
		m_DataPackage.NaviEvents.push_back(EventLeftLine);
	}

	NAVI_EVENT EventRightLine;
	EventRightLine.nMajorType = -21;
	EventRightLine.nValue = pPackData_->MobileyeInfo.RightLine.Quality;
	if (pPackData_->MobileyeInfo.RightLine.Valid != 0 && 
		pPackData_->MobileyeInfo.RightLine.Quality >= 2)
	{
		double dC0R = pPackData_->MobileyeInfo.RightLine.C0;
		double dC1R = pPackData_->MobileyeInfo.RightLine.C1;
		double dC2R = pPackData_->MobileyeInfo.RightLine.C2;
		double dC3R = pPackData_->MobileyeInfo.RightLine.C3;	
		for (double i = -50.0; i < 50.0/*pPackData_->MobileyeInfo.RightLine.Range*/; i++)
		{
			Point3f pt(dC3R*pow(i,3)+dC2R*pow(i,2)+dC1R*i+dC0R, i, 0.0);
			EventRightLine.PointsSource.push_back(pt);
		}
	}
	if (EventRightLine.PointsSource.size() > 0)
	{
		m_DataPackage.NaviEvents.push_back(EventRightLine);
	}

	if (EventLeftLine.PointsSource.size() > 0 &&
		EventRightLine.PointsSource.size() > 0)
	{
		NAVI_EVENT EventLane;
		EventLane.nMajorType = -22;
		double dMobiAngRot = m_dMobileyeHeadingOffSet/180.0*CV_PI;
		for (unsigned int i = 0; i < EventLeftLine.PointsSource.size() &&
			i < EventRightLine.PointsSource.size(); i++)
		{
			Point3f pt((EventLeftLine.PointsSource[i].x + EventRightLine.PointsSource[i].x)/2.0,
				(EventLeftLine.PointsSource[i].y + EventRightLine.PointsSource[i].y)/2.0, 
				0.0);
			Point3f pt_;
			pt_.x = (pt.x*cos(dMobiAngRot) - pt.y*sin(dMobiAngRot)) + m_MobileyeTranslate.x/1000.0;
			pt_.y = (pt.x*sin(dMobiAngRot) + pt.y*cos(dMobiAngRot)) + m_MobileyeTranslate.y/1000.0;
			EventLane.PointsSource.push_back(pt_);
		}
		m_DataPackage.NaviEvents.push_back(EventLane);
	}

}

void CLidarMappingOpr::Process_100(void* pPackData)
{
	LCM_SENSOR_FUSION_PACKAGE* pPackData_ = (LCM_SENSOR_FUSION_PACKAGE*) pPackData;

	//添加路口禁止变道
	//是否有红绿灯
	bool bIsTrafficLightFront = false;
	NAVI_EVENT EventTL;
	for (unsigned int i = 0; i < m_DataPackage.NaviEvents.size(); i++)
	{
		if (m_DataPackage.NaviEvents[i].nMajorType == 6)
		{
			if (m_DataPackage.NaviEvents[i].PointsSource[0].y >= 0.0 &&
				abs(m_DataPackage.NaviEvents[i].PointsSource[0].x) <= 15.0)
			{
				bIsTrafficLightFront = true;
				EventTL = m_DataPackage.NaviEvents[i];
				break;
			}
		}
	}

	NAVI_EVENT Event;
	bool bIsSectonExist =false;
	for (unsigned int i = 0; i < m_DataPackage.NaviEvents.size(); i++)
	{
		if (m_DataPackage.NaviEvents[i].nMajorType == 11)
		{
			bIsSectonExist = true;
			Event = m_DataPackage.NaviEvents[i];
			break;
		}
	}

	if (!bIsSectonExist)
	{
		return;
	}

	if (bIsTrafficLightFront)
	{
		if (EventTL.PointsDest[0].y >= 0.0)
		{
			if (EventTL.PointsDest[0].y <= 50.0)
			{
				NAVI_EVENT Event;
				Event.nMajorType = 100;
				m_DataPackage.NaviEvents.push_back(Event);
			}
		}
		else
		{
			if (EventTL.PointsSource[0].y >= 0.0)
			{
				NAVI_EVENT Event;
				Event.nMajorType = 100;
				m_DataPackage.NaviEvents.push_back(Event);
			}
		}
	}
	else
	{
		if (m_nTurnningInfo == 1 || m_nTurnningInfo == 2)
		{
			if (Event.PointsSource[0].y >= 0.0)
			{
				if (Event.PointsSource[0].y <= 50.0)
				{
					NAVI_EVENT Event;
					Event.nMajorType = 100;
					m_DataPackage.NaviEvents.push_back(Event);
				}
			}
			else
			{
				if (Event.PointsDest[0].y >= 0.0)
				{
					NAVI_EVENT Event;
					Event.nMajorType = 100;
					m_DataPackage.NaviEvents.push_back(Event);
				}
			}
		}
	}
}

void CLidarMappingOpr::Process_20(void* pPackData)
{
	LCM_SENSOR_FUSION_PACKAGE* pPackData_ = (LCM_SENSOR_FUSION_PACKAGE*) pPackData;

	vector<LCM_NAVI_OBJECT> Vec20;
	for (unsigned int i = 0; i < pPackData_->NaviInfo.Objs.size(); i++)
	{
		if (pPackData_->NaviInfo.Objs[i].nMajorType == 20)
		{
			Vec20.push_back(pPackData_->NaviInfo.Objs[i]);
		}
	}
	if (Vec20.size() <= 0)
		return;

	vector<int> VecNearInd;
	vector<double> VecNearDist;
	for (unsigned int i = 0; i < Vec20.size() ;i++)
	{
		int nNearInd = -1;
		double dNearDist = DBL_MAX;
		LCM_NAVI_OBJECT& ObjRef = Vec20[i];
		for (unsigned int j = 0; j < m_MainPath.points.size(); j++)
		{
			Point2d& ptLane = m_MainPath.points[j];
			double dDist = sqrt(pow(ObjRef.points[0].x-ptLane.x,2)+
				pow(ObjRef.points[0].y-ptLane.y,2));
			if (dDist <= dNearDist)
			{
				dNearDist = dDist;
				nNearInd = j;
			}
		}

		if (dNearDist > 2.0)
			continue;

		if (nNearInd < m_nNearestPointOfCurLane)
			continue;

		double dAcc = 0.0;
		for (unsigned int j = m_nNearestPointOfCurLane; j+1 <= nNearInd; j++)
		{
			Point2d pt0 = m_MainPath.points[j];
			Point2d pt1 = m_MainPath.points[j+1];
			double dDist = sqrt(pow(pt0.x-pt1.x,2)+pow(pt0.y-pt1.y,2));
			dAcc += dDist;
		}
		if (dAcc > ObjRef.nValue)
			continue;

		NAVI_EVENT Event;
		Event.nMajorType = 20;
		Event.PointsSource.push_back(Point3f(ObjRef.points[0].x,
			ObjRef.points[0].y,
			ObjRef.points[0].z));
		m_DataPackage.NaviEvents.push_back(Event);

		Event.nMajorType = 100;
		m_DataPackage.NaviEvents.push_back(Event);
	}

}

void CLidarMappingOpr::Process_11(void* pPackData)
{
	LCM_SENSOR_FUSION_PACKAGE* pPackData_ = (LCM_SENSOR_FUSION_PACKAGE*) pPackData;

	vector<NAVI_EVENT> VecEvents;
	for (unsigned int i = 0; i + 1 < pPackData_->NaviInfo.Objs.size(); i++)
	{
		LCM_NAVI_OBJECT& Obj0 = pPackData_->NaviInfo.Objs[i];
		if (Obj0.nMajorType == 11 && Obj0.nPlaceType == 2)
		{
			for (unsigned int j = i+1; j < pPackData_->NaviInfo.Objs.size(); j++)
			{
				LCM_NAVI_OBJECT& Obj1 = pPackData_->NaviInfo.Objs[j];
				if (Obj1.nMajorType == 11 && Obj1.nPlaceType == 3)
				{
					NAVI_EVENT Event;
					Event.nMajorType = 11;
					Point3f pt0(Obj0.points[0].x, Obj0.points[0].y, 0.f);
					Point3f pt1(Obj1.points[0].x, Obj1.points[0].y, 0.f);
					Event.PointsSource.push_back(pt0);
					Event.PointsDest.push_back(pt1);
					Event.nValue = Obj0.nValue;
					VecEvents.push_back(Event);
					break;
				}
			}
		}
	}

	int nNearInd = -1;
	double dNearDist = DBL_MAX;
	for (unsigned int i = 0; i < VecEvents.size(); i++)
	{
		if (VecEvents[i].PointsSource[0].y >= 0.0 /*&& 
			abs(VecEvents[i].PointsSource[0].x) <= 15.0*/)
		{
			double dDist = sqrt(pow(VecEvents[i].PointsSource[0].x,2)+
				pow(VecEvents[i].PointsSource[0].y,2));
			if (dDist <= dNearDist)
			{
				dNearDist = dDist;
				nNearInd = i;
			}
		}
		if (VecEvents[i].PointsDest[0].y >= 0.0 /*&& 
			abs(VecEvents[i].PointsDest[0].x) <= 15.0*/)
		{
			double dDist = sqrt(pow(VecEvents[i].PointsDest[0].x,2)+
				pow(VecEvents[i].PointsDest[0].y,2));
			if (dDist <= dNearDist)
			{
				dNearDist = dDist;
				nNearInd = i;
			}
		}
	}

	if (nNearInd == -1)
	{
		return;
	}

	m_DataPackage.NaviEvents.push_back(VecEvents[nNearInd]);

}

void CLidarMappingOpr::Process_30_31(void* pPackData)
{
	m_nPredictForbidState = -1;
	if (m_nCurLaneId == -1 || m_nNearestPointOfCurLane == -1 || m_dDistCar2NearLane > 3.0)
		return;

	LANE_INFO CurLane = m_DataPackage.Lanes[m_nCurLaneId];

	LCM_SENSOR_FUSION_PACKAGE* pPackData_ = (LCM_SENSOR_FUSION_PACKAGE*) pPackData;
	vector<LCM_NAVI_OBJECT> VecObj_300_301;
	for (unsigned int i = 0; i < pPackData_->NaviInfo.Objs.size(); i++)
	{
		LCM_NAVI_OBJECT& ObjRef = pPackData_->NaviInfo.Objs[i];
		if (ObjRef.nMajorType == 30 || ObjRef.nMajorType == 31)
			VecObj_300_301.push_back(ObjRef);
	}

	for (unsigned int i = 0; i < VecObj_300_301.size(); i++)
	{
		LCM_NAVI_OBJECT& ObjRef = VecObj_300_301[i];
		int nEventPointInd = -1;
		for (unsigned int j = 0; j < CurLane.points.size(); j++)
		{
			Point2d& pt = CurLane.points[j];
			double dDist = sqrt(pow((ObjRef.points[0].x - pt.x),2) + pow((ObjRef.points[0].y - pt.y),2));
			if (dDist <= 2.0)
			{
				nEventPointInd = j;
				break;
			}
		}
		if (nEventPointInd == -1)
		{
			continue;
		}
		if (nEventPointInd < m_nNearestPointOfCurLane)
		{
			continue;
		}
		double dValidRange = ObjRef.nValue;
		double dDistAcc = 0.0;
		for (unsigned int j = m_nNearestPointOfCurLane; j+1 <= nEventPointInd; j++)
		{
			Point2d pt0 = CurLane.points[j];
			Point2d pt1 = CurLane.points[j+1];
			double dDist = sqrt(pow(pt0.x - pt1.x, 2)+pow(pt0.y - pt1.y, 2));
			dDistAcc += dDist;
		}
		if (dDistAcc <= dValidRange)
		{
			NAVI_EVENT Event;
			Event.nMajorType = ObjRef.nMajorType;
			Event.PointsDest.push_back(Point3f(ObjRef.points[0].x, ObjRef.points[0].y, 0.f));
			m_DataPackage.NaviEvents.push_back(Event);
			m_nPredictForbidState = ObjRef.nMajorType;
			break;
		}
	}
}

void CLidarMappingOpr::GetPredictAbsVel()
{
	/*
	m_ListHisoryData.push_front(m_DataPackage);
	if (m_ListHisoryData.size() > m_Param.nHistoryLen)
		m_ListHisoryData.pop_back();

	if (m_ListHisoryData.size() < m_Param.nHistoryLen)
	{
		return;
	}

	DATA_PACKAGE CurPack = m_ListHisoryData.front();
	
	vector<vector<SENSOR_OBJECT> > VecHistoryObj;

	for (unsigned int i = 0; i < CurPack.ObjectList.objects.size(); i++)
	{
		SENSOR_OBJECT& ObjRef = CurPack.ObjectList.objects[i];
		vector<SENSOR_OBJECT> ObjT;
		ObjT.push_back(ObjRef);
		std::list<DATA_PACKAGE>::iterator it;
		for (it = m_ListHisoryData.begin(), it++; it != m_ListHisoryData.end(); it++)
		{
			int nMatchInd = -1;
			for (unsigned int j = 0; j < it->ObjectList.objects.size(); j++)
			{
				SENSOR_OBJECT& ObjHistory = it->ObjectList.objects[j];
				if (ObjRef.obj_id == ObjHistory.obj_id)
				{
					nMatchInd = j;
					ObjT.push_back(ObjHistory);
					break;
				}
			}
		}
		VecHistoryObj.push_back(ObjT);
	}

	
	vector<double> VecHeading;
	vector<Point2d> VecLocate;
	std::list<DATA_PACKAGE>::iterator it;
	DATA_PACKAGE PackRef = m_ListHisoryData.front();
	for (it = m_ListHisoryData.begin(); it != m_ListHisoryData.end(); it++)
	{
		VecHeading.push_back(it->ImuData.Heading);
		VecLocate.push_back(Point2d(it->ImuData.Lattitude,it->ImuData.Longitude));
// 		Point2d pt = GisWgsPoint2PointRotate(Point2d(PackRef.ImuData.Lattitude, PackRef.ImuData.Longitude)
// 			,Point2d(it->ImuData.Lattitude, it->ImuData.Longitude),
// 			it->ImuData.Heading - PackRef.ImuData.Heading);
	}

	for (unsigned int i = 0; i < VecHistoryObj.size(); i++)//第i个障碍物
	{
		for (unsigned int j = 0; j < VecHistoryObj[i].size(); j++)//第j次跟踪
		{
			SENSOR_OBJECT ObjT = VecHistoryObj[i][j];
			float fHead_ = -1.0*(VecHeading[j] - PackRef.ImuData.Heading)/180.0*CV_PI;
			Point2d ptTrans;
			ptTrans.x = cos(fHead_)*ObjT.obj_loc_x - sin(fHead_)*ObjT.obj_loc_y;
			ptTrans.y = sin(fHead_)*ObjT.obj_loc_x + cos(fHead_)*ObjT.obj_loc_y;
			Point2d T = GisWgsPoint2PointRotate(Point2d(PackRef.ImuData.Lattitude, PackRef.ImuData.Longitude), 
				VecLocate[j], PackRef.ImuData.Heading);
			ptTrans += T*1000.0;
			VecHistoryObj[i][0].HistoryTrace.push_back(ptTrans);
			m_DataPackage.ObjectList.objects[i].HistoryTrace.push_back(ptTrans);
		}
	}

	for (unsigned int i = 0; i < m_DataPackage.ObjectList.objects.size(); i++)
	{
		SENSOR_OBJECT& ObjRef = m_DataPackage.ObjectList.objects[i];
		if (ObjRef.ext_measurement < m_Param.dMinExtMeasurment)
		{
			continue;
		}
		if (ObjRef.HistoryTrace.size() < m_Param.nHistoryLen)
		{
			continue;
		}
		if (ObjRef.obj_loc_y < 0)
		{
			continue;
		}
		if (ObjRef.predict_age > 0)
		{
			continue;
		}
		if (ObjRef.from == 10 || ObjRef.from == 11)
		{
			continue;
		}
		if (m_nPredictForbidState == 30 && ObjRef.obj_loc_x < 0.0)
		{
			continue;
		}
		if (m_nPredictForbidState == 31 && ObjRef.obj_loc_x > 0.0)
		{
			continue;
		}
		Mat matStdX = Mat::zeros(1, m_Param.nHistoryLen-1, CV_32F);
		Mat matStdY = Mat::zeros(1, m_Param.nHistoryLen-1, CV_32F);
		for (unsigned int j = 1; j < ObjRef.HistoryTrace.size(); j++)
		{
			matStdX.at<float>(0,j-1) = ObjRef.HistoryTrace[j-1].x - ObjRef.HistoryTrace[j].x;
			matStdY.at<float>(0,j-1) = ObjRef.HistoryTrace[j-1].y - ObjRef.HistoryTrace[j].y;
		}
		Mat meanX,meanY,stdX,stdY;
		meanStdDev(matStdX, meanX, stdX);
		meanStdDev(matStdY, meanY, stdY);
		float fStdX = stdX.at<double>(0,0);
		float fStdY = stdY.at<double>(0,0);
		float fMeanX = meanX.at<double>(0,0);
		float fMeanY = meanY.at<double>(0,0);
		if (fStdX > m_Param.dMaxStd || fStdY > m_Param.dMaxStd)
		{
			continue;
		}
		ObjRef.v_x_abs_track = fMeanX/m_Param.dFrameInterval;
		ObjRef.v_y_abs_track = fMeanY/m_Param.dFrameInterval;
		double dVel_abs = sqrt(pow(ObjRef.v_x_abs_track,2)+pow(ObjRef.v_y_abs_track,2));
		if (dVel_abs < 5*1000*1000/3600)
		{
			continue;
		}
		ObjRef.is_abs_track_v_available = true;
	}

	*/

	for (unsigned int i = 0; i < m_DataPackage.ObjectList.objects.size(); i++)
	{
		SENSOR_OBJECT& ObjRef = m_DataPackage.ObjectList.objects[i];
		ObjRef.v_x_abs_track = ObjRef.v_x_abs_from_sensor;
		ObjRef.v_y_abs_track = ObjRef.v_y_abs_from_sensor;
		ObjRef.is_abs_track_v_available = true;
	}

	if (m_nPredictStart == -1 || m_nPredictEnd == -1)
	{
		return;
	}

	bool bCollisionHappen = false;
	for (unsigned int i = 0; i < m_DataPackage.ObjectList.objects.size(); i++)
	{
		SENSOR_OBJECT& ObjRef = m_DataPackage.ObjectList.objects[i];
		bool bIsCollision = false;
		if (ObjRef.is_abs_track_v_available)
		{
			for (unsigned int j = m_nPredictStart; j < m_nPredictEnd; j++)
			{
				Point2d ptPath = m_MainPath.points[j] * 1000.0;
				for (float k = 0; k < m_Param.dCollisionPredictTime; k+=0.1)
				{
					double dXDelta = k * ObjRef.v_x_abs_track;
					double dYDelta = k * ObjRef.v_y_abs_track;
					Point2d ptStart(ObjRef.obj_loc_x, ObjRef.obj_loc_y);
					Point2d ptEnd(ptStart.x + dXDelta, ptStart.y + dYDelta);
					double dObj2Path = sqrt(pow(ptEnd.x-ptPath.x,2)+pow(ptEnd.y-ptPath.y,2));
					if (dObj2Path < 2000.0)
					{
						bIsCollision = true;
						bCollisionHappen  = true;
					}
				}
			}
		}
		ObjRef.is_will_collision = bIsCollision;
	}

	m_ListCollision.push_front(bCollisionHappen);
	if (m_ListCollision.size() > 12)
	{
		m_ListCollision.pop_back();
	}
	int nAcc = 0;
	list<bool>::iterator it_;
	for (it_ = m_ListCollision.begin(); it_ != m_ListCollision.end(); it_++)
	{
		if (*it_)
			nAcc++;
	}
	printf("Slow down percent: %d / %d\n",nAcc,m_ListCollision.size());
	if (m_bIsInTurn&&nAcc>=4)
	{
		m_bIsBreak = true;
	}
	if (m_bIsInTurn&&nAcc<=2)
	{
		m_bIsBreak = false;
	}
	if (m_bIsBreak)
	{
		NAVI_EVENT Evt;
		Evt.nMajorType = 200;
		m_DataPackage.NaviEvents.push_back(Evt);
	}

	//Mat imgPath = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
	//vector<Point2i> VecPtImg;
	//for (int i = 0; i < MainPath.points.size(); i++)
	//{
	//	Point2d pt(MainPath.points[i].x*1000.0, MainPath.points[i].y*1000.0);
	//	Point2i ptImg = Car2Image(pt);
	//	VecPtImg.push_back(ptImg);
	//}
	//Point2i* pPt = VecPtImg.data();
	//int PtLen = VecPtImg.size();
	//int nWidth = 3000.0/m_nGridSize;
	//polylines(imgPath,(const Point2i**)&pPt,(const int*)&PtLen,1,false,Scalar(255,255,255),nWidth);
	//for (unsigned int i = 1; i < m_DataPackage.ObjectList.objects.size(); i++)
	//{
	//	SENSOR_OBJECT& ObjRef = m_DataPackage.ObjectList.objects[i];
	//	if (ObjRef.age < 25)
	//	{
	//		continue;
	//	}
	//	Mat objPath = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
	//	double dTimeDelta = m_Param.dCollisionPredictTime;
	//	double dXDelta = dTimeDelta * ObjRef.v_x_abs_track;
	//	double dYDelta = dTimeDelta * ObjRef.v_y_abs_track;
	//	Point2d ptStart(ObjRef.obj_loc_x, ObjRef.obj_loc_y);
	//	Point2d ptEnd(ptStart.x + dXDelta, ptStart.y + dYDelta);
	//	Point2i ptStart_ = Car2Image(ptStart);
	//	Point2i ptEnd_ = Car2Image(ptEnd);
	//	line(objPath,ptStart_,ptEnd_,CV_RGB(255,255,255));
	//	Mat collision = objPath.mul(imgPath);
	//	if (countNonZero(collision) > 0)
	//	{
	//		ObjRef.is_will_collision = true;
	//	}
	//}

	return;

}

int CLidarMappingOpr::GetTurnningInfoFromNavi()
{
	m_MainPath.points.clear();
	m_nTurnningInfo = 0;
	m_nTurnStartInd = -1;
	m_nTurnEndInd = -1;
	m_bIsInTurn = false;

	m_nPredictStart = -1;
	m_nPredictEnd = -1;
	//弯道检测
	if (m_nCurLaneId == -1 || m_nNearestPointOfCurLane == -1 || m_dDistCar2NearLane > 3.0)
	{
		return 0;
	}
	m_MainPath = m_DataPackage.Lanes[m_nCurLaneId];

	if (m_MainPath.points.size() <= 0)
	{
		return 0;
	}

	//找最近点
	int nNearestInd = m_nNearestPointOfCurLane;	

	//确定预测范围
	
	int nPreStart = nNearestInd;
	for ( ;nPreStart < m_MainPath.points.size(); nPreStart++)
	{
		double dDist = sqrt(pow(m_MainPath.points[nPreStart].x,2)+pow(m_MainPath.points[nPreStart].y,2));
		if (dDist >= 7.0)
		{
			break;
		}
	}
	m_nPredictStart = nPreStart;

	double dSpeed = sqrt(pow(m_DataPackage.ImuData.Ve,2) + pow(m_DataPackage.ImuData.Vn,2));
	double dPrePathLen = (m_Param.dCollisionPredictTime + 1.0) * (dSpeed + 2.0);
	m_nPredictEnd = m_nPredictStart;
	double dDistAcc = 0.0;
	for ( ;m_nPredictEnd+1 < m_MainPath.points.size(); m_nPredictEnd++)
	{
		Point2d pt0 = m_MainPath.points[m_nPredictEnd];
		Point2d pt1 = m_MainPath.points[m_nPredictEnd+1];
		double dDist = sqrt(pow(pt0.x-pt1.x,2)+pow(pt0.y-pt1.y,2));
		dDistAcc += dDist;
		if (dDistAcc >= dPrePathLen)
		{
			break;
		}
	}
	m_nPredictEnd = m_nPredictEnd+1;
	

	//找路口事件
	int nSectionId = -1;
	for (unsigned int i = 0; i < m_DataPackage.NaviEvents.size(); i++)
	{
		if (m_DataPackage.NaviEvents[i].nMajorType == 11)
		{
			nSectionId = i;
		}
	}
	if (nSectionId != -1)
	{
		NAVI_EVENT Section = m_DataPackage.NaviEvents[nSectionId];

		//找起点
		double dStartDist = DBL_MAX;
		for (unsigned int i = 0; i < m_MainPath.points.size(); i++)
		{
			Point3f pt0 = Section.PointsSource[0];
			Point2f pt1 = m_MainPath.points[i];
			double dDist = sqrt(pow(pt0.x - pt1.x,2) + pow(pt0.y - pt1.y,2));
			if (dDist <= dStartDist)
			{
				dStartDist = dDist;
				m_nTurnStartInd = i;
			}
		}
		//update start
		int nStart_ = m_nTurnStartInd;
		double dDistAcc = 0.0;
		for ( ;nStart_ >= 1; nStart_--)
		{
			Point2d pt0 = m_MainPath.points[nStart_-1];
			Point2d pt1 = m_MainPath.points[nStart_];
			double dDist = sqrt(pow(pt0.x-pt1.x,2)+pow(pt0.y-pt1.y,2));
			dDistAcc += dDist;
			if (dDistAcc >= 10.0)
			{
				break;
			}
		}
		m_nTurnStartInd = nStart_;

		double dEndDist = DBL_MAX;
		for (unsigned int i = 0; i < m_MainPath.points.size(); i++)//找终点
		{
			Point3f pt0 = Section.PointsDest[0];
			Point2f pt1 = m_MainPath.points[i];
			double dDist = sqrt(pow(pt0.x - pt1.x,2) + pow(pt0.y - pt1.y,2));
			if (dDist <= dEndDist)
			{
				dEndDist = dDist;
				m_nTurnEndInd = i;
			}
		}

// 		if (m_nTurnEndInd < m_nTurnStartInd)
// 		{
// 			m_bIsBreak = false;
// 			return 0;
// 		}

		//判断左右弯道
		vector<double> VecCurve;
		for (unsigned int i = 0; i+2 < m_MainPath.points.size(); i++)
		{
			Point2f pt0 = m_MainPath.points[i];
			Point2f pt1 = m_MainPath.points[i+1];
			Point2f pt2 = m_MainPath.points[i+2];
			double dHeading0 = atan2((pt1.y - pt0.y), (pt1.x - pt0.x));
			double dHeading1 = atan2((pt2.y - pt1.y), (pt2.x - pt1.x));
			double dDeltaAng = dHeading1-dHeading0;
			if (dDeltaAng < -1.0*CV_PI)
			{
				dDeltaAng = dHeading1 - (dHeading0 - 2.0*CV_PI);
			}
			if (dDeltaAng > CV_PI)
			{
				dDeltaAng = dHeading1 - (dHeading0 + 2.0*CV_PI);
			}
			double dDist0 = sqrt(pow((pt1.y - pt0.y),2) + pow((pt1.x - pt0.x),2));
			double dDist1 = sqrt(pow((pt2.y - pt1.y),2) + pow((pt2.x - pt1.x),2));
			double dCurve = dDeltaAng/(dDist0+dDist1);
			VecCurve.push_back(dCurve);
		}
		vector<double> VecCurveFilter;
		int nFilterLen = 10;
		for (unsigned int i = 0; i+nFilterLen < VecCurve.size(); i++)
		{
			double dDist = 0.0;
			for (unsigned int j = 0; j < nFilterLen; j++)
			{
				dDist += VecCurve[i+j];
			}
			dDist/=nFilterLen;
			VecCurveFilter.push_back(dDist);
		}
		m_nTurnningInfo = 3;
		for (unsigned int i = m_nTurnStartInd; i < m_nTurnEndInd && i < VecCurveFilter.size(); i++)
		{
			if (VecCurveFilter[i] <= -0.02)
			{
				m_nTurnningInfo = 2;
				break;
			}
			if (VecCurveFilter[i] >= 0.02)
			{
				m_nTurnningInfo = 1;
				break;
			}
		}

		//确认是否正在弯道中
		if (nNearestInd >= m_nTurnStartInd && nNearestInd <= m_nTurnEndInd &&
			(m_nTurnningInfo == 1 || m_nTurnningInfo == 2))
		{
			m_bIsInTurn = true;
		}
		else
		{
			m_bIsInTurn = false;
		}
	}

	if (m_nTurnEndInd != -1)
	{
		if (m_nTurnEndInd <= m_nPredictEnd)
		{
			m_nPredictEnd = m_nTurnEndInd;
		}
	}

	NAVI_EVENT Ent;
	Ent.nMajorType = 201;
	for (unsigned int i = m_nPredictStart; i < m_nPredictEnd- 1; i++)
	{
		Point3f pt_(m_MainPath.points[i].x, m_MainPath.points[i].y, 0);
		Ent.PointsSource.push_back(pt_);
	}
	Ent.nValue = m_bIsInTurn;
	Ent.nPlaceType = m_nTurnningInfo;
	m_DataPackage.NaviEvents.push_back(Ent);

	return 1;
}

int CLidarMappingOpr::GetTurnningInfo()
{
	m_MainPath.points.clear();
	m_nTurnningInfo = 0;
	m_nTurnStartInd = -1;
	m_nTurnEndInd = -1;
	m_bIsInTurn = false;
	//弯道检测
	for (int i = 0; i < m_DataPackage.Lanes.size(); i++)
	{
		if (i == m_DataPackage.nCurLaneId)
		{
			m_MainPath = m_DataPackage.Lanes[i];
			break;
		}
	}
	if (m_MainPath.points.size() <= 0)
	{
		return 0;
	}
	vector<double> VecCurve;
	for (unsigned int i = 0; i+2 < m_MainPath.points.size(); i++)
	{
		Point2f pt0 = m_MainPath.points[i];
		Point2f pt1 = m_MainPath.points[i+1];
		Point2f pt2 = m_MainPath.points[i+2];
		double dHeading0 = atan2((pt1.y - pt0.y), (pt1.x - pt0.x));
		double dHeading1 = atan2((pt2.y - pt1.y), (pt2.x - pt1.x));
		double dDeltaAng = dHeading1-dHeading0;
		if (dDeltaAng < -1.0*CV_PI)
		{
			dDeltaAng = dHeading1 - (dHeading0 - 2.0*CV_PI);
		}
		if (dDeltaAng > CV_PI)
		{
			dDeltaAng = dHeading1 - (dHeading0 + 2.0*CV_PI);
		}
		double dDist0 = sqrt(pow((pt1.y - pt0.y),2) + pow((pt1.x - pt0.x),2));
		double dDist1 = sqrt(pow((pt2.y - pt1.y),2) + pow((pt2.x - pt1.x),2));
		double dCurve = dDeltaAng/(dDist0+dDist1);
		VecCurve.push_back(dCurve);
	}
	vector<double> VecCurveFilter;
	int nFilterLen = 10;
	for (unsigned int i = 0; i+nFilterLen < VecCurve.size(); i++)
	{
		double dDist = 0.0;
		for (unsigned int j = 0; j < nFilterLen; j++)
		{
			dDist += VecCurve[i+j];
		}
		dDist/=nFilterLen;
		VecCurveFilter.push_back(dDist);
	}
	int nNearestInd = -1;										//找最近点
	double dDistMin = 100000000.0;
	for (unsigned int i = 0; i < VecCurveFilter.size(); i++)
	{
		double dDist = sqrt(pow(m_MainPath.points[i].x,2)+pow(m_MainPath.points[i].y,2));
		if (dDist <= dDistMin)
		{
			dDistMin = dDist;
			nNearestInd = i;
		}
	}
	if (nNearestInd == -1)
	{
		m_bIsBreak = false;
		return 0;
	}
	for (unsigned int i = nNearestInd; i < VecCurveFilter.size(); i++)//最近点后面的弯道起点
	{
		if (VecCurveFilter[i] >= 0.02)
		{
			m_nTurnStartInd = i;
			m_nTurnningInfo = 1;
			break;
		}
		if (VecCurveFilter[i] <= -0.02)
		{
			m_nTurnStartInd = i;
			m_nTurnningInfo = 2;
			break;
		}
	}
	if (m_nTurnStartInd == -1)
	{
		m_bIsBreak = false;
		return 0;
	}
	for (int i = m_nTurnStartInd; i < VecCurveFilter.size(); i++)//弯道起点后面的弯道终点
	{
		if (abs(VecCurveFilter[i]) <= 0.01)
		{
			m_nTurnEndInd = i;
			break;
		}
	}
	if (m_nTurnEndInd == -1)
	{
		m_bIsBreak = false;
		return 0;
	}
	int nNewStart = -1;
	for (int i = m_nTurnStartInd; i >= nNearestInd; i--)//更新弯道起点
	{
		if (abs(VecCurveFilter[i]) > 0.01)
		{
			nNewStart = i;
		}
	}
	if (nNewStart != -1)
	{
		m_nTurnStartInd = nNewStart;
	}
	//确认是否正在变道中
	double dDist2Start = sqrt(pow(m_MainPath.points[m_nTurnStartInd].x,2)+pow(m_MainPath.points[m_nTurnStartInd].y,2));
	if (dDist2Start <= 5.0)
	{
		m_bIsInTurn = true;
	}
	else
	{
		m_bIsInTurn = false;
		m_bIsBreak = false;
	}
	//弯道起点更新至车头位置
	int nNewStart_i = m_nTurnStartInd;
	for ( ;nNewStart_i < m_nTurnEndInd; nNewStart_i++)
	{
		double dDist = sqrt(pow(m_MainPath.points[nNewStart_i].x,2)+pow(m_MainPath.points[nNewStart_i].y,2));
		if (dDist >= 7.0)
		{
			break;
		}
	}
	m_nTurnStartInd = nNewStart_i;
	//弯道终点更新至3秒后的位置
	double dSpeed = sqrt(pow(m_DataPackage.ImuData.Ve,2) + pow(m_DataPackage.ImuData.Vn,2));
	double dPrePathLen = (m_Param.dCollisionPredictTime + 1.0) * (dSpeed + 2.0);
	int nNewEnd_i = m_nTurnStartInd;
	double dDistAcc = 0.0;
	for ( ;nNewEnd_i+1 < m_nTurnEndInd; nNewEnd_i++)
	{
		Point2d pt0 = m_MainPath.points[nNewEnd_i];
		Point2d pt1 = m_MainPath.points[nNewEnd_i+1];
		double dDist = sqrt(pow(pt0.x-pt1.x,2)+pow(pt0.y-pt1.y,2));
		dDistAcc += dDist;
		if (dDistAcc >= dPrePathLen)
		{
			break;
		}
	}
	m_nTurnEndInd = nNewEnd_i+1;

	NAVI_EVENT Ent;
	Ent.nMajorType = 201;
	for (unsigned int i = m_nTurnStartInd; i < m_nTurnEndInd - 1; i++)
	{
		Point3f pt_(m_MainPath.points[i].x, m_MainPath.points[i].y, 0);
		Ent.PointsSource.push_back(pt_);
	}
	Ent.nValue = m_bIsInTurn;
	Ent.nPlaceType = m_nTurnningInfo;
	m_DataPackage.NaviEvents.push_back(Ent);

	return 1;
}

int CLidarMappingOpr::GetCurLaneId()
{
	m_nCurLaneId = -1;
	m_nNearestPointOfCurLane = -1;
	m_dDistCar2NearLane = DBL_MAX;

	vector<int> VecNearPointId;
	vector<double> VecNearDist;
	for (unsigned int i = 0; i < m_DataPackage.Lanes.size(); i++)
	{
		int nNearId = -1;
		double dNearDist = DBL_MAX;
		LANE_INFO& LaneRef = m_DataPackage.Lanes[i];
		for (unsigned int j = 0; j < LaneRef.points.size(); j++)
		{
			double dDist = sqrt(pow(LaneRef.points[j].x,2) + pow(LaneRef.points[j].y,2));
			if (dDist < dNearDist)
			{
				dNearDist = dDist;
				nNearId = j;
			}
		}
		VecNearPointId.push_back(nNearId);
		VecNearDist.push_back(dNearDist);
	}

	double dCurLaneDist = DBL_MAX;
	for (unsigned int i = 0; i < VecNearDist.size(); i++)
	{
		if (VecNearDist[i] < dCurLaneDist)
		{
			dCurLaneDist = VecNearDist[i];
			m_nCurLaneId = i;
		}
	}

	if (m_nCurLaneId == -1)
	{
		return 0;
	}

	m_nNearestPointOfCurLane = VecNearPointId[m_nCurLaneId];
	m_dDistCar2NearLane = VecNearDist[m_nCurLaneId];

	NAVI_EVENT Event;
	Event.nMajorType = -10;
	Event.PointsSource.push_back(Point3f(m_DataPackage.Lanes[m_nCurLaneId].points[m_nNearestPointOfCurLane].x,
		m_DataPackage.Lanes[m_nCurLaneId].points[m_nNearestPointOfCurLane].y, 0.f));
	m_DataPackage.NaviEvents.push_back(Event);

	return 1;
}

void CLidarMappingOpr::OtherWork()
{
	//变道路径快要到头时，提前缩短50米
	for (unsigned int i = 0; i < m_DataPackage.Lanes.size(); i++)
	{
		LANE_INFO& LaneRef = m_DataPackage.Lanes[i];
		if (LaneRef.nType != 0)
		{
			double dPathLen = sqrt(pow(LaneRef.points.back().x,2)+pow(LaneRef.points.back().y,2));
			if (dPathLen <= 90.0)
			{
				double dPathRemainLen = dPathLen - 30.0;
				vector<Point2d>::reverse_iterator rit;
				for (rit = LaneRef.points.rbegin(); rit != LaneRef.points.rend(); )
				{
					double dDist = sqrt(pow(rit->x,2)+pow(rit->y,2));
					if (dDist > dPathRemainLen)
					{
						rit = vector<Point2d>::reverse_iterator(LaneRef.points.erase((++rit).base()));
					}
					else
					{
						++rit;
					}
				}
			}
		}
	}
}

int CLidarMappingOpr::CollisionDetect(DATA_PACKAGE& Data)
{
	LANE_INFO MainPath;
	for (unsigned int i = 0; i < Data.Lanes.size(); i++)
	{
		if (Data.Lanes[i].nType == 0)
		{
			MainPath = Data.Lanes[i];
			break;
		}
	}

// 	int nNearInd = -1;
// 	double dNearDist = 1000000.0;
// 	for (unsigned int i = 0; i < MainPath.points.size(); i++)
// 	{
// 		double dDist = sqrt(pow(MainPath.points[i].x,2)+pow(MainPath.points[i].y,2));
// 		if (dDist < dNearDist)
// 		{
// 			dNearDist = dDist;
// 			nNearInd = i;
// 		}
// 	}
// 	if (nNearInd < 0)
// 	{
// 		return 0;
// 	}

	vector<double> VecCurve;
	for (unsigned i = 0; i+2 < MainPath.points.size(); i++)
	{
		Point2f pt0 = MainPath.points[i];
		Point2f pt1 = MainPath.points[i+1];
		Point2f pt2 = MainPath.points[i+2];
		double dHeading0 = atan2((pt1.y - pt0.y), (pt1.x - pt0.x));
		double dHeading1 = atan2((pt2.y - pt1.y), (pt2.x - pt1.x));
		double dDist0 = sqrt(pow((pt1.y - pt0.y),2) + pow((pt1.x - pt0.x),2));
		double dDist1 = sqrt(pow((pt2.y - pt1.y),2) + pow((pt2.x - pt1.x),2));
		double dCurve = abs(dHeading1-dHeading0)/(dDist0+dDist1);
		VecCurve.push_back(dCurve);
	}

	vector<double> VecCurveFilter;
	int nFilterLen = 5;
	for (unsigned int i = 0; i+nFilterLen < VecCurve.size(); i++)
	{
		double dDist = 0.0;
		for (unsigned int j = 0; j < nFilterLen; j++)
		{
			dDist += VecCurve[i+j];
		}
		dDist/=nFilterLen;
		VecCurveFilter.push_back(dDist);
	}

	int nCurveBeginInd = -1;
	for (unsigned int i = 0; i < VecCurveFilter.size(); i++)
	{
		if (VecCurveFilter[i] >= 0.03)
		{
			nCurveBeginInd = i;
			break;
		}
	}
	if (nCurveBeginInd < 0)
	{
		return 0;
	}

	int nStraightEndInd = -1;
	for (int i = nCurveBeginInd; i >= 0; i--)
	{
		if (VecCurveFilter[i] <= 0.01)
		{
			nStraightEndInd = i;
			break;
		}
	}
	if (nStraightEndInd < 0)
	{
		return 0;
	}

	Point2f ptCurve = MainPath.points[nCurveBeginInd];
	if (ptCurve.y >= 15.0 || ptCurve.y <= 0.0)
	{
		return 0;
	}
	NAVI_EVENT evt;
	evt.nMajorType = 201;
	Point3f pt;
	pt.x = ptCurve.x;
	pt.y = ptCurve.y;
	pt.z = 0.f;
	evt.PointsSource.push_back(pt);
	evt.nValue = 5;
	Data.NaviEvents.push_back(evt);

	double dMeetTimeMax = -5.0;
	for (unsigned int i = 0; i < Data.ObjectList.objects.size(); i++)
	{
		SENSOR_OBJECT& ObjT = Data.ObjectList.objects[i];
 		if (ObjT.obj_loc_x >= -2000.0 || ObjT.obj_loc_x <= -6000.0)
 			continue;
		double dMeetTime = ObjT.obj_loc_y/ObjT.v_y_abs;
		if (dMeetTime >= dMeetTimeMax && dMeetTime <= 0)
		{
			NAVI_EVENT evt;
			evt.nMajorType = 200;
			Point3f pt;
			pt.x = ptCurve.x;
			pt.y = ptCurve.y;
			pt.z = 0.f;
			evt.PointsSource.push_back(pt);
			evt.nValue = 5;
			Data.NaviEvents.push_back(evt);
			break;
		}
	}

	return 1;
}

void CLidarMappingOpr::SetCallBack(lpLidarDataRecFunc hLidarDataCallBack)
{
	m_LidarDataRecFunc = hLidarDataCallBack;
}

long CLidarMappingOpr::Start()
{
// 	((CGpsManager*)m_pGpsMgr)->Start();
// 	((CGpsManager*)m_pGpsMgr)->EnablePrintLog(true);
	((CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE>*)m_pLcmLidarCloud)->SetCallBack(PackDataCallBack,this);
	((CLcmRevicer<LCM_SENSOR_FUSION_PACKAGE>*)m_pLcmLidarCloud)->Start();

	return 1;
}

Mat CLidarMappingOpr::DrawLidarPoints()
{
	list<Mat>::const_iterator its = m_ListGroundPt.begin();
	list<Mat>::const_iterator ite = m_ListGroundPt.end();

	vector<Mat> VecCloud;
	Mat PoseRef = m_ListPose.front();
	for (;its != ite;its++)
	{
		Mat CloudPt = *its;
		//		cout << PoseRef << endl;
		Mat aaa = PoseRef.inv() * CloudPt;
		//		cout << CloudPt << endl;
		VecCloud.push_back(aaa);
	}


	Mat img0 = Gridding(VecCloud[0].t());

	Mat img0_;
	dilate(img0,img0_,Mat::ones(5,5,CV_8U),cvPoint(2,2));
	Mat img;
	img0_.copyTo(img);
	//xinzi1113
	for (int i = 1; i < VecCloud.size(); i++)
	{
		Mat img1 = Gridding(VecCloud[i].t());
		Mat img1_;
		dilate(img1,img1_,Mat::ones(5,5,CV_8U),cvPoint(2,2));
		img = img + img1_;
	}

	return (img);
}

Point2d CLidarMappingOpr::CarXY2ImageUV(Point2d CarXY)
{
	Point2d out;
	double X = CarXY.x;
	double Y = CarXY.y;
	X = m_nGridSide + X;
	Y = m_nGridFront - Y;
	out.x = floor(X/m_nGridSize);
	out.y = floor(Y/m_nGridSize);

	return out;
}

Mat CLidarMappingOpr::Gridding(Mat CloudPt)
{
	Mat Lidar1 = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8U);
	for (int i = 0; i < CloudPt.rows; i++)
	{
		double X = CloudPt.at<double>(i,0);
		double Z = CloudPt.at<double>(i,1);
		if (X>=-1*m_nGridSide && X<=m_nGridSide && Z<=m_nGridFront && Z>=-1.f*m_nGridBack)
		{
			X += m_nGridSide;
			Z += m_nGridBack;
			double XX = floor(X/m_nGridSize);
			double ZZ = floor(Z/m_nGridSize);
			Lidar1.at<UCHAR>(ZZ,XX) = 1;
		}
	}
	flip(Lidar1,Lidar1,0);
	//	cvFlip(&CvMat(Lidar1),NULL,0);
	//	Lidar = Grey2FateColor(Lidar1);

	return Lidar1;
}


Mat CLidarMappingOpr::Grey2FateColor(Mat& mat_in)
{
	Mat temp(mat_in.rows,mat_in.cols,CV_8UC1);
	cvNormalize( &CvMat(mat_in), &CvMat(temp), 255, 0, CV_MINMAX );

	CvMat* red = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* green = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* blue = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* mask = cvCreateMat(temp.rows, temp.cols, CV_8U);

	Mat color_mat(temp.rows,temp.cols,CV_8UC3);
	// 计算各彩色通道的像素值
	cvSubRS(&CvMat(temp), cvScalar(255), blue);	// blue(I) = 255 - gray(I)
	cvCopy(&CvMat(temp), red);			// red(I) = gray(I)
	cvCopy(&CvMat(temp), green);			// green(I) = gray(I),if gray(I) < 128
	cvCmpS(green, 128, mask, CV_CMP_GE );	// green(I) = 255 - gray(I), if gray(I) >= 128
	cvSubRS(green, cvScalar(255), green, mask);
	cvConvertScale(green, green, 2.0, 0.0);

	// 合成伪彩色图
	cvMerge(blue, green, red, NULL, &CvMat(color_mat));

	cvReleaseMat( &red );
	cvReleaseMat( &green );
	cvReleaseMat( &blue );
	cvReleaseMat( &mask );

	return color_mat;
}

void CLidarMappingOpr::saveXYZ(const char* filename, const Mat& mat)
{
	char szLine[20000] = {0};
	char szWord[100] = {0};
	FILE* fp = fopen(filename, "wt");
	for(int x = 0; x < mat.rows; x++)
	{
		strcpy(szLine,"");
		for(int y = 0; y < mat.cols; y++)
		{
			double point = cvGetReal2D(&CvMat(mat),x,y);
			sprintf(szWord,"%.2f,",point);
			strcat(szLine,szWord);
		}
		fprintf(fp,"%s\n",szLine);
	}
	fclose(fp);
}

Mat Grey2FateColor(Mat& mat_in)
{
	Mat temp(mat_in.rows,mat_in.cols,CV_8UC1);
	cvNormalize( &CvMat(mat_in), &CvMat(temp), 255, 0, CV_MINMAX );

	CvMat* red = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* green = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* blue = cvCreateMat(temp.rows, temp.cols, CV_8U);
	CvMat* mask = cvCreateMat(temp.rows, temp.cols, CV_8U);

	Mat color_mat(temp.rows,temp.cols,CV_8UC3);
	// 计算各彩色通道的像素值
	cvSubRS(&CvMat(temp), cvScalar(255), blue);	// blue(I) = 255 - gray(I)
	cvCopy(&CvMat(temp), red);			// red(I) = gray(I)
	cvCopy(&CvMat(temp), green);			// green(I) = gray(I),if gray(I) < 128
	cvCmpS(green, 128, mask, CV_CMP_GE );	// green(I) = 255 - gray(I), if gray(I) >= 128
	cvSubRS(green, cvScalar(255), green, mask);
	cvConvertScale(green, green, 2.0, 0.0);

	// 合成伪彩色图
	cvMerge(blue, green, red, NULL, &CvMat(color_mat));

	cvReleaseMat( &red );
	cvReleaseMat( &green );
	cvReleaseMat( &blue );
	cvReleaseMat( &mask );

	return color_mat;
}

long CLidarMappingOpr::GetData(DATA_PACKAGE* pData)
{
	if (m_bIsOldData)
	{
		return -1;
	}

	EnterCriticalSection(&m_cs);
	*pData = m_DataPackage;
	m_bIsOldData = true;
	LeaveCriticalSection(&m_cs);

	return m_nIndOut;
}

void CLidarMappingOpr::DrawTrack(Mat& img, vector<Point2d>& Track)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC3);
		img.col((m_nGridSide/m_nGridSize)+1) = cvScalar(255,255,255);
		img.row((m_nGridFront/m_nGridSize)+1) = cvScalar(255,255,255);
	}
	for (int i = 0; i < Track.size(); i++)
	{
		double X = m_nGridSide + Track[i].x*1000;
		double Z = m_nGridFront - Track[i].y*1000;
		double XX = floor(X/m_nGridSize);
		double ZZ = floor(Z/m_nGridSize);
		if (ZZ>=0 && ZZ < img.rows && XX>=0 && XX < img.cols)
		{
			if (img.channels() == 3)
			{
				Vec<uchar,3> color_(255,255,255);
				img.at<Vec<uchar,3>>(ZZ,XX) = color_;
			}
			else
			{
				img.at<uchar>(ZZ,XX) = 255;
			}
		}
	}
}

void CLidarMappingOpr::DrawLine(Mat& img, vector<vector<Point2d>>& Lines)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC3);
		img.col((m_nGridSide/m_nGridSize)+1) = cvScalar(255,255,255);
		img.row((m_nGridFront/m_nGridSize)+1) = cvScalar(255,255,255);
	}
	for (int i = 0; i < Lines.size(); i++)
	{
		for (int j = 0; j < Lines[i].size(); j++)
		{
			Point2d& ptT = Lines[i][j];
			double X = m_nGridSide + ptT.x*1000;
			double Z = m_nGridFront - ptT.y*1000;
			double XX = floor(X/m_nGridSize);
			double ZZ = floor(Z/m_nGridSize);
			if (ZZ>=0 && ZZ < img.rows && XX>=0 && XX < img.cols)
			{
				if (img.channels() == 3)
				{
					Vec<uchar,3> color_(0,0,255);
					img.at<Vec<uchar,3>>(ZZ,XX) = color_;
				}
				else
				{
					img.at<uchar>(ZZ,XX) = 255;
				}
			}
		}
	}

}

void CLidarMappingOpr::DrawNavi(Mat& img, DATA_PACKAGE& Data)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC3);
		img.col((m_nGridSide/m_nGridSize)+1) = cvScalar(255,255,255);
		img.row((m_nGridFront/m_nGridSize)+1) = cvScalar(255,255,255);
	}

	Scalar sc = CV_RGB(0.0, 40.0 ,0.0);
	Mat PathGreen((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC3,sc);
	cv::add(img, PathGreen, img, Data.LidarMapping, CV_8UC3);

	//车道
	for (int i = 0; i < Data.Lanes.size(); i++)
	{
		LANE_INFO& Lane = Data.Lanes[i];
		Vec<uchar,3> color(255,255,255);
		CvScalar colorT;
		if (i == Data.nCurLaneId)
		{
			color = Vec<uchar,3>(0,255,0);//green
			colorT = CV_RGB(0,255,0);
		}
		else
		{
			color = Vec<uchar,3>(0,0,255);//blue
			colorT = CV_RGB(0,0,255);
		}
		for (int j = 0; j < Lane.points.size(); j++)
		{
			Point2d& pt = Lane.points[j];
			double X = m_nGridSide + pt.x*1000;
			double Z = m_nGridFront - pt.y*1000;
			double XX = floor(X/m_nGridSize);
			double ZZ = floor(Z/m_nGridSize);
			if (ZZ>=0 && ZZ < img.rows && XX>=0 && XX < img.cols)
			{
				circle(img,Point(XX,ZZ),1,colorT,-1);
			}
		}
	}

	//车道线
	for (int i = 0; i < Data.Lines.size(); i++)
	{
		LINE_INFO& Line = Data.Lines[i];
		for (int j = 0; j < Line.points.size(); j++)
		{
			Point2d& pt = Line.points[j];
			double X = m_nGridSide + pt.x*1000;
			double Z = m_nGridFront - pt.y*1000;
			double XX = floor(X/m_nGridSize);
			double ZZ = floor(Z/m_nGridSize);
			if (ZZ>=0 && ZZ < img.rows && XX>=0 && XX < img.cols)
			{
				if (img.channels() == 3)
				{
					Vec<uchar,3> color_(255,255,255);
					img.at<Vec<uchar,3>>(ZZ,XX) = color_;
				}
				else
				{
					img.at<uchar>(ZZ,XX) = 255;
				}
			}
		}
	}

	//红绿灯
	for (int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == 6)
		{
			CvScalar TLcolor;
			printf("Traffic Light State: %d\n", Data.NaviEvents[i].nValue);
			if(Data.NaviEvents[i].nValue == 0)
				TLcolor = CV_RGB(0,255,0);
			else if(Data.NaviEvents[i].nValue == 1)
				TLcolor = CV_RGB(255,255,0);
			else if(Data.NaviEvents[i].nValue == 2)
				TLcolor = CV_RGB(255,0,0);
			else
				TLcolor = CV_RGB(0,0,255);
			Point3f& pt = Data.NaviEvents[i].PointsSource[0];
			double X = m_nGridSide + pt.x*1000;
			double Z = m_nGridFront - pt.y*1000;
			double XX = floor(X/m_nGridSize);
			double ZZ = floor(Z/m_nGridSize);
			if (ZZ>=0 && ZZ < img.rows && XX>=0 && XX < img.cols)
			{
				if (img.channels() == 3)
				{
					circle(img,Point(XX,ZZ),6,CV_RGB(255,0,0),-1);
					circle(img,Point(XX+12,ZZ),6,CV_RGB(255,255,0),-1);
					circle(img,Point(XX+24,ZZ),6,CV_RGB(0,255,0),-1);
					circle(img,Point(XX+48,ZZ),6,TLcolor,-1);
//					Vec<uchar,3> color(255,0,0);
//					img.at<Vec<uchar,3>>(ZZ,XX) = color;
				}
				else
				{
//					img.at<uchar>(ZZ,XX) = 255;
					circle(img,Point(XX,ZZ),6,CV_RGB(255,255,255),-1);
				}
			}

			pt = Data.NaviEvents[i].PointsDest[0];
			X = m_nGridSide + pt.x*1000;
			Z = m_nGridFront - pt.y*1000;
			XX = floor(X/m_nGridSize);
			ZZ = floor(Z/m_nGridSize);
			if (ZZ>=0 && ZZ < img.rows && XX>=0 && XX < img.cols)
			{
				if (img.channels() == 3)
				{
					Vec<uchar,3> color(255,255,0);
					circle(img,Point(XX,ZZ),6,CV_RGB(255,255,0),-1);
//					img.at<Vec<uchar,3>>(ZZ,XX) = color;
				}
				else
				{
					circle(img,Point(XX,ZZ),6,CV_RGB(255,255,255),-1);
//					img.at<uchar>(ZZ,XX) = 255;
				}
			}
		}
	}

	//禁止变道
	for (unsigned int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == 100)
		{
			putText(img,"Lane change forbid!",Point(0,50),0,0.5,CV_RGB(255,0,0),2);
		}
	}

	//弯道减速信号
	bool bIsTurnSlow = false;
	for (unsigned int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == 200)
		{
			bIsTurnSlow = true;
			putText(img,"Curve slow!",Point(0,50),0,0.5,CV_RGB(255,0,0),2);
// 			Point3f pt = Data.NaviEvents[i].PointsSource[0];
// 			double X = m_nGridSide + pt.x*1000;
// 			double Z = m_nGridFront - pt.y*1000;
// 			double XX = floor(X/m_nGridSize);
// 			double ZZ = floor(Z/m_nGridSize);
// 			circle(img,Point(XX,ZZ),2,CV_RGB(255,0,0),-1);
		}
	}
	//
	for (unsigned int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == 201)
		{
			CvScalar color_;
			if (!bIsTurnSlow)
				color_ = CV_RGB(255,255,0);
			else
				color_ = CV_RGB(255,0,0);
			for (unsigned int j = 0; j < Data.NaviEvents[i].PointsSource.size(); j++)
			{
				Point3f pt = Data.NaviEvents[i].PointsSource[j];
				double X = m_nGridSide + pt.x*1000;
				double Z = m_nGridFront - pt.y*1000;
				double XX = floor(X/m_nGridSize);
				double ZZ = floor(Z/m_nGridSize);
				circle(img,Point(XX,ZZ),2,color_,-1);
			}
			string szDisp = "Turn Unkown";
			if (Data.NaviEvents[i].nPlaceType == 1)
				szDisp = "Turn left";
			if (Data.NaviEvents[i].nPlaceType == 2)
				szDisp = "Turn right";
			if (Data.NaviEvents[i].nPlaceType == 3)
				szDisp = "Straight";
			if (Data.NaviEvents[i].nValue != 0)
				szDisp += "  Turning";
			putText(img,szDisp,Point(0,100),0,0.5,CV_RGB(255,0,0),2);
		}
	}

	//减速带
	for (unsigned int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == 8)
		{
			Point3d pt = Data.NaviEvents[i].PointsDest[0];
			double X = m_nGridSide + pt.x*1000;
			double Z = m_nGridFront - pt.y*1000;
			double XX = floor(X/m_nGridSize);
			double ZZ = floor(Z/m_nGridSize);
			circle(img,Point(XX,ZZ),3,CV_RGB(255,255,255),-1);
		}
	}

	//禁止预测
	for (unsigned int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == 30 || Data.NaviEvents[i].nMajorType == 31)
		{
			Point3d pt = Data.NaviEvents[i].PointsDest[0];
			double X = m_nGridSide + pt.x*1000;
			double Z = m_nGridFront - pt.y*1000;
			double XX = floor(X/m_nGridSize);
			double ZZ = floor(Z/m_nGridSize);
			circle(img,Point(XX,ZZ),3,CV_RGB(255,255,255),-1);
			char szDisp[256] = {0};
			sprintf(szDisp, "Predict state: %d", Data.NaviEvents[i].nMajorType);
			putText(img,szDisp,Point(0,150),0,0.5,CV_RGB(255,255,255),2);
		}
	}

	//当前车道指示
	for (unsigned int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == -10)
		{
			Point3d pt = Data.NaviEvents[i].PointsSource[0];
			double X = m_nGridSide + pt.x*1000;
			double Z = m_nGridFront - pt.y*1000;
			double XX = floor(X/m_nGridSize);
			double ZZ = floor(Z/m_nGridSize);
			circle(img,Point(XX,ZZ),3,CV_RGB(255,0,0),-1);
		}
	}

	//路口信息
	for (unsigned int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == 11)
		{
			Point3d pt0 = Data.NaviEvents[i].PointsSource[0];
			double X = m_nGridSide + pt0.x*1000;
			double Z = m_nGridFront - pt0.y*1000;
			Point pt0_(floor(X/m_nGridSize), floor(Z/m_nGridSize));
			Point3d pt1 = Data.NaviEvents[i].PointsDest[0];
			X = m_nGridSide + pt1.x*1000;
			Z = m_nGridFront - pt1.y*1000;
			Point pt1_(floor(X/m_nGridSize), floor(Z/m_nGridSize));
			line(img, pt0_, pt1_, CV_RGB(255,255,0), 1);
		}
	}

	for (unsigned int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == 20)
		{
			Point3d pt = Data.NaviEvents[i].PointsSource[0];
			double X = m_nGridSide + pt.x*1000;
			double Z = m_nGridFront - pt.y*1000;
			double XX = floor(X/m_nGridSize);
			double ZZ = floor(Z/m_nGridSize);
			circle(img,Point(XX,ZZ),3,CV_RGB(255,255,255),-1);
		}
	}

	//Mobileye车道线信息
	for (unsigned int i = 0; i < Data.NaviEvents.size(); i++)
	{
		if (Data.NaviEvents[i].nMajorType == -22)
		{
			for (unsigned int j = 0; j+1 < Data.NaviEvents[i].PointsSource.size(); j++)
			{
				Point3f pt0 = Data.NaviEvents[i].PointsSource[j];
				double X0 = m_nGridSide + pt0.x*1000;
				double Z0 = m_nGridFront - pt0.y*1000;
				double XX0 = floor(X0/m_nGridSize);
				double ZZ0 = floor(Z0/m_nGridSize);
				Point3f pt1 = Data.NaviEvents[i].PointsSource[j+1];
				double X1 = m_nGridSide + pt1.x*1000;
				double Z1 = m_nGridFront - pt1.y*1000;
				double XX1 = floor(X1/m_nGridSize);
				double ZZ1 = floor(Z1/m_nGridSize);
				line(img, Point(XX0,ZZ0), Point(XX1,ZZ1), CV_RGB(255,0,0), 1);
			}
		}
	}

	if (Data.ImuData.Status != 2)
	{
		char szDispGpsState[256] = {0};
		sprintf(szDispGpsState, "Gps state:%d", Data.ImuData.Status);
		putText(img,szDispGpsState,Point(0,200),0,0.5,CV_RGB(255,0,0),2);
	}
	
}

void CLidarMappingOpr::DrawRadar(Mat& img, DATA_PACKAGE& Data)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC3);
		img.col((m_nGridSide/m_nGridSize)+1) = cvScalar(255,255,255);
		img.row((m_nGridFront/m_nGridSize)+1) = cvScalar(255,255,255);
	}
	for (int i = 0; i < Data.ObjectList.objects.size(); i++)
	{
		SENSOR_OBJECT& Obj = Data.ObjectList.objects[i];
		double X = m_nGridSide + Obj.obj_loc_x*1000;
		double Z = m_nGridFront - Obj.obj_loc_y*1000;
		double XX = floor(X/m_nGridSize);
		double ZZ = floor(Z/m_nGridSize);
		if (ZZ>=0 && ZZ < img.rows && XX>=0 && XX < img.cols)
		{
			if (img.channels() == 3)
			{
				Vec<uchar,3> color_(255,255,255);
//				img.at<Vec<uchar,3>>(ZZ,XX) = color_;
				circle(img,Point(XX,ZZ),3,CV_RGB(255,255,255),1);
			}
			else
			{
//				img.at<uchar>(ZZ,XX) = 255;
				circle(img,Point(XX,ZZ),3,CV_RGB(255,255,255),1);
			}
		}
	}
}

void CLidarMappingOpr::DrawCar(Mat& img, int nFront, int nRear, int nSide)
{
	if (img.data == NULL)
	{
		img = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC3);
		img.col((m_nGridSide/m_nGridSize)+1) = CV_RGB(90,90,90);
		img.row((m_nGridFront/m_nGridSize)+1) = CV_RGB(90,90,90);
	}
	img.col((m_nGridSide/m_nGridSize)+1) = CV_RGB(90,90,90);
	img.row((m_nGridFront/m_nGridSize)+1) = CV_RGB(90,90,90);

	Point2f Coners[4];
	Coners[0] = Point2f(-1.f*nSide,nFront);
	Coners[1] = Point2f(nSide,nFront);
	Coners[2] = Point2f(nSide,-1.f*nRear);
	Coners[3] = Point2f(-1.f*nSide,-1.f*nRear);

	Point2f Coners_[4];
	for (int i = 0; i < 4; i++)
	{
		Coners_[i] = CarXY2ImageUV(Coners[i]);
	}
	for (int i = 0; i < 4; i++)
	{
		line(img, Coners_[i], Coners_[(i+1)%4], CV_RGB(90,90,90),1);
	}
}

Point2i CLidarMappingOpr::Car2Image(Point2d PtCar)
{
	return Point2i((m_nGridSide+(PtCar.x + m_nGridSize/2))/m_nGridSize,(m_nGridFront-(PtCar.y + m_nGridSize/2))/m_nGridSize);
}

void CLidarMappingOpr::DrawIbeoObjects(Mat& img, DATA_PACKAGE& Data, Scalar color)
{
	SENSOR_OBJECT_LIST* pIbeoObjects = &(Data.ObjectList);
	if (img.data == NULL)
	{
		img = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_8UC3);
		img.col((m_nGridSide/m_nGridSize)+1) = cvScalar(255,255,255);
		img.row((m_nGridFront/m_nGridSize)+1) = cvScalar(255,255,255);
	}

	for (int i = 0; i < pIbeoObjects->objects.size(); i++)
	{
		SENSOR_OBJECT& ObjRef = pIbeoObjects->objects[i];

		if (ObjRef.predict_age > 1)
		{
			continue;
		}

		//Create color
// 		bool isRandMatchColor = color == Scalar::all(-1);
// 		Scalar color_ = isRandMatchColor ? m_ColorList[ObjRef.obj_id%1000] : color;
		Scalar color_;
		if (ObjRef.lane_loc == 0)
		{
			if (ObjRef.state >= 0)
			{
				color_ = CV_RGB(255, 0, 0);
			}
			else
			{
				color_ = CV_RGB(122, 122, 0);
			}
		}
		else
		{
			color_ = CV_RGB(177, 177, 0);
		}
		if (ObjRef.from == 9 ||ObjRef.from == 10 ||ObjRef.from == 11 || ObjRef.from == 20)
		{
			color_ = CV_RGB(0,122,122);
		}
		if (ObjRef.from == 12 ||ObjRef.from == 13)
		{
			color_ = CV_RGB(122,0,122);
		}
		////Draw counter points
		vector< Point2d >& ContPointsRef = ObjRef.contour;
		vector<double> VecXX,VecYY;
		for (int j = 0; j < ContPointsRef.size(); j++)
		{
			double X = ContPointsRef[j].x;
			double Y = ContPointsRef[j].y;
			//           if (X>=-1*nGridSide && X<=nGridSide && Y<=nGridFront && Y>=-1.f*nGridBack)
			X = m_nGridSide + X;
			Y = m_nGridFront - Y;
			double XX = floor(X/m_nGridSize);
			double YY = floor(Y/m_nGridSize);
			circle(img,cvPoint(XX,YY),3,color_,-1);
			VecXX.push_back(XX);
			VecYY.push_back(YY);
		}

		//Draw counter lines
 		for (int j = 1; j < VecXX.size(); j++)
 		{
 			line(img, cvPoint(VecXX[j-1],VecYY[j-1]), cvPoint(VecXX[j],VecYY[j]), color_, 1);
 		}

		//Draw objects rect
//		if (ObjRef.lane_loc == 0)
		{
// 			double X = ObjRef.obj_loc_x;
// 			double Y = ObjRef.obj_loc_y;
// 			Point2f center(X, Y);
// 			Point2f center_((center.x + m_nGridSide) / m_nGridSize, (m_nGridFront - center.y) / m_nGridSize);
// 			Point2f size(ObjRef.obj_size_x, ObjRef.obj_size_y);
// 			Point2f size_(size.x / m_nGridSize, size.y / m_nGridSize);
// 			RotatedRect rectOrient(center_, size_, -1.f*ObjRef.obj_heading);
// 			Point2f vertices_[4];
// 			rectOrient.points(vertices_);
// 			for (int j = 0; j < 4; j++)
// 			{
// 				line(img, vertices_[j], vertices_[(j+1)%4], color_,1);
// 			}

			Point2f vertices_[4];
			for (int j = 0 ; j < 4; j++)
			{
				vertices_[j] = Point2f((ObjRef.rotate_rect[j].x + m_nGridSide) / m_nGridSize, (m_nGridFront - ObjRef.rotate_rect[j].y) / m_nGridSize);
			}
			for (int j = 0; j < 4; j++)
			{
			 	line(img, vertices_[j], vertices_[(j+1)%4], color_,1);
			}
		}

		//Draw relative velocity
//		if (ObjRef.lane_loc == 0)
		{
			double dTimeDelta = 3.0;
			double dXDelta = dTimeDelta * ObjRef.v_x;
			double dYDelta = dTimeDelta * ObjRef.v_y;
// 			double dXDelta = dTimeDelta * ObjRef.v_x_abs;
// 			double dYDelta = dTimeDelta * ObjRef.v_y_abs;
			Point2d ptStart(ObjRef.obj_loc_x, ObjRef.obj_loc_y);
			Point2d ptEnd(ptStart.x + dXDelta, ptStart.y + dYDelta);
			Point2i ptStart_ = Car2Image(ptStart);
			Point2i ptEnd_ = Car2Image(ptEnd);
//			cv::line(img,ptStart_,ptEnd_,CV_RGB(0,255,255));
		}

		//Draw classification
		if (ObjRef.lane_loc == 0)
		{
			char szClassify[256] = {0};
			if (ObjRef.classify == 0)
				strcpy_s(szClassify,"UnKn");
			else if (ObjRef.classify == 1)
				strcpy_s(szClassify,"Small");
			else if (ObjRef.classify == 2)
				strcpy_s(szClassify,"Bis");
			else if (ObjRef.classify == 3)
				strcpy_s(szClassify,"Ped");
			else if (ObjRef.classify == 4)
				strcpy_s(szClassify,"Bike");
			else if (ObjRef.classify == 5)
				strcpy_s(szClassify,"Car");
			else if (ObjRef.classify == 6)
				strcpy_s(szClassify,"Truck");
			else if (ObjRef.classify == 12)
				strcpy_s(szClassify,"Under");
			else
				strcpy_s(szClassify,"UnKn");
			Point2d ptRef = ObjRef.ref_point;
			Point2i ptCar = Car2Image(ptRef);
//			putText(img,szClassify,ptCar,0,0.8,color_);
		}
		
		Point2d ptStart(ObjRef.obj_loc_x, ObjRef.obj_loc_y);
		Point2i ptStart_ = Car2Image(ptStart);
		cv::circle(img, ptStart_, 2, CV_RGB(255,255,255), -1);

// 		Point2d ptStartRef(ObjRef.ref_point.x, ObjRef.ref_point.y);
// 		Point2i ptStartRef_ = Car2Image(ptStart);
// 		cv::circle(img, ptStartRef_, 2, CV_RGB(0,255,0), -1);


// 		Point2d ptRef = ObjRef.ref_point;
// 		Point2i ptCar = Car2Image(ptRef);
// 		char szPreAge[256] = {0};
// 		sprintf(szPreAge,"%d",ObjRef.predict_age);
// 		putText(img,szPreAge,ptCar,0,0.8,color_);

// 		Point2d ptRef = ObjRef.ref_point;
// 		Point2i ptCar = Car2Image(ptRef);
// 		char szPreAge[256] = {0};
// 		sprintf(szPreAge,"%.2f",ObjRef.ext_measurement);
// 		putText(img,szPreAge,ptCar,0,0.8,color_);


		
		if (ObjRef.is_abs_track_v_available)
		{
			//Draw history trace
			for (unsigned int j = 1; j < ObjRef.HistoryTrace.size(); j++)
			{
				Point2d ptStart = ObjRef.HistoryTrace[j-1];
				Point2i ptStart_ = Car2Image(ptStart);
				Point2d ptEnd = ObjRef.HistoryTrace[j];
				Point2i ptEnd_ = Car2Image(ptEnd);
				line(img,ptStart_,ptEnd_,CV_RGB(255,255,255));
			}
			//Draw predict vel
			double dTimeDelta = m_Param.dCollisionPredictTime;
			double dXDelta = dTimeDelta * ObjRef.v_x_abs_track;
			double dYDelta = dTimeDelta * ObjRef.v_y_abs_track;
			Point2d ptStart(ObjRef.obj_loc_x, ObjRef.obj_loc_y);
			Point2d ptEnd(ptStart.x + dXDelta, ptStart.y + dYDelta);
			Point2i ptStart_ = Car2Image(ptStart);
			Point2i ptEnd_ = Car2Image(ptEnd);
			arrowedLine(img,ptStart_,ptEnd_,CV_RGB(0,255,255));
		}
		if (ObjRef.is_will_collision)
		{
			//Draw predict vel
			double dTimeDelta = m_Param.dCollisionPredictTime;
			double dXDelta = dTimeDelta * ObjRef.v_x_abs_track;
			double dYDelta = dTimeDelta * ObjRef.v_y_abs_track;
			Point2d ptStart(ObjRef.obj_loc_x, ObjRef.obj_loc_y);
			Point2d ptEnd(ptStart.x + dXDelta, ptStart.y + dYDelta);
			Point2i ptStart_ = Car2Image(ptStart);
			Point2i ptEnd_ = Car2Image(ptEnd);
			arrowedLine(img,ptStart_,ptEnd_,CV_RGB(255,0,0));
		}
 	}
}

void CLidarMappingOpr::SetScale(int nFront, int nRear, int nSide, int nGridSize/* = 100*/)
{
	EnterCriticalSection(&m_cs);
	m_nGridFront = nFront;
	m_nGridSide = nSide;
	m_nGridBack = nRear;
	m_nGridSize = nGridSize;
	LeaveCriticalSection(&m_cs);
}

int CLidarMappingOpr::FilterObj(void* pPackage)
{
	DATA_PACKAGE* pPack = (DATA_PACKAGE*) pPackage;

	vector<SENSOR_OBJECT>::iterator it;
	for (it = pPack->ObjectList.objects.begin(); it != pPack->ObjectList.objects.end(); )
	{
		if (it->lane_loc == -100 || it->predict_age > 0)
		{
			it = pPack->ObjectList.objects.erase(it);
		}
		else
		{
			++it;
		}
	}

	return 1;
}

int CLidarMappingOpr::LinkObj2Path(void* pPackage)
{
	DATA_PACKAGE* pPack = (DATA_PACKAGE*) pPackage;

	Mat imgPath = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_32S);
	LANE_INFO MainPath;
	for (int i = 0; i < pPack->Lanes.size(); i++)
	{
		if (i == pPack->nCurLaneId)
		{
			MainPath = pPack->Lanes[i];
			break;
		}
	}
	vector<Point2i> VecPtImg;
	for (int i = 0; i < MainPath.points.size(); i++)
	{
		Point2d pt(MainPath.points[i].x*1000.0, MainPath.points[i].y*1000.0);
		Point2i ptImg = Car2Image(pt);
		VecPtImg.push_back(ptImg);
	}
	Point2i* pPt = VecPtImg.data();
	int PtLen = VecPtImg.size();
	int nWidth = 16000.0/m_nGridSize;
	polylines(imgPath,(const Point2i**)&pPt,(const int*)&PtLen,1,false,Scalar(1,1,1),nWidth);

	Mat imgObj = Mat::zeros((m_nGridFront+m_nGridBack)/m_nGridSize+1,m_nGridSide*2/m_nGridSize+1,CV_32S);
	for (int i = 0; i < pPack->ObjectList.objects.size(); i++)
	{
		SENSOR_OBJECT& OjbT = pPack->ObjectList.objects[i];
		for (int j = 0; j < OjbT.contour.size(); j++)
		{
			Point2d& ptT = OjbT.contour[j];
			Point2d ptCar(ptT.x, ptT.y);
			Point2i ptImg = Car2Image(ptCar);
			if (ptImg.x < 0 || ptImg.y < 0 || ptImg.x >= imgObj.cols || ptImg.y >= imgObj.rows)
			{
				continue;
			}
			imgObj.at<int>(ptImg) = i + 1;	
		}
	}

	Mat imgObjPath = imgPath.mul(imgObj);	//考虑使用batchDistance
	Mat locations;
	findNonZero(imgObjPath!=0,locations);
	for (int i = 0; i < locations.rows; i++)
	{
		Vec<int,2> ptT = locations.at<Vec<int,2>>(i,0);
		int id = imgObjPath.at<int>(Point2i(ptT.val[0],ptT.val[1])) - 1;
		pPack->ObjectList.objects[id].lane_loc = 0;
	}
	
	imgPath.convertTo(pPack->LidarMapping, CV_8U);

	return 1;
}

int CLidarMappingOpr::StateCheck(SENSOR_OBJECT& obj)
{
	int state = 0;
	if (obj.classify == 1)		//未识别的小障碍物
	{
		state = -1;
	}
	else
	{
		state = 1;
	}

	return state;
}

int CLidarMappingOpr::AddNaviEvents(DATA_PACKAGE& Data, void* pPackage)
{
	LCM_SENSOR_FUSION_PACKAGE* pPack = (LCM_SENSOR_FUSION_PACKAGE*)pPackage;

	//红绿灯
	if (m_nTurnningInfo != 2)//右转时忽略红绿灯
	{
		for (int i = 0; i < pPack->NaviInfo.Objs.size(); i++)
		{
			LCM_NAVI_OBJECT& ObjT = pPack->NaviInfo.Objs[i];
			if (ObjT.nMajorType == 6)
			{
				bool bIsFindStopLine = false;
				for (int j = 0; j < pPack->NaviInfo.Objs.size(); j++)
				{
					LCM_NAVI_OBJECT& ObjT_ = pPack->NaviInfo.Objs[j];
					if (ObjT.nValue == ObjT_.nMinorType)
					{
						NAVI_EVENT Event;
						//					Event.nValue = ObjT.nValue;
						Event.nMajorType = 6;
						Event.nValue = ObjT.nPlaceType;
						Point3f PointsTL(ObjT.points[0].x, ObjT.points[0].y, ObjT.points[0].z);
						Event.PointsSource.push_back(PointsTL);
						Point3f PointsSL(ObjT_.points[0].x, ObjT_.points[0].y, ObjT_.points[0].z);
						Event.PointsDest.push_back(PointsSL);
						if (Event.PointsDest[0].y >= 15.0)
						{
							Event.nValue = 2;
						}
						if (abs(Event.PointsDest[0].x) >= 10.0)//过滤掉侧向红绿灯
						{
							break;
						}
						Data.NaviEvents.push_back(Event);
						break;
					}
				}
			}
		}
	}

	//如果有多个停止线和红绿灯，只选取路径上的停止线-红绿灯
	vector<NAVI_EVENT>::iterator it;
	for (it = Data.NaviEvents.begin(); it != Data.NaviEvents.end();)
	{
		if (it->nMajorType != 6)
		{
			++it;
			continue;
		}
		double dDistMin = DBL_MAX;
		for (unsigned int j = 0; j < m_MainPath.points.size(); j++)
		{
			double dDist = sqrt(pow(it->PointsDest[0].x - m_MainPath.points[j].x,2) + 
				pow(it->PointsDest[0].y - m_MainPath.points[j].y,2));
			if (dDist <= dDistMin)
			{
				dDistMin = dDist;
			}
		}
		if (dDistMin >= 2.0)
		{
			it = Data.NaviEvents.erase(it);
		}
		else
		{
			++it;
		}
	}

	//禁止变道,如果前方有红绿灯，则禁止变道
// 	for (int i = 0; i < Data.NaviEvents.size(); i++)
// 	{
// 		NAVI_EVENT& ObjT = Data.NaviEvents[i];
// 		if (ObjT.nMajorType == 6)
// 		{
// 			if (ObjT.PointsSource[0].y >= 0.0)
// 			{
// 				NAVI_EVENT Event;
// 				Event.nMajorType = 100;
// 				Data.NaviEvents.push_back(Event);
// 			}
// 		}
// 	}

	//减速带

	for (unsigned int i = 0; i < pPack->NaviInfo.Objs.size(); i++)
	{
		LCM_NAVI_OBJECT& ObjT = pPack->NaviInfo.Objs[i];
		if (ObjT.nMajorType == 8)
		{
			LCM_POINT3D_F ptObjT = ObjT.points[0];
			for (unsigned int j = 0; j < pPack->NaviInfo.Paths.size(); j++)
			{
				LCM_NAVI_PATH& PathT = pPack->NaviInfo.Paths[j];
				for (unsigned int k = 0; k < PathT.Path.size(); k++)
				{
					double dDist = sqrt(pow(PathT.Path[k].x - ptObjT.x,2)+pow(PathT.Path[k].y - ptObjT.y,2));
					if (dDist <= 2.0)
					{
						NAVI_EVENT Event;
						Event.nMajorType = 8;
						Point3d pt(ObjT.points[0].x,ObjT.points[0].y,0);
						Event.PointsDest.push_back(pt);
						Data.NaviEvents.push_back(Event);
						break;
					}
				}
			}
		}
	}

	return Data.NaviEvents.size();
}