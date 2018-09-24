#pragma once
#include <Windows.h>
#include <string>
#include "OpenCVInc.h"
#include <list>
using namespace std;

#ifdef LIDARMAPPINGOPR_EXPORTS
#define LIDARMAPPINGOPR_API __declspec(dllexport)
#else
#define LIDARMAPPINGOPR_API __declspec(dllimport)
#endif

typedef struct LIDARMAPPINGOPR_API tag_LidarMappingParam
{
	int nFrameCont;				//缓存雷达数据的帧数，[1，255]
	bool bIsMapping;			//是否制图并显示
	double dCalibLongOffset;	//制图位置偏移量mm，使用默认值
	double dCalibLatOffset;		//
	double dCalibHeadingOffset;	//制图角度偏移量degree，使用默认值
	int nGroundLine;			//打到地面的激光线层数，制图时只是用>nGroundLine层的激光点云，从0开始
	int nSkyLine;				//打向天空的激光线层数，制图时只是用<nSkyLine层的激光点云，从0开始，
								//有效的层数(nGroundLine, nSkyLine)
	int nValidCont;				//每nFrameCont中出现nValidCont次的点云为合法点云
	int nHistoryLen;			//跟踪历史周期数
	float dMaxStd;			//跟踪最大均方差mm
	float dFrameInterval;	//帧间隔时间seconds	 
	float dCollisionPredictTime;
	float dMinExtMeasurment;
	tag_LidarMappingParam::tag_LidarMappingParam();
	int LoadParam();
	int WriteParam();
}CLidarMappingParam;

typedef void(WINAPI *lpLidarDataRecFunc)(void*);

class LIDARMAPPINGOPR_API CLidarMappingOpr 
{
public:
	CLidarMappingOpr(CLidarMappingParam Param);
	~CLidarMappingOpr();

	typedef struct GPS_IMU_INFO_
	{
		double          GPSTime;        // 自本周日 0:00:00 至当前的秒数（格林尼治时间）
		float           Heading;        // 偏航角（0～359.99）
		float           Pitch;          // 俯仰角（-90～90）
		float           Roll;           // 横滚角（-180～180）
		double          Lattitude;      // 纬度（-90～90）
		double          Longitude;      // 经度（-180～180）
		double          Altitude;       // 高度，单位（米）
		float           Ve;             // 东向速度，单位（米/秒）
		float           Vn;             // 北向速度，单位（米/秒）
		float           Vu;             // 天向速度，单位（米/秒）
		float           Baseline;       // 基线长度，单位（米）
		int             NSV1;           // 天线1 卫星数
		int             NSV2;           // 天线2 卫星数
		unsigned char   Status;         // 系统状态 系统状态
		int				Type;			//0:gps，Dgps  1：IMU
		GPS_IMU_INFO_::GPS_IMU_INFO_()
		{
			memset(this,0,sizeof(GPS_IMU_INFO_));
		}
	}GPS_IMU_INFO;

	typedef struct LINE_INFO_
	{
		int nInd;						//车道线编号
		int nType;						//车道线类型
		vector<Point2d> points;			//车道线点
		LINE_INFO_::LINE_INFO_()
		{
			nInd = 0;
			nType = 0;
		}
	}LINE_INFO;

	typedef struct LANE_INFO_
	{
		int nInd;						//车道编号
		int nType;						//车道类型 0：一次规划车道，-1：一次规划左侧相邻车道 +1：一次规划右侧相邻车道
		vector<Point2d> points;			//车道中心线点
		int nLeftLineInd;				//左侧车道线编号
		int nRightLineInd;				//右侧车道线编号
		LANE_INFO_::LANE_INFO_()
		{
			nInd = 0;
			nType = 0;
			nLeftLineInd = 0;
			nRightLineInd = 0;
		}
	}LANE_INFO;

	typedef struct NAVI_EVENT_
	{
		//30：禁止左侧预测  31：禁止右侧预测
		//6:Traffic Light(TL) and its Stop Line(SL) 8:减速带 
		//100:禁止变道  200：弯道预测减速  201：弯道信息
		//11：导航路口信息
		//-10：当前车道信息
		//20：从yml文件获取的禁止变道信息
		//-20:mobileye左车道线信息
		//-21：mobileye右车道线信息
		//-22：mobileye车道中心线信息
		int nMajorType;		
		int nMinorType;		//reserve
		int nPlaceType;		//reserve
		float fOffSet;		//reserve
		int nPtCont;		//reserve
		vector< Point3f > PointsSource; //TL locate
		vector< Point3f > PointsDest;	//SL locate for the TL
		int    nValue;		//-1:not detected 0:green 1:yellow 2:red
		NAVI_EVENT_::NAVI_EVENT_()
		{
			nMajorType = 0;
			nMinorType = 0;
			nPlaceType = 0;
			fOffSet = 0.0;
			PointsSource.clear();
			PointsDest.clear();
			nValue = -1;
		}
	}NAVI_EVENT;

	typedef struct SENSOR_OBJECT_
	{
		int             obj_id;                                 // 障碍物ID
		float			ext_measurement;
		int				from;									// 数据传感器来源 0:Ibeo 9:Esr 10:RSDS_RL 11:RSDS_RR 12:RSDS_FL 13:RSDS_FR 20:Mobileye
		unsigned char   valid;                                  // 有效位
		unsigned int	age;									// 跟踪时间
		unsigned int	predict_age;							// 预测时间
		unsigned int	classify;								// 分类
		int				state;									// 状态位 state<0：疑似 | state<0：确定 | state==0: 未分类
		int				lane_loc;                               // 车道标志
		float			obj_heading;                            // 障碍物航向角，相对于车道中心线
		float           obj_loc_x;                              // 障碍物方位，横向,单位：米
		float           obj_loc_y;                              // 障碍物方位，纵向,单位：米
		float           obj_loc_z;                              // 障碍物方位，高度,单位：米
		float           obj_size_x;                             // 障碍物尺寸，横向,单位：米
		float           obj_size_y;                             // 障碍物尺寸，纵向,单位：米
		float           obj_size_z;                             // 障碍物尺寸，高度,单位：米
		float           v_x;                                    // 车身坐标系横向速度（m/s）
		float           v_y;                                    // 车身坐标系纵向速度
		float			v_x_abs;								// 绝对速度（m/s，东向）
		float			v_y_abs;								// 绝对速度（m/s，北向）
		float			v_x_abs_from_sensor;					// 绝对速度（m/s，传感器原始数据）
		float			v_y_abs_from_sensor;					// 绝对速度（m/s，传感器原始数据）
		float			v_x_abs_track;
		float			v_y_abs_track;
		bool			is_abs_track_v_available;
		float           a_x;                                    // 车身坐标系横向加速度（m/s^2）
		float           a_y;                                    // 车身坐标系纵向加速度
		Point2d			ref_point;								//参考点
		vector<Point2d>				contour;
		vector<Point2d>             points;                     // 轮廓描述，自左下沿顺时针方向
		vector<Vec<double,4> >      predict;                    // 预测轨迹，每隔100ms
		vector<Point2d>				rotate_rect;				// 外包框4个顶点
		vector<Point2d>				HistoryTrace;				// 历史轨迹
		float			pre_abs_v_x;
		float			pre_abs_v_y;
		bool			is_will_collision;
		list<bool>	HistoryCollision;
		SENSOR_OBJECT_::SENSOR_OBJECT_()
		{
			ext_measurement = 1.0;
			predict_age = 0;
			lane_loc = -100;
			state = 0;
			v_x_abs_track = 0.f;
			v_y_abs_track = 0.f;
			is_abs_track_v_available = false;
			is_will_collision = false;
		}
	}SENSOR_OBJECT;

	typedef struct SENSOR_OBJECT_LIST_
	{
		long		time_stamp;                             // 时间戳，为格林尼治时间至当前的毫秒数
		long		frame_ind;                              // 帧序号
		char            dev_ind;                                // 设备编号
		char            dev_type;                               // 雷达数据来源 
		vector<SENSOR_OBJECT>   objects;                        // 障碍物列表
	}SENSOR_OBJECT_LIST;

	typedef struct DATA_PACKAGE_
	{
		int nPackType;					//0: CloudPoint  1:Ibeo Objects
		long nFrame;
		GPS_IMU_INFO ImuData;
		int nCurLaneId;					//主路径的id，-1：无主路径
		vector<LANE_INFO> Lanes;		//路径id从左至右，从0递增
		vector<LINE_INFO> Lines;
		Mat LidarMapping;
		SENSOR_OBJECT_LIST ObjectList;
		vector<NAVI_EVENT> NaviEvents;
		DATA_PACKAGE_::DATA_PACKAGE_()
		{
			nCurLaneId = -1;
			nPackType = 0;
			nFrame = 0;
		}
		void DATA_PACKAGE_::clear()
		{
			memset(&ImuData,0,sizeof(GPS_IMU_INFO));
			Lanes.clear();
			Lines.clear();
			LidarMapping.release();
			ObjectList.objects.clear();
			NaviEvents.clear();
		}
	}DATA_PACKAGE;

	friend void WINAPI PackDataCallBack(void* pCloudPoints, void* pUser);

public:
	void SetScale(int nFront, int nRear, int nSide, int nGridSize = 100);
	void SetCallBack(lpLidarDataRecFunc hLidarDataCallBack);
	long Start();
	Mat Grey2FateColor(Mat& mat_in);
	long GetData(DATA_PACKAGE* pData);
	void DrawTrack(Mat& img, vector<Point2d>& Track);
	void DrawLine(Mat& img, vector<vector<Point2d>>& Lines);
	void DrawNavi(Mat& img, DATA_PACKAGE& Data);
	void DrawRadar(Mat& img, DATA_PACKAGE& Data);
	void DrawCar(Mat& img, int nFront, int nRear, int nSide);
	void DrawIbeoObjects(Mat& img, DATA_PACKAGE& Data, Scalar color = Scalar::all(-1));
	Point2i Car2Image(Point2d PtCar);

private:
	Point2d CarXY2ImageUV(Point2d CarXY);
	Mat DrawLidarPoints();
	Mat Gridding(Mat CloudPt);
	void saveXYZ(const char* filename, const Mat& mat);
	long LidarCloudMapping(void* pPackData);
	long LidarObjects(void* pPackData);
	int LinkObj2Path(void* pPackage);
	int FilterObj(void* pPackage);
	int StateCheck(SENSOR_OBJECT& obj);
	int AddNaviEvents(DATA_PACKAGE& Data, void* pPackage);
	int CollisionDetect(DATA_PACKAGE& Data);
	int GetTurnningInfo();
	int GetTurnningInfoFromNavi();
	void OtherWork();
	int GetCurLaneId();
	void GetPredictAbsVel();
	void Process_30_31(void* pPackData);
	void Process_11(void* pPackData);
	void Process_100(void* pPackData);
	void Process_20(void* pPackData);
	void Process_Mobileye(void* pPackData);
	void Process_LaneKeep(void* pPackData);

private:
	lpLidarDataRecFunc m_LidarDataRecFunc;
	CLidarMappingParam m_Param;
	void* m_pLcmLidarCloud;
	void* m_phHandle;

	std::list<Mat> m_ListPose;
	std::list<Mat> m_ListMatCloudPt;
	std::list<Mat> m_ListGroundPt;
	GPS_IMU_INFO m_ImuData;

	DATA_PACKAGE m_DataPackage;
	bool m_bIsOldData;
	long m_nIndOut;
	CRITICAL_SECTION m_cs;

	long m_nGridFront;
	long m_nGridBack;
	long m_nGridSide;
	long m_nGridSize;

	Scalar m_ColorList[1000];

	std::list<DATA_PACKAGE> m_ListHisoryData;
	bool m_bIsTurning;
	DATA_PACKAGE m_DataBefor;

	LANE_INFO m_MainPath;
	int m_nTurnningInfo;			//0：无弯道 1：左拐弯 2：右拐弯 3：路口直行
	int m_nTurnStartInd;
	int m_nTurnEndInd;
	bool m_bIsInTurn;
	list<bool> m_ListCollision;
	bool m_bIsBreak;

	int m_nCurLaneId;				//当前车道ID
	int m_nNearestPointOfCurLane;	//当前车道最近的点的ID
	double m_dDistCar2NearLane;		//当前车道最近的点的距离

	int m_nPredictForbidState;		//-1：no  30：left forbid  31：right forbid

	double m_dMobileyeHeadingOffSet;
	Point3d m_MobileyeTranslate;
	list<int> m_GpsStateList;
};

Mat LIDARMAPPINGOPR_API Grey2FateColor(Mat& mat_in);