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
	int nFrameCont;				//�����״����ݵ�֡����[1��255]
	bool bIsMapping;			//�Ƿ���ͼ����ʾ
	double dCalibLongOffset;	//��ͼλ��ƫ����mm��ʹ��Ĭ��ֵ
	double dCalibLatOffset;		//
	double dCalibHeadingOffset;	//��ͼ�Ƕ�ƫ����degree��ʹ��Ĭ��ֵ
	int nGroundLine;			//�򵽵���ļ����߲�������ͼʱֻ����>nGroundLine��ļ�����ƣ���0��ʼ
	int nSkyLine;				//������յļ����߲�������ͼʱֻ����<nSkyLine��ļ�����ƣ���0��ʼ��
								//��Ч�Ĳ���(nGroundLine, nSkyLine)
	int nValidCont;				//ÿnFrameCont�г���nValidCont�εĵ���Ϊ�Ϸ�����
	int nHistoryLen;			//������ʷ������
	float dMaxStd;			//������������mm
	float dFrameInterval;	//֡���ʱ��seconds	 
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
		double          GPSTime;        // �Ա����� 0:00:00 ����ǰ����������������ʱ�䣩
		float           Heading;        // ƫ���ǣ�0��359.99��
		float           Pitch;          // �����ǣ�-90��90��
		float           Roll;           // ����ǣ�-180��180��
		double          Lattitude;      // γ�ȣ�-90��90��
		double          Longitude;      // ���ȣ�-180��180��
		double          Altitude;       // �߶ȣ���λ���ף�
		float           Ve;             // �����ٶȣ���λ����/�룩
		float           Vn;             // �����ٶȣ���λ����/�룩
		float           Vu;             // �����ٶȣ���λ����/�룩
		float           Baseline;       // ���߳��ȣ���λ���ף�
		int             NSV1;           // ����1 ������
		int             NSV2;           // ����2 ������
		unsigned char   Status;         // ϵͳ״̬ ϵͳ״̬
		int				Type;			//0:gps��Dgps  1��IMU
		GPS_IMU_INFO_::GPS_IMU_INFO_()
		{
			memset(this,0,sizeof(GPS_IMU_INFO_));
		}
	}GPS_IMU_INFO;

	typedef struct LINE_INFO_
	{
		int nInd;						//�����߱��
		int nType;						//����������
		vector<Point2d> points;			//�����ߵ�
		LINE_INFO_::LINE_INFO_()
		{
			nInd = 0;
			nType = 0;
		}
	}LINE_INFO;

	typedef struct LANE_INFO_
	{
		int nInd;						//�������
		int nType;						//�������� 0��һ�ι滮������-1��һ�ι滮������ڳ��� +1��һ�ι滮�Ҳ����ڳ���
		vector<Point2d> points;			//���������ߵ�
		int nLeftLineInd;				//��೵���߱��
		int nRightLineInd;				//�Ҳ೵���߱��
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
		//30����ֹ���Ԥ��  31����ֹ�Ҳ�Ԥ��
		//6:Traffic Light(TL) and its Stop Line(SL) 8:���ٴ� 
		//100:��ֹ���  200�����Ԥ�����  201�������Ϣ
		//11������·����Ϣ
		//-10����ǰ������Ϣ
		//20����yml�ļ���ȡ�Ľ�ֹ�����Ϣ
		//-20:mobileye�󳵵�����Ϣ
		//-21��mobileye�ҳ�������Ϣ
		//-22��mobileye������������Ϣ
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
		int             obj_id;                                 // �ϰ���ID
		float			ext_measurement;
		int				from;									// ���ݴ�������Դ 0:Ibeo 9:Esr 10:RSDS_RL 11:RSDS_RR 12:RSDS_FL 13:RSDS_FR 20:Mobileye
		unsigned char   valid;                                  // ��Чλ
		unsigned int	age;									// ����ʱ��
		unsigned int	predict_age;							// Ԥ��ʱ��
		unsigned int	classify;								// ����
		int				state;									// ״̬λ state<0������ | state<0��ȷ�� | state==0: δ����
		int				lane_loc;                               // ������־
		float			obj_heading;                            // �ϰ��ﺽ��ǣ�����ڳ���������
		float           obj_loc_x;                              // �ϰ��﷽λ������,��λ����
		float           obj_loc_y;                              // �ϰ��﷽λ������,��λ����
		float           obj_loc_z;                              // �ϰ��﷽λ���߶�,��λ����
		float           obj_size_x;                             // �ϰ���ߴ磬����,��λ����
		float           obj_size_y;                             // �ϰ���ߴ磬����,��λ����
		float           obj_size_z;                             // �ϰ���ߴ磬�߶�,��λ����
		float           v_x;                                    // ��������ϵ�����ٶȣ�m/s��
		float           v_y;                                    // ��������ϵ�����ٶ�
		float			v_x_abs;								// �����ٶȣ�m/s������
		float			v_y_abs;								// �����ٶȣ�m/s������
		float			v_x_abs_from_sensor;					// �����ٶȣ�m/s��������ԭʼ���ݣ�
		float			v_y_abs_from_sensor;					// �����ٶȣ�m/s��������ԭʼ���ݣ�
		float			v_x_abs_track;
		float			v_y_abs_track;
		bool			is_abs_track_v_available;
		float           a_x;                                    // ��������ϵ������ٶȣ�m/s^2��
		float           a_y;                                    // ��������ϵ������ٶ�
		Point2d			ref_point;								//�ο���
		vector<Point2d>				contour;
		vector<Point2d>             points;                     // ������������������˳ʱ�뷽��
		vector<Vec<double,4> >      predict;                    // Ԥ��켣��ÿ��100ms
		vector<Point2d>				rotate_rect;				// �����4������
		vector<Point2d>				HistoryTrace;				// ��ʷ�켣
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
		long		time_stamp;                             // ʱ�����Ϊ��������ʱ������ǰ�ĺ�����
		long		frame_ind;                              // ֡���
		char            dev_ind;                                // �豸���
		char            dev_type;                               // �״�������Դ 
		vector<SENSOR_OBJECT>   objects;                        // �ϰ����б�
	}SENSOR_OBJECT_LIST;

	typedef struct DATA_PACKAGE_
	{
		int nPackType;					//0: CloudPoint  1:Ibeo Objects
		long nFrame;
		GPS_IMU_INFO ImuData;
		int nCurLaneId;					//��·����id��-1������·��
		vector<LANE_INFO> Lanes;		//·��id�������ң���0����
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
	int m_nTurnningInfo;			//0������� 1������� 2���ҹ��� 3��·��ֱ��
	int m_nTurnStartInd;
	int m_nTurnEndInd;
	bool m_bIsInTurn;
	list<bool> m_ListCollision;
	bool m_bIsBreak;

	int m_nCurLaneId;				//��ǰ����ID
	int m_nNearestPointOfCurLane;	//��ǰ��������ĵ��ID
	double m_dDistCar2NearLane;		//��ǰ��������ĵ�ľ���

	int m_nPredictForbidState;		//-1��no  30��left forbid  31��right forbid

	double m_dMobileyeHeadingOffSet;
	Point3d m_MobileyeTranslate;
	list<int> m_GpsStateList;
};

Mat LIDARMAPPINGOPR_API Grey2FateColor(Mat& mat_in);