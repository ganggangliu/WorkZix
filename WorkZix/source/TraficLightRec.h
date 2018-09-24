#ifndef TRAFICLIGHTREC_H
#define TRAFICLIGHTREC_H

#include <Windows.h>
#include "OpenCVInc.h"
#include "CalibSensor.h"

//using namespace cv;
using namespace std;

typedef struct tagTraficLightResult
{
	long nIndex;
	double dX;
	double dY;
	double dZ;
	Rect RecBox;
}TraficLightResult;

class CTraficLightRec:public CCamImuCalib
{
public:
	enum TargetDataSegment
	{
		TARGET_INDEX,
		TARGET_LATTITUDE,
		TARGET_LONGITUDE,
		TARGET_ALTITUDE,
		TARGET_RECT_WIDTH,
		TARGET_RECT_HIGHT
	};
	CTraficLightRec(void);
	
	//读取交通灯信息
	long LoadTraficLightInfo();

	//获取交通灯区域
//	vector<TraficLightResult> TraficLightLocate(CGpsManager::GPS_IMU_INFO ImuData); 
	vector<TraficLightResult> TraficLightLocate(Mat& ImuData); 

	//Gps信息格式转换
//	Mat GpsData2Mat(CGpsManager::GPS_IMU_INFO ImuData);

	//画出交通灯区域
	long DrawRect(Mat& img, vector<TraficLightResult>& Result);

	//画参考坐标轴，相对IMU空间位置不变
	void DrawRefAxis(Mat& image, Mat& ImuData);

	//以当前IMU点为中心生成经纬度网格坐标
	long DrawGrid(Mat& image, Mat& ptLocal);
	
	//画参考点
private:
// 	double GetDist(Mat& GpsInfo0, Mat& GpsInfo1);
// 	double GetDist(double lat0, double long0, double lat1, double long1);
	double rad(double d);
	CCamImuCalib m_CamImuCalib;
	vector<Mat> m_TraficLightInfo;
};

#endif