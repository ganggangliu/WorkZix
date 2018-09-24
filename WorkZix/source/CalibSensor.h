#ifndef CALIBSENSOR_H
#define CALIBSENSOR_H

#include <Windows.h>
#include "OpenCVInc.h"

using namespace std;

class CCalibSensor 
{
public:
	CCalibSensor(void);
	
};

//CCamImuCalib
//参考坐标轴说明：
//相机坐标向右X，向前Z，向下Y
//惯导坐标系向右X，向前Y，向上Z
class CCamImuCalib
{
public:
	CCamImuCalib();
	~CCamImuCalib();

	enum ImuDataSegment
	{
		HEADING,
		PITCH,
		ROLL,
		LATITUDE,
		LONGTITUDE,
		ALTITUDE
	};

public:
	//使用CameraImuCalib.yml内的数据进行标定并生成标定结果文件extrinsicsCamAndImu.yml
	long CalibByFiles();

	////装载相机内参以及与惯导的外参extrinsicsCamAndImu.yml
	//long LoadParams();

	//ImuData：当前IMU数据，输入一行高精度定位数据,数据格式参照ImuDataSegment 如：128.99,2.0,3.0,30.4093913,114.4425757,19.8800000
	//Target：目标物的绝对坐标， 每一行是一个目标物的坐标，如：30.4093509,114.4426281,19.88
	//返回图像中的坐标
	Mat LocaleAbsolute(Mat& ImuData, Mat& ptTarget);
	//LocaleAbsolute分解版
	Mat LocaleAbsoluteEx(Mat& ImuData, Mat& ptTarget);

	//Target：目标物相对于IMU的坐标， 返回图像中的坐标
	Mat LocaleRelative(Mat& ptTarget);

	//画参考坐标轴，相对IMU空间位置不变
	void DrawRefAxis(Mat& image, Mat& ImuData);

	//以当前IMU点为中心生成经纬度网格坐标
	long DrawGrid(Mat& image, Mat& ImuData);

	//画目标点
	long DrawAxis(Mat& image, Mat& ptTarget);

	//获取相机参数
	Mat GetCameraMatrix(){return m_cameraMatrix;};
	Mat m_ptCart;
	Mat m_ptImu;
	Mat	m_ptCamera;
	Mat	m_ptImage;
private:
	double rad(double d);
	Mat Gps2Plant(Mat ptLocal, Mat ptTarget);

	//WGS48转=》大地直角坐标系
	Mat WGS48ToCart(Mat ptLocal, Mat ptTarget);

	//大地直角坐标=》惯导坐标
	Mat CartToImu(Mat ImuData, Mat Targets);

	//惯导坐标系=》相机坐标系
	Mat ImuToCamera(Mat CalibCam2Imu, Mat Targets);

	//相机坐标系=》像素坐标系
	Mat CameraToImage(Mat CamMatrix, Mat targets);

	Mat m_cameraMatrix;
	Mat m_rvec0;
	Mat m_tvec0;
	Mat m_RTImu2Cam;
	Mat m_RefHight2Ground;

	int m_nGridSize;
};		

#endif