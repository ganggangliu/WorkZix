#pragma once
#include "OpenCVInc.h"
// #include <cv.h>
// using namespace cv;
// ���� ifdef ���Ǵ���ʹ�� DLL �������򵥵�
// ��ı�׼�������� DLL �е������ļ��������������϶���� LIBGUIOPENGL_EXPORTS
// ���ű���ġ���ʹ�ô� DLL ��
// �κ�������Ŀ�ϲ�Ӧ����˷��š�������Դ�ļ��а������ļ����κ�������Ŀ���Ὣ
// LIBGUIOPENGL_API ������Ϊ�Ǵ� DLL ����ģ����� DLL ���ô˺궨���
// ������Ϊ�Ǳ������ġ�
#ifdef LIBGUIOPENGL_EXPORTS
#define LIBGUIOPENGL_API __declspec(dllexport)
#else
#define LIBGUIOPENGL_API __declspec(dllimport)
#endif

enum OpenGlLibDispMode
{
	AUTO_FOCUS_SIZE,
	AUTO_FOCUS,
	AUTO_FREE,
	AUTO_ANGLE
};

struct ILibGuiOpenGL
{
	virtual void UnInit() = 0;
	
	virtual	void AddRefPoints(Mat RefPoints, Mat Color = Mat()) = 0;
	virtual void AddRefPoints(double x, double y, double heading, Mat Color = Mat()) = 0;
	virtual	void AddPointsRelative(Mat Points, Mat Pose = Mat(), Mat Color = Mat()) = 0;
	virtual	void AddPointsRelative(double x, double y, double heading, Mat Points, Mat Color = Mat()) = 0;
	virtual	void AddLinesRelative(Mat Lines, Mat Pose = Mat(), Mat Color = Mat()) = 0;
	virtual	void AddBreakLinesRelative(Mat Lines, Mat Pose = Mat(), Mat Color = Mat()) = 0;
	virtual void AddSphereRelative(Point3d pt,float sz, Mat Pose = Mat(), Mat Color = Mat()) = 0;
	virtual void Clear() = 0;
	virtual void ClearRefPoints() = 0;
	virtual void ClearPoints() = 0;
	virtual void ClearLines() = 0;
	virtual void ClearBreakLines() = 0;
	virtual void ClearSphere() = 0;
	virtual void SetCameraMode(int nMode) = 0;
	virtual void SetScale(double dScale) = 0;
	virtual void SetAxisLenth(float dLenth) = 0;
};

typedef ILibGuiOpenGL* LIBGUIOPENGL_HANDLE;

extern "C" LIBGUIOPENGL_API LIBGUIOPENGL_HANDLE GetLibGuiOpenGL(const std::string &LibXml = "LibGuiOpenGL_Params.xml");

//extern "C"
//{
//
//LIBGUIOPENGL_API long LibGuiOpenGL_Init(const string &LibXml = "LibGuiOpenGL_Params.xml");
//LIBGUIOPENGL_API void LibGuiOpenGL_UnInit(long &lHandle);
//
//LIBGUIOPENGL_API void LibGuiOpenGL_AddPoints(long lHandle, cv::Point3f& PoseData, cv::Mat& Cloud);
//LIBGUIOPENGL_API void LibGuiOpenGL_AddPoints1_Limited(long lHandle, cv::Mat& pose, const cv::Mat& Cloud, const std::vector<Vec3b> &tColorList);
//
//}

