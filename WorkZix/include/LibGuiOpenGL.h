#pragma once
#include "OpenCVInc.h"
// #include <cv.h>
// using namespace cv;
// 下列 ifdef 块是创建使从 DLL 导出更简单的
// 宏的标准方法。此 DLL 中的所有文件都是用命令行上定义的 LIBGUIOPENGL_EXPORTS
// 符号编译的。在使用此 DLL 的
// 任何其他项目上不应定义此符号。这样，源文件中包含此文件的任何其他项目都会将
// LIBGUIOPENGL_API 函数视为是从 DLL 导入的，而此 DLL 则将用此宏定义的
// 符号视为是被导出的。
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

