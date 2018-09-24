#pragma once

#ifndef _TOOL_H_
#define _TOOL_H_// 定义内容嵌入这里
#include <Windows.h>
#endif // 条件结束标志 不要漏了


#include <list>
#include "cv.h"
#include "opencv2/opencv.hpp"

using namespace cv;

struct stVisualRange
{
	int x_min, x_max;
	int y_min, y_max;
	int z_min, z_max;

	bool b_x_min_nil, b_x_max_nil;
	bool b_y_min_nil, b_y_max_nil;
	bool b_z_min_nil, b_z_max_nil;

	stVisualRange():b_x_min_nil(true),b_x_max_nil(true),b_y_min_nil(true),b_y_max_nil(true),b_z_min_nil(true),b_z_max_nil(true) {}
};

class COpenGl
{
public:
	~COpenGl(void);

	static COpenGl* GetInstance();
	static void Destory();

	void Show(int w,int h);

	static void Set_gluPerspective_zfar(double dbzfar);
	static void Set_VisualRange(stVisualRange *pRange);

private:
	COpenGl();
	static COpenGl* m_pGl;

	int m_Width;
	int m_Height;
	int m_hwnd;

	bool m_bAvaliable;
	HANDLE m_hThread;

	static double param_gluPerspective_zfar;
	static stVisualRange *tVisualRange;

	static std::list<Mat> m_RefPointList;
	static std::list<Point3d> m_RefPointList_;
	static std::list<Mat> m_RefPointColor;

	static std::list<Mat> m_PointsPose;
	static std::list<Mat> m_PointsList;
	static std::list<Mat> m_PointsColor;

	static std::list<Mat> m_LinePose;
	static std::list<Mat> m_LineList;
	static std::list<Mat> m_LineColor;

	static std::list<Point3d> m_SpherePt;
	static std::list<Mat> m_SpherePose;
	static std::list<float> m_SphereSizeList;
	static std::list<Mat> m_SphereColor;

	static std::list<Mat> m_BreakLinePose;
	static std::list<Mat> m_BreakLineList;
	static std::list<Mat> m_BreakLineColor;
	static float m_dAxisLonth;

protected:
	static DWORD WINAPI ThreadProcess(LPVOID lparam);

	static void SetupRC();
	static void ChangeSize(int w, int h);
	static void RenderScene();

	static void SpecialKeys(int key, int x, int y);
	static void KeyboardKeys(unsigned char key, int x, int y);
	static void MouseKeys(int button, int state, int x, int y);
	static void Idle();

	static void BuildFont();
	static void glText(char* fmt, ...);

	static void DrawAxis();
	static void DrawPoints();
	static void DrawRefPoints();
	static void DrawLines();
	static void DrawBreakLines();
	static void DrawSphere();
	static void CheckSize();
	static void AutoSize();
	static void ResetSize();

public:
	void AddRefPoints(Mat RefPoint, Mat Color = Mat());
	void AddRefPoints(double x, double y, double heading, Mat Color = Mat());
	void AddPointsRelative(Mat Points, Mat Pose = Mat(), Mat Color = Mat());
	void AddPointsRelative(double x, double y, double heading, Mat Points, Mat Color = Mat());
	void AddLinesRelative(Mat Lines, Mat Pose = Mat(), Mat Color = Mat());
	void AddBreakLinesRelative(Mat Lines, Mat Pose = Mat(), Mat Color = Mat());
	void AddSphereRelative(Point3d pt ,float sz, Mat Pose, Mat Color);
	void Clear();
	void ClearRefPoints();
	void ClearPoints();
	void ClearLines();
	void ClearBreakLines();
	void ClearSphere();
	void SetCameraMode(int nMode);
	void SetScale(double dScale);
	void SetAxisLenth(float dLenth);
};

