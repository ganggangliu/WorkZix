#ifndef OPENGL_H
#define OPENGL_H

#include <vector>
#include "OpenCVInc.h"

class COpenGlOpr
{
public:
	COpenGlOpr();
	~COpenGlOpr();
	int addRefPoints(const cv::Point3f& point,
		const cv::Scalar& color = CV_RGB(255,0,0));
	int addPolyLine(const std::vector<cv::Point3f>& lines,
		const cv::Scalar& color = CV_RGB(0,255,0));

protected:
#ifdef WIN32
	static unsigned long __stdcall ThreadProcess(void* lparam);
#else
    static void* ThreadProcess(void* lparam);
#endif
	static void SetupRC();
	static void ChangeSize(int w, int h);
	static void RenderScene();
	static void SpecialKeys(int key, int x, int y);
	static void KeyboardKeys(unsigned char key, int x, int y);
	static void MouseKeys(int button, int state, int x, int y);
	static void Idle();
	static void CheckSize();
	static void AutoSize();
	static void ResetSize();
	static void DrawAxis();
	static void DrawRefPoints();
	static void DrawPolyLines();

private:
	int m_nHandle;
	int m_nHeight;
	int m_nWidth;

	static int m_nDispType;

	static float m_xRot;
	static float m_yRot;
	static float m_zRot;
	static float m_xOffset;
	static float m_yOffset;
	static float m_zOffset;
	static float m_scale;
	static float m_scalePro;

	static float m_dXmax;
	static float m_dXmin;
	static float m_dYmax;
	static float m_dYmin;

	static std::vector<cv::Point3f> m_refPoints;
	static std::vector<cv::Scalar> m_refPointsColor;
	static std::vector<std::vector<cv::Point3f> > m_polyLines;
	static std::vector<cv::Scalar> m_polyLinesColor;
};

#endif
