#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif
#include "OpenGL.h"

using namespace cv;
using namespace std;

// int main(int argc, char* argv[])
// {
// 	COpenGlOpr Opr;
// 
// 	vector<Point2f> Points;
// 	Points.push_back(Point2f(100.f, 100.f));
// 	Points.push_back(Point2f(-100.f, -100.f));
// 	Points.push_back(Point2f(0.f, 0.f));
// 
// 	for (int i = 1; i < 10000; i++)
// 	{
// 		Opr.addRefPoints(Point3f(i-1, 0, 0));
// 		vector<Point3f> line;
// 		line.push_back(Point3f(i-1, 0, 0));
// 		line.push_back(Point3f(i, 0, 0));
// 		Opr.addPolyLine(line);
// #ifdef WIN32
//         Sleep(100);
// #else
//         usleep(100000);
// #endif
// 	}
// 
// 	Opr.addRefPoints(Point3f(100.f, 100.f, 0));
// 	Opr.addRefPoints(Point3f(-100.f, -100.f, 0));
// 	Opr.addRefPoints(Point3f(0.f, 0.f, 0));
// 
// #ifdef WIN32
//     Sleep(INFINITE);
// #else
//     sleep(10000000);
// #endif
// 
// 	return 0;
// }


#include "TrackProcess .h"

int main(int argc, char* argv[])
{
	if (argc < 2)
	{
		printf("Path not found!\n");
		getchar();
	}

	CTrackProcess TrackOpr;
	TrackOpr.Process(argv[1]);

	COpenGlOpr Opr;
	double dMaxDeltaAng = DBL_MIN;
	double dMinDeltaAng = DBL_MAX;
	for (unsigned int i = 0; i < TrackOpr.m_MotionR.size(); i++)
	{
		Point3f pt(TrackOpr.m_MotionR[i].dCartX, TrackOpr.m_MotionR[i].dCartY,0);
		Opr.addRefPoints(pt, CV_RGB(0,0,255));
		if (TrackOpr.m_MotionR[i].dAngleDelta >= dMaxDeltaAng)
		{
			dMaxDeltaAng = TrackOpr.m_MotionR[i].dAngleDelta;
		}
		if (TrackOpr.m_MotionR[i].dAngleDelta <= dMinDeltaAng)
		{
			dMinDeltaAng = TrackOpr.m_MotionR[i].dAngleDelta;
		}
	}
	printf("dMinDeltaAng:%.5f, dMaxDeltaAng:%.5f\n", dMinDeltaAng, dMaxDeltaAng);

	dMaxDeltaAng = DBL_MIN;
	dMinDeltaAng = DBL_MAX;
	for (unsigned int i = 0; i < TrackOpr.m_MotionL.size(); i++)
	{
		Point3f pt(TrackOpr.m_MotionL[i].dCartX, TrackOpr.m_MotionL[i].dCartY,0);
		Opr.addRefPoints(pt, CV_RGB(0,255,0));
		if (TrackOpr.m_MotionL[i].dAngleDelta >= dMaxDeltaAng)
		{
			dMaxDeltaAng = TrackOpr.m_MotionL[i].dAngleDelta;
		}
		if (TrackOpr.m_MotionL[i].dAngleDelta <= dMinDeltaAng)
		{
			dMinDeltaAng = TrackOpr.m_MotionL[i].dAngleDelta;
		}
	}
	printf("dMinDeltaAng:%.5f, dMaxDeltaAng:%.5f\n", dMinDeltaAng, dMaxDeltaAng);

	for (unsigned int i = 0; i < TrackOpr.m_UbloxInCart.size(); i++)
	{
		Point3f pt(TrackOpr.m_UbloxInCart[i].x, TrackOpr.m_UbloxInCart[i].y,0);
		Opr.addRefPoints(pt, CV_RGB(255,0,0));
	}
	for (unsigned int i = 0; i < TrackOpr.m_MotionStereo.size(); i++)
	{
		Point3f pt(TrackOpr.m_MotionStereo[i].dCartX, TrackOpr.m_MotionStereo[i].dCartY,0);
		Opr.addRefPoints(pt, CV_RGB(255,0,255));
//		printf("%d,%.5f\n", i, TrackOpr.m_MotionStereo[i].dAngleDelta);
//		Sleep(10);
	}


#ifdef WIN32
	Sleep(INFINITE);
#else
	sleep(10000000);
#endif

	return 0;
}