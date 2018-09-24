#include "OpenCVInc.h"
#include "UbloxReader.h"

using namespace std;
using namespace cv;

string g_szWindowName("Ublox Replay");

std::vector<CUbloxData> g_UbloxInfo;	//Ublox数据
int g_nCurInd(0);						//当前显示的数据帧号
bool g_bIsAutoRun(false);				//是否自动播放
int g_nSleepTime(50);					//播放间隔时间mm

void on_trackbar(int nValue, void* pUser)
{
	if (g_nCurInd != nValue)
	{
		g_bIsAutoRun = false;
	}
	g_nCurInd = nValue;
}

int main(int argc, char* argv[])
{
	if (argc != 2)
	{
		printf("Error param cont!\n");
		getchar();
	}

	CUbloxReader UbloxOpr;
	if (UbloxOpr.ReadLog(argv[1]) <= 0)
	{
		printf("Read ublox log failed!\n");
		getchar();
	}
	g_UbloxInfo = UbloxOpr.GetUbloxInfo();
	if (g_UbloxInfo.size() <= 0)
	{
		printf("Read ublox log failed!\n");
		getchar();
	}

	namedWindow(g_szWindowName, CV_WINDOW_NORMAL);
	resizeWindow("Ublox Replay", 800, 600);
	createTrackbar("Frame", g_szWindowName, 0, g_UbloxInfo.size()-1, on_trackbar, 0);
	
	g_nCurInd = 0;
	while(1)
	{
		if (g_nCurInd <= 0)
		{
			g_nCurInd = 0;
		}
		if (g_nCurInd >= g_UbloxInfo.size())
		{
			g_nCurInd = g_UbloxInfo.size()-1;
		}
		Mat UbloxImage = UbloxOpr.GetImageByInd(g_nCurInd);

		setTrackbarPos("Frame", g_szWindowName, g_nCurInd);
		imshow(g_szWindowName, UbloxImage);

		int nKey = -1;
		if (g_bIsAutoRun)
		{
			nKey = waitKey(g_nSleepTime);
			g_nCurInd++;
		}
		else
		{
			nKey = waitKey(1);
		}
		if (nKey == 32)
		{
			g_bIsAutoRun = !g_bIsAutoRun;
		}
		if (nKey == 'a' || nKey == 'A')
		{
			g_bIsAutoRun = false;
			g_nCurInd--;
		}
		if (nKey == 'd' || nKey == 'D')
		{
			g_bIsAutoRun = false;
			g_nCurInd++;
		} 
	}

	return 0;
}

