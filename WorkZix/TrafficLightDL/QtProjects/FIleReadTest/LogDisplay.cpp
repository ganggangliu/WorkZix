#include "LogDisplay.h"

using namespace std;
using namespace cv;

CLogDisplay::CLogDisplay()
{
	m_szWindName = "Log Replay";
	m_bIsAutoRun = false;
	m_nAutoRunInter = 10;
	m_nAutoSleep = m_nAutoRunInter*2;
	m_nCurAbsInd = 0;
}

int CLogDisplay::Init(string szLogDir)
{
	return m_Df.Init(szLogDir);
}

void on_trackbar(int nValue, void* pUser)
{
	CLogDisplay* pLd = (CLogDisplay*) pUser;
	if (pLd->m_nCurAbsInd != nValue)
	{
		pLd->m_bIsAutoRun = false;
	}
	pLd->m_nCurAbsInd = nValue;
}

int CLogDisplay::Run()
{
    namedWindow(m_szWindName, CV_WINDOW_NORMAL);
	resizeWindow(m_szWindName, 800, 600);
	createTrackbar("Frame", m_szWindName, 0, m_Df.GetTotalCount()-1, on_trackbar, this);

	m_nCurAbsInd = m_Df.m_nCoverStartIndL;
	while(1)
	{
		int64_t nRt = m_Df.GetFrameByInd(m_nCurAbsInd, m_matDisp);
		m_nCurAbsInd = nRt;
		setTrackbarPos("Frame", m_szWindName, m_nCurAbsInd);
		imshow(m_szWindName, m_matDisp);

		int nKey = -1;
		if (m_bIsAutoRun)
		{
			nKey = waitKey(m_nAutoSleep);
			m_nCurAbsInd += m_nAutoRunInter;
		}
		else
		{
			nKey = waitKey(1);
		}
		if (nKey == 32)
		{
			m_bIsAutoRun = !m_bIsAutoRun;
		}
		if (nKey == 'a' || nKey == 'A')
		{
			m_bIsAutoRun = false;
			m_nCurAbsInd--;
		}
		if (nKey == 'd' || nKey == 'D')
		{
			m_bIsAutoRun = false;
			m_nCurAbsInd++;
		}
	}

	return 1;
}
