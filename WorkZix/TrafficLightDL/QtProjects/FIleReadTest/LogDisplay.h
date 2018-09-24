#ifndef LOG_DISPLAY_H
#define LOG_DISPLAY_H

#include "DataFusion.h"

class CLogDisplay
{
public:
	CLogDisplay();
    int Init(std::string szLogDir);
	int Run();
	friend void on_trackbar(int, void*);
private:
    std::string m_szWindName;
	CDataFusion m_Df;
    cv::Mat m_matDisp;
	bool m_bIsAutoRun;
	int m_nAutoRunInter;
	int m_nAutoSleep;
	int64_t m_nCurAbsInd;
};

#endif
