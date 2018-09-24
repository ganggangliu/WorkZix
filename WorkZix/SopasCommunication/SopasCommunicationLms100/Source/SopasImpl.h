#include <windows.h>
#include "SopasOpr.h"
#include "Doxygen_doc.h"
#include <stdio.h>
#include "TcpCommunication.h"
#include "SerialCommunication.h"
#include "SopasInterface.h"
#include "SopasAsciiFramer.h"
#include "ColaAProtocol.h"
#include "Lms100SopasInterface.h"
#include "Lms100State.h"
#include "Lms100Scan.h"
#include "OpenCVInc.h"


class  CSopasImpl
{
public:
	CSopasImpl();
	~CSopasImpl();

	int Init(CSopasParam& Param);
	void SetCallBack(lpSopasDataCallBack hHander, void* pUser);
	int Start();
	int GetData(vector<cv::Point2f>& Points);
	int Stop();

friend void static SopasAsyncCallback1(const SDecodedAnswer& rAnswer, CBinaryDataStream& rStream, CDeserializer& rDeserializer, void* pUser = 0);

private:
	static DWORD __stdcall ThreadFunc(LPVOID pParam);
	void InitLms1001(CLms100SopasInterface& rSopas);
	void WaitForMeasureState1(CLms100SopasInterface& rSopas);
	void SubscribeScans1(CLms100SopasInterface& rSopas);

	CSopasParam m_Param;
	lpSopasDataCallBack m_hCallBack;
	void* m_pUser;
	CLms100SopasInterface* m_pSopas;
	vector<cv::Point2f> m_Points;
	int m_nDataState;
	CTcpCommunication m_ipComm;
	HANDLE m_hThread;
	CRITICAL_SECTION m_cs;
};


