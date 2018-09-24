#include "SopasImpl.h"

void SopasAsyncCallback1(const SDecodedAnswer& rAnswer, CBinaryDataStream& rStream, CDeserializer& rDeserializer, void* pUser)
{
	CSopasImpl* pSopasImpl = (CSopasImpl*)pUser;
	if(rAnswer.eAnswerType == SDecodedAnswer::EVENT_RESULT)
	{
		// Process subscribed events
		if(rAnswer.coName.compare("LMDscandata") == 0)
		{
			// Process scandata event
			SLms100Scan soScan;
			if(soScan.Deserialize(rDeserializer, rStream) == true)
			{
				// Initialize average and display detailed scan information on first run
				//        (void)ProcessScan(soScan, bFirstCall);
				//       bFirstCall = false;

				int nDataLen = soScan.aDataChannel16.aFlexArrayData[0].aData.uiFlexArrayLength;
				double dAngRes = soScan.aDataChannel16.aFlexArrayData[0].DataChannelHdr.uiAngleRes/10000.0;
				SOPAS_UInt16* pDist = soScan.aDataChannel16.aFlexArrayData[0].aData.aFlexArrayData;
				double dStartAng = soScan.aDataChannel16.aFlexArrayData[0].DataChannelHdr.diStartAngle/10000.0;

				EnterCriticalSection(&(pSopasImpl->m_cs));
				pSopasImpl->m_Points.clear();
				for (int i = 0; i < nDataLen; i++)
				{
					cv::Point2f pt;
					double dAng = dStartAng + i*dAngRes;
					if(dAng < pSopasImpl->m_Param.dAngleMin || dAng > pSopasImpl->m_Param.dAngleMax)
						continue;
					dAng += pSopasImpl->m_Param.dHeading;
					double dDist = (double)pDist[i]*2.0;//mm
					if (dDist <= 100)
						dDist = 1000000;
					double X = cos(dAng/180.f*CV_PI)*dDist + pSopasImpl->m_Param.T.x;
					double Y = sin(dAng/180.f*CV_PI)*dDist + pSopasImpl->m_Param.T.y;
					pt.x = X;
					pt.y = Y;
					pSopasImpl->m_Points.push_back(pt);
				}
				pSopasImpl->m_nDataState = 0;
				LeaveCriticalSection(&(pSopasImpl->m_cs));
				if (pSopasImpl->m_hCallBack)
				{
					(*(pSopasImpl->m_hCallBack))(&(pSopasImpl->m_Points), pSopasImpl->m_pUser);
				}

//				printf("Sick lms511 data send: %d\n",nDataLen);
			}
			else
			{
//				printf("Error during scandata deserialization (event based)\n");
			}
		}
	}
}

CSopasImpl::CSopasImpl()
{
	m_hCallBack = 0;
	m_pUser = 0;
	m_pSopas = 0;
	m_nDataState = -1;
	m_hThread = 0;
	InitializeCriticalSection(&m_cs);
}
CSopasImpl::~CSopasImpl()
{
	if (m_pSopas)
	{
		delete m_pSopas;
		m_pSopas = 0;
	}
	DeleteCriticalSection(&m_cs);
}

int CSopasImpl::Init(CSopasParam& Param)
{
	m_Param = Param;

	return 1;
}
void CSopasImpl::SetCallBack(lpSopasDataCallBack hHander, void* pUser)
{
	m_hCallBack = hHander;
	m_pUser = pUser;
}
DWORD __stdcall CSopasImpl::ThreadFunc(LPVOID pParam)
{
	CSopasImpl* pCSopasImpl = (CSopasImpl*)pParam;

	CSopasAsciiFramer framer;
	CColaAProtocol protocol(pCSopasImpl->m_ipComm, framer);
	CLms100SopasInterface sopas(protocol);

	pCSopasImpl->InitLms1001(sopas);

	pCSopasImpl->WaitForMeasureState1(sopas);

	sopas.SetAsyncCallback(&SopasAsyncCallback1, pParam);

	pCSopasImpl->SubscribeScans1(sopas);

	return 1;
}

int CSopasImpl::Start()
{
	m_nDataState = -1;
	IPAddress^ coAdr = IPAddress::Parse(gcnew System::String(m_Param.szIp));
	
	if(m_ipComm.Connect(coAdr) == true)
	{
		m_hThread = CreateThread(NULL,0,ThreadFunc,this,0,NULL);
		if (m_hThread == 0)
		{
			printf("CreateThread Failed \n");
			return 0;
		}
	} 
	else
	{
		return 0;
	}

	return 1;
}
int CSopasImpl::GetData(vector<cv::Point2f>& Points)
{
	EnterCriticalSection(&m_cs);
	if (m_nDataState < 0)
	{
		LeaveCriticalSection(&m_cs);
		return m_nDataState;
	}
	Points = m_Points;
	m_nDataState++;
	LeaveCriticalSection(&m_cs);
	return m_nDataState;

	return 1;
}
int CSopasImpl::Stop()
{
	DWORD nRt = 0;
	TerminateThread(m_hThread, nRt);

	m_ipComm.Disconnect();

	return 1;
}

void CSopasImpl::InitLms1001(CLms100SopasInterface& rSopas)
{
	printf("Started\n");
	std::string version;
	std::string name;
	if (rSopas.GetVersionString(name, version) == true)  
	{
		printf("Name: %s, Version: %s\n", name.c_str(), version.c_str());    
	}
	else
	{
		printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());
	}

	if(rSopas.Login(SopasLoginLevel::AUTHORIZEDCLIENT) == true)
	{
		printf("Login successful\n");
	}
	else
	{
		printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());    
	}

	// Set scan config to 50Hz, 0.5 deg resolution and angle from -45 deg to 225 deg.
	CLms100SopasInterface::EScanConfigError eError;
	if(rSopas.SetScanConfig(1000/*5000*/, 1000/*5000*/, 0/*-450000*/, 1800000/*2250000*/, &eError) == true)
	{
		// Communication was OK, there might be an error code though
		printf("SetScanConfig: %d (0 = OK)\n", eError);
	}
	else
	{
		printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());    
	}

	CLms100SopasInterface::EMeasureError eRetVal;
	if(rSopas.StartMeasure(&eRetVal) == true)
	{
		// Communication was OK, there might be an error code though
		printf("StartMeasure: %d (0 = OK)\n", eRetVal);
	}
	else
	{
		printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());
	}  

	// Logout to make parameters active (must be logged in for StartMeasure)
	if(rSopas.Logout() == true)
	{
		printf("Logout successful\n");
	}
	else
	{
		printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());    
	}

	// NOTE: It might take some time until the LMS100 is in the MEASURE-state.
	// PollScans waits until measure state is reached
}
void CSopasImpl::WaitForMeasureState1(CLms100SopasInterface& rSopas)
{
	// --- Wait for measure state --- //
	printf("Wait for MEASURE state (7)\n");
	SLms100State soState;
	soState.Clear();  
	while((rSopas.GetState(soState) == true) && (soState.eFSMState != SLms100State::LMSSTATE_MEASURE))
	{
		printf("\rstate: %u", soState.eFSMState);
		System::Threading::Thread::CurrentThread->Sleep(1000); // C++/CLI to wait some time
	}
	if(soState.eFSMState != SLms100State::LMSSTATE_MEASURE)
	{
		// Meas-state not reached but while loop was exited => GetState returned false
		printf("\nSopas error code: %04x\n", rSopas.GetSopasErrorCode());
	}
	else
	{
		printf("\nLMS100 is in measure state\n");
	}
}
void CSopasImpl::SubscribeScans1(CLms100SopasInterface& rSopas)
{
	printf("\n=> Event based scandata processing\n");
	if(rSopas.SubscribeScanDataEvent(true) == true)
	{
		printf("Polling\n");
		rSopas.PollAsyncAnswers(10000); // Using default callback set by SetAsyncCallback    

		printf("\nUnsubscribing:");
		if(rSopas.SubscribeScanDataEvent(false) == true)
		{
			printf("\ndone");
		}
		else
		{
			printf("\nSopas error code: %04x", rSopas.GetSopasErrorCode());
		}
		// There might be still an event in the queue 
		// if sent during or shortly after unsubscribing.
		// Usually processed by call to SubscribeEvent itself...
		printf("\nClearing command queue"); 
		rSopas.PollAsyncAnswers(1000);
	}
	else
	{
		printf("Sopas error code: %04x\n", rSopas.GetSopasErrorCode());
	}
}

