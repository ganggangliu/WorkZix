#include "IbeoEcuImpl.h"

// #ifndef WIN32_LEAN_AND_MEAN
// #define WIN32_LEAN_AND_MEAN
// #endif

#if _DEBUG
#pragma comment(lib,"ibeosdkD.lib")
#else
#pragma comment(lib,"ibeosdk.lib")
#endif


const ibeosdk::Version::MajorVersion majorVersion(4);
const ibeosdk::Version::MinorVersion minorVersion(5);
const ibeosdk::Version::Revision revision(1);
const ibeosdk::Version::PatchLevel patchLevel;
const ibeosdk::Version::Build build;
const std::string info = "Demo";

ibeosdk::Version appVersionECU(majorVersion, minorVersion, revision, patchLevel, build, info);
IbeoSDK ibeoSDKECU;

//======================================================================

void live_demoECU(LogFileManager& logFileManager, std::string ip);

//======================================================================

TimeConversion tcECU;

//======================================================================
AllEcuListener::AllEcuListener()
{
	m_nType = ECU_CONFUTION_SYSTEM;
	m_bIsNewPoints = false;
	m_bIsNewObjects = false;
	m_bIsNewVS = false;
	m_CallBackPoints = NULL;
	m_CallBackObjects = NULL;
	m_CallBackVehicleState = NULL;
	m_pUserPoints = NULL;
	m_pUserObjects = NULL;
	m_pUserVehicleState = NULL;
	InitializeCriticalSection(&m_mutexPoints);
	InitializeCriticalSection(&m_mutexObjects);
	InitializeCriticalSection(&m_mutexVehicleState);
	m_nObjFrameInd = 0;
	m_nObjCallBackInd = 0;
	m_hHandle = NULL;
}

AllEcuListener::~AllEcuListener()
{
	DeleteCriticalSection(&m_mutexPoints);
	DeleteCriticalSection(&m_mutexObjects);
	DeleteCriticalSection(&m_mutexVehicleState);
}

void AllEcuListener::Init(char* pszIp, IBEO_DATA_TYPE nType)
{
	m_szIp = string(pszIp);
	m_nType = nType;
}
void AllEcuListener::SetCallBackPoints(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_CallBackPoints = hCallBack;
	m_pUserPoints = pUser;
}
void AllEcuListener::SetCallBackObjects(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_CallBackObjects = hCallBack;
	m_pUserObjects = pUser;
}
void AllEcuListener::SetCallBackVehicleState(lpIbeoDataRecFunc hCallBack, void* pUser)
{
	m_CallBackVehicleState = hCallBack;
	m_pUserVehicleState = pUser;
}
int AllEcuListener::Start()
{
	if (m_hHandle != NULL)
	{
		printf("m_hThread != NULL\n");
		return 0;
	}

	m_hHandle = CreateThread(NULL,0,IbeothreadReadData,this,0,NULL);
	if (m_hHandle == NULL)
	{
		printf("CreateThread Failed \n");
		return 0;
	}
	return 1;
}
long AllEcuListener::GetPoints(LCM_IBEO_CLOUD_POINTS& points)
{
	EnterCriticalSection(&m_mutexPoints);
	if (!m_bIsNewPoints)
	{
		LeaveCriticalSection(&m_mutexPoints);
		return 0;
	}
	points = m_CurPoints;
	m_bIsNewPoints = false;
	LeaveCriticalSection(&m_mutexPoints);
	return 1;
}
long AllEcuListener::GetObjects(LCM_IBEO_OBJECT_LIST& objects)
{
	EnterCriticalSection(&m_mutexObjects);
	if (!m_bIsNewObjects)
	{
		LeaveCriticalSection(&m_mutexObjects);
		return 0;
	}
	objects = m_CurObjects;
	m_bIsNewObjects = false;
	LeaveCriticalSection(&m_mutexObjects);
	return 1;
}
long AllEcuListener::GetVehicleState(LCM_IBEO_VEHICLE_STATE& VehicleState)
{
	EnterCriticalSection(&m_mutexVehicleState);
	if (!m_bIsNewVS)
	{
		LeaveCriticalSection(&m_mutexVehicleState);
		return 0;
	}
	VehicleState = m_VehicleState;
	m_bIsNewVS = false;
	LeaveCriticalSection(&m_mutexVehicleState);
	return 1;
}

static DWORD WINAPI IbeothreadReadData(LPVOID pParam)
{
	AllEcuListener* pIbeoOpr = (AllEcuListener*)pParam;
	pIbeoOpr->runIbeoDataThread();
	return 0;
}

void AllEcuListener::runIbeoDataThread()
{
	//////////////////////////////////////////////////////////////////////////
	const off_t maxLogFileSize = 1000000;
	LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Warning");
	ibeosdk::LogFile::setLogLevel(ll);
	logFileManager.start();
	//////////////////////////////////////////////////////////////////////////
	string szIp(m_szIp);
	const uint16_t port = getPort(szIp, 12002);

	IbeoEcu ecu(szIp, port);
	ecu.setLogFileManager(&logFileManager);
	ecu.registerListener(this);
	ecu.getConnected();

	CommandManagerAppBaseStatus cmabs;
	ReplyEcuAppBaseStatus cmabsr;
	logInfo << "     ==================== Status ======================" << std::endl;
	ecu.sendCommand(cmabs, cmabsr, boost::posix_time::milliseconds(500));
	logError << "CommandManagerAppBaseStatusReply: " << cmabsr.getData().size() << "  '"<< cmabsr.getData() << "'" << std::endl;

	logInfo << "==================== Start Recording =======================" << std::endl;
	CommandEcuAppBaseCtrl cmabcStart(CommandEcuAppBaseCtrl::AppBaseCtrlId_StartRecording);
	ReplyEcuAppBaseCtrl cmabcr;
	ecu.sendCommand(cmabcStart, cmabcr, boost::posix_time::milliseconds(1500));
	logError << "CommandManagerAppBaseCtrlReply: " << toHex(cmabcr.getCommandId()) << "'" << std::endl;

	logInfo << "     ==================== Status ======================" << std::endl;
	ecu.sendCommand(cmabs, cmabsr, boost::posix_time::milliseconds(500));
	logError << "CommandManagerAppBaseStatusReply: " << cmabsr.getData().size() << "  '"<< cmabsr.getData() << "'" << std::endl;

# ifdef _WIN32
	::Sleep(1);
# else // _WIN32
	sleep(1);
# endif // _WIN32
	logInfo << "==================== Stop Recording =======================" << std::endl;
	CommandEcuAppBaseCtrl cmabcStop(CommandEcuAppBaseCtrl::AppBaseCtrlId_StopRecording);
	ecu.sendCommand(cmabcStop, cmabcr, boost::posix_time::milliseconds(1500));
	logError << "CommandManagerAppBaseCtrlReply: " << toHex(cmabcr.getCommandId()) << "'" << std::endl;

	logInfo << "     ==================== Status ======================" << std::endl;
	ecu.sendCommand(cmabs, cmabsr, boost::posix_time::milliseconds(500));
	logError << "CommandManagerAppBaseStatusReply: " << cmabsr.getData().size() << "  '"<< cmabsr.getData() << "'" << std::endl;

	// Just to keep the program alive
	while (true) 
	{
		if (!ecu.isConnected())
			return;
#		ifdef _WIN32
		::Sleep(1);
#		else // _WIN32
		sleep(1);
#		endif // _WIN32
	}
}

void AllEcuListener::onData(const ScanEcu* const scan)
{
// 	logInfo << "Scan received: # " << scan->getScanNumber()
// 		<<"  time: " << tcECU.toString(scan->getStartTimestamp().toPtime(), 3)
// 		<< std::endl;
// 	NTPTime TimeS = scan->getStartTimestamp();
// 	uint32_t nSec = TimeS.getSeconds();
// 	uint32_t nMilSec = TimeS.getMilliseconds();
// 	uint32_t nMicroSec = TimeS.getMicroseconds();
// 	std::cout << "Time0:" << nSec << ":" << nMilSec << ":" << nMicroSec << std::endl;

	std::vector<ScanPointEcu> ScanPt = scan->getScanPoints();
	ibeosdk::INT16 nFlag = scan->getFlags();
	bool bIsMirrorFaceUp = (nFlag & 0x0400);
	int nLayerAdd = 0;
	if (bIsMirrorFaceUp == true)
		nLayerAdd = 4;
	LCM_IBEO_CLOUD_POINTS CloudPt;
	CloudPt.TimeStamp = scan->getStartTimestamp().getMilliseconds();
	CloudPt.FrameInd = scan->getScanNumber();
	CloudPt.DevInd = 0;
	CloudPt.ContPoints = ScanPt.size();
	for (int i = 0; i < ScanPt.size(); i++)
	{
		LCM_IBEO_POINT ptT;
		ptT.layer = ScanPt[i].getLayer() + nLayerAdd;
		ptT.flag = 0x00;
		ptT.distance = sqrt(pow((float)ScanPt[i].getPositionX(),2)+
			pow((float)ScanPt[i].getPositionY(),2))*10.f;
		ptT.angle = atan2((float)ScanPt[i].getPositionY(),(float)ScanPt[i].getPositionX())/
			M_PI*180.f*32;
		ptT.echo = ScanPt[i].getEcho();
		ptT.epw = ScanPt[i].getEchoPulseWidth()*10.f;
		CloudPt.IbeoPoints.push_back(ptT);
	}
	EnterCriticalSection(&m_mutexPoints);
	m_CurPoints = CloudPt;
	m_bIsNewPoints = true;
	LeaveCriticalSection(&m_mutexPoints);
	if(m_CallBackPoints)
	{
		(*m_CallBackPoints)(&CloudPt,m_pUserPoints);
	}
}

	//========================================
void AllEcuListener::onData(const ObjectListEcu* const objectList)
{
//	logInfo << "Objects received: # " << objectList->getNumberOfObjects() << std::endl;
}

	//========================================
void AllEcuListener::onData(const ObjectListEcuEt* const objectList)
{
//	logInfo << "ET Objects received: # " << objectList->getNbOfObjects() << std::endl;

// 	NTPTime TimeS = objectList->getTimestamp();
// 	boost::posix_time::ptime TimeNow = TimeS.toPtime();
// 	boost::posix_time::millisec_posix_time_system_config::time_duration_type time_elapse;
// 	boost::posix_time::ptime Time1970(boost::gregorian::date (1980, 1, 6));
// 	time_elapse = TimeNow - Time1970;
// 	boost::int64_t aaaa = time_elapse.total_milliseconds();
// 	std::cout << aaaa << "  ";
// 	string szTime =  tcECU.toString(TimeS.toPtime(),3);
// 	std::cout << szTime << std::endl;

	m_nObjCallBackInd++;
	if (m_nObjCallBackInd%2 == 0)
		return;
	m_nObjFrameInd++;
	const std::vector<ObjectEcuEt>& Objs =  objectList->getObjects();
	LCM_IBEO_OBJECT_LIST ObjList;
	ObjList.TimeStamp = objectList->getScanStartTimestamp().getMilliseconds();
	ObjList.FrameInd = m_nObjFrameInd;
	for (int i = 0; i < Objs.size(); i++)
	{
		const std::vector<Point2dFloat>& contPointsRef = Objs[i].getContourPoints();
		LCM_IBEO_OBJECT ObjT;
		for (int j = 0; j < contPointsRef.size(); j++)
		{
			LCM_POINT2D_F ptT;
			ptT.x = contPointsRef[j].getX() * 1000.f;
			ptT.y = contPointsRef[j].getY() * 1000.f;
			ObjT.ContourPts.push_back(ptT);
		}
		ObjT.ContContourPt = ObjT.ContourPts.size();

		ObjT.Age = Objs[i].getObjectAge();
		ObjT.ObjExtMeasurement = Objs[i].getObjExtMeasurement();
		ObjT.PredictAge = Objs[i].getObjectPredAge();
		ObjT.AbsVelocity.x = (float)Objs[i].getAbsVelocity().getX() * 1000.f;
		ObjT.AbsVelocity.y = (float)Objs[i].getAbsVelocity().getY() * 1000.f;
		Point2dFloat ptCenter = Objs[i].convertRefPoint(RPL_CenterOfGravity);
		ObjT.RefPoint.x = ptCenter.getX() * 1000.f;
		ObjT.RefPoint.y = ptCenter.getY() * 1000.f;
		ObjT.Classification = Objs[i].getClassification();
		ObjT.Id = Objs[i].getObjectId();
		ObjT.ObjBoxCenter.x = (float)Objs[i].getObjBoxCenter().getX() * 1000.f;
		ObjT.ObjBoxCenter.y = (float)Objs[i].getObjBoxCenter().getY() * 1000.f;
		ObjT.ObjBoxSize.x = (float)Objs[i].getObjBoxSize().getX() * 1000.f;
		ObjT.ObjBoxSize.y = (float)Objs[i].getObjBoxSize().getY() * 1000.f;
		ObjT.RelativeVelocity.x = (float)Objs[i].getRelVelocity().getX() * 1000.f;
		ObjT.RelativeVelocity.y = (float)Objs[i].getRelVelocity().getY() * 1000.f;
		ObjT.ObjOrientation = Objs[i].getObjBoxOrientation() / M_PI * 180.f;
		ObjList.IbeoObjects.push_back(ObjT);
	}
	ObjList.ContObjects = ObjList.IbeoObjects.size();

	EnterCriticalSection(&m_mutexObjects);
	m_CurObjects= ObjList;
	m_bIsNewObjects = true;
	LeaveCriticalSection(&m_mutexObjects);
	if(m_CallBackObjects)
	{
		(*m_CallBackObjects)(&ObjList,m_pUserObjects);
	}
}

	//========================================
void AllEcuListener::onData(const Image* const image)
{
	logInfo << std::setw(5) << image->getSerializedSize() << " Bytes  " << "Image received: time: " << tcECU.toString(image->getTimestamp().toPtime()) << std::endl;
}

	//========================================
void AllEcuListener::onData(const PositionWgs84_2604* const wgs84)
{
	logInfo << std::setw(5) << wgs84->getSerializedSize() << " Bytes  "
		<< "PositionWGS84 received: time: " << tcECU.toString(wgs84->getPosition().getTimestamp().toPtime())
		<< std::endl;
}

	//========================================
void AllEcuListener::onData(const VehicleStateBasicEcu2806* const vsb)
{
	logInfo << "VSB (0x2806) " << tcECU.toString(vsb->getTimestamp().toPtime(), 3) << std::endl;
}

	//========================================
void AllEcuListener::onData(const VehicleStateBasicEcu* const vsb)
{
	logInfo << "VSB " << tcECU.toString(vsb->getTimestamp().toPtime(), 3) << std::endl;
}

	//========================================
void AllEcuListener::onData(const MeasurementList2821* const ml)
{
	logInfo << std::setw(5) << ml->getSerializedSize() << " Bytes  "
		<< "MeasurementList received: time: " << tcECU.toString(ml->getTimestamp().toPtime())
		<< " LN: '" << ml->getListName() << "' GN: '" << ml->getGroupName() << "'"
		<< std::endl;
}

	//========================================
void AllEcuListener::onData(const DeviceStatus* const devStat)
{
	logInfo << std::setw(5) << devStat->getSerializedSize() << " Bytes  " << "DevStat received" << std::endl;
}

	//========================================
void AllEcuListener::onData(const DeviceStatus6303* const devStat)
{
	logInfo << std::setw(5) << devStat->getSerializedSize() << " Bytes  " << "DevStat 0x6303 received" << std::endl;
}

	//========================================
void AllEcuListener::onData(const LogMessageError* const logMsg)
{
	logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
		<< "LogMessage (Error) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
}

	//========================================
void AllEcuListener::onData(const LogMessageWarning* const logMsg)
{
	logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
		<< "LogMessage (Warning) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
}

	//========================================
void AllEcuListener::onData(const LogMessageNote* const logMsg)
{
	logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
		<< "LogMessage (Note) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
}

	//========================================
void AllEcuListener::onData(const LogMessageDebug* const logMsg)
{
	logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
		<< "LogMessage (Debug) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
}

int checkArgumentsECU(const int argc, const char** argv, bool& hasLogFile)
{
	const int minNbOfNeededArguments = 2;
	const int maxNbOfNeededArguments = 3;

	bool wrongNbOfArguments = false;
	if (argc < minNbOfNeededArguments) {
		std::cerr << "Missing argument" << std::endl;
		wrongNbOfArguments = true;
	}
	else if (argc > maxNbOfNeededArguments) {
		std::cerr << "Too many argument" << std::endl;
		wrongNbOfArguments = true;
	}

	if (wrongNbOfArguments) {
		std::cerr << argv[0] << " " << " IP [LOGFILE]" << std::endl;
		std::cerr << "\tIP is the ip address of the Ibeo Ecu, e.g. 192.168.0.1." << std::endl;
		std::cerr << "\tLOGFILE name of the log file. If ommitted, the log output will be performed to stderr." << std::endl;
		return 1;
	}

	hasLogFile = (argc == maxNbOfNeededArguments);
	return 0;
}

//======================================================================

int mainECU(const int argc, const char** argv)
{
	std::cerr << argv[0] << " Version " << appVersionECU.toString();
	std::cerr << "  using IbeoSDK " << ibeoSDKECU.getVersion().toString() << std::endl;

	bool hasLogFile;
	const int checkResult = checkArgumentsECU(argc, argv, hasLogFile);
	if (checkResult != 0)
		exit(checkResult);
	int currArg = 1;

	std::string ip = argv[currArg++];

	const off_t maxLogFileSize = 1000000;

	LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);

	if (hasLogFile) {
		ibeosdk::LogFile::setLogFileBaseName(argv[currArg++]);
	}
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
	ibeosdk::LogFile::setLogLevel(ll);

	logFileManager.start();

	if (hasLogFile) {
		logInfo << argv[0] << " Version " << appVersionECU.toString()
			<< "  using IbeoSDK " << ibeoSDKECU.getVersion().toString() << std::endl;
	}

	live_demoECU(logFileManager, ip);

	exit(0);
}

//======================================================================

void live_demoECU(LogFileManager& logFileManager, std::string ip)
{
	AllEcuListener allEcuListener;

	const uint16_t port = getPort(ip, 12002);
	IbeoEcu ecu(ip, port);
	ecu.setLogFileManager(&logFileManager);

	ecu.registerListener(&allEcuListener);
	ecu.getConnected();

	CommandManagerAppBaseStatus cmabs;
	ReplyEcuAppBaseStatus cmabsr;
	logInfo << "     ==================== Status ======================" << std::endl;
	ecu.sendCommand(cmabs, cmabsr, boost::posix_time::milliseconds(500));
	logError << "CommandManagerAppBaseStatusReply: " << cmabsr.getData().size() << "  '"<< cmabsr.getData() << "'" << std::endl;

	logInfo << "==================== Start Recording =======================" << std::endl;
	CommandEcuAppBaseCtrl cmabcStart(CommandEcuAppBaseCtrl::AppBaseCtrlId_StartRecording);
	ReplyEcuAppBaseCtrl cmabcr;
	ecu.sendCommand(cmabcStart, cmabcr, boost::posix_time::milliseconds(1500));
	logError << "CommandManagerAppBaseCtrlReply: " << toHex(cmabcr.getCommandId()) << "'" << std::endl;

	logInfo << "     ==================== Status ======================" << std::endl;
	ecu.sendCommand(cmabs, cmabsr, boost::posix_time::milliseconds(500));
	logError << "CommandManagerAppBaseStatusReply: " << cmabsr.getData().size() << "  '"<< cmabsr.getData() << "'" << std::endl;

# ifdef _WIN32
	::Sleep(1);
# else // _WIN32
	sleep(1);
# endif // _WIN32
	logInfo << "==================== Stop Recording =======================" << std::endl;
	CommandEcuAppBaseCtrl cmabcStop(CommandEcuAppBaseCtrl::AppBaseCtrlId_StopRecording);
	ecu.sendCommand(cmabcStop, cmabcr, boost::posix_time::milliseconds(1500));
	logError << "CommandManagerAppBaseCtrlReply: " << toHex(cmabcr.getCommandId()) << "'" << std::endl;

	logInfo << "     ==================== Status ======================" << std::endl;
	ecu.sendCommand(cmabs, cmabsr, boost::posix_time::milliseconds(500));
	logError << "CommandManagerAppBaseStatusReply: " << cmabsr.getData().size() << "  '"<< cmabsr.getData() << "'" << std::endl;

	// Just to keep the program alive
	while (true) {
		if (!ecu.isConnected())
			return;
#		ifdef _WIN32
		::Sleep(1);
#		else // _WIN32
		sleep(1);
#		endif // _WIN32
	}
}

//======================================================================
