#include <iostream>
#include <fstream>
#include <iomanip>

#include "gpscontroler.h"

using namespace std;

int      use_3660_or_7660 = 0;
double   gps_status = 0;

//////////////////////////////////////////////////////////////////////////xz
DWORD WINAPI ThreadFunc(LPVOID pParam)
{
	GPSControler* pCtrl = (GPSControler*)pParam;

	if (!pCtrl->m_bIsGpsPortOpened) 
	{
		return 0;
	}

	bool bRet = false;
	char headBuff[2] = { 0 };
	int index = 0;
	enum 
	{
		STATUS_WAIT_HEAD,
		STATUS_WAIT_DATA,
		STATUS_WAIT_CHACK,
		STATUS_WAIT_END,
		STATIS_READ_LINE_OK
	};
	int status = STATUS_WAIT_HEAD;
	char dataBuff;
	while (pCtrl->m_bRecvGpsFlag)
	{
		DWORD dataLen = pCtrl->m_gpsPort.ReadData(&dataBuff, 1);
		if (0 == dataLen) 
		{
			Sleep(1);
		}
		if (1 == dataLen) 
		{
			switch (status) 
			{
			case (STATUS_WAIT_HEAD):
				headBuff[0] = headBuff[1];
				headBuff[1] = dataBuff;
				if ((char)0xAA == headBuff[0] && (char)0x55 == headBuff[1]) 				 
				{
					EnterCriticalSection(&pCtrl->cs);//lock
					pCtrl->m_BinData.data[index++] = headBuff[0];
					pCtrl->m_BinData.data[index++] = headBuff[1];
					memset(headBuff,0,sizeof(headBuff));
					status = STATUS_WAIT_DATA;
				}
				break;
			case (STATUS_WAIT_DATA):
				pCtrl->m_BinData.data[index++] = dataBuff;
				if (index >= GPFPD_BIN_DATA_LEN) 
				{
					status = STATIS_READ_LINE_OK;
				}
				break;
			default:
				break;
			}

			if (STATIS_READ_LINE_OK == status) 
			{
				status = STATUS_WAIT_HEAD;
				index = 0;
				bRet = true;
				pCtrl->m_RepeatGetCont = 0;
				LeaveCriticalSection(&pCtrl->cs);//unlock
				if (pCtrl->m_hCallBack)
				{
					(*pCtrl->m_hCallBack)(&pCtrl->m_BinData);
				}
				//				break;
			}
		}

	}

	return 1;
}

DWORD WINAPI ThreadFuncPos(LPVOID pParam)
{
	GPSControler* pCtrl = (GPSControler*)pParam;

	if (!pCtrl->m_bIsGpsPortOpened) 
	{
		return 0;
	}

	bool bRet = false;
	char headBuff[2] = { 0 };
	int index = 0;
	enum 
	{
		STATUS_WAIT_HEAD,
		STATUS_WAIT_DATA,
		STATUS_WAIT_CHACK,
		STATUS_WAIT_END,
		STATIS_READ_LINE_OK
	};
	int status = STATUS_WAIT_HEAD;
	char dataBuff;
	while (pCtrl->m_bRecvGpsFlag)
	{
		DWORD dataLen = pCtrl->m_gpsPort.ReadData(&dataBuff, 1);
		if (0 == dataLen) 
		{
			Sleep(1);
		}
		if (1 == dataLen) 
		{
			switch (status) 
			{
			case (STATUS_WAIT_HEAD):
				headBuff[0] = headBuff[1];
				headBuff[1] = dataBuff;
				if ((char)0xAA == headBuff[0] && (char)0x55 == headBuff[1]) 				 
				{
					EnterCriticalSection(&pCtrl->cs);//lock
					pCtrl->m_BinDataPos.data[index++] = headBuff[0];
					pCtrl->m_BinDataPos.data[index++] = headBuff[1];
					memset(headBuff,0,sizeof(headBuff));
					status = STATUS_WAIT_DATA;
				}
				break;
			case (STATUS_WAIT_DATA):
				pCtrl->m_BinDataPos.data[index++] = dataBuff;
				if (index >= POS_BIN_DATA_LEN) 
				{
					status = STATIS_READ_LINE_OK;
				}
				break;
			default:
				break;
			}

			if (STATIS_READ_LINE_OK == status) 
			{
				status = STATUS_WAIT_HEAD;
				index = 0;
				bRet = true;
				pCtrl->m_RepeatGetCont = 0;
				LeaveCriticalSection(&pCtrl->cs);//unlock
				if (pCtrl->m_hCallBack)
				{
					(*pCtrl->m_hCallBack)(&pCtrl->m_BinDataPos);
				}
				//				break;
			}
		}

	}

	return 1;
}
//////////////////////////////////////////////////////////////////////////

GPSControler::GPSControler()
{
    m_gpsDataLineInterpret.SetSeparator(",");

    m_bIsGpsPortOpened = false;
    strncpy(m_gpsPortParam.portName, "COM1", GPS_PORT::MAX_PORT_NAME-1);
    m_gpsPortParam.baudRate = 115200;
    m_gpsPortParam.parity = NOPARITY;
    m_gpsPortParam.byteSize = 8;
    m_gpsPortParam.stopBits = ONESTOPBIT;

    m_bRecvGpsFlag = false;
	//////////////////////////////////////////////////////////////////////////xz
	m_hCallBack = NULL;
	InitializeCriticalSection(&cs);
	m_RepeatGetCont = 0;
	m_hThread = NULL;
	//////////////////////////////////////////////////////////////////////////

}

GPSControler::~GPSControler()
{
    //
}

bool GPSControler::getGpsDataFromFile(const char* fileNameIn, std::vector<GPS_INFO>& gpsDataOut)
{
    ifstream infile(fileNameIn);
    if (!infile) {
        printf("Open file error!\n");
        return (false);
    }

    bool bRet = false;
    enum { MAX_DATA_BUFF_SIZE = 4096 };
    char gpsDataLineBuff[MAX_DATA_BUFF_SIZE];
    GPS_INFO gpsDataLineInfo;

    // 用于调试
//    ofstream outfile_debug("../../gps_data_debug.txt");

    // 读取文件
    while(true) {
        memset(gpsDataLineBuff, 0, MAX_DATA_BUFF_SIZE);

        if (!infile.getline(gpsDataLineBuff, MAX_DATA_BUFF_SIZE)) {
            break;
        }

        bRet = analyzeGpsData(gpsDataLineBuff, gpsDataLineInfo);
        use_3660_or_7660 = use_3660_or_7660;
        if (true == bRet) {
            // convert corrdinate
            MAP_COORDINATE coordinate;
            convertCoordinate(gpsDataLineInfo.Longitude, gpsDataLineInfo.Lattitude, coordinate);
            gpsDataLineInfo.Lattitude = coordinate.y;
            gpsDataLineInfo.Longitude = coordinate.x;

            gpsDataOut.push_back(gpsDataLineInfo);

            // 用于调试
//            outfile_debug << setiosflags(ios::fixed) << setprecision(6)
//                          << gpsDataLineInfo.GPSTime << "\t"
//                          << gpsDataLineInfo.Heading << "\t"
//                          << gpsDataLineInfo.Pitch << "\t"
//                          << gpsDataLineInfo.Roll << "\t"
//                          << gpsDataLineInfo.Lattitude << "\t"
//                          << gpsDataLineInfo.Longitude << "\t"
//                          << gpsDataLineInfo.Altitude << "\t"
//                          << gpsDataLineInfo.Ve << "\t"
//                          << gpsDataLineInfo.Vn << "\t"
//                          << gpsDataLineInfo.Vu << "\t"
//                          << gpsDataLineInfo.Status << "\t"
//                          << endl;
        }
    }

    return (true);
}
bool GPSControler::analyzeGpsData_XW_GI3660(const char* arg_, GPS_INFO &gpsDataOut)
{
     vector<string> argTable;
     std::string   temp = arg_;
     std::string   data = "";
     int nStart = 0;
     int nStop  = 0;
     nStop = temp.find (",");
     data = temp.substr(nStart, nStop);
     argTable.push_back(data);
     while (1)
     {
          nStart = nStop + 1;
          nStop = temp.find (",", nStart);
          if (nStop == -1)
          {
              break;
          }
          if (nStop - nStart == 0)
          {
              argTable.push_back("0");
          }
          else
          {
              data = temp.substr(nStart, nStop - nStart);
              argTable.push_back(data);
          }
     }

     //double          GPSTime;        // 自本周日 0:00:00 至当前的秒数（格林尼治时间）
     gpsDataOut.GPSTime = atof(argTable[2].c_str());
     //float           Heading;        // 偏航角（0～359.99）
     gpsDataOut.Heading = atof(argTable[3].c_str());
     //float           Pitch;          // 俯仰角（-90～90）
     gpsDataOut.Pitch = atof(argTable[4].c_str());
     //float           Roll;           // 横滚角（-180～180）
     gpsDataOut.Roll = atof(argTable[5].c_str());
     //double          Lattitude;      // 纬度（-90～90）
     gpsDataOut.Lattitude = atof(argTable[6].c_str());
     if (0 == gpsDataOut.Lattitude) {
         return (false);
     }
     //double          Longitude;      // 经度（-180～180）
     gpsDataOut.Longitude = atof(argTable[7].c_str());
     if (0 == gpsDataOut.Longitude) {
         return (false);
     }
//     qDebug () << "gpstime = " <<gpsDataOut.GPSTime << "lon = " << gpsDataOut.Longitude << "lat = " << gpsDataOut.Lattitude;
     //double          Altitude;       // 高度，单位（米）
     gpsDataOut.Altitude = atof(argTable[8].c_str());
     //float           Ve;             // 东向速度，单位（米/秒）
     gpsDataOut.Ve = atof(argTable[9].c_str());
     //float           Vn;             // 北向速度，单位（米/秒）
     gpsDataOut.Vn = atof(argTable[10].c_str());
     //float           Vu;             // 天向速度，单位（米/秒）
     gpsDataOut.Vu = atof(argTable[11].c_str());
     //float           Baseline;       // 基线长度，单位（米）
     gpsDataOut.Baseline = atof(argTable[12].c_str());
     //int             NSV1;           // 天线1 卫星数
     gpsDataOut.NSV1 = atoi(argTable[13].c_str());
     //int             NSV2;           // 天线2 卫星数
     gpsDataOut.NSV2 = atoi(argTable[14].c_str());
     //unsigned char   Status;         // 系统状态 系统状态
     gpsDataOut.Status = argTable[15].c_str()[1];

     return  true ;
}

bool GPSControler::analyzeGpsData_XW_GI7660(const char* gpsDataLineIn, GPS_INFO &gpsDataOut)
{
    vector<string> argTable;

    m_gpsDataLineInterpret.GetAllCommand(gpsDataLineIn, argTable);
    if (argTable.size() < 16) {
        //MY_DEBUG("analyzeGpsData_XW_GI7660, arg size not enough (%d)!\n", argTable.size());
        return (false);
    }
    static const string Header = "$GPFPD";
  //  static const string Header = "$GPHPD";
    if (argTable[0] != Header) {
        //MY_DEBUG("analyzeGpsData_XW_GI7660, Header error (%s)!\n", argTable[0].c_str());
        return (false);
   }

    //double          GPSTime;        // 自本周日 0:00:00 至当前的秒数（格林尼治时间）
    gpsDataOut.GPSTime = atof(argTable[2].c_str());
    //float           Heading;        // 偏航角（0～359.99）
    gpsDataOut.Heading = atof(argTable[3].c_str());
    //float           Pitch;          // 俯仰角（-90～90）
    gpsDataOut.Pitch = atof(argTable[4].c_str());
    //float           Roll;           // 横滚角（-180～180）
    gpsDataOut.Roll = atof(argTable[5].c_str());
    //double          Lattitude;      // 纬度（-90～90）
    gpsDataOut.Lattitude = atof(argTable[6].c_str());
    if (0 == gpsDataOut.Lattitude) {
        return (false);
    }
    //double          Longitude;      // 经度（-180～180）
    gpsDataOut.Longitude = atof(argTable[7].c_str());
    if (0 == gpsDataOut.Longitude) {
        return (false);
    }
  //  qDebug () << "gpstime = " <<gpsDataOut.GPSTime << "lon = " << gpsDataOut.Longitude << "lat = " << gpsDataOut.Lattitude;
    //double          Altitude;       // 高度，单位（米）
    gpsDataOut.Altitude = atof(argTable[8].c_str());
    //float           Ve;             // 东向速度，单位（米/秒）
    gpsDataOut.Ve = atof(argTable[9].c_str());
    //float           Vn;             // 北向速度，单位（米/秒）
    gpsDataOut.Vn = atof(argTable[10].c_str());
    //float           Vu;             // 天向速度，单位（米/秒）
    gpsDataOut.Vu = atof(argTable[11].c_str());
    //float           Baseline;       // 基线长度，单位（米）
    gpsDataOut.Baseline = atof(argTable[12].c_str());
    //int             NSV1;           // 天线1 卫星数
    gpsDataOut.NSV1 = atoi(argTable[13].c_str());
    //int             NSV2;           // 天线2 卫星数
    gpsDataOut.NSV2 = atoi(argTable[14].c_str());
    //unsigned char   Status;         // 系统状态 系统状态
    gpsDataOut.Status = argTable[15].c_str()[1];

//    if (gpsDataOut.Status == 'B')
//    {
//        qDebug() << gpsDataOut.Status;
//    }
//    else
//    {
//        qDebug("status = %c", gpsDataOut.Status);
//    }

    // convert corrdinate
//    MAP_COORDINATE coordinate;
//    convertCoordinate(gpsDataOut.Longitude, gpsDataOut.Lattitude, coordinate);
//    gpsDataOut.Lattitude = coordinate.y;
//    gpsDataOut.Longitude = coordinate.x;

    return (true);
}
void GPSControler::analyzeGpsData_GGA(const char* gpsDataLineIn)
{
    vector<string> argTable;
    std::string   temp = gpsDataLineIn;
    std::string   data = "";
    int nStart = 0;
    int nStop  = 0;
    nStop = temp.find (",");
    data = temp.substr(nStart, nStop);
    argTable.push_back(data);
    while (1)
    {
         nStart = nStop + 1;
         nStop = temp.find (",", nStart);
         if (nStop == -1)
         {
             break;
         }
         if (nStop - nStart == 0)
         {
             argTable.push_back("0");
         }
         else
         {
             data = temp.substr(nStart, nStop - nStart);
             argTable.push_back(data);
         }
    }
    gps_status = atof(argTable[6].c_str());
}
void GPSControler::convertCoordinate(double longitudeIn, double lattitudeIn, MAP_COORDINATE& coordinateOut)
{
    coordinateOut.x = longitudeIn*3600*1024;
    coordinateOut.y = lattitudeIn*3600*1024;
}

bool GPSControler::openGPSPort(GPS_PORT& port)
{
    if (m_bIsGpsPortOpened) {
        m_gpsPort.ClosePort();
    }

    strncpy(m_gpsPortParam.portName, port.portName, GPS_PORT::MAX_PORT_NAME-1);
    m_gpsPortParam.baudRate = port.baudRate;
    m_gpsPortParam.parity = port.parity;
    m_gpsPortParam.byteSize = port.byteSize;
    m_gpsPortParam.stopBits = port.stopBits;

    strncpy(m_gpsPort.portParam.portName, m_gpsPortParam.portName, CSerialPort::PORT_PARAM::MAX_NAME-1);
    m_gpsPort.portParam.baudRate = m_gpsPortParam.baudRate;
    m_gpsPort.portParam.parity = m_gpsPortParam.parity;
    m_gpsPort.portParam.byteSize = m_gpsPortParam.byteSize;
    m_gpsPort.portParam.stopBits = m_gpsPortParam.stopBits;
    m_gpsPort.portParam.inQueue = 1024; // 输入缓冲区的大小（字节数）

    BOOL bRet = m_gpsPort.InitPort();
    if (FALSE == bRet) {
        printf("[ERR], InitPort err\n");
        return (false);
    }

    m_bIsGpsPortOpened = true;

    return (true);
}

void GPSControler::closeGPSPort(void)
{
    if (m_bIsGpsPortOpened) {
        m_gpsPort.ClosePort();
    }
}

bool GPSControler::sendGPSCmd(const char* cmd)
{
    if ((NULL == cmd) || (!m_bIsGpsPortOpened)) {
        return (false);
    }

    int cmdLen = strlen(cmd);
    DWORD writeBytes = m_gpsPort.WriteData(cmd, cmdLen, SEND_CMD_TIMEOUT);
    if (writeBytes < cmdLen) {
        printf("[ERR], send command err\n");
        return (false);
    }

    return (true);
}

void GPSControler::enableRecvGps()
{
    m_bRecvGpsFlag = true;
    printf("ADC: GPSControler::enableRecvGps, run flag %d", m_bRecvGpsFlag);
    m_gpsPort.Start();
}

void GPSControler::disableRecvGps()
{
    m_bRecvGpsFlag = false;
    printf("ADC: GPSControler::disableRecvGps, run flag %d", m_bRecvGpsFlag);
    m_gpsPort.Stop();
}

 bool GPSControler::recvGPSInfo(GPS_INFO& gpsDataOut, char* dataLineOut, int dataLineLenIn)
 {
     if (!m_bIsGpsPortOpened) {
         return (false);
     }

     bool bRet = false;
     enum { MAX_DATA_BUFF_SIZE = 512 };
     char gpsDataLineBuff[MAX_DATA_BUFF_SIZE];
     //GPS_INFO gpsDataLineInfo;

     // 读取串口
     memset(gpsDataLineBuff, 0, MAX_DATA_BUFF_SIZE);
     // 读取一行
     // 用于写入行缓存
     int index = 0;
     // 用于状态控制
     enum {
         STATUS_WAIT_HEAD,
         STATUS_WAIT_DATA,
         STATUS_WAIT_CHACK,
         STATUS_WAIT_END,
         STATIS_READ_LINE_OK
     };
     int status = STATUS_WAIT_HEAD;
     // 用于校验
     //unsigned char CS = 0;
     //char checkCode[4] = { 0 };
     int checkCodeIndex = 0;

     //int testCount = 0;
     // 开始读取串口（读一行）
     while ((m_bRecvGpsFlag) && (index < MAX_DATA_BUFF_SIZE-1)) {
         char dataBuff[4] = { 0 };
         //COM_DEBUG(5, ("ADC: GPSControler::recvGPSInfo, ReadData"));
         DWORD dataLen = m_gpsPort.ReadData(dataBuff, 1);
         //COM_DEBUG(5, ("ADC: GPSControler::recvGPSInfo, ReadData, data len %d, run flag %d", dataLen, m_bRecvGpsFlag));
         if (0 == dataLen) {
             if (false == m_bRecvGpsFlag) {
                 printf("ADC: GPSControler::recvGPSInfo, 1. Stop receiving gps");
                 break;
             }
             DWORD dwEvtMask;
             //COM_DEBUG(5, ("ADC: GPSControler::recvGPSInfo, WaitSerialComEvent"));
             m_gpsPort.WaitSerialComEvent(dwEvtMask);
             if (false == m_bRecvGpsFlag) {
                 printf("ADC: GPSControler::recvGPSInfo, 2. Stop receiving gps");
                 break;
             }
         }

         if (1 == dataLen) {
             switch (status) {
             case (STATUS_WAIT_HEAD):
                 if ('$' == dataBuff[0]) {
                     gpsDataLineBuff[index++] = dataBuff[0];
                     status = STATUS_WAIT_DATA;
                 }
                 break;
             case (STATUS_WAIT_DATA):
                 gpsDataLineBuff[index++] = dataBuff[0];
                 if ('*' != dataBuff[0]) {
                     //CS += dataBuff[0];
                 } else {
                     status = STATUS_WAIT_CHACK;
                 }
                 break;
             case (STATUS_WAIT_CHACK):
                 gpsDataLineBuff[index++] = dataBuff[0];
                 //checkCode[checkCodeIndex++] = dataBuff[0];
                 checkCodeIndex++;
                 if (checkCodeIndex > 1) {
                     status = STATUS_WAIT_END;
                 }
                 break;
             case (STATUS_WAIT_END):
                 //gpsDataLineBuff[index++] = dataBuff[0];
                 if ('\n' == dataBuff[0]) {
                     status = STATIS_READ_LINE_OK;
                 }
                 break;
             default:
                 break;
             }

             if (STATIS_READ_LINE_OK == status) {
                 status = STATUS_WAIT_HEAD;
                 bRet = true;
                 break;
             }
         }
     }

     //qDebug(gpsDataLineBuff);

     if (bRet) {
         // 数据校验
         //char checkSum = (convertHexCharToDecimal(checkCode[0]) << 4) + convertHexCharToDecimal(checkCode[1]);
         //qDebug("CS = %d, checkCode = %s, checkSum=%d", CS, checkCode, checkSum);
         //if (CS != checkSum) {
         //    // 校验失败
         //    MY_DEBUG("[ERR] %s, check sum err\n", __FUNCTION__);
         //    return (false);
         //}

         bRet = analyzeGpsData(gpsDataLineBuff, gpsDataOut);

         // 用于调试
         //            outfile_debug << setiosflags(ios::fixed) << setprecision(6)
         //                          << gpsDataLineInfo.GPSTime << "\t"
         //                          << gpsDataLineInfo.Heading << "\t"
         //                          << gpsDataLineInfo.Pitch << "\t"
         //                          << gpsDataLineInfo.Roll << "\t"
         //                          << gpsDataLineInfo.Lattitude << "\t"
         //                          << gpsDataLineInfo.Longitude << "\t"
         //                          << gpsDataLineInfo.Altitude << "\t"
         //                          << gpsDataLineInfo.Ve << "\t"
         //                          << gpsDataLineInfo.Vn << "\t"
         //                          << gpsDataLineInfo.Vu << "\t"
         //                          << gpsDataLineInfo.Status << "\t"
         //                          << endl;

         if ((0 != dataLineOut) && (dataLineLenIn > 0)) {
             strncpy(dataLineOut, gpsDataLineBuff, dataLineLenIn);
         }
     }

     return (bRet);
 }

 // other functions
// char convertHexCharToDecimal(char hexChar)
// {
//     if ( ('0' <= hexChar) && (hexChar <= '9') ) {
//         return (hexChar - '0');
//     } else if ( ('a' <= hexChar) && (hexChar <= 'f') ) {
//         return (hexChar - 'a' + 10);
//     } else if ( ('A' <= hexChar) && (hexChar <= 'F') ) {
//         return (hexChar - 'A' + 10);
//     }
//     return (0);
// }
bool GPSControler::analyzeGpsData(const char* gpsDataLineIn, GPS_INFO& gpsDataOut)
{
    std::string temp = gpsDataLineIn;
    if (temp.size() < 6)
    {
        return   false;
    }
    std::string  heading = temp.substr(0,6);
    if (heading == "$BDGGA" || heading == "$GPGGA")
    {
        analyzeGpsData_GGA(gpsDataLineIn);
    }
    if (heading == "$GPFPD")
    {
        use_3660_or_7660 = 1;
        return (analyzeGpsData_XW_GI7660(gpsDataLineIn, gpsDataOut));
    }
    if (heading == "$GPHPD")
    {
        use_3660_or_7660 = 2;
        return (analyzeGpsData_XW_GI3660(gpsDataLineIn, gpsDataOut));
    }
    return  (false);
}

//////////////////////////////////////////////////////////////////////////xz
//  bool GPSControler::recvBinData(GPFPD_BIN_DATA& BinData)
//  {
// 	 if (!m_bIsGpsPortOpened) 
// 	 {
// 		 return (false);
// 	 }
// 
// 	 bool bRet = false;
// 	 char headBuff[2] = { 0 };
// 	 int index = 0;
// 	 enum 
// 	 {
// 		 STATUS_WAIT_HEAD,
// 		 STATUS_WAIT_DATA,
// 		 STATUS_WAIT_CHACK,
// 		 STATUS_WAIT_END,
// 		 STATIS_READ_LINE_OK
// 	 };
// 	 int status = STATUS_WAIT_HEAD;
// 	 char dataBuff;
// 	 while (m_bRecvGpsFlag)
// 	 {
// 		 DWORD dataLen = m_gpsPort.ReadData(&dataBuff, 1);
// 		 if (0 == dataLen) 
// 		 {
// 			 if (false == m_bRecvGpsFlag) 
// 			 {
// 				 printf("ADC: GPSControler::recvGPSInfo, 1. Stop receiving gps");
// 				 break;
// 			 }
// 			 DWORD dwEvtMask;
// 			 m_gpsPort.WaitSerialComEvent(dwEvtMask);
// 			 if (false == m_bRecvGpsFlag) 
// 			 {
// 				 printf("ADC: GPSControler::recvGPSInfo, 2. Stop receiving gps");
// 				 break;
// 			 }
// 		 }
// 		 if (1 == dataLen) 
// 		 {
// 			 switch (status) 
// 			 {
// 			 case (STATUS_WAIT_HEAD):
// 				 headBuff[0] = headBuff[1];
// 				 headBuff[1] = dataBuff;
// 				 if ((char)0xAA == headBuff[0] && (char)0x55 == headBuff[1]) 				 
// 				 {
// 					 BinData.data[index++] = headBuff[0];
// 					 BinData.data[index++] = headBuff[1];
// 					 memset(headBuff,0,sizeof(headBuff));
// 					 status = STATUS_WAIT_DATA;
// 				 }
// 				 break;
// 			 case (STATUS_WAIT_DATA):
// 				 BinData.data[index++] = dataBuff;
// 				 if (index >= GPFPD_BIN_DATA_LEN) 
// 				 {
// 					 status = STATIS_READ_LINE_OK;
// 				 }
// 				 break;
// 			 default:
// 				 break;
// 			 }
// 
// 			 if (STATIS_READ_LINE_OK == status) 
// 			 {
// 				 status = STATUS_WAIT_HEAD;
// 				 bRet = true;
// 				 break;
// 			 }
// 		 }
// 	 }
// 
// 	 return (bRet);
//  }

bool GPSControler::ParseBinData(LCM_GPS_DATA& gpsDataOut, LCM_GPFPD_BIN_DATA& BinData)
{
	if (CRCheck((unsigned char*)BinData.data,GPFPD_BIN_DATA_LEN) == false)
		return false;

	float fTemp = 0;
	unsigned int nTemp = 0;
	unsigned char cTemp = 0;

	memcpy(&nTemp,BinData.data+3+2,sizeof(unsigned int));
	gpsDataOut.GPS_TIME = (double)nTemp/1000.f;

	memcpy(&fTemp,BinData.data+3+6,sizeof(float));
	gpsDataOut.GPS_HEADING = fTemp;

	memcpy(&fTemp,BinData.data+3+10,sizeof(float));
	gpsDataOut.GPS_PITCH = fTemp;

	memcpy(&fTemp,BinData.data+3+14,sizeof(float));
	gpsDataOut.GPS_ROLL = fTemp;

	memcpy(&nTemp,BinData.data+3+18,sizeof(unsigned int));
	gpsDataOut.GPS_LATITUDE = (double)nTemp*1e-7;

	memcpy(&nTemp,BinData.data+3+22,sizeof(unsigned int));
	gpsDataOut.GPS_LONGITUDE = (double)nTemp*1e-7;

	memcpy(&nTemp,BinData.data+3+26,sizeof(unsigned int));
	gpsDataOut.GPS_ALTITUDE = (double)nTemp*0.001f;

	memcpy(&fTemp,BinData.data+3+30,sizeof(float));
	gpsDataOut.GPS_VE = fTemp;

	memcpy(&fTemp,BinData.data+3+34,sizeof(float));
	gpsDataOut.GPS_VN = fTemp;

	memcpy(&fTemp,BinData.data+3+38,sizeof(float));
	gpsDataOut.GPS_VU = fTemp;

	memcpy(&fTemp,BinData.data+3+42,sizeof(float));
	gpsDataOut.GPS_BASELINE = fTemp;

	memcpy(&cTemp,BinData.data+3+46,sizeof(unsigned char));
	gpsDataOut.GPS_NSV1 = cTemp;

	memcpy(&cTemp,BinData.data+3+47,sizeof(unsigned char));
	gpsDataOut.GPS_NSV2 = cTemp;

	memcpy(&cTemp,BinData.data+3+48,sizeof(unsigned char));
	gpsDataOut.GPS_STATE = cTemp;

	return true;
}

bool GPSControler::ParseBinDataPos(LCM_GPS_DATA& gpsDataOut, LCM_POS_BIN_DATA& BinData)
{
	if (CRCheck((unsigned char*)(BinData.data+2),POS_BIN_DATA_LEN-2) == false)
		return false;

	double dTemp = 0.f;
	unsigned char cTemp = 0;
	long lTemp = 0;
	unsigned long ulTemp = 0;
	short sTemp = 0;
	unsigned short usTemp = 0;


// 	memcpy(&dTemp,BinData.data+3,sizeof(double));
// 	gpsDataOut.GPSTime = dTemp;

	memcpy(&usTemp,BinData.data+4,sizeof(usTemp));
	unsigned short usWeek = usTemp;
	memcpy(&lTemp,BinData.data+6,sizeof(lTemp));
	long nMiliSecend = lTemp;
	gpsDataOut.GPS_TIME = usWeek*7.0*24.0*60.0*60.0 + lTemp/1000.0;
	
	memcpy(&cTemp,BinData.data+10,sizeof(cTemp));
	gpsDataOut.GPS_NSV1 = cTemp;
	//////////////////////////////////////////////////////////////////////////

	memcpy(&lTemp,BinData.data+11,sizeof(long));
	gpsDataOut.GPS_LATITUDE = (double)lTemp*1e-7;

	memcpy(&lTemp,BinData.data+15,sizeof(long));
	gpsDataOut.GPS_LONGITUDE = (double)lTemp*1e-7;

	memcpy(&lTemp,BinData.data+19,sizeof(long));
	gpsDataOut.GPS_ALTITUDE = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+23,sizeof(long));
	gpsDataOut.GPS_VN = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+27,sizeof(long));
	gpsDataOut.GPS_VE = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+31,sizeof(long));
	gpsDataOut.GPS_VU = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+35,sizeof(long));
	gpsDataOut.GPS_ROLL = (double)lTemp*1e-3;

	memcpy(&lTemp,BinData.data+39,sizeof(long));
	gpsDataOut.GPS_PITCH = (double)lTemp*1e-3;

	memcpy(&ulTemp,BinData.data+43,sizeof(unsigned long));
	gpsDataOut.GPS_HEADING = (double)lTemp*1e-3;

	memcpy(&sTemp,BinData.data+47,sizeof(short));
	double dAccn = (double)lTemp*1e-3;//北向加速度，m/s^2

	memcpy(&sTemp,BinData.data+49,sizeof(short));
	double dAcce = (double)lTemp*1e-3;//东向加速度，m/s^2

	memcpy(&sTemp,BinData.data+51,sizeof(short));
	double dAccu = (double)lTemp*1e-3;//地向加速度，m/s^2

	memcpy(&sTemp,BinData.data+53,sizeof(short));
	double dAccRoll = (double)lTemp*1e-3;//横滚角速度，degree/s

	memcpy(&sTemp,BinData.data+55,sizeof(short));
	double dAccePitch = (double)lTemp*1e-3;//俯仰角速度，degree/s

	memcpy(&sTemp,BinData.data+57,sizeof(short));
	double dAccuHeading = (double)lTemp*1e-3;//航向角速度，degree/s

	memcpy(&cTemp,BinData.data+59,sizeof(unsigned char));
	gpsDataOut.GPS_STATE = cTemp;

	return true;
}

bool GPSControler::CRCheck(unsigned char* pdata, int datalen)
{
	unsigned char CS = 0; 
	for (int i=0; i< datalen-1; i++) 
	{ 
		CS += pdata[i]; 
	}

	return (CS == pdata[datalen-1]);
}

void GPSControler::SetCallBack(lpBinRecFunc hCallBack)
{
	m_hCallBack = hCallBack;
}

long GPSControler::GetGpfpdBinData(LCM_GPFPD_BIN_DATA& BinData)
{
	EnterCriticalSection(&cs);
	BinData = m_BinData;
	m_RepeatGetCont++;
	if (m_RepeatGetCont >= 10000)
		m_RepeatGetCont = 10000;
	LeaveCriticalSection(&cs);
	return m_RepeatGetCont;
}

long GPSControler::GetGpfpdBinDataPos(LCM_POS_BIN_DATA& BinData)
{
	EnterCriticalSection(&cs);
	BinData = m_BinDataPos;
	m_RepeatGetCont++;
	if (m_RepeatGetCont >= 10000)
		m_RepeatGetCont = 10000;
	LeaveCriticalSection(&cs);
	return m_RepeatGetCont;
}

void GPSControler::BeginRecvBinData()
{
	m_hThread = CreateThread(NULL,0,ThreadFunc,this,0,NULL);
	if (m_hThread == 0)
	{
		printf("CreateThread Failed \n");
		return;
	}
}

void GPSControler::BeginRecvBinDataPos()
{
	m_hThread = CreateThread(NULL,0,ThreadFuncPos,this,0,NULL);
	if (m_hThread == 0)
	{
		printf("CreateThread Failed \n");
		return;
	}
}
//////////////////////////////////////////////////////////////////////////
