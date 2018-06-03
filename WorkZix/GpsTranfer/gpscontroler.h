#ifndef RECVGPSDATA_H
#define RECVGPSDATA_H

#include <vector>
#include "CmdLineInterpret.h"
#include "SerialPortWin32.h"

//////////////////////////////////////////////////////////////////////////xz
#include "LCM_GPFPD_BIN_DATA.hpp"
#define GPFPD_BIN_DATA_LEN 53
typedef void(WINAPI *lpBinRecFunc)(void*); 

#include "LCM_POS_BIN_DATA.hpp"
#define POS_BIN_DATA_LEN 61

#include "LCM_GPS_DATA.hpp"
//using namespace lcmtypes;
//////////////////////////////////////////////////////////////////////////

typedef struct {
    double          GPSTime;        // �Ա����� 0:00:00 ����ǰ����������������ʱ�䣩
    float           Heading;        // ƫ���ǣ�0��359.99��
    float           Pitch;          // �����ǣ�-90��90��
    float           Roll;           // ����ǣ�-180��180��
    double          Lattitude;      // γ�ȣ�-90��90��
    double          Longitude;      // ���ȣ�-180��180��
    double          Altitude;       // �߶ȣ���λ���ף�
    float           Ve;             // �����ٶȣ���λ����/�룩
    float           Vn;             // �����ٶȣ���λ����/�룩
    float           Vu;             // �����ٶȣ���λ����/�룩
    float           Baseline;       // ���߳��ȣ���λ���ף�
    int             NSV1;           // ����1 ������
    int             NSV2;           // ����2 ������
    unsigned char   Status;         // ϵͳ״̬ ϵͳ״̬
}GPS_INFO;

// coordinate
typedef struct {
    double x;
    double y;
}MAP_COORDINATE;

typedef struct {
    enum { MAX_PORT_NAME = 16 };
    char portName[MAX_PORT_NAME];
    unsigned int baudRate;
    unsigned char parity;
    unsigned char byteSize;
    unsigned char stopBits;
}GPS_PORT;

class GPSControler
{
public:
    GPSControler();
    ~GPSControler();

    bool getGpsDataFromFile(const char* fileNameIn, std::vector<GPS_INFO>& gpsDataOut);
    bool analyzeGpsData_XW_GI7660(const char* gpsDataLineIn, GPS_INFO& gpsDataOut);
    void analyzeGpsData_GGA(const char* gpsDataLineIn);
    bool analyzeGpsData_XW_GI3660(const char* arg_, GPS_INFO& gpsDataOut);
    bool analyzeGpsData(const char* gpsDataLineIn, GPS_INFO& gpsDataOut);
    bool openGPSPort(GPS_PORT& port);
    void closeGPSPort(void);
    bool isGpsPortOpened(void) { return (m_bIsGpsPortOpened); }
    bool sendGPSCmd(const char *cmd);
    bool recvGPSInfo(GPS_INFO& gpsDataOut, char *dataLineOut = 0, int dataLineLenIn = 0);
	//////////////////////////////////////////////////////////////////////////xz
	bool ParseBinData(LCM_GPS_DATA& gpsDataOut, LCM_GPFPD_BIN_DATA& BinData);
	bool ParseBinDataPos(LCM_GPS_DATA& gpsDataOut, LCM_POS_BIN_DATA& BinData);
	bool CRCheck(unsigned char* pdata, int datalen);
	void SetCallBack(lpBinRecFunc hCallBack);
	long GetGpfpdBinData(LCM_GPFPD_BIN_DATA& BinData);
	long GetGpfpdBinDataPos(LCM_POS_BIN_DATA& BinData);
	void BeginRecvBinData();
	void BeginRecvBinDataPos();
	lpBinRecFunc m_hCallBack;
	LCM_GPFPD_BIN_DATA m_BinData;
	LCM_POS_BIN_DATA m_BinDataPos;
	CRITICAL_SECTION cs;
	long m_RepeatGetCont;
	HANDLE m_hThread;
	//////////////////////////////////////////////////////////////////////////

    void convertCoordinate(double longitudeIn, double lattitudeIn, MAP_COORDINATE& coordinateOut);

    void enableRecvGps();
    void disableRecvGps();

public:
    CmdLineInterpret m_gpsDataLineInterpret;

    enum { SEND_CMD_TIMEOUT = 3000 };
    CSerialPort m_gpsPort;
    GPS_PORT m_gpsPortParam;
    bool m_bIsGpsPortOpened;
    volatile bool m_bRecvGpsFlag;
//    enum { MAX_DATA_BUFF_SIZE = 1024*4 };
//    char m_gpsDataLineBuff[MAX_DATA_BUFF_SIZE];
};

// other functions
char convertHexCharToDecimal(char hexChar);

#endif // RECVGPSDATA_H
