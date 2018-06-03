#define _CRT_SECURE_NO_WARNINGS

#include "debugfunction.h"
//#include "ringbuffer.h"
#ifdef WIN32
    #include <Windows.h>
#else
    #include <stdlib.h>
    #include <stdio.h>
    #include <time.h>
#endif
#include "transplant.h"

unsigned int GetSysTick(void)
{
#ifdef WIN32
    return (unsigned int)GetTickCount();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
#endif
}

unsigned int CalcElapsedTime(unsigned int nStart_, unsigned int nEnd_)
{
    if (nEnd_ >= nStart_) {
        return (nEnd_ - nStart_);
    }
    else {
        return (0xFFFFFFFF - nStart_ + nEnd_ + 1);
    }
}


/*
 * 日志
*/
//enum {
//    LOG_BUFF_SIZE = 256
//};
//static RingBuff<LOG> s_logBuff(LOG_BUFF_SIZE);
//static QMutex        s_logBuffMutex;

static bool s_bLogFuncInitFlag = false;
//MY_RING_BUF s_myDebugRingBuff;
#define MAX_RING_BUFF_SIZE (1024 * 1024 * 4)
char s_ringBuff[MAX_RING_BUFF_SIZE];

void OutputLogToRingBuff(const LOG& log_)
{
//    s_logBuffMutex.lock();
//    //s_logBuff.PushBackCoverageOfFullRecord(log_);
//    if (false == s_bLogFuncInitFlag) {
//        InitMyRingBuf(&s_myDebugRingBuff, s_ringBuff, MAX_RING_BUFF_SIZE, false);
//        s_bLogFuncInitFlag = true;
//    }
//    int strLen = strlen(log_.info);
//    WriteToMyRingBufCoverageOfFullRecord(&s_myDebugRingBuff, log_.info, strLen);
//    s_logBuffMutex.unlock();
    printf("%s", log_.info);
}

//bool GetLogFromRingBuff(LOG& log_)
//{
//    bool bRet = false;

//    s_logBuffMutex.lock();
//    bRet = s_logBuff.GetFront(log_);
//    s_logBuffMutex.unlock();

//    return (bRet);
//}
int GetLogFromRingBuff(char* log_, int size_)
{
    int nRet = 0;

    //s_logBuffMutex.lock();
    //if (true == s_bLogFuncInitFlag) {
        //InitMyRingBuf(&s_myDebugRingBuff, s_ringBuff, MAX_RING_BUFF_SIZE, false);
        //s_bLogFuncInitFlag = true;
//        nRet = ReadFromMyRingBufReadCanRead(&s_myDebugRingBuff, log_, size_);
    //}
    //s_logBuffMutex.unlock();

    return (nRet);
}

void OutputErr( const char* fmt, ... )
{
    va_list argptr;
    LOG log;

    if ( NULL == fmt ) {
        return;
    }

    memset(log.info, 0, sizeof(log.info));
#ifdef QT_VERSION_STR
    QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
    QString timeStr = time.toString("yyyy-MM-dd hh:mm:ss zzz "); //设置显示格式
#ifdef WIN32
    sprintf_s(&log.info[0], LOG::MAX_CHAR_SIZE - 1, timeStr.toUtf8().constData());
    //sprintf_s(&log.info[24], LOG::MAX_CHAR_SIZE - 25, "[ERR] ");
#else
    sprintf_s(&log.info[0], sizeof(log.info), timeStr.toUtf8().constData());
    //sprintf_s(&log.info[24], sizeof(log.info), "[ERR] ");
#endif
#endif
    int iStrLen = strlen(log.info);
    sprintf_s(&log.info[iStrLen], LOG::MAX_CHAR_SIZE - 1 - iStrLen, "[ERR] ");

    va_start(argptr, fmt);
    iStrLen = strlen(log.info);
#ifdef WIN32
    _vsnprintf(&log.info[iStrLen], LOG::MAX_CHAR_SIZE - 1 - iStrLen, fmt, argptr);
#else
    vsnprintf(&log.info[iStrLen], LOG::MAX_CHAR_SIZE - 1 - iStrLen, fmt, argptr);
#endif
    va_end(argptr);

    log.type = LOG::ERR;

    OutputLogToRingBuff(log);
}

void OutputWarn( const char* fmt, ... )
{
    va_list argptr;
    LOG log;

    if ( NULL == fmt ) {
        return;
    }

    memset(log.info, 0, sizeof(log.info));
#ifdef QT_VERSION_STR
    QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
    QString timeStr = time.toString("yyyy-MM-dd hh:mm:ss zzz "); //设置显示格式
#ifdef WIN32
    sprintf_s(&log.info[0], LOG::MAX_CHAR_SIZE - 1, timeStr.toUtf8().constData());
    //sprintf_s(&log.info[24], LOG::MAX_CHAR_SIZE - 25, "[WARN] ");
#else
    sprintf(&log.info[0], timeStr.toUtf8().constData());
    //sprintf(&log.info[24], "[WARN] ");
#endif
#endif

    int iStrLen = strlen(log.info);
    sprintf_s(&log.info[iStrLen], LOG::MAX_CHAR_SIZE - 1 - iStrLen, "[WARN] ");

    va_start(argptr, fmt);
    iStrLen = strlen(log.info);
#ifdef WIN32
    _vsnprintf(&log.info[iStrLen], LOG::MAX_CHAR_SIZE - 1 - iStrLen, fmt, argptr);
#else
    vsnprintf(&log.info[iStrLen], LOG::MAX_CHAR_SIZE - 1 - iStrLen, fmt, argptr);
#endif
    va_end(argptr);

    log.type = LOG::WARN;

    OutputLogToRingBuff(log);
}

void OutputInfo( const char* fmt, ... )
{
    va_list argptr;
    LOG log;
    int iStrLen = 0;

    if ( NULL == fmt ) {
        return;
    }

    //memset(log.info, 0, sizeof(log.info));
    log.info[0] = 0;
#ifdef QT_VERSION_STR
    QDateTime time = QDateTime::currentDateTime();//获取系统现在的时间
    QString timeStr = time.toString("yyyy-MM-dd hh:mm:ss zzz "); //设置显示格式
#ifdef WIN32
    sprintf_s(&log.info[0], LOG::MAX_CHAR_SIZE - 1, timeStr.toUtf8().constData());
    //sprintf_s(&log.info[24], LOG::MAX_CHAR_SIZE - 25, "[INFO] ");
#else
    sprintf(&log.info[0], timeStr.toUtf8().constData());
    //sprintf(&log.info[24], "[INFO] ");
#endif
#endif

    iStrLen = strlen(log.info);
    sprintf_s(&log.info[iStrLen], LOG::MAX_CHAR_SIZE - 1 - iStrLen, "[INFO] ");
    va_start(argptr, fmt);
    iStrLen = strlen(log.info);
#ifdef WIN32
    _vsnprintf(&log.info[iStrLen], LOG::MAX_CHAR_SIZE - 1 - iStrLen, fmt, argptr);
#else
    vsnprintf(&log.info[iStrLen], LOG::MAX_CHAR_SIZE - 1 - iStrLen, fmt, argptr);
#endif
    va_end(argptr);

    log.type = LOG::INFO;

    OutputLogToRingBuff(log);
}

/*
  * 日志文件（开始）
*/
LogFile::LogFile(const char* moduleName)
{
    static int fileIndex = 1;

    if (NULL != moduleName) {
#ifdef WIN32
        sprintf_s(m_moduleName, MAX_NAME_SIZE, "%s", moduleName);
#else
        sprintf(m_moduleName, "%s", moduleName);
#endif
    } else {
#ifdef WIN32
        sprintf_s(m_moduleName, MAX_NAME_SIZE, "null_name_%d", fileIndex);
#else
        sprintf(m_moduleName, "null_name_%d", fileIndex);
#endif
    }

    m_bFileIsOpen = false;
    m_bEnable = false;
}

LogFile::~LogFile()
{
    Close();
}

bool LogFile::Open(void)
{
#ifdef QT_VERSION_STR 
    if (m_bFileIsOpen) {
        COM_WARN("Open log file(\"%s\") err, can not open file again\n", m_moduleName);
        return (false);
    }

    char filePath[MAX_FILE_NAME-64] = { 0 };
#ifdef WIN32
    sprintf_s(filePath, MAX_FILE_NAME-64, ".\\log");
#else
    sprintf(filePath, ".\\log");
#endif

    QDir dir;
    //CreateDirectoryA(filePath, NULL);
    dir.mkdir(filePath);

//    SYSTEMTIME st;
//    GetLocalTime(&st);
//    sprintf_s(m_fileName, MAX_FILE_NAME, "%s\\%s_%04d_%02d_%02d_%02d_%02d_%02d_%03d.log",
//              filePath,
//              m_moduleName,
//              st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
    QDateTime time = QDateTime::currentDateTime();
    QString timeStr = time.toString("yyyy_MM_dd_hh_mm_ss_zzz");
#ifdef WIN32
    sprintf_s(m_fileName, MAX_FILE_NAME, "%s\\%s_%s.log", filePath, m_moduleName, timeStr.toUtf8().constData());
#else
    sprintf(m_fileName, "%s\\%s_%s.log", filePath, m_moduleName, timeStr.toUtf8().constData());
#endif
    m_pFile = NULL;
    m_pFile = fopen(m_fileName, "w+t");
    if (NULL == m_pFile) {
        COM_ERR("fopen(\"%s\") err\n", m_fileName);
        return (false);
    }

    m_bFileIsOpen = true;
#endif
    return (true);
}

bool LogFile::Close(void)
{
    if (m_bFileIsOpen) {
        m_bFileIsOpen = false;
        m_bEnable = false;
        if (NULL != m_pFile) {
            fclose(m_pFile);
            m_pFile = NULL;
        }
    }

    return (true);
}

void LogFile::Write(const char* str)
{
    if (m_bFileIsOpen && m_bEnable && NULL != str) {
        fputs(str, m_pFile);
    }
}
/*
  * 日志文件（结束）
*/


/*
  * 调试信息（开始）
*/
DebugInfo::DebugInfo()
{
}

DebugInfo::~DebugInfo()
{
}

DebugInfo& DebugInfo::getInstance()
{
    static DebugInfo s_debugInfo;
    return (s_debugInfo);
}

void DebugInfo::setDebugInfo(VEH_DEBUG_INFO& in_)
{
    m_vehDebugInfo = in_;
}

/*
  * 调试信息（结束）
*/
