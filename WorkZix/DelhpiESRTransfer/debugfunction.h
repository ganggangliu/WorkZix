#ifndef DEBUGFUNCTION_H
#define DEBUGFUNCTION_H

#ifdef QT_VERSION_STR 
    #include <QtCore>
    #include <QMutex>
#endif
#include <stdio.h>
#include <memory.h>
//#include "commgeometry.h"


unsigned int GetSysTick(void);
unsigned int CalcElapsedTime(unsigned int nStart_, unsigned int nEnd_);

/*
 * 日志
*/
typedef struct LOG_
{
    enum
    {
        ERR = 0,
        WARN = 1,
        INFO
    };
    enum { MAX_CHAR_SIZE = 512 };
    int type;
    char info[MAX_CHAR_SIZE];
}LOG;

//bool GetLogFromRingBuff(LOG& log_);
int GetLogFromRingBuff(char* log_, int size_);
void OutputErr( const char* fmt, ... );
void OutputWarn( const char* fmt, ... );
void OutputInfo( const char* fmt, ... );
#define COM_ERR OutputErr
#define COM_WARN OutputWarn
#define COM_INFO(level, info)	\
    if(level <= 6) {		    \
        OutputInfo info ;		\
    }

/*
  * 日志文件（开始）
*/
class LogFile
{
public:
    LogFile(const char* moduleName);
    ~LogFile();

    bool isEnable(void) { return (m_bEnable); }
    void Enable(bool enable_)
    {
        if (m_bFileIsOpen) {
            m_bEnable = enable_;
        } else {
            m_bEnable = false;
        }
    }
    bool Open(void);
    bool Close(void);
    void Write(const char* str);

private:
    enum { MAX_NAME_SIZE = 64 };
    char m_moduleName[MAX_NAME_SIZE];
    bool m_bEnable;
    bool m_bFileIsOpen;
    FILE* m_pFile;
    enum { MAX_FILE_NAME = 128 };
    char m_fileName[MAX_FILE_NAME];
};
/*
  * 日志文件（结束）
*/


/*
  * 调试信息（开始）
*/
class DebugInfo
{
public:
    typedef struct VEH_DEBUG_INFO_ {
        struct {
            double proportional_gain;   // 当前比例系数
            double integral_gain;       // 当前积分系数
            double differential_gain;   // 当前微分系数
            double proportion;          // 比例值
            double integral;            // 积分值
            double differential;        // 微分值
        }acc_ctl, brake_ctl;
        float target_wheel_angle;       // 目标方向盘转角
        float cur_wheel_angle;          // 当前方向盘转角
        float target_velocity;          // 目标速度
        float current_velocity;         // 当前速度
        float target_acc;               // 目标加速值
        float current_acc;              // 节气门开度
        float target_brake;             // 制动目标值
        float current_brake;            // 当前制动压力

        unsigned int update_count;      // 数据更新计数
        VEH_DEBUG_INFO_()
        {
            memset(&acc_ctl, 0, sizeof(acc_ctl));
            memset(&brake_ctl, 0, sizeof(brake_ctl));

            target_wheel_angle = 0.0f; cur_wheel_angle = 0.0f;
            target_velocity = 0.0f; current_velocity = 0.0f;
            target_acc = 0.0f; target_brake = 0.0f;
            current_acc = 0.0f; current_brake = 0.0f;

            update_count = 0;
        }
    }VEH_DEBUG_INFO;

private:
    DebugInfo();
    ~DebugInfo();

public:
    static DebugInfo& getInstance();
    void setDebugInfo(VEH_DEBUG_INFO& in_);

public:
    VEH_DEBUG_INFO m_vehDebugInfo;
};
/*
  * 调试信息（结束）
*/

#endif // DEBUGFUNCTION_H
