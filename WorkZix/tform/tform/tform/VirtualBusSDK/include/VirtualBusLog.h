#ifndef VIRTUALBUS_LOG_H
#define VIRTUALBUS_LOG_H
#define  g_nLogLevel 2

//#include <windows.h>
#define DEBUG_LEVEL1 1
#define DEBUG_LEVEL2 2
#define DEBUG_LEVEL3 3
#define DEBUG_LEVEL4 4


#define VIRTUALBUS_INFO_LOG(level, arg)					\
	if(level <= g_nLogLevel) {					\
		PrintToTerminal arg ;					\
	}
#define VIRTUALBUS_WARN_LOG PrintToTerminal
#define VIRTUALBUS_ERR_LOG  PrintToTerminal

void PrintToTerminal( const char* fmt, ... );

#endif // COMMON_LOG_H
