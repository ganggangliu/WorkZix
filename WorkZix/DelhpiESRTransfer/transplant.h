
// 文件名: transplant.h
// 说明: 为了方便在Windows与Linux之间快速地进行移植
//          而添加的头文件。
//
// 作者: 陈波
// 创建日期: 2016-06-01
//

#ifndef TRANSPLANT_H
#define TRANSPLANT_H

#include <stdio.h>

#ifdef _MSC_VER
    #define _CRT_SECURE_NO_WARNINGS
#endif

#ifdef linux

// winnt.h
    typedef unsigned char UCHAR;
    typedef unsigned char BYTE;
    typedef char CHAR;
    typedef unsigned short USHORT;
    typedef unsigned int UINT;
    typedef unsigned int ULONG;
    //typedef int BOOL; 与canlib.h中的定义冲突
    typedef int INT;
    typedef unsigned short WORD;
    typedef unsigned int DWORD;
    typedef void * PVOID;

    //typedef PVOID HANDLE; 与canlib.h中的定义冲突

    #define __stdcall
    #define FALSE 0
    #define TRUE 1

// winbase.h
    #define INFINITE            0xFFFFFFFF  // Infinite timeout

//    int sprintf_s(char *str, size_t size, const char *format, ...)
//    {
//        va_list argptr;
//        return snprintf(str, size, format, argptr);
//    }
#define sprintf_s snprintf
#endif


// 使用周立功的CAN驱动
#define CAN_DEV_TYPE_ZLG            1
// 使用Kavser的CAN驱动
#define CAN_DEV_TYPE_KVASER     2
// 另外一种类型的CAN驱动
#define CAN_DEV_TYPE_VECTOR     3

#if defined(WIN32)
    #define CAN_DEV_TYPE CAN_DEV_TYPE_ZLG
#elif defined(linux)
    #define CAN_DEV_TYPE CAN_DEV_TYPE_KVASER
#endif


#endif // TRANSPLANT_H


