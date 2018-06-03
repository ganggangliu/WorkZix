// 文件名: cancommon.h
// 说明: CAN所需的一些结构体定义
// 作者: 陈波
// 创建日期: 2016-06-01
//
#ifndef CAN_COMMON_H
#define CAN_COMMON_H


typedef  struct  _CAN_OBJ
{
    unsigned int        id;
    unsigned int        time_stamp;
    unsigned char       time_flag;
    unsigned char       send_type;   // 0表示正常发送
    unsigned char       remote_flag; // 是否是远程帧 0表示不是远程帧，1表示是远程帧
    unsigned char       extern_flag; // 是否是扩展帧 0表示不是扩展真，1表示是扩展帧
    unsigned char       data_len;
    unsigned char       data[8];
    unsigned char       reserved[3];
} CAN_OBJ, *P_CAN_OBJ;


typedef struct CAN_CONFIG_
{
    unsigned long	acc_code;//AccCode;    // 验收码
    unsigned long	acc_mask;//AccMask;    // 屏蔽码
    unsigned long	reserved;//Reserved;   // 保留
    unsigned char	filter;//Filter;     // 滤波方式，1表示单滤波，0表示双滤波
    // Timing0和Timing1用来设置CAN波特率, 00 1C - 500Kbps
    unsigned int       baud_rate; // BaudRate; unit is bps
    unsigned char	mode;//Mode;       // 模式，0表示正常模式，1表示只听模式
} CAN_CONFIG;

#endif // CANCOMMON_H
