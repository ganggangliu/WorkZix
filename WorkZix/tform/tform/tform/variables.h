#include <iostream>
#include <vector>
using namespace std;


typedef     signed char            d_int8;
typedef     unsigned char          d_uint8;
typedef     short                  d_int16;
typedef     unsigned short         d_uint16;
typedef     int                    d_int32;
typedef     unsigned int           d_uint32;
typedef     long long              d_int64;
typedef     unsigned long long     d_uint64;
typedef     float                  d_float;
typedef     double                 d_double;

struct AD_POINT_3D
{
    d_double        x;							// 横向坐标
    d_double        y;							// 纵向坐标
    d_double        z;							// 高度坐标
};

// 行为规划
#define MAX_CHARACTER_COUNT (200)                               // 字符最大数量
typedef struct
{
    d_int64         time_stamp;                         		// 时间戳，为格林尼治时间至当前的毫秒数
    d_int8          valid;                              		// 有效位
    d_int8          honk;										// 按喇叭
    d_int8  	    emergency_lamp;								// 应急灯
    d_int8          cornering_lamp;								// 转向灯
    d_int8          stop_lamp;									// 刹车灯
    d_int8          driving_lamp;								// 远近光灯

    d_int8          braking;                                    // 1：刹车
    d_int8          avoiding;                                   // 1：左避障， 2：右避障
    d_int8          turning;                                    // 1：左转弯， 2：右转弯
    d_int8          changing_lane;                              // 1：左变道， 2：右变道

    d_int8          prompt_id;                                  // 提示信息编号
    d_int8  	    prompt[MAX_CHARACTER_COUNT];                // 警示信息内容
}SIGNAL_BEHAVIOR_PLAN;

// 运动规划，二次规划轨迹
#define MAX_TRAJECTORY_POINT (300)
typedef struct TRAJECTORY_MOTION_PLAN_
{
	d_int64         		time_stamp;                             // 时间戳，为格林尼治时间至当前的毫秒数
	d_int8          		valid;                                  // 有效位, 0x00无效, 0x01横向, 0x02纵向
	d_int32         		count;                                  // 轨迹点数量
	struct AD_POINT_3D    	trajectory[MAX_TRAJECTORY_POINT];       // 轨迹点列表
	d_float                 target_velocity[MAX_TRAJECTORY_POINT];  // 车速（m/s）
	d_float                 tar_velocity;                           // 预瞄车速（m/s）
	d_float                 tar_acceleration;                       // 预瞄加速度（m/s^2）
}TRAJECTORY_MOTION_PLAN;