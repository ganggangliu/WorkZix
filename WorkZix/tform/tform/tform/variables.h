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
    d_double        x;							// ��������
    d_double        y;							// ��������
    d_double        z;							// �߶�����
};

// ��Ϊ�滮
#define MAX_CHARACTER_COUNT (200)                               // �ַ��������
typedef struct
{
    d_int64         time_stamp;                         		// ʱ�����Ϊ��������ʱ������ǰ�ĺ�����
    d_int8          valid;                              		// ��Чλ
    d_int8          honk;										// ������
    d_int8  	    emergency_lamp;								// Ӧ����
    d_int8          cornering_lamp;								// ת���
    d_int8          stop_lamp;									// ɲ����
    d_int8          driving_lamp;								// Զ�����

    d_int8          braking;                                    // 1��ɲ��
    d_int8          avoiding;                                   // 1������ϣ� 2���ұ���
    d_int8          turning;                                    // 1����ת�䣬 2����ת��
    d_int8          changing_lane;                              // 1�������� 2���ұ��

    d_int8          prompt_id;                                  // ��ʾ��Ϣ���
    d_int8  	    prompt[MAX_CHARACTER_COUNT];                // ��ʾ��Ϣ����
}SIGNAL_BEHAVIOR_PLAN;

// �˶��滮�����ι滮�켣
#define MAX_TRAJECTORY_POINT (300)
typedef struct TRAJECTORY_MOTION_PLAN_
{
	d_int64         		time_stamp;                             // ʱ�����Ϊ��������ʱ������ǰ�ĺ�����
	d_int8          		valid;                                  // ��Чλ, 0x00��Ч, 0x01����, 0x02����
	d_int32         		count;                                  // �켣������
	struct AD_POINT_3D    	trajectory[MAX_TRAJECTORY_POINT];       // �켣���б�
	d_float                 target_velocity[MAX_TRAJECTORY_POINT];  // ���٣�m/s��
	d_float                 tar_velocity;                           // Ԥ�鳵�٣�m/s��
	d_float                 tar_acceleration;                       // Ԥ����ٶȣ�m/s^2��
}TRAJECTORY_MOTION_PLAN;