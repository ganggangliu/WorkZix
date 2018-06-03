#include "OpenCVInc.h"


#ifdef ELASOPR_EXPORTS
#define ELASOPR_API __declspec(dllexport)
#else
#define ELASOPR_API __declspec(dllimport)
#endif

typedef struct ELASOPR_API tag_ElasOprParam
{
	int nMaxDisp;
	tag_ElasOprParam()
	{
		nMaxDisp = 255;
	};
}CElasOprParam;

class ELASOPR_API CElasOpr 
{
public:
	//��ʼ������
	CElasOpr(CElasOprParam Param);
	~CElasOpr(void);

	//�����������˫ĿͼƬ�������Ŀ�Ӳ�ͼ
	Mat process(Mat imgL, Mat imgR);

private:
	void* m_pElas;
};

