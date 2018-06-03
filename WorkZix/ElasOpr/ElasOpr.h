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
	//初始化参数
	CElasOpr(CElasOprParam Param);
	~CElasOpr(void);

	//输入矫正过的双目图片，输出左目视差图
	Mat process(Mat imgL, Mat imgR);

private:
	void* m_pElas;
};

