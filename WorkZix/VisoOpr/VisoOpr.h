#include "OpenCVInc.h"

#ifdef VISOOPR_EXPORTS
#define VISOOPR_API __declspec(dllexport)
#else
#define VISOOPR_API __declspec(dllimport)
#endif

typedef struct VISOOPR_API tag_VisoOprParam
{
	double f;
	double cu;
	double cv;
	double base;
	double height;
	double pitch;
	int match_radius;
	int bucket_height;
	int bucket_width;
	int	ransac_iters;
	double	inlier_threshold;
	bool reweighting;
	bool doReconstruction;
	bool doForceHorizon;
	tag_VisoOprParam()
	{
		f = 0;
		cu = 0;
		cv = 0;
		base = 1000;
		height = 1000;
		pitch = 0;
		match_radius = 200;
		bucket_height = 20;
		bucket_width = 20;
		ransac_iters = 200;
		inlier_threshold = 2.0;
		reweighting = false;
		doReconstruction = false;
		doForceHorizon = true;
	};
}CVisoOprParam;

class VISOOPR_API CVisoOpr 
{
public:
	enum VISO_TYPE
	{
		STEREO = 0,
		MONO = 1
	};
	//��ʼ����̼�
	//nType��0-˫Ŀ��̼�   1-��Ŀ��̼�
	//Param����̼Ʋ���
	CVisoOpr(CVisoOprParam Param, double dScale=1.f, VISO_TYPE type=STEREO);
	~CVisoOpr();

	//˫Ŀ��̼Ƽ���,����˫Ŀ�Ҷ�ͼ
	bool process(Mat ImgL,Mat ImgR);

	//��Ŀ��̼Ƽ���,����˫Ŀ�Ҷ�ͼ
	bool process(Mat ImgL);

	//������ͼ
	//nType��0-��Ч����  1-���й���
	long DrawOpticFlow(Mat& img, long nType=0);

	//��ȡ���˶�����
	Mat getMotion();

	//��ȡ��ǰ��̬
	Mat getPose();

// 	//��ȡ��ǰ����
 	Mat GetMonoCloudPoints();

	long DrawStereoMatch(Mat& imgMatch, Mat& imgL, Mat& imgR, vector<DMatch>& matchesT, vector<KeyPoint>& kpt0, vector<KeyPoint>& kpt1);
//	long GetPoint

	Mat GetReconstructionPoints();

private:
	long m_nType;
	
	
	void* m_pVisualOdometry;
	void* m_pReconstruction;
	Mat m_Pose;
	Mat m_Motion;
	double m_Scale;
	CVisoOprParam m_Param;
	long m_nPtsContCur;
};
