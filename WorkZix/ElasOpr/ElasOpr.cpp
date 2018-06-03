#include "ElasOpr.h"
#include "elas.h"

CElasOpr::CElasOpr(CElasOprParam Param)
{
	Elas::parameters ElasParam;
	ElasParam.postprocess_only_left = true;
	ElasParam.disp_max = Param.nMaxDisp;
	m_pElas = new Elas(ElasParam);
	return;
}

CElasOpr::~CElasOpr()
{
	if (m_pElas)
	{
		delete m_pElas;
		m_pElas = NULL;
	}
	return;
}

Mat CElasOpr::process(Mat imgL, Mat imgR)
{
	if (imgL.data == NULL || imgR.data == NULL ||
		imgL.channels() == 3 || imgR.channels() == 3)
	{
		return Mat();
	}

	const int32_t dims[3] = {imgL.cols,imgL.rows,imgL.cols};
	Mat out1_(imgL.rows,imgL.cols,CV_32F);
	Mat out2_(imgR.rows,imgR.cols,CV_32F);
	((Elas*)m_pElas)->process(imgL.data,imgR.data,(float*)out1_.data,(float*)out2_.data,dims);

	return out1_;
}