#include <iostream>

#include "CameraOpr.h"

using namespace std;

int main(int argc, char *argv[])
{
	CCameraParam Param;
	Param.ReadParam("xmltest.xml", "test0");
// 	std::ofstream fout("file.xml");// �Ѷ���д��file.txt�ļ���  
// 	boost::archive::xml_oarchive oa(fout); // �ı�������鵵�࣬ʹ��һ��ostream������   
// //	oa << BOOST_SERIALIZATION_NVP(Param);
// 	oa << boost::serialization::make_nvp("abc123"/*BOOST_PP_STRINGIZE(Param)*/, Param);
// 	fout.close();// �ر�
	
	hCameraHandle p = CreateCameraHandle(Param);

	ReleaseCameraHandle(p);

    return 0;
}
