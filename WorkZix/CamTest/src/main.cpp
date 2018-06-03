#include <iostream>

#include "CameraOpr.h"

using namespace std;

int main(int argc, char *argv[])
{
	CCameraParam Param;
	Param.ReadParam("xmltest.xml", "test0");
// 	std::ofstream fout("file.xml");// 把对象写到file.txt文件中  
// 	boost::archive::xml_oarchive oa(fout); // 文本的输出归档类，使用一个ostream来构造   
// //	oa << BOOST_SERIALIZATION_NVP(Param);
// 	oa << boost::serialization::make_nvp("abc123"/*BOOST_PP_STRINGIZE(Param)*/, Param);
// 	fout.close();// 关闭
	
	hCameraHandle p = CreateCameraHandle(Param);

	ReleaseCameraHandle(p);

    return 0;
}
