#include "BaslerProcessor.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    string szProjPath = argv[1];
	CBaslerProParam Param;
    Param.nRunMode = 1;
    if (argc >= 3)
    {
        Param.nRunMode = atoi(argv[2]);
    }
	CBaslerProcessor Opr;
	Opr.Init(Param);
	Opr.Process(szProjPath);

	printf("Finish!\n");

	getchar();
	return 1;
}
